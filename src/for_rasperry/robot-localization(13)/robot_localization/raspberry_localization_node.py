# spiral_motion/spiral_motion/spiral_node.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
import math
import time

class PIDController:
    def __init__(self, kp, ki, kd, output_min, output_max, filter_coefficient=0.2):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max
        self.filter_coefficient = filter_coefficient
        self.filtered_output = 0.0
        self.last_derivative = 0.0
        self.derivative_filter = 0.5
        
        self.previous_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()
    
    def compute(self, setpoint, measured_value):
        # Calculate time since last computation
        current_time = time.time()
        dt = current_time - self.last_time
        
        # Avoid division by zero or very small dt
        if dt < 0.001:
            dt = 0.001
            
        # Calculate error
        error = setpoint - measured_value
        
        # Calculate integral term with anti-windup
        self.integral += error * dt
        
        # Calculate and filter derivative term
        raw_derivative = (error - self.previous_error) / dt
        self.last_derivative = (self.derivative_filter * raw_derivative) + ((1 - self.derivative_filter) * self.last_derivative)
        
        # Calculate raw output
        raw_output = (self.kp * error) + (self.ki * self.integral) + (self.kd * self.last_derivative)
        
        # Apply low-pass filter
        self.filtered_output = (self.filter_coefficient * raw_output) + ((1 - self.filter_coefficient) * self.filtered_output)
        
        # Clamp output to limits
        output = max(self.output_min, min(self.filtered_output, self.output_max))
        
        # Enhanced anti-windup
        if output >= self.output_max:
            self.integral -= error * dt
        elif output <= self.output_min:
            self.integral += error * dt
        
        # Save values for next computation
        self.previous_error = error
        self.last_time = current_time
        
        return output
    
    def reset(self):
        self.previous_error = 0.0
        self.integral = 0.0
        self.filtered_output = 0.0
        self.last_derivative = 0.0
        self.last_time = time.time()


class SpiralMotionNode(Node):
    def __init__(self):
        super().__init__('spiral_motion_node')

        # Create publisher for cmd_vel topic
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Create subscriber for wheel_pose topic
        self.wheel_pose_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/wheel_pose',
            self.wheel_pose_callback,
            10)

        # Spiral motion parameters
        self.desired_angular_speed = 0.5  # rad/s
        self.initial_linear_speed = 0.1  # m/s
        self.linear_speed_increment = 0.001  # m/s per cycle
        self.max_linear_speed = 12  # m/s
        self.update_rate = 0.1  # seconds

        # Current target speeds
        self.target_linear_speed = self.initial_linear_speed
        
        # Actual speeds from wheel pose (calculated from position changes)
        self.current_linear_speed = 0.0
        self.current_angular_speed = 0.0
        
        # Previous values for velocity calculation
        self.prev_x = None
        self.prev_y = None
        self.prev_yaw = None
        self.prev_time = None
        
        # Odometry tracking
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.start_x = None
        self.start_y = None
        self.distance_from_start = 0.0
        
        # PID controllers
        self.linear_pid = PIDController(kp=0.8, ki=0.1, kd=0.05, output_min=0.0, output_max=self.max_linear_speed, filter_coefficient=0.3)
        self.angular_pid = PIDController(kp=1.0, ki=0.1, kd=0.1, output_min=-1.0, output_max=1.0, filter_coefficient=0.3)
        
        # Boundary parameters - will be set when initial position is known
        self.boundary_radius = 2.0  # 2 meters in each direction from center
        self.boundary_x_min = 0.0  # To be set after initialization
        self.boundary_x_max = 0.0  # To be set after initialization
        self.boundary_y_min = 0.0  # To be set after initialization
        self.boundary_y_max = 0.0  # To be set after initialization
        self.max_distance = self.boundary_radius  # Set max distance to boundary radius
        
        # Boundary approach parameters
        self.boundary_slowdown_distance = 0.3  # meters
        self.boundary_approaching = False

        # Create timer
        self.timer = self.create_timer(self.update_rate, self.timer_callback)

        # Variables to track motion progress
        self.last_radius_check_time = time.time()
        self.radius_check_interval = 3.0  # Check every 3 seconds
        self.last_radius = 0.0
        self.stalled_count = 0

        self.get_logger().info('Spiral motion node with PID control has been started')

        # Publish an initial zero message
        zero_twist = Twist()
        self.publisher_.publish(zero_twist)
        self.get_logger().info('Published initial zero command')

    def wheel_pose_callback(self, msg):
        # Extract position from wheel pose message (PoseWithCovarianceStamped)
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        # Extract orientation and convert to yaw
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        self.current_yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
        
        # Calculate velocities from position changes
        current_time = time.time()
        if self.prev_x is not None and self.prev_time is not None:
            dt = current_time - self.prev_time
            if dt > 0.01:  # Avoid too frequent calculations
                # Calculate linear velocity
                dx = self.current_x - self.prev_x
                dy = self.current_y - self.prev_y
                self.current_linear_speed = math.sqrt(dx*dx + dy*dy) / dt
                
                # Calculate angular velocity
                dyaw = self.current_yaw - self.prev_yaw
                # Handle angle wrapping
                if dyaw > math.pi:
                    dyaw -= 2 * math.pi
                elif dyaw < -math.pi:
                    dyaw += 2 * math.pi
                self.current_angular_speed = dyaw / dt
                
                # Store current values for next iteration
                self.prev_x = self.current_x
                self.prev_y = self.current_y
                self.prev_yaw = self.current_yaw
                self.prev_time = current_time
        
        # Store initial values if this is the first callback
        if self.prev_x is None:
            self.prev_x = self.current_x
            self.prev_y = self.current_y
            self.prev_yaw = self.current_yaw
            self.prev_time = current_time
        
        # Set initial position if not set and define boundaries based on it
        if self.start_x is None:
            self.start_x = self.current_x
            self.start_y = self.current_y
            
            # Set boundaries as ±2 meters from center point
            self.boundary_x_min = self.start_x - self.boundary_radius
            self.boundary_x_max = self.start_x + self.boundary_radius
            self.boundary_y_min = self.start_y - self.boundary_radius
            self.boundary_y_max = self.start_y + self.boundary_radius
            
            self.get_logger().info(
                f'Initial position set: x={self.start_x:.2f}, y={self.start_y:.2f}\n' +
                f'Boundaries set: x=[{self.boundary_x_min:.2f}, {self.boundary_x_max:.2f}], ' +
                f'y=[{self.boundary_y_min:.2f}, {self.boundary_y_max:.2f}]'
            )
        
        # Calculate distance from start (center)
        self.distance_from_start = math.sqrt(
            (self.current_x - self.start_x) ** 2 + 
            (self.current_y - self.start_y) ** 2
        )
        
        # Check if spiral is expanding
        current_time = time.time()
        if current_time - self.last_radius_check_time > self.radius_check_interval:
            radius_change = self.distance_from_start - self.last_radius
            self.get_logger().info(f'Radius change over {self.radius_check_interval}s: {radius_change:.3f}m')
            
            # If radius hasn't changed significantly, we might be stuck
            if abs(radius_change) < 0.05 and self.target_linear_speed > 0.2:
                self.stalled_count += 1
                if self.stalled_count >= 2:  # If stalled for multiple checks
                    self.get_logger().warning('Spiral not expanding, increasing linear_speed_increment')
                    self.linear_speed_increment *= 1.5  # Increase speed increment
                    self.stalled_count = 0
            else:
                self.stalled_count = 0
                
            self.last_radius = self.distance_from_start
            self.last_radius_check_time = current_time
        
        self.get_logger().debug(f'Current position: x={self.current_x:.2f}, y={self.current_y:.2f}, ' +
                              f'distance from center: {self.distance_from_start:.2f}m')

    def check_boundaries(self):
        # Calculate distances to each boundary
        distance_to_x_min = abs(self.current_x - self.boundary_x_min)
        distance_to_x_max = abs(self.boundary_x_max - self.current_x)
        distance_to_y_min = abs(self.current_y - self.boundary_y_min)
        distance_to_y_max = abs(self.boundary_y_max - self.current_y)
        
        # Calculate distance to circular boundary from center
        distance_to_circular_boundary = self.boundary_radius - self.distance_from_start
        
        # Find the closest boundary
        min_distance = min(distance_to_x_min, distance_to_x_max,
                          distance_to_y_min, distance_to_y_max, 
                          distance_to_circular_boundary)
        
        # Check if approaching boundary
        if min_distance < self.boundary_slowdown_distance:
            self.boundary_approaching = True
            # Slow down proportionally to boundary proximity
            slowdown_factor = min_distance / self.boundary_slowdown_distance
            return slowdown_factor
        else:
            self.boundary_approaching = False
            
        # Check if the robot is within defined boundaries
        if (self.current_x < self.boundary_x_min or 
            self.current_x > self.boundary_x_max or
            self.current_y < self.boundary_y_min or
            self.current_y > self.boundary_y_max or
            self.distance_from_start > self.boundary_radius):
            
            self.get_logger().warning(f'Boundary reached at position x={self.current_x:.2f}, y={self.current_y:.2f}')
            return 0.0  # Return 0 to stop the robot
        
        return 1.0  # Normal operation

    def timer_callback(self):
        # Skip if we don't have wheel pose data yet
        if self.start_x is None:
            self.get_logger().info('Waiting for initial wheel pose data...')
            return

        # Check boundaries and get slowdown factor
        boundary_factor = self.check_boundaries()
        
        if boundary_factor <= 0:
            stop_msg = Twist()
            self.publisher_.publish(stop_msg)
            self.get_logger().info('Boundary reached. Stopping robot and shutting down node.')
            self.timer.cancel()  # Stop the timer
            rclpy.shutdown()  # Shut down ROS
            return
        
        # If approaching boundary, adjust target speeds
        if self.boundary_approaching:
            adjusted_target_linear = self.target_linear_speed * boundary_factor
            adjusted_target_angular = self.desired_angular_speed * boundary_factor
            self.get_logger().info(f'Approaching boundary. Slowing down: factor={boundary_factor:.2f}')
        else:
            adjusted_target_linear = self.target_linear_speed
            adjusted_target_angular = self.desired_angular_speed
        
        # Check if we've reached the maximum speed
        if self.target_linear_speed >= self.max_linear_speed:
            stop_msg = Twist()
            self.publisher_.publish(stop_msg)
            self.get_logger().info('Max speed reached. Stopping robot and shutting down node.')
            self.timer.cancel()  # Stop the timer
            rclpy.shutdown()  # Shut down ROS
            return

        # Use PID to compute linear and angular velocity commands
        linear_output = self.linear_pid.compute(adjusted_target_linear, self.current_linear_speed)
        angular_output = self.angular_pid.compute(adjusted_target_angular, self.current_angular_speed)
        
        # Create Twist message
        twist_msg = Twist()
        twist_msg.linear.x = linear_output
        twist_msg.angular.z = angular_output

        # Publish Twist message
        self.publisher_.publish(twist_msg)

        # Increment target linear speed according to spiral pattern
        self.target_linear_speed += self.linear_speed_increment

        # Log current speeds and position
        self.get_logger().info(
            f'Target: lin={adjusted_target_linear:.3f}, ang={adjusted_target_angular:.3f} | ' +
            f'Command: lin={linear_output:.3f}, ang={angular_output:.3f} | ' +
            f'Position: x={self.current_x:.2f}, y={self.current_y:.2f}, dist={self.distance_from_start:.2f}m'
        )

def main(args=None):
    rclpy.init(args=args)

    spiral_motion_node = SpiralMotionNode()

    try:
        rclpy.spin(spiral_motion_node)
    except KeyboardInterrupt:
        stop_msg = Twist()
        spiral_motion_node.publisher_.publish(stop_msg)
        spiral_motion_node.get_logger().info('KeyboardInterrupt: Stopping robot and shutting down node')
    finally:
        spiral_motion_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__': 
    main()
