# spiral_motion/spiral_motion/spiral_node.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
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
        self.derivative_filter = 0.7  # Increased filtering for real hardware
        
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
        
        # Create subscriber for odometry
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)

        # Spiral motion parameters - SIGNIFICANTLY INCREASED for faster expansion
        self.desired_angular_speed = 0.4  # rad/s - slightly reduced for better stability
        self.initial_linear_speed = 0.08  # m/s - increased starting value
        self.linear_speed_increment = 0.004  # m/s per cycle - GREATLY increased for faster expansion
        self.max_linear_speed = 0.25  # m/s - based on 37 RPM motors with 13cm wheels
        self.update_rate = 0.6  # seconds - match control node's 0.5s delay plus buffer
        
        # Flag to ensure commands are sent
        self.last_cmd_time = time.time()
        self.cmd_timeout = 0.7  # seconds - longer than control node delay
        self.last_published_cmd = Twist()

        # Current target speeds
        self.target_linear_speed = self.initial_linear_speed
        
        # Actual speeds from odometry
        self.current_linear_speed = 0.0
        self.current_angular_speed = 0.0
        
        # Odometry tracking
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.start_x = None
        self.start_y = None
        self.distance_from_start = 0.0
        self.last_odom_time = None
        self.odom_timeout = 2.0  # seconds - increased for slower updates
        
        # PID controllers - adjusted for more aggressive acceleration
        self.linear_pid = PIDController(kp=1.0, ki=0.2, kd=0.05, output_min=0.0, output_max=self.max_linear_speed, filter_coefficient=0.5)
        self.angular_pid = PIDController(kp=1.0, ki=0.2, kd=0.1, output_min=-0.5, output_max=0.5, filter_coefficient=0.5)
        
        # Boundary parameters - INCREASED for larger spiral
        self.boundary_radius = 5.0  # meters - INCREASED from 2.0 to 5.0
        self.boundary_x_min = 0.0  # To be set after initialization
        self.boundary_x_max = 0.0  # To be set after initialization
        self.boundary_y_min = 0.0  # To be set after initialization
        self.boundary_y_max = 0.0  # To be set after initialization
        self.max_distance = self.boundary_radius  # Set max distance to boundary radius
        
        # Boundary approach parameters
        self.boundary_slowdown_distance = 0.5  # meters - increased for earlier slowdown with control delay
        self.boundary_approaching = False

        # Create timer with rate matching control node delay
        self.timer = self.create_timer(self.update_rate, self.timer_callback)
        
        # Variables to track motion progress
        self.last_radius_check_time = time.time()
        self.radius_check_interval = 6.0  # Check every 6 seconds - increased for slower updates
        self.last_radius = 0.0
        self.stalled_count = 0
        self.movement_detection_threshold = 0.03  # meters - increased threshold for detection

        # Flag to ensure we accelerate quickly at the beginning
        self.startup_phase = True
        self.startup_phase_duration = 5.0  # seconds
        self.startup_time = time.time()

        self.get_logger().info('Spiral motion node with PID control has been started (optimized for larger spiral)')

        # Publish an initial zero message
        zero_twist = Twist()
        self.publisher_.publish(zero_twist)
        self.last_published_cmd = zero_twist
        self.get_logger().info('Published initial zero command')

    def odom_callback(self, msg):
        # Update last odometry time
        self.last_odom_time = time.time()
        
        # Extract position from odometry message
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        # Extract orientation and convert to yaw
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        self.current_yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
        
        # Extract linear and angular velocities
        self.current_linear_speed = msg.twist.twist.linear.x
        self.current_angular_speed = msg.twist.twist.angular.z
        
        # Set initial position if not set and define boundaries based on it
        if self.start_x is None:
            self.start_x = self.current_x
            self.start_y = self.current_y
            
            # Set boundaries as ±boundary_radius meters from center point
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
            if abs(radius_change) < self.movement_detection_threshold and self.target_linear_speed > 0.05:
                self.stalled_count += 1
                if self.stalled_count >= 2:  # If stalled for multiple checks
                    self.get_logger().warning('Spiral not expanding, GREATLY increasing linear_speed_increment')
                    self.linear_speed_increment *= 2.0  # DOUBLED speed increment for stuck conditions
                    # Try to break out of stall by temporarily increasing angular speed
                    self.desired_angular_speed *= 1.2
                    self.stalled_count = 0
            else:
                self.stalled_count = 0
                # Reset angular speed if we were previously stalled
                if self.desired_angular_speed > 0.4:
                    self.desired_angular_speed = 0.4
                
            self.last_radius = self.distance_from_start
            self.last_radius_check_time = current_time

        # Handle startup phase - extra boost to get moving quickly
        if self.startup_phase and current_time - self.startup_time > self.startup_phase_duration:
            self.startup_phase = False
            self.get_logger().info('Startup phase complete, continuing with normal expansion')
        
        self.get_logger().debug(f'Position: x={self.current_x:.2f}, y={self.current_y:.2f}, ' +
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
        
        # Predict position after control delay for early boundary detection
        delay_time = 0.5  # Control node delay in seconds
        predicted_x = self.current_x + self.current_linear_speed * math.cos(self.current_yaw) * delay_time
        predicted_y = self.current_y + self.current_linear_speed * math.sin(self.current_yaw) * delay_time
        
        # Calculate predicted distance from center
        predicted_distance = math.sqrt(
            (predicted_x - self.start_x) ** 2 + 
            (predicted_y - self.start_y) ** 2
        )
        
        # Calculate predicted distances to boundaries
        pred_distance_to_x_min = abs(predicted_x - self.boundary_x_min)
        pred_distance_to_x_max = abs(self.boundary_x_max - predicted_x)
        pred_distance_to_y_min = abs(predicted_y - self.boundary_y_min)
        pred_distance_to_y_max = abs(self.boundary_y_max - predicted_y)
        pred_distance_to_circular = self.boundary_radius - predicted_distance
        
        # Find the closest boundary based on prediction
        pred_min_distance = min(pred_distance_to_x_min, pred_distance_to_x_max,
                               pred_distance_to_y_min, pred_distance_to_y_max, 
                               pred_distance_to_circular)
        
        # Use the smaller of current or predicted distance for safety
        effective_distance = min(min_distance, pred_min_distance)
        
        # Check if approaching boundary
        if effective_distance < self.boundary_slowdown_distance:
            self.boundary_approaching = True
            # Slow down proportionally to boundary proximity
            slowdown_factor = effective_distance / self.boundary_slowdown_distance
            return slowdown_factor
        else:
            self.boundary_approaching = False
            
        # Check if the robot is within defined boundaries (current or predicted)
        if (predicted_x < self.boundary_x_min or 
            predicted_x > self.boundary_x_max or
            predicted_y < self.boundary_y_min or
            predicted_y > self.boundary_y_max or
            predicted_distance > self.boundary_radius or
            self.current_x < self.boundary_x_min or 
            self.current_x > self.boundary_x_max or
            self.current_y < self.boundary_y_min or
            self.current_y > self.boundary_y_max or
            self.distance_from_start > self.boundary_radius):
            
            self.get_logger().warning(f'Boundary reached at position x={self.current_x:.2f}, y={self.current_y:.2f}')
            return 0.0  # Return 0 to stop the robot
        
        return 1.0  # Normal operation

    def timer_callback(self):
        # Check if odometry data is too old
        current_time = time.time()
        if self.last_odom_time is not None and current_time - self.last_odom_time > self.odom_timeout:
            self.get_logger().warning('Odometry data timeout! Stopping for safety.')
            stop_msg = Twist()
            self.publisher_.publish(stop_msg)
            self.last_published_cmd = stop_msg
            self.last_cmd_time = current_time
            return
            
        # Skip if we don't have odometry data yet
        if self.start_x is None:
            self.get_logger().info('Waiting for initial odometry data...')
            return

        # Check boundaries and get slowdown factor
        boundary_factor = self.check_boundaries()
        
        if boundary_factor <= 0:
            stop_msg = Twist()
            self.publisher_.publish(stop_msg)
            self.last_published_cmd = stop_msg
            self.last_cmd_time = current_time
            self.get_logger().info('Boundary reached. Stopping robot and shutting down node.')
            self.timer.cancel()  # Stop the timer
            rclpy.shutdown()  # Shut down ROS
            return
        
        # If in startup phase, apply extra acceleration
        if self.startup_phase:
            # Higher linear speed increment during startup
            self.linear_speed_increment = 0.006  # Very aggressive during startup
        else:
            # Return to normal increment after startup
            self.linear_speed_increment = 0.004
        
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
            self.last_published_cmd = stop_msg
            self.last_cmd_time = current_time
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

        # Always publish the command since our rate is matched to control node's delay
        self.publisher_.publish(twist_msg)
        self.last_published_cmd = twist_msg
        self.last_cmd_time = current_time
        
        # Log that we published a command
        self.get_logger().info(
            f'CMD: lin={linear_output:.3f}, ang={angular_output:.3f} | ' +
            f'Pos: x={self.current_x:.2f}, y={self.current_y:.2f}, dist={self.distance_from_start:.2f}m | ' +
            f'Target lin={adjusted_target_linear:.3f}, increment={self.linear_speed_increment:.5f}'
        )
        
        # Increment target linear speed according to spiral pattern
        self.target_linear_speed += self.linear_speed_increment

def main(args=None):
    rclpy.init(args=args)

    spiral_motion_node = SpiralMotionNode()
    rclpy.spin(spiral_motion_node)
