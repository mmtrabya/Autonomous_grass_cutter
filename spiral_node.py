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
        self.derivative_filter = 0.5
        self.previous_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()
    
    def compute(self, setpoint, measured_value):
        current_time = time.time()
        dt = current_time - self.last_time
        if dt < 0.001:
            dt = 0.001

        error = setpoint - measured_value
        self.integral += error * dt

        raw_derivative = (error - self.previous_error) / dt
        self.last_derivative = (self.derivative_filter * raw_derivative) + ((1 - self.derivative_filter) * self.last_derivative)

        raw_output = (self.kp * error) + (self.ki * self.integral) + (self.kd * self.last_derivative)
        self.filtered_output = (self.filter_coefficient * raw_output) + ((1 - self.filter_coefficient) * self.filtered_output)
        output = max(self.output_min, min(self.filtered_output, self.output_max))

        if output >= self.output_max:
            self.integral -= error * dt
        elif output <= self.output_min:
            self.integral += error * dt

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

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)

        # Spiral parameters
        self.initial_radius = 0.3  # Start with 0.3m radius
        self.radius_increment = 0.3  # Increase radius by 0.3m per revolution
        self.current_target_radius = self.initial_radius
        self.angular_speed = 0.5  # rad/s for circular motion
        self.max_radius = 3.0  # Maximum allowed radius
        
        # Center point (will be set to starting position)
        self.center_x = None
        self.center_y = None
        
        # Boundary settings
        self.boundary_size = 2.0  # ±2m from center
        self.boundary_check_enabled = True
        
        # Position tracking
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.start_x = None
        self.start_y = None
        self.distance_from_center = 0.0
        
        # Revolution tracking
        self.last_yaw = 0.0
        self.revolutions_completed = 0
        self.yaw_initialized = False
        self.initial_yaw = None
        self.yaw_crossings = 0  # Track how many times we cross the initial yaw
        self.last_yaw_quadrant = 0  # Track which quadrant we're in
        self.completed_half_revolution = False
        
        # Speed control
        self.current_linear_speed = 0.0
        self.current_angular_speed = 0.0
        self.max_linear_speed = 2.0
        
        # PID controllers
        self.linear_pid = PIDController(1.5, 0.2, 0.1, 0.0, self.max_linear_speed, 0.3)
        self.angular_pid = PIDController(2.0, 0.1, 0.1, -2.0, 2.0, 0.3)
        
        # Control parameters
        self.update_rate = 0.05  # 20Hz update rate
        self.timer = self.create_timer(self.update_rate, self.timer_callback)
        
        self.get_logger().info(f'Spiral motion node started - Initial radius: {self.initial_radius}m, Increment: {self.radius_increment}m per revolution')
        self.publisher_.publish(Twist())

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        # Convert quaternion to yaw
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        self.current_yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))

        # Get current velocities
        self.current_linear_speed = msg.twist.twist.linear.x
        self.current_angular_speed = msg.twist.twist.angular.z

        # Set initial position as center
        if self.start_x is None:
            self.start_x = self.current_x
            self.start_y = self.current_y
            self.center_x = self.current_x
            self.center_y = self.current_y
            self.last_yaw = self.current_yaw
            self.initial_yaw = self.current_yaw
            self.last_yaw_quadrant = self.get_yaw_quadrant(self.current_yaw)
            self.yaw_initialized = True
            self.get_logger().info(
                f'Center point set: x={self.center_x:.2f}, y={self.center_y:.2f}, yaw={self.current_yaw:.2f}\n'
                f'Boundaries: x=[{self.center_x-self.boundary_size:.2f}, {self.center_x+self.boundary_size:.2f}], '
                f'y=[{self.center_y-self.boundary_size:.2f}, {self.center_y+self.boundary_size:.2f}]'
            )

        # Calculate distance from center
        if self.center_x is not None and self.center_y is not None:
            self.distance_from_center = math.sqrt(
                (self.current_x - self.center_x)**2 + (self.current_y - self.center_y)**2)
            
            # Track revolutions
            self.track_revolutions()

    def get_yaw_quadrant(self, yaw):
        """Get quadrant (0-3) based on yaw angle"""
        # Normalize yaw to [0, 2π]
        normalized_yaw = yaw
        while normalized_yaw < 0:
            normalized_yaw += 2 * math.pi
        while normalized_yaw >= 2 * math.pi:
            normalized_yaw -= 2 * math.pi
            
        if normalized_yaw < math.pi / 2:
            return 0
        elif normalized_yaw < math.pi:
            return 1
        elif normalized_yaw < 3 * math.pi / 2:
            return 2
        else:
            return 3

    def track_revolutions(self):
        if not self.yaw_initialized or self.initial_yaw is None:
            return
            
        current_quadrant = self.get_yaw_quadrant(self.current_yaw)
        
        # Simple revolution tracking: count transitions through quadrants
        # A full revolution means going through all 4 quadrants in order
        
        # Check if we moved to the next quadrant
        if current_quadrant != self.last_yaw_quadrant:
            # Check if we're progressing through quadrants (clockwise: 0->1->2->3->0)
            next_expected_quadrant = (self.last_yaw_quadrant + 1) % 4
            
            if current_quadrant == next_expected_quadrant:
                # We're progressing correctly
                if current_quadrant == 2:  # Reached quadrant 2 (π to 3π/2)
                    self.completed_half_revolution = True
                elif current_quadrant == 0 and self.completed_half_revolution:
                    # Completed a full revolution (back to quadrant 0 after going through 2)
                    self.revolutions_completed += 1
                    old_radius = self.current_target_radius
                    self.current_target_radius = self.initial_radius + (self.revolutions_completed * self.radius_increment)
                    self.completed_half_revolution = False
                    
                    self.get_logger().info(
                        f'🔄 Revolution {self.revolutions_completed} completed! '
                        f'Radius: {old_radius:.2f}m -> {self.current_target_radius:.2f}m'
                    )
                    
                    # Check if we've reached maximum radius
                    if self.current_target_radius >= self.max_radius:
                        self.get_logger().info(f'Maximum radius {self.max_radius}m reached. Stopping.')
                        self.stop_robot()
                        return
            
            self.last_yaw_quadrant = current_quadrant

    def check_boundaries(self):
        """Check if robot is approaching or has exceeded boundaries"""
        if not self.boundary_check_enabled or self.center_x is None or self.center_y is None:
            return True
            
        # Calculate distances to each boundary
        distance_to_left = abs(self.current_x - (self.center_x - self.boundary_size))
        distance_to_right = abs((self.center_x + self.boundary_size) - self.current_x)
        distance_to_bottom = abs(self.current_y - (self.center_y - self.boundary_size))
        distance_to_top = abs((self.center_y + self.boundary_size) - self.current_y)
        
        # Check if outside boundaries
        if (self.current_x < self.center_x - self.boundary_size or 
            self.current_x > self.center_x + self.boundary_size or
            self.current_y < self.center_y - self.boundary_size or
            self.current_y > self.center_y + self.boundary_size):
            
            self.get_logger().warning(
                f'Boundary exceeded! Position: x={self.current_x:.2f}, y={self.current_y:.2f}, '
                f'Center: x={self.center_x:.2f}, y={self.center_y:.2f}'
            )
            return False
            
        return True

    def calculate_target_speeds(self):
        """Calculate target linear and angular speeds for spiral motion around center"""
        # For circular motion around center: v = ω * r
        target_linear_speed = self.angular_speed * self.current_target_radius
        target_angular_speed = self.angular_speed
        
        # Add a small centripetal force correction to maintain circular motion
        # The robot needs to "pull" towards the center while moving forward
        if self.center_x is not None and self.center_y is not None:
            # Calculate angle from center to current position
            angle_to_center = math.atan2(self.current_y - self.center_y, self.current_x - self.center_x)
            # Calculate desired heading (perpendicular to radius for circular motion)
            desired_heading = angle_to_center + math.pi/2  # 90 degrees ahead for counterclockwise
            
            # Calculate heading error
            heading_error = desired_heading - self.current_yaw
            
            # Normalize heading error
            while heading_error > math.pi:
                heading_error -= 2 * math.pi
            while heading_error < -math.pi:
                heading_error += 2 * math.pi
            
            # Adjust angular speed based on heading error
            target_angular_speed += heading_error * 0.5  # Proportional control
        
        # Limit maximum linear speed
        if target_linear_speed > self.max_linear_speed:
            target_linear_speed = self.max_linear_speed
            target_angular_speed = target_linear_speed / self.current_target_radius
            
        return target_linear_speed, target_angular_speed

    def stop_robot(self):
        """Stop the robot and shutdown the node"""
        stop_msg = Twist()
        self.publisher_.publish(stop_msg)
        self.timer.cancel()
        rclpy.shutdown()

    def timer_callback(self):
        if self.start_x is None:
            self.get_logger().info('Waiting for initial odometry data...')
            return

        # Check boundaries
        if not self.check_boundaries():
            self.stop_robot()
            return

        # Check if maximum radius reached
        if self.current_target_radius >= self.max_radius:
            self.get_logger().info(f'Maximum radius {self.max_radius}m reached. Stopping.')
            self.stop_robot()
            return

        # Calculate target speeds for current radius
        target_linear_speed, target_angular_speed = self.calculate_target_speeds()

        # Use PID controllers to achieve target speeds
        linear_output = self.linear_pid.compute(target_linear_speed, self.current_linear_speed)
        angular_output = self.angular_pid.compute(target_angular_speed, self.current_angular_speed)

        # Create and publish velocity command
        twist_msg = Twist()
        twist_msg.linear.x = linear_output
        twist_msg.angular.z = angular_output
        self.publisher_.publish(twist_msg)

        # Log current status
        current_quadrant = self.get_yaw_quadrant(self.current_yaw) if self.yaw_initialized else 0
        self.get_logger().info(
            f'Rev: {self.revolutions_completed} | Target radius: {self.current_target_radius:.2f}m | '
            f'Dist from center: {self.distance_from_center:.2f}m | '
            f'Quadrant: {current_quadrant} | Half-rev: {self.completed_half_revolution} | '
            f'Target speeds: lin={target_linear_speed:.3f}, ang={target_angular_speed:.3f} | '
            f'Command: lin={linear_output:.3f}, ang={angular_output:.3f} | '
            f'Position: x={self.current_x:.2f}, y={self.current_y:.2f}, yaw={self.current_yaw:.2f}'
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
