import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
import math
import time

class PIDController:
    def __init__(self, kp, ki, kd, output_min, output_max):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max
        
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
        
        # Calculate derivative term
        derivative = (error - self.previous_error) / dt
        
        # Calculate output
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        
        # Clamp output to limits
        output = max(self.output_min, min(output, self.output_max))
        
        # Save values for next computation
        self.previous_error = error
        self.last_time = current_time
        
        return output
    
    def reset(self):
        self.previous_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()


class SpiralMotionNode(Node):
    def __init__(self):
        super().__init__('spiral_motion_node')

        # Create publisher for cmd_vel topic
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Create subscriber for localization pose
        self.pose_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/localization/pose',
            self.pose_callback,
            10)

        # Spiral motion parameters - ALWAYS MAXIMUM for low torque motors
        self.max_linear_speed = 1.0   # m/s - Set your robot's maximum safe speed
        self.max_angular_speed = 1.0  # rad/s - Set your robot's maximum angular speed
        self.update_rate = 0.1  # seconds

        # Always use maximum speeds
        self.target_linear_speed = self.max_linear_speed
        self.target_angular_speed = self.max_angular_speed
        
        # Actual speeds (will need to be estimated from position changes)
        self.current_linear_speed = 0.0
        self.current_angular_speed = 0.0
        self.previous_time = None
        self.previous_x = None
        self.previous_y = None
        
        # Center point (will be set from first odometry reading)
        self.center_x = None
        self.center_y = None
        
        # Current position and orientation
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        
        # Revolution tracking for corrections
        self.previous_yaw = 0.0
        self.total_rotation = 0.0  # Total rotation in radians
        self.revolutions_completed = 0
        self.last_correction_revolution = -1
        
        # Boundary corners (will be calculated once center is set)
        self.boundary_corners = []
        self.current_target_corner = 0
        self.boundary_size = 2.0  # Changed from 1.0 to 2.0 meters from center
        self.boundary_tolerance = 0.10  # 10 cm tolerance
        
        # Spiral completion tracking
        self.corners_reached = [False, False, False, False]
        self.spiral_complete = False
        
        # Initial position for center point
        self.first_position_received = False
        self.initial_x = None
        self.initial_y = None
        
        # Position correction parameters
        self.position_error_threshold = 0.15  # meters
        self.correction_active = False
        self.target_x = 0.0
        self.target_y = 0.0

        # Create timer
        self.timer = self.create_timer(self.update_rate, self.timer_callback)

        self.get_logger().info('Spiral motion node with 2-meter boundary started')

        # Publish an initial zero message
        zero_twist = Twist()
        self.publisher_.publish(zero_twist)

    def pose_callback(self, msg):
        # Extract position from PoseWithCovarianceStamped message
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        # Extract orientation and convert to yaw
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        current_yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
        
        # Save first position as initial and center point
        if not self.first_position_received:
            self.initial_x = self.current_x
            self.initial_y = self.current_y
            self.center_x = self.initial_x
            self.center_y = self.initial_y
            self.setup_boundary_corners()
            self.first_position_received = True
            self.get_logger().info(f'Initial position set: x={self.initial_x:.2f}, y={self.initial_y:.2f}')
            self.get_logger().info(f'Center position set: x={self.center_x:.2f}, y={self.center_y:.2f}')
        
        # Optional: Check pose quality from covariance
        # Position uncertainty (x, y variances)
        x_variance = msg.pose.covariance[0]   # covariance[0] = x variance
        y_variance = msg.pose.covariance[7]   # covariance[7] = y variance
        yaw_variance = msg.pose.covariance[35] # covariance[35] = yaw variance
        
        # Log warning if localization uncertainty is high
        if x_variance > 0.1 or y_variance > 0.1:  # 10cm variance threshold
            self.get_logger().warning(f'High localization uncertainty: x_var={x_variance:.3f}, y_var={y_variance:.3f}')
        
        # Estimate velocities from position changes
        current_time = time.time()
        if self.previous_time is not None and self.previous_x is not None:
            dt = current_time - self.previous_time
            if dt > 0.001:  # Avoid division by zero
                # Calculate linear velocity
                dx = self.current_x - self.previous_x
                dy = self.current_y - self.previous_y
                self.current_linear_speed = math.sqrt(dx*dx + dy*dy) / dt
                
                # Calculate angular velocity
                dyaw = current_yaw - self.previous_yaw
                # Handle angle wraparound
                if dyaw > math.pi:
                    dyaw -= 2 * math.pi
                elif dyaw < -math.pi:
                    dyaw += 2 * math.pi
                self.current_angular_speed = dyaw / dt
        
        # Store previous values
        self.previous_time = current_time
        self.previous_x = self.current_x
        self.previous_y = self.current_y
        
        # Track rotation for revolution counting
        if self.previous_yaw is not None:
            yaw_diff = current_yaw - self.previous_yaw
            # Handle angle wraparound
            if yaw_diff > math.pi:
                yaw_diff -= 2 * math.pi
            elif yaw_diff < -math.pi:
                yaw_diff += 2 * math.pi
            
            self.total_rotation += abs(yaw_diff)
            
            # Check for completed revolution
            if self.total_rotation >= 2 * math.pi:
                self.revolutions_completed += 1
                self.total_rotation = 0.0
                self.get_logger().info(f'Revolution {self.revolutions_completed} completed')
        
        self.current_yaw = current_yaw
        self.previous_yaw = current_yaw

    def setup_boundary_corners(self):
        """Setup the four boundary corners based on center position with 2m distance"""
        if self.center_x is None or self.center_y is None:
            return
            
        # Define corners: [x+2, y+2], [x-2, y+2], [x-2, y-2], [x+2, y-2]
        self.boundary_corners = [
            (self.center_x + self.boundary_size, self.center_y + self.boundary_size),  # Top-right
            (self.center_x - self.boundary_size, self.center_y + self.boundary_size),  # Top-left
            (self.center_x - self.boundary_size, self.center_y - self.boundary_size),  # Bottom-left
            (self.center_x + self.boundary_size, self.center_y - self.boundary_size)   # Bottom-right
        ]
        
        self.get_logger().info(f'Boundary corners set with 2m distance:')
        for i, corner in enumerate(self.boundary_corners):
            self.get_logger().info(f'  Corner {i+1}: ({corner[0]:.2f}, {corner[1]:.2f})')

    def check_corner_reached(self, corner_idx):
        """Check if robot has reached a specific corner within tolerance"""
        if corner_idx >= len(self.boundary_corners):
            return False
            
        corner_x, corner_y = self.boundary_corners[corner_idx]
        distance = math.sqrt((self.current_x - corner_x)**2 + (self.current_y - corner_y)**2)
        
        return distance <= self.boundary_tolerance

    def check_all_corners_reached(self):
        """Check if robot has been near all four corners"""
        for i, corner in enumerate(self.boundary_corners):
            if not self.corners_reached[i]:
                if self.check_corner_reached(i):
                    self.corners_reached[i] = True
                    self.get_logger().info(f'Corner {i+1} reached: {corner}')
        
        # Check if all corners have been reached
        if all(self.corners_reached) and not self.spiral_complete:
            self.spiral_complete = True
            self.get_logger().info('All boundary corners reached! Spiral complete.')
            return True
        
        return False

    def calculate_expected_position(self):
        """Calculate expected position based on spiral pattern"""
        # Simple spiral calculation based on revolutions and angular position
        initial_radius = 0.1  # Initial radius
        radius = initial_radius * (self.revolutions_completed + 1)
        angle = self.current_yaw
        
        expected_x = self.center_x + radius * math.cos(angle)
        expected_y = self.center_y + radius * math.sin(angle)
        
        return expected_x, expected_y

    def should_apply_correction(self):
        """Determine if position correction should be applied"""
        if self.revolutions_completed <= self.last_correction_revolution:
            return False
            
        if self.revolutions_completed == 0:
            return False
            
        # Calculate position error
        expected_x, expected_y = self.calculate_expected_position()
        error = math.sqrt((self.current_x - expected_x)**2 + (self.current_y - expected_y)**2)
        
        if error > self.position_error_threshold:
            self.target_x = expected_x
            self.target_y = expected_y
            self.last_correction_revolution = self.revolutions_completed
            self.get_logger().info(f'Position correction needed. Error: {error:.3f}m')
            return True
            
        return False

    def apply_position_correction(self, twist_msg):
        """Apply position correction to the twist message"""
        # Calculate distance and angle to target
        dx = self.target_x - self.current_x
        dy = self.target_y - self.current_y
        distance_to_target = math.sqrt(dx**2 + dy**2)
        angle_to_target = math.atan2(dy, dx)
        
        # Calculate angle error
        angle_error = angle_to_target - self.current_yaw
        
        # Normalize angle error to [-pi, pi]
        while angle_error > math.pi:
            angle_error -= 2 * math.pi
        while angle_error < -math.pi:
            angle_error += 2 * math.pi
        
        # If we're close enough, stop correction
        if distance_to_target < 0.05:  # 5cm tolerance
            self.correction_active = False
            self.get_logger().info('Position correction complete')
            return
        
        # Apply correction commands
        correction_linear = min(0.2, distance_to_target * 2.0)  # Proportional control
        correction_angular = angle_error * 2.0  # Proportional control
        
        # Blend correction with spiral motion
        blend_factor = min(1.0, distance_to_target / 0.2)
        
        twist_msg.linear.x = twist_msg.linear.x * (1 - blend_factor) + correction_linear * blend_factor
        twist_msg.angular.z = twist_msg.angular.z * (1 - blend_factor) + correction_angular * blend_factor

    def timer_callback(self):
        # Skip if we don't have pose data yet
        if not self.first_position_received:
            self.get_logger().info('Waiting for initial pose data...')
            return

        # Check if spiral is complete
        if self.check_all_corners_reached():
            stop_msg = Twist()
            self.publisher_.publish(stop_msg)
            self.get_logger().info('Grass cutting complete! All boundaries reached.')
            self.timer.cancel()
            rclpy.shutdown()
            return

        # Check for position correction need
        if not self.correction_active and self.should_apply_correction():
            self.correction_active = True

        # Always use maximum velocities for low torque motors
        # Create Twist message with maximum speeds
        twist_msg = Twist()
        twist_msg.linear.x = self.max_linear_speed
        twist_msg.angular.z = self.max_angular_speed

        # Apply position correction if active
        if self.correction_active:
            self.apply_position_correction(twist_msg)

        # Publish Twist message
        self.publisher_.publish(twist_msg)

        # Log current status
        distance_from_center = math.sqrt(
            (self.current_x - self.center_x)**2 + 
            (self.current_y - self.center_y)**2
        )
        
        # Log current status with information about corners reached
        corners_status = ' '.join([
            f"C{i+1}:{'✓' if reached else '✗'}" 
            for i, reached in enumerate(self.corners_reached)
        ])
        
        self.get_logger().info(
            f'Rev: {self.revolutions_completed} | Command: lin={twist_msg.linear.x:.3f}, ang={twist_msg.angular.z:.3f} | ' +
            f'Pos: x={self.current_x:.2f}, y={self.current_y:.2f} | Dist from center: {distance_from_center:.2f}m | ' +
            f'Corners: {corners_status} | Correction: {"ON" if self.correction_active else "OFF"}'
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
