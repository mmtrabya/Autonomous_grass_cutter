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
        self.odom_subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.desired_angular_speed = 0.5
        self.initial_linear_speed = 0.1
        self.linear_speed_increment = 0.001
        self.max_linear_speed = 12
        self.update_rate = 0.1

        self.target_linear_speed = self.initial_linear_speed
        self.current_linear_speed = 0.0
        self.current_angular_speed = 0.0

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.start_x = None
        self.start_y = None
        self.distance_from_start = 0.0

        self.linear_pid = PIDController(0.8, 0.1, 0.05, 0.0, self.max_linear_speed, 0.3)
        self.angular_pid = PIDController(1.0, 0.1, 0.1, -1.0, 1.0, 0.3)

        self.boundary_radius = 4.0
        self.boundary_x_min = 0.0
        self.boundary_x_max = 0.0
        self.boundary_y_min = 0.0
        self.boundary_y_max = 0.0
        self.max_distance = self.boundary_radius

        self.boundary_slowdown_distance = 0.3
        self.boundary_approaching = False

        self.timer = self.create_timer(self.update_rate, self.timer_callback)

        self.last_radius_check_time = time.time()
        self.radius_check_interval = 3.0
        self.last_radius = 0.0
        self.stalled_count = 0

        self.get_logger().info('Spiral motion node with PID control has been started')
        zero_twist = Twist()
        self.publisher_.publish(zero_twist)
        self.get_logger().info('Published initial zero command')

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        self.current_yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))

        self.current_linear_speed = msg.twist.twist.linear.x
        self.current_angular_speed = msg.twist.twist.angular.z

        if self.start_x is None:
            self.start_x = self.current_x
            self.start_y = self.current_y
            self.boundary_x_min = self.start_x - self.boundary_radius
            self.boundary_x_max = self.start_x + self.boundary_radius
            self.boundary_y_min = self.start_y - self.boundary_radius
            self.boundary_y_max = self.start_y + self.boundary_radius
            self.get_logger().info(f'Initial position set: x={self.start_x:.2f}, y={self.start_y:.2f}\n' +
                                   f'Boundaries set: x=[{self.boundary_x_min:.2f}, {self.boundary_x_max:.2f}], ' +
                                   f'y=[{self.boundary_y_min:.2f}, {self.boundary_y_max:.2f}]')

        self.distance_from_start = math.sqrt((self.current_x - self.start_x) ** 2 + (self.current_y - self.start_y) ** 2)

        current_time = time.time()
        if current_time - self.last_radius_check_time > self.radius_check_interval:
            radius_change = self.distance_from_start - self.last_radius
            self.get_logger().info(f'Radius change over {self.radius_check_interval}s: {radius_change:.3f}m')
            if abs(radius_change) < 0.05 and self.target_linear_speed > 0.2:
                self.stalled_count += 1
                if self.stalled_count >= 2:
                    self.get_logger().warning('Spiral not expanding, increasing linear_speed_increment')
                    self.linear_speed_increment *= 1.5
                    self.stalled_count = 0
            else:
                self.stalled_count = 0

            self.last_radius = self.distance_from_start
            self.last_radius_check_time = current_time

    def check_boundaries(self):
        distance_to_x_min = abs(self.current_x - self.boundary_x_min)
        distance_to_x_max = abs(self.boundary_x_max - self.current_x)
        distance_to_y_min = abs(self.current_y - self.boundary_y_min)
        distance_to_y_max = abs(self.boundary_y_max - self.current_y)
        distance_to_circular_boundary = self.boundary_radius - self.distance_from_start
        min_distance = min(distance_to_x_min, distance_to_x_max, distance_to_y_min, distance_to_y_max, distance_to_circular_boundary)

        if min_distance < self.boundary_slowdown_distance:
            self.boundary_approaching = True
            slowdown_factor = min_distance / self.boundary_slowdown_distance
            return slowdown_factor
        else:
            self.boundary_approaching = False

        if (self.current_x < self.boundary_x_min or self.current_x > self.boundary_x_max or
            self.current_y < self.boundary_y_min or self.current_y > self.boundary_y_max or
            self.distance_from_start > self.boundary_radius):
            self.get_logger().warning(f'Boundary reached at position x={self.current_x:.2f}, y={self.current_y:.2f}')
            return 0.0

        return 1.0

    def timer_callback(self):
        if self.start_x is None:
            self.get_logger().info('Waiting for initial odometry data...')
            return

        boundary_factor = self.check_boundaries()
        if boundary_factor <= 0:
            stop_msg = Twist()
            self.publisher_.publish(stop_msg)
            self.get_logger().info('Boundary reached. Stopping robot and shutting down node.')
            self.timer.cancel()
            rclpy.shutdown()
            return

        if self.boundary_approaching:
            adjusted_target_linear = self.target_linear_speed * boundary_factor
            adjusted_target_angular = self.desired_angular_speed * boundary_factor
            self.get_logger().info(f'Approaching boundary. Slowing down: factor={boundary_factor:.2f}')
        else:
            adjusted_target_linear = self.target_linear_speed
            adjusted_target_angular = self.desired_angular_speed

        if self.target_linear_speed >= self.max_linear_speed:
            stop_msg = Twist()
            self.publisher_.publish(stop_msg)
            self.get_logger().info('Max speed reached. Stopping robot and shutting down node.')
            self.timer.cancel()
            rclpy.shutdown()
            return

        linear_output = self.linear_pid.compute(adjusted_target_linear, self.current_linear_speed)
        angular_output = self.angular_pid.compute(adjusted_target_angular, self.current_angular_speed)

        twist_msg = Twist()
        twist_msg.linear.x = linear_output
        twist_msg.angular.z = angular_output
        self.publisher_.publish(twist_msg)
        self.target_linear_speed += self.linear_speed_increment

        self.get_logger().info(
            f'Target: lin={adjusted_target_linear:.3f}, ang={adjusted_target_angular:.3f} | '
            f'Command: lin={linear_output:.3f}, ang={angular_output:.3f} | '
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
