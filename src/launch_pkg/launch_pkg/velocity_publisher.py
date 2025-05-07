#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import Bool
from turtlesim.msg import Pose as TurtlePose

class TurtleSimSpiralPlanner(Node):
    def __init__(self):
        super().__init__('turtle_sim_spiral_planner')
        
        # Parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('step_size', 1.0),
                ('linear_velocity', 0.2),
                ('angular_velocity', 0.3),
                ('position_tolerance', 0.5),
                ('orientation_tolerance', 0.1),
                ('safe_margin', 0.5)
            ]
        )
        
        self.step_size = self.get_parameter('step_size').value
        self.linear_vel = self.get_parameter('linear_velocity').value
        self.angular_vel = self.get_parameter('angular_velocity').value
        self.pos_tolerance = self.get_parameter('position_tolerance').value
        self.orient_tolerance = self.get_parameter('orientation_tolerance').value
        self.safe_margin = self.get_parameter('safe_margin').value

        # Subscribers
        self.pose_sub = self.create_subscription(
            TurtlePose, '/turtle1/pose', self.pose_callback, 10)
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.completion_pub = self.create_publisher(Bool, '/spiral_complete', 10)

        # State variables
        self.current_pose = None
        self.waypoints = []
        self.current_waypoint_idx = 0
        self.path_completed = False
        self.spiral_initialized = False

        # Timers
        self.control_timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info("🌀 TurtleSim Spiral Planner Initialized")
        self.initialize_spiral_path()

    def pose_callback(self, msg):
        self.current_pose = msg

    def initialize_spiral_path(self):
        self.waypoints = []
        self.current_waypoint_idx = 0
        self.path_completed = False

        # Generate a simple square boundary for testing
        size = 5  # You can adjust this size as needed
        self.waypoints = [
            (2.0, 2.0),
            (2.0, -2.0),
            (-2.0, -2.0),
            (-2.0, 2.0)
        ]
        self.get_logger().info(f"Generated {len(self.waypoints)} waypoints")

    def control_loop(self):
        if not self.current_pose:
            return

        if self.current_waypoint_idx >= len(self.waypoints):
            self.stop_robot()
            return

        current_x = self.current_pose.x
        current_y = self.current_pose.y
        target_x, target_y = self.waypoints[self.current_waypoint_idx]
        
        distance = math.sqrt((target_x - current_x)**2 + (target_y - current_y)**2)
        target_heading = math.atan2(target_y - current_y, target_x - current_x)
        current_heading = self.current_pose.theta
        heading_error = self.normalize_angle(target_heading - current_heading)

        cmd = Twist()

        if distance < self.pos_tolerance:
            self.current_waypoint_idx += 1
        else:
            if abs(heading_error) > self.orient_tolerance:
                cmd.angular.z = self.angular_vel * (1.0 if heading_error > 0 else -1.0)
            else:
                cmd.linear.x = min(self.linear_vel, self.linear_vel * distance / 2.0)

        self.cmd_vel_pub.publish(cmd)

    def stop_robot(self):
        self.cmd_vel_pub.publish(Twist())
        self.completion_pub.publish(Bool(data=True))
        self.path_completed = True
        self.get_logger().info("Spiral path completed!")

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    planner = TurtleSimSpiralPlanner()
    try:
        rclpy.spin(planner)
    except KeyboardInterrupt:
        planner.get_logger().info("Planner shutting down...")
    finally:
        planner.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
