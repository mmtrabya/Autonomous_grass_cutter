#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class UltrasonicNode(Node):
    def __init__(self):
        super().__init__('ultrasonic_node')
        self.safe_distance = 0.5  # meters
        
        # Subscribers
        self.create_subscription(Range, '/ultrasonic/front', self.front_cb, 10)
        self.create_subscription(Range, '/ultrasonic/left', self.left_cb, 10)
        self.create_subscription(Range, '/ultrasonic/right', self.right_cb, 10)
        self.create_subscription(Twist, '/cmd_vel_raw', self.cmd_vel_cb, 10)
        
        # Publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Sensor data storage
        self.front_dist = 4.0
        self.left_dist = 4.0
        self.right_dist = 4.0
        self.last_cmd = Twist()

    def front_cb(self, msg):
        self.front_dist = msg.range

    def left_cb(self, msg):
        self.left_dist = msg.range

    def right_cb(self, msg):
        self.right_dist = msg.range

    def cmd_vel_cb(self, msg):
        modified_cmd = Twist()
        modified_cmd.linear.x = msg.linear.x
        modified_cmd.angular.z = msg.angular.z
        
        # Collision avoidance logic
        if self.front_dist < self.safe_distance:
            modified_cmd.linear.x = 0.0
            self.get_logger().warn("Front obstacle! Stopping.")
        
        if self.left_dist < self.safe_distance:
            modified_cmd.angular.z -= 0.5
            self.get_logger().warn("Left obstacle! Turning right.")
        
        if self.right_dist < self.safe_distance:
            modified_cmd.angular.z += 0.5
            self.get_logger().warn("Right obstacle! Turning left.")
        
        self.cmd_pub.publish(modified_cmd)
        
        # Front obstacle - full stop
        if self.front_dist < self.safe_distance:
            modified_cmd.linear.x = 0.0
            modified_cmd.angular.z = 0.0
            safe = False
            self.get_logger().warn("Front obstacle detected!")
            
        # Left obstacle - slight right turn
        elif self.left_dist < self.safe_distance:
            modified_cmd.linear.x = msg.linear.x * 0.5
            modified_cmd.angular.z = -0.5
            safe = False
            self.get_logger().warn("Left obstacle detected!")
            
        # Right obstacle - slight left turn
        elif self.right_dist < self.safe_distance:
            modified_cmd.linear.x = msg.linear.x * 0.5
            modified_cmd.angular.z = 0.5
            safe = False
            self.get_logger().warn("Right obstacle detected!")
            
        if safe:
            modified_cmd = msg  # Forward original command
            
        self.cmd_pub.publish(modified_cmd)

def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()