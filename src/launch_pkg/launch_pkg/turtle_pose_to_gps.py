#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from sensor_msgs.msg import NavSatFix

class TurtlePoseToGPS(Node):
    def __init__(self):
        super().__init__('turtle_pose_to_gps')

        self.pose_sub = self.create_subscription(
            Pose, '/turtle1/pose', self.pose_callback, 10)

        self.gps_pub = self.create_publisher(
            NavSatFix, '/gps', 10)

        # Fake GPS origin
        self.origin_lat = 30.0  # Fake latitude origin
        self.origin_lon = 31.0  # Fake longitude origin
        self.scale = 0.00001    # Degrees per turtle unit

    def pose_callback(self, msg):
        gps_msg = NavSatFix()
        gps_msg.latitude = self.origin_lat + msg.x * self.scale
        gps_msg.longitude = self.origin_lon + msg.y * self.scale
        gps_msg.altitude = 0.0
        gps_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

        self.gps_pub.publish(gps_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TurtlePoseToGPS()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
