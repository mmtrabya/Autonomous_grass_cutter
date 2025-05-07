#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

class LocationSubscriber(Node):
    def __init__(self):
        super().__init__('location_subscriber')
        self.subscription = self.create_subscription(
            NavSatFix,
            '/gps',
            self.listener_callback,
            10)
        self.get_logger().info('Location Subscriber Node Started')

    def listener_callback(self, msg):
        self.get_logger().info(
            f'GPS Position - Lat: {msg.latitude:.6f}, '
            f'Lon: {msg.longitude:.6f}, '
            f'Alt: {msg.altitude:.2f}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = LocationSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()