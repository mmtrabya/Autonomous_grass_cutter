#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from time import sleep

class BoundaryPublisher(Node):
    def __init__(self):
        super().__init__('boundary_publisher')
        self.publisher = self.create_publisher(Point, 'boundary_points', 10)
        
        # Timer to publish boundary points
        self.timer = self.create_timer(1.0, self.publish_boundaries)
        
        # Example boundary points for the TurtleSim
        self.boundary_points = [
            (2.0, 2.0),
            (2.0, -2.0),
            (-2.0, -2.0),
            (-2.0, 2.0)
        ]
        self.index = 0

    def publish_boundaries(self):
        if self.index < len(self.boundary_points):
            point = Point()
            point.x = self.boundary_points[self.index][0]
            point.y = self.boundary_points[self.index][1]
            self.publisher.publish(point)
            self.get_logger().info(f"Published boundary point: {point.x}, {point.y}")
            self.index += 1
        else:
            self.get_logger().info("All boundary points published.")
            self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    boundary_publisher = BoundaryPublisher()
    try:
        rclpy.spin(boundary_publisher)
    except KeyboardInterrupt:
        boundary_publisher.get_logger().info('Boundary Publisher shutting down...')
    finally:
        boundary_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
