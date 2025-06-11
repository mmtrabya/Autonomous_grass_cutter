# gps_tracker.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import math

class GPSTracker(Node):
    def __init__(self):
        super().__init__('gps_tracker')
        self.subscription = self.create_subscription(
            NavSatFix,
            '/gps',
            self.gps_callback,
            10)
        self.initial_position = None
        self.get_logger().info('GPS Tracker started')

    def gps_callback(self, msg):
        if self.initial_position is None and msg.status.status >= 0:
            self.initial_position = (msg.latitude, msg.longitude, msg.altitude)
            self.get_logger().info(f'Initial position: Lat={msg.latitude}, Lon={msg.longitude}, Alt={msg.altitude}')
        
        # Calculate distance from initial position if available
        if self.initial_position:
            distance = self.haversine(
                self.initial_position[0], self.initial_position[1],
                msg.latitude, msg.longitude
            )
            self.get_logger().info(
                f'Position: Lat={msg.latitude}, Lon={msg.longitude}, Alt={msg.altitude}, '
                f'Distance from start: {distance:.2f} meters'
            )
        else:
            self.get_logger().info(f'Position: Lat={msg.latitude}, Lon={msg.longitude}, Alt={msg.altitude}')

    def haversine(self, lat1, lon1, lat2, lon2):
        # Calculate the great circle distance between two points
        # on the earth (specified in decimal degrees)
        # Convert decimal degrees to radians
        lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
        
        # Haversine formula
        dlon = lon2 - lon1
        dlat = lat2 - lat1
        a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
        c = 2 * math.asin(math.sqrt(a))
        r = 6371000  # Radius of earth in meters
        return c * r

def main(args=None):
    rclpy.init(args=args)
    gps_tracker = GPSTracker()
    rclpy.spin(gps_tracker)
    gps_tracker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()