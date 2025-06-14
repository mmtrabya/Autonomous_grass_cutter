from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='velocity_controller',
            executable='velocity_controller_node',
            name='velocity_controller_node',
            output='screen',
            parameters=[
                {'serial_port': '/dev/ttyUSB0'},
                {'baud_rate': 9600},
                {'track_width': 0.5}
            ]
        )
    ])