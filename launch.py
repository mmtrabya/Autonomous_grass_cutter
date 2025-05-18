from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_sensor_pkg',
            executable='imu',
            name='imu',
            output='screen',
            parameters=[{'use_sim_time': False}]  # adjust params if needed
        ),
        Node(
            package='my_sensor_pkg',
            executable='encoder',
            name='encoder',
            output='screen',
            parameters=[{'use_sim_time': False}]  # adjust params if needed
        )
    ])