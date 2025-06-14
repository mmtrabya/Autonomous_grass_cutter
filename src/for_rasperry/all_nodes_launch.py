from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Raspberry Pi Localization Node (Python)
       

        # IMU Reader Node (C++)
        Node(
            package='my_sensor_pkg',
            executable='imu',
            name='imu_node',
            output='screen'
        ),

        # Encoder Reader Node (C++)
        Node(
            package='my_sensor_pkg',
            executable='encoder',
            name='encoder_node',
            output='screen'
        ), 
        Node(
            package='robot_localization',
            executable='raspberry_localization_node',
            name='raspberry_localization_node',
            output='screen'
        ) ,

        # Spiral Motion Controller (Python)
        Node(
            package='spiral_motion',
            executable='spiral_node',
            name='spiral_node',
            output='screen'
        ),

        # Final Velocity Control Node (C++)
        Node(
            package='control_node',
            executable='control_node',
            name='control_node',
            output='screen'
        ),
    ])

