# robot_localization_launch.py
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Launch arguments
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    # Parameters
    wheel_radius = DeclareLaunchArgument(
        'wheel_radius',
        default_value='0.09',
        description='Wheel radius in meters'
    )
    
    wheel_base = DeclareLaunchArgument(
        'wheel_base',
        default_value='0.45',
        description='Wheel base in meters'
    )

    # Get robot_localization parameters
    ekf_config_path = os.path.join(
        get_package_share_directory('your_robot_package'),
        'config',
        'ekf_config.yaml'
    )

    return LaunchDescription([
        use_sim_time,
        wheel_radius,
        wheel_base,
        
        # Launch IMU driver node
        Node(
            package='my_sensor_pkg',
            executable='imu',
            name='sensor_driver',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'publish_rate': 30.0,
                'imu_frame_id': 'imu_link'
            }],
            output='screen'
        ),
        
        # Launch Encoder Odometry node
        Node(
            package='my_sensor_pkg',
            executable='encoder',
            name='encoder_odometry',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'wheel_radius': LaunchConfiguration('wheel_radius'),
                'wheel_base': LaunchConfiguration('wheel_base'),
                'encoder_resolution': 4096,
                'publish_rate': 10.0,
                'odom_frame_id': 'odom',
                'base_frame_id': 'base_link'
            }],
            output='screen'
        ),
        
        # Launch robot_localization EKF
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[
                ekf_config_path,
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ],
            remappings=[
                ('/odometry/filtered', '/odom/filtered'),
                ('/imu_data', '/imu/data'),  # Connect to your IMU topic
                ('/wheel_odom', '/odometry/wheel')  # Connect to your wheel odometry topic
            ]
        ),
        
        # Add static transforms if needed
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='imu_to_base_link',
            arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'imu_link'],
            output='screen'
        )
    ])