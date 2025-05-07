import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    pkg_path = os.path.join(get_package_share_directory('model_description'), 'models', 'model_copy.sdf')
    

    return LaunchDescription([
        # Launch Ignition Gazebo with the world
        ExecuteProcess(
            cmd=['ign', 'gazebo', '-r', pkg_path],
            output='screen'
        ),

        # ROS-Ignition Bridge
        #Node(
            #package='ros_ign_bridge',
            #executable='parameter_bridge',
            #arguments=[
    #'/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
   # '/gps@ignition.msgs.NavSat@sensor_msgs/msg/NavSatFix',
#],
           # output='screen'
       # ),
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim'
        ),
        # Velocity Publisher Node
        Node(
            package='launch_pkg',
            executable='velocity_publisher',
            output='screen'
        ),

        Node(
            package='launch_pkg',
            executable='boundary_publisher',
            output='screen'

        ),


        Node(
            package='launch_pkg',
            executable='turtle_pose_to_gps',
            name='pose_to_gps'
        ),

        # Location Subscriber Node
        #Node(
            #package='launch_pkg',
            #executable='location_subscriber',
            #output='screen'
        #),


        #Node(
           # package='launch_pkg',
           # executable='ultrasonic_node',
           # output='screen'
        #),


        #Node(
            #package='launch_pkg',
            #executable='path_planning_spiral',
            #output='screen'
        #)
    ])