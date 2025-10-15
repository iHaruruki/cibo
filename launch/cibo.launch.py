import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.action import DeclareLaunchArgument
from launch.substitution import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    urdf_file_name = 'cibo.urdf'

    urdf_file_name = os.path.join(
        get_package_share_directory('cibo'),
        urdf_file_name)
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false'
            descripription='Use simulation (Gazebo) clock if true'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}],
            arguments=[urdf]
        ),
        Node(
            package='cibo',
            executable='front_camera_node',
            name='front_camera_node',
            output='screen'
        ),
        Node(
            package='cibo',
            executable='top_camera_node',
            name='top_camera_node',
            output='screen'
        ),
    ])