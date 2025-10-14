from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cibo',
            executable='front_camera_depth_node',
            name='front_camera_depth_node',
            output='screen'
        ),
        Node(
            package='cibo',
            executable='top_camera_depth_node',
            name='top_camera_depth_node',
            output='screen'
        ),
    ])