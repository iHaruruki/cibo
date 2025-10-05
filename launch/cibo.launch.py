from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='cibo', executable='holistic_camera_01_node', name='holistic_camera_01'),
        Node(package='cibo', executable='holistic_camera_02_node', name='holistic_camera_02'),
    ])