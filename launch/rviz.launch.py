from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=[
            '-d', './rviz/cibo_camera.rviz'])

    return LaunchDescription([node])