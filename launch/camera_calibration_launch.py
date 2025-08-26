#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
import os


def generate_launch_description():
    """
    8x6チェスボードを使った2カメラ間キャリブレーション用のlaunchファイル
    
    このlaunchファイルは以下を実行します：
    1. camera_calibration_nodeの起動
    2. 静的TFの配信（base_linkから各カメラへ）
    3. RViz2の起動（オプション）
    """
    
    # Launch引数の定義
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='RViz2を起動するかどうか'
    )
    
    auto_load_calibration_arg = DeclareLaunchArgument(
        'auto_load_calibration',
        default_value='true',
        description='起動時に保存済みキャリブレーション結果を自動読み込みするかどうか'
    )
    
    # カメラキャリブレーションノード
    camera_calibration_node = Node(
        package='cibo',
        executable='camera_calibration_node',
        name='camera_calibration_node',
        output='screen',
        parameters=[{
            'use_sim_time': False,
        }],
        remappings=[
            # 必要に応じてトピック名をリマップ
            # ('/camera_01/color/image_raw', '/your_camera1_topic'),
            # ('/camera_02/color/image_raw', '/your_camera2_topic'),
        ]
    )
    
    # 静的TF配信（base_linkからcamera_01_linkへ）
    static_tf_camera1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_camera1',
        arguments=[
            '0', '0', '0',  # x, y, z
            '0', '0', '0', '1',  # qx, qy, qz, qw
            'base_link',
            'camera_01_link'
        ]
    )
    
    # RViz2起動（オプション）
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        output='screen'
    )
    
    return LaunchDescription([
        use_rviz_arg,
        auto_load_calibration_arg,
        camera_calibration_node,
        static_tf_camera1,
        rviz_node,
    ])
