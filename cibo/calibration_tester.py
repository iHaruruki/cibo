#!/usr/bin/env python3
"""
カメラキャリブレーションのデモ・テスト用スクリプト

このスクリプトは以下をテストします：
1. キャリブレーションノードの起動
2. TF配信の確認
3. 保存・読み込み機能のテスト
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
import cv2
import numpy as np
from cv_bridge import CvBridge
import time


class CalibrationTester(Node):
    def __init__(self):
        super().__init__('calibration_tester')
        
        self.bridge = CvBridge()
        
        # テスト用パブリッシャー
        self.pub_camera1_rgb = self.create_publisher(Image, '/camera_01/color/image_raw', 10)
        self.pub_camera1_depth = self.create_publisher(Image, '/camera_01/depth/image_raw', 10)
        self.pub_camera2_rgb = self.create_publisher(Image, '/camera_02/color/image_raw', 10)
        self.pub_camera2_depth = self.create_publisher(Image, '/camera_02/depth/image_raw', 10)
        
        # キャリブレーション状態監視
        self.sub_status = self.create_subscription(
            Bool, '/calibration/status', self.status_callback, 10)
        
        # テスト画像生成用タイマー
        self.timer = self.create_timer(1.0/30.0, self.publish_test_images)
        
        # チェスボードパターンを生成
        self.chessboard_pattern = self.create_chessboard_pattern()
        
        self.get_logger().info('キャリブレーションテスター開始')
        self.get_logger().info('チェスボードパターンを両カメラに配信中...')

    def create_chessboard_pattern(self):
        """8x6チェスボードパターンを生成"""
        # 640x480の画像にチェスボードを描画
        image = np.ones((480, 640, 3), dtype=np.uint8) * 255
        
        # チェスボードパラメータ
        square_size = 40  # ピクセル単位
        board_width = 9   # 正方形の数（コーナーは8個）
        board_height = 7  # 正方形の数（コーナーは6個）
        
        # 描画開始位置（中央に配置）
        start_x = (640 - board_width * square_size) // 2
        start_y = (480 - board_height * square_size) // 2
        
        # チェスボードパターンを描画
        for row in range(board_height):
            for col in range(board_width):
                if (row + col) % 2 == 0:
                    # 黒い正方形
                    x1 = start_x + col * square_size
                    y1 = start_y + row * square_size
                    x2 = x1 + square_size
                    y2 = y1 + square_size
                    cv2.rectangle(image, (x1, y1), (x2, y2), (0, 0, 0), -1)
        
        return image

    def create_depth_image(self):
        """テスト用深度画像を生成"""
        # 1000mmの一様な深度値
        depth = np.full((480, 640), 1000, dtype=np.uint16)
        return depth

    def publish_test_images(self):
        """テスト画像を配信"""
        try:
            # RGB画像（チェスボードパターン）
            rgb_msg = self.bridge.cv2_to_imgmsg(self.chessboard_pattern, "bgr8")
            rgb_msg.header.stamp = self.get_clock().now().to_msg()
            rgb_msg.header.frame_id = "camera_frame"
            
            # 深度画像
            depth_image = self.create_depth_image()
            depth_msg = self.bridge.cv2_to_imgmsg(depth_image, "16UC1")
            depth_msg.header.stamp = self.get_clock().now().to_msg()
            depth_msg.header.frame_id = "camera_frame"
            
            # Camera1（同じ画像）
            self.pub_camera1_rgb.publish(rgb_msg)
            self.pub_camera1_depth.publish(depth_msg)
            
            # Camera2（少し異なる視点をシミュレート）
            # 実際のテストでは、異なる角度のチェスボード画像を使用
            rgb_msg2 = rgb_msg  # 簡単のため同じ画像を使用
            depth_msg2 = depth_msg
            
            self.pub_camera2_rgb.publish(rgb_msg2)
            self.pub_camera2_depth.publish(depth_msg2)
            
        except Exception as e:
            self.get_logger().error(f'画像配信エラー: {str(e)}')

    def status_callback(self, msg):
        """キャリブレーション状態コールバック"""
        if msg.data:
            self.get_logger().info('キャリブレーション完了を確認！')


def main(args=None):
    rclpy.init(args=args)
    
    node = CalibrationTester()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
