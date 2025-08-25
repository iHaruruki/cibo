#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import cv2
import mediapipe as mp
import numpy as np


class MediaPipeHandsNode(Node):
    def __init__(self):
        super().__init__('mediapipe_hands_processor')
        
        # CV Bridgeの初期化
        self.bridge = CvBridge()
        
        # MediaPipe Handsの初期化
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_hands = mp.solutions.hands
        
        # MediaPipe Handsモデルの初期化
        self.hands = mp.solutions.hands.Hands(
            max_num_hands=2,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )
        
        # カメラフィードの購読者
        self.camera1_sub = self.create_subscription(
            Image,
            '/camera_01/color/image_raw',
            self.camera1_callback,
            10
        )
        
        self.camera2_sub = self.create_subscription(
            Image,
            '/camera_02/color/image_raw',
            self.camera2_callback,
            10
        )
        
        # 注釈付き画像の配信者（手の検出結果付き）
        self.camera1_annotated_pub = self.create_publisher(
            Image,
            '/mp/camera_01/hands_annotated_image',
            10
        )
        
        self.camera2_annotated_pub = self.create_publisher(
            Image,
            '/mp/camera_02/hands_annotated_image',
            10
        )
        
        # 手のランドマークの配信者
        self.camera1_hands_pub = self.create_publisher(
            Float32MultiArray,
            '/mp/camera_01/hand_landmarks',
            10
        )
        
        self.camera2_hands_pub = self.create_publisher(
            Float32MultiArray,
            '/mp/camera_02/hand_landmarks',
            10
        )
        
        self.get_logger().info('MediaPipe Hands ROS2ノードが初期化されました')

    def process_image(self, cv_image, camera_name):
        """MediaPipeで手の検出を行い、注釈付き画像とランドマークを返す"""
        
        # MediaPipe用にBGRからRGBに変換
        image_rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        image_rgb.flags.writeable = False
        
        # MediaPipe Handsで処理
        results = self.hands.process(image_rgb)
        
        # 可視化用にBGRに戻す
        image_rgb.flags.writeable = True
        annotated_image = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR)
        
        # 画像の寸法を取得
        height, width = cv_image.shape[:2]
        
        # 手のランドマークを描画
        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                self.mp_drawing.draw_landmarks(
                    annotated_image, 
                    hand_landmarks, 
                    self.mp_hands.HAND_CONNECTIONS
                )
        
        # ランドマークデータを抽出
        landmarks = []
        
        # 手のランドマークを追加
        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                for landmark in hand_landmarks.landmark:
                    landmarks.append(landmark.x * width)
                    landmarks.append(landmark.y * height)
            self.get_logger().debug(f'{camera_name}: 手のランドマーク数: {len(landmarks)}')
        else:
            self.get_logger().debug(f'{camera_name}: 手のデータなし')
            landmarks = [0.0, 0.0]  # デフォルト値
        
        return annotated_image, landmarks

    def camera1_callback(self, msg):
        """カメラ1のコールバック"""
        try:
            # ROS ImageをOpenCVに変換
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # 画像を処理
            annotated_image, landmarks = self.process_image(cv_image, "camera_01")
            
            # 注釈付き画像を配信
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated_image, "bgr8")
            annotated_msg.header = msg.header
            self.camera1_annotated_pub.publish(annotated_msg)
            
            # 手のランドマークを配信
            landmarks_msg = Float32MultiArray()
            landmarks_msg.data = landmarks
            self.camera1_hands_pub.publish(landmarks_msg)
            
        except Exception as e:
            self.get_logger().error(f'カメラ1の処理エラー: {str(e)}')

    def camera2_callback(self, msg):
        """カメラ2のコールバック"""
        try:
            # ROS ImageをOpenCVに変換
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # 画像を処理
            annotated_image, landmarks = self.process_image(cv_image, "camera_02")
            
            # 注釈付き画像を配信
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated_image, "bgr8")
            annotated_msg.header = msg.header
            self.camera2_annotated_pub.publish(annotated_msg)
            
            # 手のランドマークを配信
            landmarks_msg = Float32MultiArray()
            landmarks_msg.data = landmarks
            self.camera2_hands_pub.publish(landmarks_msg)
            
        except Exception as e:
            self.get_logger().error(f'カメラ2の処理エラー: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    
    node = MediaPipeHandsNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()