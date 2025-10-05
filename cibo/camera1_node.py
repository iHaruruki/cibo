#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import cv2
import mediapipe as mp
import numpy as np
import os
import sys


class HolisticPoseEstimatorNode(Node):
    def __init__(self):
        super().__init__('holistic_pose_estimator')
        
        self.get_logger().info("Initializing Holistic Pose Estimator Node...")
        
        # ROS Parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('min_detection_confidence', 0.5),
                ('min_tracking_confidence', 0.5),
                ('model_complexity', 2),
                ('smooth_landmarks', True),
                ('enable_segmentation', True),
                ('smooth_segmentation', True),
                ('refine_face_landmarks', True),
                ('roi_enabled', False),
                ('roi_x', 100),
                ('roi_y', 100),
                ('roi_width', 400),
                ('roi_height', 400),
                ('show_opencv_window', True)
            ]
        )
        
        # Get parameters
        self.min_detection_confidence = self.get_parameter('min_detection_confidence').value
        self.min_tracking_confidence = self.get_parameter('min_tracking_confidence').value
        self.model_complexity = self.get_parameter('model_complexity').value
        self.smooth_landmarks = self.get_parameter('smooth_landmarks').value
        self.enable_segmentation = self.get_parameter('enable_segmentation').value
        self.smooth_segmentation = self.get_parameter('smooth_segmentation').value
        self.refine_face_landmarks = self.get_parameter('refine_face_landmarks').value
        self.show_opencv_window = self.get_parameter('show_opencv_window').value
        
        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # Initialize MediaPipe Holistic
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_drawing_styles = mp.solutions.drawing_styles
        self.mp_holistic = mp.solutions.holistic
        
        self.holistic = self.mp_holistic.Holistic(
            min_detection_confidence=self.min_detection_confidence,
            min_tracking_confidence=self.min_tracking_confidence,
            model_complexity=self.model_complexity,
            smooth_landmarks=self.smooth_landmarks,
            enable_segmentation=self.enable_segmentation,
            smooth_segmentation=self.smooth_segmentation,
            refine_face_landmarks=self.refine_face_landmarks
        )
        
        # ROI state
        self.roi_state = {
            'enabled': self.get_parameter('roi_enabled').value,
            'x': self.get_parameter('roi_x').value,
            'y': self.get_parameter('roi_y').value,
            'width': self.get_parameter('roi_width').value,
            'height': self.get_parameter('roi_height').value,
            'cx': self.get_parameter('roi_x').value + self.get_parameter('roi_width').value // 2,
            'cy': self.get_parameter('roi_y').value + self.get_parameter('roi_height').value // 2
        }
        
        # OpenCV window setup
        self.opencv_enabled = False
        if self.show_opencv_window:
            self.opencv_enabled = self.setup_opencv_window()
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image, 
            '/camera_01/color/image_raw', 
            self.image_callback, 
            10
        )
        
        # Publishers
        self.annotated_image_pub = self.create_publisher(
            Image, 
            '/holistic/annotated_image', 
            10
        )
        
        self.pose_landmarks_pub = self.create_publisher(
            Float32MultiArray, 
            '/holistic/pose_landmarks', 
            10
        )
        
        self.face_landmarks_pub = self.create_publisher(
            Float32MultiArray, 
            '/holistic/face_landmarks', 
            10
        )
        
        self.left_hand_landmarks_pub = self.create_publisher(
            Float32MultiArray, 
            '/holistic/left_hand_landmarks', 
            10
        )
        
        self.right_hand_landmarks_pub = self.create_publisher(
            Float32MultiArray, 
            '/holistic/right_hand_landmarks', 
            10
        )
        
        self.get_logger().info('Holistic Pose Estimator Node initialized successfully')
        self.get_logger().info(f'ROI enabled: {self.roi_state["enabled"]}')
        if self.roi_state['enabled']:
            self.get_logger().info(f'ROI: ({self.roi_state["x"]}, {self.roi_state["y"]}) - '
                                 f'{self.roi_state["width"]}x{self.roi_state["height"]}')

    def setup_opencv_window(self):
        """OpenCVウィンドウのセットアップ"""
        try:
            cv2.namedWindow('Holistic Pose Estimator', cv2.WINDOW_AUTOSIZE)
            cv2.setMouseCallback('Holistic Pose Estimator', self.mouse_callback)
            
            self.get_logger().info("OpenCV window setup successful")
            return True
            
        except Exception as e:
            self.get_logger().error(f"Failed to setup OpenCV window: {str(e)}")
            return False

    def mouse_callback(self, event, x, y, flags, param):
        """マウスイベントハンドラ"""
        if event == cv2.EVENT_LBUTTONDOWN:
            self.get_logger().info(f"Mouse clicked at ({x}, {y})")
            # クリック位置を中心にROIを設定
            half_width = self.roi_state['width'] // 2
            half_height = self.roi_state['height'] // 2
            
            self.roi_state['cx'] = x
            self.roi_state['cy'] = y
            self.roi_state['x'] = max(0, x - half_width)
            self.roi_state['y'] = max(0, y - half_height)
            self.roi_state['enabled'] = True
            
            self.get_logger().info(f"ROI set: center=({x}, {y}), "
                                 f"region=({self.roi_state['x']}, {self.roi_state['y']}) - "
                                 f"{self.roi_state['width']}x{self.roi_state['height']}")

    def extract_landmarks(self, results, image_width, image_height):
        """ランドマークデータを抽出"""
        
        # Pose landmarks
        pose_landmarks = []
        if results.pose_landmarks:
            for landmark in results.pose_landmarks.landmark:
                pose_landmarks.extend([
                    landmark.x * image_width,
                    landmark.y * image_height,
                    landmark.z,
                    landmark.visibility
                ])
        
        # Face landmarks
        face_landmarks = []
        if results.face_landmarks:
            for landmark in results.face_landmarks.landmark:
                face_landmarks.extend([
                    landmark.x * image_width,
                    landmark.y * image_height,
                    landmark.z
                ])
        
        # Left hand landmarks
        left_hand_landmarks = []
        if results.left_hand_landmarks:
            for landmark in results.left_hand_landmarks.landmark:
                left_hand_landmarks.extend([
                    landmark.x * image_width,
                    landmark.y * image_height,
                    landmark.z
                ])
        
        # Right hand landmarks
        right_hand_landmarks = []
        if results.right_hand_landmarks:
            for landmark in results.right_hand_landmarks.landmark:
                right_hand_landmarks.extend([
                    landmark.x * image_width,
                    landmark.y * image_height,
                    landmark.z
                ])
        
        return pose_landmarks, face_landmarks, left_hand_landmarks, right_hand_landmarks

    def draw_landmarks(self, image, results):
        """ランドマークを画像に描画"""
        
        # Draw pose landmarks
        if results.pose_landmarks:
            self.mp_drawing.draw_landmarks(
                image,
                results.pose_landmarks,
                self.mp_holistic.POSE_CONNECTIONS,
                landmark_drawing_spec=self.mp_drawing_styles.get_default_pose_landmarks_style()
            )
        
        # Draw face landmarks
        if results.face_landmarks:
            self.mp_drawing.draw_landmarks(
                image,
                results.face_landmarks,
                self.mp_holistic.FACEMESH_CONTOURS,
                landmark_drawing_spec=None,
                connection_drawing_spec=self.mp_drawing_styles.get_default_face_mesh_contours_style()
            )
        
        # Draw left hand landmarks
        if results.left_hand_landmarks:
            self.mp_drawing.draw_landmarks(
                image,
                results.left_hand_landmarks,
                self.mp_holistic.HAND_CONNECTIONS,
                landmark_drawing_spec=self.mp_drawing_styles.get_default_hand_landmarks_style(),
                connection_drawing_spec=self.mp_drawing_styles.get_default_hand_connections_style()
            )
        
        # Draw right hand landmarks
        if results.right_hand_landmarks:
            self.mp_drawing.draw_landmarks(
                image,
                results.right_hand_landmarks,
                self.mp_holistic.HAND_CONNECTIONS,
                landmark_drawing_spec=self.mp_drawing_styles.get_default_hand_landmarks_style(),
                connection_drawing_spec=self.mp_drawing_styles.get_default_hand_connections_style()
            )

    def process_image(self, cv_image):
        """MediaPipe Holisticで画像処理"""
        height, width = cv_image.shape[:2]
        
        # ROI抽出
        if self.roi_state['enabled']:
            x1 = max(0, self.roi_state['x'])
            y1 = max(0, self.roi_state['y'])
            x2 = min(width, x1 + self.roi_state['width'])
            y2 = min(height, y1 + self.roi_state['height'])
            
            processing_image = cv_image[y1:y2, x1:x2]
            roi_width, roi_height = x2 - x1, y2 - y1
        else:
            processing_image = cv_image
            roi_width, roi_height = width, height
            x1 = y1 = 0
        
        # MediaPipe処理
        image_rgb = cv2.cvtColor(processing_image, cv2.COLOR_BGR2RGB)
        image_rgb.flags.writeable = False
        
        results = self.holistic.process(image_rgb)
        
        image_rgb.flags.writeable = True
        annotated_roi = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR)
        
        # ランドマーク描画（ROI内の画像に描画）
        self.draw_landmarks(annotated_roi, results)
        
        # 元画像にROI部分を埋め込み
        annotated_image = cv_image.copy()
        if self.roi_state['enabled']:
            annotated_image[y1:y1+roi_height, x1:x1+roi_width] = annotated_roi
        else:
            annotated_image = annotated_roi
        
        # ランドマークデータ抽出
        pose_landmarks, face_landmarks, left_hand_landmarks, right_hand_landmarks = \
            self.extract_landmarks(results, roi_width, roi_height)
        
        return annotated_image, pose_landmarks, face_landmarks, left_hand_landmarks, right_hand_landmarks

    def image_callback(self, msg):
        """画像コールバック"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # MediaPipe処理
            annotated_image, pose_landmarks, face_landmarks, left_hand_landmarks, right_hand_landmarks = \
                self.process_image(cv_image)
            
            # アノテーション画像をパブリッシュ
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated_image, "bgr8")
            annotated_msg.header = msg.header
            self.annotated_image_pub.publish(annotated_msg)
            
            # ランドマークデータをパブリッシュ
            pose_msg = Float32MultiArray()
            pose_msg.data = pose_landmarks
            self.pose_landmarks_pub.publish(pose_msg)
            
            face_msg = Float32MultiArray()
            face_msg.data = face_landmarks
            self.face_landmarks_pub.publish(face_msg)
            
            left_hand_msg = Float32MultiArray()
            left_hand_msg.data = left_hand_landmarks
            self.left_hand_landmarks_pub.publish(left_hand_msg)
            
            right_hand_msg = Float32MultiArray()
            right_hand_msg.data = right_hand_landmarks
            self.right_hand_landmarks_pub.publish(right_hand_msg)
            
            # OpenCV表示
            if self.opencv_enabled:
                try:
                    display_image = annotated_image.copy()
                    
                    # ROI矩形を描画
                    if self.roi_state['enabled']:
                        x1 = max(0, self.roi_state['x'])
                        y1 = max(0, self.roi_state['y'])
                        x2 = min(cv_image.shape[1], x1 + self.roi_state['width'])
                        y2 = min(cv_image.shape[0], y1 + self.roi_state['height'])
                        
                        cv2.rectangle(display_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        cv2.putText(display_image, 'ROI', (x1, y1-10), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    
                    # 操作説明を追加
                    cv2.putText(display_image, 'Click to set ROI center', 
                               (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                    cv2.putText(display_image, 'Press: Q=quit, R=reset ROI, +/-=resize', 
                               (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)
                    
                    cv2.imshow('Holistic Pose Estimator', display_image)
                    
                    # キー入力処理
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord('q'):
                        self.get_logger().info("Closing OpenCV window")
                        cv2.destroyAllWindows()
                        self.opencv_enabled = False
                    elif key == ord('r'):
                        # ROIリセット
                        self.roi_state['enabled'] = False
                        self.get_logger().info("ROI reset")
                    elif key in (ord('+'), ord('=')):
                        # ROIサイズ増加
                        self.roi_state['width'] = min(cv_image.shape[1], self.roi_state['width'] + 40)
                        self.roi_state['height'] = min(cv_image.shape[0], self.roi_state['height'] + 40)
                        self.get_logger().info(f"ROI size increased: {self.roi_state['width']}x{self.roi_state['height']}")
                    elif key == ord('-'):
                        # ROIサイズ減少
                        self.roi_state['width'] = max(100, self.roi_state['width'] - 40)
                        self.roi_state['height'] = max(100, self.roi_state['height'] - 40)
                        self.get_logger().info(f"ROI size decreased: {self.roi_state['width']}x{self.roi_state['height']}")
                        
                except Exception as e:
                    self.get_logger().debug(f"OpenCV display error: {str(e)}")
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    
    node = HolisticPoseEstimatorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(node, 'opencv_enabled') and node.opencv_enabled:
            cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()