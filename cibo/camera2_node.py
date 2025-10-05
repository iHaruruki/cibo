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


class HolisticPoseEstimator(Node):
    def __init__(self):
        super().__init__('holistic_pose_estimator')
        
        # Declare ROS parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('min_detection_confidence', 0.5),
                ('min_tracking_confidence', 0.5),
                ('model_complexity', 1),
                ('enable_segmentation', False),
                ('refine_face_landmarks', True),
                ('roi_x', 0),
                ('roi_y', 0),
                ('roi_width', 640),
                ('roi_height', 480),
                ('enable_roi', False),
                ('show_landmarks', True),
                ('output_topic_prefix', '/holistic')
            ]
        )
        
        # Get parameters
        self.min_detection_confidence = self.get_parameter('min_detection_confidence').value
        self.min_tracking_confidence = self.get_parameter('min_tracking_confidence').value
        self.model_complexity = self.get_parameter('model_complexity').value
        self.enable_segmentation = self.get_parameter('enable_segmentation').value
        self.refine_face_landmarks = self.get_parameter('refine_face_landmarks').value
        self.roi_x = self.get_parameter('roi_x').value
        self.roi_y = self.get_parameter('roi_y').value
        self.roi_width = self.get_parameter('roi_width').value
        self.roi_height = self.get_parameter('roi_height').value
        self.enable_roi = self.get_parameter('enable_roi').value
        self.show_landmarks = self.get_parameter('show_landmarks').value
        self.output_topic_prefix = self.get_parameter('output_topic_prefix').value
        
        self.get_logger().info(f"Python executable: {sys.executable}")
        
        # OpenCV環境チェック
        self.check_opencv_environment()
        
        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # Initialize MediaPipe Holistic
        self.mp_holistic = mp.solutions.holistic
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_drawing_styles = mp.solutions.drawing_styles
        
        # Initialize Holistic model
        self.holistic = self.mp_holistic.Holistic(
            min_detection_confidence=self.min_detection_confidence,
            min_tracking_confidence=self.min_tracking_confidence,
            model_complexity=self.model_complexity,
            enable_segmentation=self.enable_segmentation,
            refine_face_landmarks=self.refine_face_landmarks
        )
        
        # ROI setup variables
        self.roi_selecting = False
        self.roi_start_point = None
        self.roi_end_point = None
        self.roi_set = False
        
        # OpenCV windows setup
        self.opencv_enabled = self.setup_opencv_windows()
        
        # Subscriber
        self.image_sub = self.create_subscription(
            Image, '/camera_01/color/image_raw', self.image_callback, 10)
        
        # Publishers
        self.annotated_image_pub = self.create_publisher(
            Image, f'{self.output_topic_prefix}/annotated_image', 10)
        self.pose_landmarks_pub = self.create_publisher(
            Float32MultiArray, f'{self.output_topic_prefix}/pose_landmarks', 10)
        self.face_landmarks_pub = self.create_publisher(
            Float32MultiArray, f'{self.output_topic_prefix}/face_landmarks', 10)
        self.left_hand_landmarks_pub = self.create_publisher(
            Float32MultiArray, f'{self.output_topic_prefix}/left_hand_landmarks', 10)
        self.right_hand_landmarks_pub = self.create_publisher(
            Float32MultiArray, f'{self.output_topic_prefix}/right_hand_landmarks', 10)
        
        self.get_logger().info('Holistic Pose Estimator initialized')
        self.get_logger().info(f'Parameters:')
        self.get_logger().info(f'  min_detection_confidence: {self.min_detection_confidence}')
        self.get_logger().info(f'  min_tracking_confidence: {self.min_tracking_confidence}')
        self.get_logger().info(f'  model_complexity: {self.model_complexity}')
        self.get_logger().info(f'  enable_segmentation: {self.enable_segmentation}')
        self.get_logger().info(f'  refine_face_landmarks: {self.refine_face_landmarks}')
        self.get_logger().info(f'  ROI: ({self.roi_x}, {self.roi_y}, {self.roi_width}, {self.roi_height})')
        self.get_logger().info(f'  enable_roi: {self.enable_roi}')

    def check_opencv_environment(self):
        """OpenCV環境をチェック"""
        self.get_logger().info(f"OpenCV version: {cv2.__version__}")
        self.get_logger().info(f"DISPLAY: {os.environ.get('DISPLAY', 'Not set')}")
        
        try:
            test_img = np.zeros((100, 300, 3), dtype=np.uint8)
            cv2.putText(test_img, 'OpenCV Test', (10, 50), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            cv2.namedWindow('Test', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('Test', test_img)
            cv2.waitKey(1000)
            cv2.destroyWindow('Test')
            
            self.get_logger().info("OpenCV window test: SUCCESS")
            return True
            
        except Exception as e:
            self.get_logger().error(f"OpenCV window test failed: {str(e)}")
            return False

    def setup_opencv_windows(self):
        """OpenCVウィンドウのセットアップ"""
        try:
            cv2.namedWindow('Holistic Pose Estimation', cv2.WINDOW_AUTOSIZE)
            cv2.setMouseCallback('Holistic Pose Estimation', self.mouse_callback)
            
            self.get_logger().info("OpenCV window setup successful")
            self.get_logger().info("Instructions:")
            self.get_logger().info("  - Drag to select ROI")
            self.get_logger().info("  - Press 'r' to reset ROI")
            self.get_logger().info("  - Press 'q' to quit OpenCV display")
            self.get_logger().info("  - Press 's' to save current ROI as parameters")
            return True
            
        except Exception as e:
            self.get_logger().error(f"Failed to setup OpenCV windows: {str(e)}")
            return False

    def mouse_callback(self, event, x, y, flags, param):
        """マウスイベントハンドラ（ROI選択）"""
        if event == cv2.EVENT_LBUTTONDOWN:
            self.roi_selecting = True
            self.roi_start_point = (x, y)
            self.roi_end_point = (x, y)
            self.get_logger().info(f"ROI selection started at ({x}, {y})")
            
        elif event == cv2.EVENT_MOUSEMOVE and self.roi_selecting:
            self.roi_end_point = (x, y)
            
        elif event == cv2.EVENT_LBUTTONUP:
            if self.roi_selecting:
                self.roi_selecting = False
                self.roi_end_point = (x, y)
                
                # ROI座標を計算
                x1 = min(self.roi_start_point[0], self.roi_end_point[0])
                y1 = min(self.roi_start_point[1], self.roi_end_point[1])
                x2 = max(self.roi_start_point[0], self.roi_end_point[0])
                y2 = max(self.roi_start_point[1], self.roi_end_point[1])
                
                self.roi_x = x1
                self.roi_y = y1
                self.roi_width = x2 - x1
                self.roi_height = y2 - y1
                self.enable_roi = True
                self.roi_set = True
                
                self.get_logger().info(f"ROI set: ({self.roi_x}, {self.roi_y}, {self.roi_width}, {self.roi_height})")

    def extract_landmarks_array(self, landmarks, image_width, image_height):
        """ランドマークを配列に変換"""
        if landmarks is None:
            return []
        
        landmarks_array = []
        for landmark in landmarks.landmark:
            landmarks_array.extend([
                landmark.x * image_width,
                landmark.y * image_height,
                landmark.z * image_width if hasattr(landmark, 'z') else 0.0
            ])
        return landmarks_array

    def process_image(self, cv_image):
        """画像処理（MediaPipe Holistic）"""
        height, width = cv_image.shape[:2]
        
        # ROI処理
        if self.enable_roi and self.roi_width > 0 and self.roi_height > 0:
            # ROI範囲をクランプ
            roi_x = max(0, min(self.roi_x, width - 1))
            roi_y = max(0, min(self.roi_y, height - 1))
            roi_x2 = min(width, roi_x + self.roi_width)
            roi_y2 = min(height, roi_y + self.roi_height)
            
            processing_image = cv_image[roi_y:roi_y2, roi_x:roi_x2]
            roi_offset = (roi_x, roi_y)
        else:
            processing_image = cv_image
            roi_offset = (0, 0)
        
        # MediaPipe処理
        image_rgb = cv2.cvtColor(processing_image, cv2.COLOR_BGR2RGB)
        image_rgb.flags.writeable = False
        
        try:
            results = self.holistic.process(image_rgb)
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")
            return cv_image, [], [], [], []
        
        image_rgb.flags.writeable = True
        annotated_image = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR)
        
        # ランドマーク描画（ROI内）
        if self.show_landmarks:
            # 顔のランドマーク
            if results.face_landmarks:
                self.mp_drawing.draw_landmarks(
                    annotated_image,
                    results.face_landmarks,
                    self.mp_holistic.FACEMESH_CONTOURS,
                    landmark_drawing_spec=None,
                    connection_drawing_spec=self.mp_drawing_styles
                    .get_default_face_mesh_contours_style())
            
            # ポーズのランドマーク
            if results.pose_landmarks:
                self.mp_drawing.draw_landmarks(
                    annotated_image,
                    results.pose_landmarks,
                    self.mp_holistic.POSE_CONNECTIONS,
                    landmark_drawing_spec=self.mp_drawing_styles
                    .get_default_pose_landmarks_style())
            
            # 左手のランドマーク
            if results.left_hand_landmarks:
                self.mp_drawing.draw_landmarks(
                    annotated_image,
                    results.left_hand_landmarks,
                    self.mp_holistic.HAND_CONNECTIONS,
                    landmark_drawing_spec=self.mp_drawing_styles
                    .get_default_hand_landmarks_style(),
                    connection_drawing_spec=self.mp_drawing_styles
                    .get_default_hand_connections_style())
            
            # 右手のランドマーク
            if results.right_hand_landmarks:
                self.mp_drawing.draw_landmarks(
                    annotated_image,
                    results.right_hand_landmarks,
                    self.mp_holistic.HAND_CONNECTIONS,
                    landmark_drawing_spec=self.mp_drawing_styles
                    .get_default_hand_landmarks_style(),
                    connection_drawing_spec=self.mp_drawing_styles
                    .get_default_hand_connections_style())
        
        # 元画像にROI処理結果を埋め込み
        output_image = cv_image.copy()
        if self.enable_roi and self.roi_width > 0 and self.roi_height > 0:
            roi_x = max(0, min(self.roi_x, width - 1))
            roi_y = max(0, min(self.roi_y, height - 1))
            roi_x2 = min(width, roi_x + self.roi_width)
            roi_y2 = min(height, roi_y + self.roi_height)
            output_image[roi_y:roi_y2, roi_x:roi_x2] = annotated_image
        else:
            output_image = annotated_image
        
        # ランドマーク座標抽出
        roi_width = processing_image.shape[1]
        roi_height = processing_image.shape[0]
        
        pose_landmarks = self.extract_landmarks_array(results.pose_landmarks, roi_width, roi_height)
        face_landmarks = self.extract_landmarks_array(results.face_landmarks, roi_width, roi_height)
        left_hand_landmarks = self.extract_landmarks_array(results.left_hand_landmarks, roi_width, roi_height)
        right_hand_landmarks = self.extract_landmarks_array(results.right_hand_landmarks, roi_width, roi_height)
        
        return output_image, pose_landmarks, face_landmarks, left_hand_landmarks, right_hand_landmarks

    def image_callback(self, msg):
        """画像コールバック"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # 画像処理
            annotated_image, pose_landmarks, face_landmarks, left_hand_landmarks, right_hand_landmarks = \
                self.process_image(cv_image)
            
            # OpenCV表示
            if self.opencv_enabled:
                display_image = annotated_image.copy()
                
                # ROI描画
                if self.enable_roi and self.roi_width > 0 and self.roi_height > 0:
                    cv2.rectangle(display_image, 
                                (self.roi_x, self.roi_y), 
                                (self.roi_x + self.roi_width, self.roi_y + self.roi_height),
                                (0, 255, 0), 2)
                    cv2.putText(display_image, 'ROI', 
                              (self.roi_x, self.roi_y - 10),
                              cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                # ROI選択中の描画
                if self.roi_selecting and self.roi_start_point and self.roi_end_point:
                    cv2.rectangle(display_image, self.roi_start_point, self.roi_end_point, (255, 0, 0), 2)
                
                # 操作説明
                cv2.putText(display_image, 'Drag to select ROI, R:reset, S:save, Q:quit', 
                          (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                
                # ランドマーク数表示
                info_y = 60
                cv2.putText(display_image, f'Pose: {len(pose_landmarks)//3} landmarks', 
                          (10, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                cv2.putText(display_image, f'Face: {len(face_landmarks)//3} landmarks', 
                          (10, info_y + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                cv2.putText(display_image, f'L.Hand: {len(left_hand_landmarks)//3} landmarks', 
                          (10, info_y + 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                cv2.putText(display_image, f'R.Hand: {len(right_hand_landmarks)//3} landmarks', 
                          (10, info_y + 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                
                cv2.imshow('Holistic Pose Estimation', display_image)
                
                # キー入力処理
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    self.get_logger().info("Closing OpenCV window")
                    cv2.destroyAllWindows()
                    self.opencv_enabled = False
                elif key == ord('r'):
                    self.enable_roi = False
                    self.roi_set = False
                    self.get_logger().info("ROI reset")
                elif key == ord('s') and self.roi_set:
                    self.get_logger().info(f"Current ROI parameters:")
                    self.get_logger().info(f"  roi_x: {self.roi_x}")
                    self.get_logger().info(f"  roi_y: {self.roi_y}")
                    self.get_logger().info(f"  roi_width: {self.roi_width}")
                    self.get_logger().info(f"  roi_height: {self.roi_height}")
                    self.get_logger().info(f"  enable_roi: {self.enable_roi}")
            
            # ROSメッセージとしてpublish
            # アノテーション付き画像
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated_image, "bgr8")
            annotated_msg.header = msg.header
            self.annotated_image_pub.publish(annotated_msg)
            
            # ランドマーク座標
            if pose_landmarks:
                pose_msg = Float32MultiArray()
                pose_msg.data = pose_landmarks
                self.pose_landmarks_pub.publish(pose_msg)
            
            if face_landmarks:
                face_msg = Float32MultiArray()
                face_msg.data = face_landmarks
                self.face_landmarks_pub.publish(face_msg)
            
            if left_hand_landmarks:
                left_hand_msg = Float32MultiArray()
                left_hand_msg.data = left_hand_landmarks
                self.left_hand_landmarks_pub.publish(left_hand_msg)
            
            if right_hand_landmarks:
                right_hand_msg = Float32MultiArray()
                right_hand_msg.data = right_hand_landmarks
                self.right_hand_landmarks_pub.publish(right_hand_msg)
                
        except Exception as e:
            self.get_logger().error(f'Error in image callback: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    
    node = HolisticPoseEstimator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()