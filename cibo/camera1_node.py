#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point32
from sensor_msgs.msg import PointCloud
from std_msgs.msg import Float32MultiArray, Header
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
        self.declare_parameter('detection_confidence', 0.5)
        self.declare_parameter('tracking_confidence', 0.5)
        self.declare_parameter('roi_size', 300)
        self.declare_parameter('enable_face', True)
        self.declare_parameter('enable_pose', True)
        self.declare_parameter('enable_hands', True)
        self.declare_parameter('model_complexity', 1)
        self.declare_parameter('smooth_landmarks', True)
        
        # Get parameters
        self.detection_confidence = self.get_parameter('detection_confidence').get_parameter_value().double_value
        self.tracking_confidence = self.get_parameter('tracking_confidence').get_parameter_value().double_value
        self.roi_size = self.get_parameter('roi_size').get_parameter_value().integer_value
        self.enable_face = self.get_parameter('enable_face').get_parameter_value().bool_value
        self.enable_pose = self.get_parameter('enable_pose').get_parameter_value().bool_value
        self.enable_hands = self.get_parameter('enable_hands').get_parameter_value().bool_value
        self.model_complexity = self.get_parameter('model_complexity').get_parameter_value().integer_value
        self.smooth_landmarks = self.get_parameter('smooth_landmarks').get_parameter_value().bool_value
        
        self.get_logger().info(f"Parameters loaded - Detection: {self.detection_confidence}, "
                              f"Tracking: {self.tracking_confidence}, ROI Size: {self.roi_size}")
        
        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # Initialize MediaPipe
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_drawing_styles = mp.solutions.drawing_styles
        self.mp_holistic = mp.solutions.holistic
        
        # Initialize MediaPipe Holistic model
        try:
            self.holistic = self.mp_holistic.Holistic(
                static_image_mode=False,
                model_complexity=self.model_complexity,
                smooth_landmarks=self.smooth_landmarks,
                enable_segmentation=False,  # Disable segmentation to avoid size mismatch errors
                refine_face_landmarks=self.enable_face,
                min_detection_confidence=self.detection_confidence,
                min_tracking_confidence=self.tracking_confidence
            )
            self.get_logger().info("MediaPipe Holistic model initialized successfully")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize MediaPipe Holistic: {str(e)}")
            return
        
        # ROI state
        self.roi_state = {
            'enabled': False,
            'cx': 320,
            'cy': 240,
            'size': self.roi_size,
            'x1': 120,
            'y1': 40,
            'x2': 520,
            'y2': 440
        }
        
        # Image processing state
        self.last_image_size = None
        self.frame_count = 0
        
        # OpenCV windows setup
        self.opencv_enabled = self.setup_opencv_windows()
        
        # Subscriber
        self.image_sub = self.create_subscription(
            Image, 
            '/camera_01/color/image_raw', 
            self.image_callback, 
            10
        )
        
        # Publishers
        self.annotated_pub = self.create_publisher(
            Image, 
            '/mp/camera_01/annotated_image', 
            10
        )
        
        self.holistic_landmarks_pub = self.create_publisher(
            Float32MultiArray, 
            '/mp/camera_01/holistic_landmarks', 
            10
        )
        
        self.pose_landmarks_pub = self.create_publisher(
            PointCloud, 
            '/mp/camera_01/pose_landmarks', 
            10
        )
        
        self.face_landmarks_pub = self.create_publisher(
            PointCloud, 
            '/mp/camera_01/face_landmarks', 
            10
        )
        
        self.hand_landmarks_pub = self.create_publisher(
            PointCloud, 
            '/mp/camera_01/hand_landmarks', 
            10
        )
        
        self.get_logger().info('Holistic Pose Estimator Node initialized')

    def setup_opencv_windows(self):
        """OpenCVウィンドウのセットアップ"""
        try:
            # Check if display is available
            if 'DISPLAY' not in os.environ:
                self.get_logger().warn("No DISPLAY environment variable found. OpenCV display disabled.")
                return False
            
            cv2.namedWindow('Holistic Pose Estimation', cv2.WINDOW_AUTOSIZE)
            cv2.setMouseCallback('Holistic Pose Estimation', self.mouse_callback)
            
            self.get_logger().info("OpenCV window setup successful")
            return True
            
        except Exception as e:
            self.get_logger().error(f"Failed to setup OpenCV windows: {str(e)}")
            return False

    def mouse_callback(self, event, x, y, flags, param):
        """マウスイベントハンドラ"""
        if event == cv2.EVENT_LBUTTONDOWN:
            self.get_logger().info(f"Mouse clicked at ({x}, {y})")
            # クリック位置を中心にROIを設定
            roi = self.roi_state
            roi['cx'] = x
            roi['cy'] = y
            half = roi['size'] // 2
            roi['x1'] = max(0, x - half)
            roi['y1'] = max(0, y - half)
            roi['x2'] = x + half
            roi['y2'] = y + half
            roi['enabled'] = True
            self.get_logger().info(f"ROI set: center=({roi['cx']},{roi['cy']}), "
                                 f"size={roi['size']} -> ({roi['x1']}, {roi['y1']}) to ({roi['x2']}, {roi['y2']})")

    def ensure_image_consistency(self, cv_image):
        """画像サイズの一貫性を確保"""
        current_size = (cv_image.shape[1], cv_image.shape[0])  # (width, height)
        
        if self.last_image_size is None:
            self.last_image_size = current_size
            self.get_logger().info(f"Initial image size set to: {current_size}")
        elif self.last_image_size != current_size:
            self.get_logger().warn(f"Image size changed from {self.last_image_size} to {current_size}. "
                                 f"Resizing to maintain consistency.")
            # Resize to the first image size to maintain consistency
            cv_image = cv2.resize(cv_image, self.last_image_size)
        
        return cv_image

    def extract_roi(self, cv_image):
        """ROI抽出"""
        height, width = cv_image.shape[:2]
        roi_state = self.roi_state
        
        if roi_state['enabled']:
            # ROI境界をチェック
            x1 = max(0, min(roi_state['x1'], width - 1))
            y1 = max(0, min(roi_state['y1'], height - 1))
            x2 = max(x1 + 1, min(roi_state['x2'], width))
            y2 = max(y1 + 1, min(roi_state['y2'], height))
            
            # Update ROI state with corrected boundaries
            roi_state['x1'], roi_state['y1'], roi_state['x2'], roi_state['y2'] = x1, y1, x2, y2
            
            roi_image = cv_image[y1:y2, x1:x2]
            
            # ROIが空でないことを確認
            if roi_image.size == 0:
                self.get_logger().warn("ROI is empty, using full image")
                return cv_image, (0, 0, width, height)
            
            return roi_image, (x1, y1, x2, y2)
        else:
            return cv_image, (0, 0, width, height)

    def process_holistic(self, image):
        """MediaPipe Holistic処理"""
        try:
            # Convert BGR to RGB
            image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            image_rgb.flags.writeable = False
            
            # Process with MediaPipe Holistic
            results = self.holistic.process(image_rgb)
            
            # Convert back to BGR for OpenCV
            image_rgb.flags.writeable = True
            annotated_image = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR)
            
            return results, annotated_image
            
        except Exception as e:
            self.get_logger().error(f"Error in holistic processing: {str(e)}")
            return None, image

    def draw_landmarks(self, image, results):
        """ランドマーク描画"""
        try:
            # Draw pose landmarks
            if self.enable_pose and results.pose_landmarks:
                self.mp_drawing.draw_landmarks(
                    image, 
                    results.pose_landmarks, 
                    self.mp_holistic.POSE_CONNECTIONS,
                    landmark_drawing_spec=self.mp_drawing_styles.get_default_pose_landmarks_style()
                )
            
            # Draw face landmarks
            if self.enable_face and results.face_landmarks:
                self.mp_drawing.draw_landmarks(
                    image, 
                    results.face_landmarks, 
                    self.mp_holistic.FACEMESH_CONTOURS,
                    landmark_drawing_spec=None,
                    connection_drawing_spec=self.mp_drawing_styles.get_default_face_mesh_contours_style()
                )
            
            # Draw hand landmarks
            if self.enable_hands:
                if results.left_hand_landmarks:
                    self.mp_drawing.draw_landmarks(
                        image, 
                        results.left_hand_landmarks, 
                        self.mp_holistic.HAND_CONNECTIONS,
                        landmark_drawing_spec=self.mp_drawing_styles.get_default_hand_landmarks_style(),
                        connection_drawing_spec=self.mp_drawing_styles.get_default_hand_connections_style()
                    )
                
                if results.right_hand_landmarks:
                    self.mp_drawing.draw_landmarks(
                        image, 
                        results.right_hand_landmarks, 
                        self.mp_holistic.HAND_CONNECTIONS,
                        landmark_drawing_spec=self.mp_drawing_styles.get_default_hand_landmarks_style(),
                        connection_drawing_spec=self.mp_drawing_styles.get_default_hand_connections_style()
                    )
            
        except Exception as e:
            self.get_logger().error(f"Error drawing landmarks: {str(e)}")

    def extract_landmarks_data(self, results, roi_bounds, original_size):
        """ランドマークデータ抽出"""
        landmarks_data = []
        x_offset, y_offset, _, _ = roi_bounds
        orig_width, orig_height = original_size
        
        try:
            # Pose landmarks (33 points)
            if self.enable_pose and results.pose_landmarks:
                for landmark in results.pose_landmarks.landmark:
                    # ROI座標を元画像座標に変換
                    x = (landmark.x * (roi_bounds[2] - roi_bounds[0])) + x_offset
                    y = (landmark.y * (roi_bounds[3] - roi_bounds[1])) + y_offset
                    z = landmark.z
                    landmarks_data.extend([x / orig_width, y / orig_height, z])
            else:
                landmarks_data.extend([0.0] * 99)  # 33 * 3 = 99
            
            # Face landmarks (468 points)
            if self.enable_face and results.face_landmarks:
                for landmark in results.face_landmarks.landmark:
                    x = (landmark.x * (roi_bounds[2] - roi_bounds[0])) + x_offset
                    y = (landmark.y * (roi_bounds[3] - roi_bounds[1])) + y_offset
                    z = landmark.z
                    landmarks_data.extend([x / orig_width, y / orig_height, z])
            else:
                landmarks_data.extend([0.0] * 1404)  # 468 * 3 = 1404
            
            # Left hand landmarks (21 points)
            if self.enable_hands and results.left_hand_landmarks:
                for landmark in results.left_hand_landmarks.landmark:
                    x = (landmark.x * (roi_bounds[2] - roi_bounds[0])) + x_offset
                    y = (landmark.y * (roi_bounds[3] - roi_bounds[1])) + y_offset
                    z = landmark.z
                    landmarks_data.extend([x / orig_width, y / orig_height, z])
            else:
                landmarks_data.extend([0.0] * 63)  # 21 * 3 = 63
            
            # Right hand landmarks (21 points)
            if self.enable_hands and results.right_hand_landmarks:
                for landmark in results.right_hand_landmarks.landmark:
                    x = (landmark.x * (roi_bounds[2] - roi_bounds[0])) + x_offset
                    y = (landmark.y * (roi_bounds[3] - roi_bounds[1])) + y_offset
                    z = landmark.z
                    landmarks_data.extend([x / orig_width, y / orig_height, z])
            else:
                landmarks_data.extend([0.0] * 63)  # 21 * 3 = 63
                
        except Exception as e:
            self.get_logger().error(f"Error extracting landmarks: {str(e)}")
            # Return zeros if extraction fails
            total_landmarks = 99 + 1404 + 63 + 63  # Total expected size
            landmarks_data = [0.0] * total_landmarks
        
        return landmarks_data

    def create_point_cloud(self, landmarks, landmark_type, header):
        """PointCloudメッセージ作成"""
        point_cloud = PointCloud()
        point_cloud.header = header
        
        # Convert landmarks to Point32 array
        for i in range(0, len(landmarks), 3):
            if i + 2 < len(landmarks):
                point = Point32()
                point.x = landmarks[i]
                point.y = landmarks[i + 1]
                point.z = landmarks[i + 2]
                point_cloud.points.append(point)
        
        return point_cloud

    def handle_opencv_display(self, display_image):
        """OpenCV表示処理"""
        if not self.opencv_enabled:
            return
        
        try:
            # ROI矩形を描画
            roi = self.roi_state
            if roi['enabled']:
                cv2.rectangle(display_image, (roi['x1'], roi['y1']), (roi['x2'], roi['y2']), (0, 255, 0), 2)
                cv2.putText(display_image, f"ROI: {roi['size']}px", (roi['x1'], roi['y1']-10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # 操作説明を追加
            cv2.putText(display_image, 'Click to set ROI center', 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(display_image, 'Keys: +/- (ROI size), r (reset), q (quit)', 
                       (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)
            cv2.putText(display_image, f'Frame: {self.frame_count}', 
                       (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)
            
            cv2.imshow('Holistic Pose Estimation', display_image)
            
            # キー入力処理
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                self.get_logger().info("Closing OpenCV windows")
                cv2.destroyAllWindows()
                self.opencv_enabled = False
            elif key == ord('r'):
                self.roi_state['enabled'] = False
                self.get_logger().info("ROI reset")
            elif key in (ord('+'), ord('=')):
                self.roi_state['size'] = min(800, self.roi_state['size'] + 40)
                self.get_logger().info(f"ROI size increased to: {self.roi_state['size']}")
            elif key == ord('-'):
                self.roi_state['size'] = max(40, self.roi_state['size'] - 40)
                self.get_logger().info(f"ROI size decreased to: {self.roi_state['size']}")
                
        except Exception as e:
            self.get_logger().debug(f"OpenCV display error: {str(e)}")

    def image_callback(self, msg):
        """画像コールバック"""
        try:
            self.frame_count += 1
            
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Ensure image size consistency to prevent MediaPipe errors
            cv_image = self.ensure_image_consistency(cv_image)
            
            original_size = (cv_image.shape[1], cv_image.shape[0])  # (width, height)
            
            # Extract ROI
            roi_image, roi_bounds = self.extract_roi(cv_image)
            
            # Process with MediaPipe Holistic
            results, annotated_roi = self.process_holistic(roi_image)
            
            if results is None:
                return
            
            # Draw landmarks on ROI
            self.draw_landmarks(annotated_roi, results)
            
            # Create full annotated image
            annotated_full = cv_image.copy()
            x1, y1, x2, y2 = roi_bounds
            if annotated_roi.shape[:2] == (y2-y1, x2-x1):  # Size check
                annotated_full[y1:y2, x1:x2] = annotated_roi
            
            # Extract landmark data
            landmarks_data = self.extract_landmarks_data(results, roi_bounds, original_size)
            
            # Create header for messages
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = "camera_01"
            
            # Publish annotated image
            try:
                annotated_msg = self.bridge.cv2_to_imgmsg(annotated_full, "bgr8")
                annotated_msg.header = header
                self.annotated_pub.publish(annotated_msg)
            except Exception as e:
                self.get_logger().error(f"Error publishing annotated image: {str(e)}")
            
            # Publish holistic landmarks array
            try:
                landmarks_msg = Float32MultiArray()
                landmarks_msg.data = landmarks_data
                self.holistic_landmarks_pub.publish(landmarks_msg)
            except Exception as e:
                self.get_logger().error(f"Error publishing holistic landmarks: {str(e)}")
            
            # Publish individual landmark point clouds
            try:
                if self.enable_pose and results.pose_landmarks:
                    pose_data = landmarks_data[0:99]  # First 99 elements (33*3)
                    pose_cloud = self.create_point_cloud(pose_data, "pose", header)
                    self.pose_landmarks_pub.publish(pose_cloud)
                
                if self.enable_face and results.face_landmarks:
                    face_data = landmarks_data[99:1503]  # Next 1404 elements (468*3)
                    face_cloud = self.create_point_cloud(face_data, "face", header)
                    self.face_landmarks_pub.publish(face_cloud)
                
                if self.enable_hands:
                    hand_data = landmarks_data[1503:]  # Last 126 elements (42*3)
                    hand_cloud = self.create_point_cloud(hand_data, "hands", header)
                    self.hand_landmarks_pub.publish(hand_cloud)
                    
            except Exception as e:
                self.get_logger().error(f"Error publishing point clouds: {str(e)}")
            
            # Handle OpenCV display
            self.handle_opencv_display(annotated_full)
            
            # Log periodic status
            if self.frame_count % 100 == 0:
                self.get_logger().info(f"Processed {self.frame_count} frames")
                
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

    def destroy_node(self):
        """ノード終了処理"""
        try:
            if hasattr(self, 'holistic'):
                self.holistic.close()
            cv2.destroyAllWindows()
        except Exception as e:
            self.get_logger().error(f"Error during cleanup: {str(e)}")
        
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    node = HolisticPoseEstimator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()