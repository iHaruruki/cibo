#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import cv2
import mediapipe as mp
import numpy as np


class FrontCameraNode(Node):
    def __init__(self):
        super().__init__('front_camera')
        
        # ROS Parameters
        self.declare_parameter('roi_enabled', False)
        self.declare_parameter('roi_x', 100)
        self.declare_parameter('roi_y', 100)
        self.declare_parameter('roi_width', 400)
        self.declare_parameter('roi_height', 300)
        self.declare_parameter('detection_confidence', 0.5)
        self.declare_parameter('tracking_confidence', 0.5)
        self.declare_parameter('face_detection_confidence', 0.5)
        self.declare_parameter('face_tracking_confidence', 0.5)
        
        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # Initialize MediaPipe
        self.mp_holistic = mp.solutions.holistic
        self.mp_face_mesh = mp.solutions.face_mesh
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_drawing_styles = mp.solutions.drawing_styles
        
        # Get parameters
        detection_conf = self.get_parameter('detection_confidence').get_parameter_value().double_value
        tracking_conf = self.get_parameter('tracking_confidence').get_parameter_value().double_value
        face_detection_conf = self.get_parameter('face_detection_confidence').get_parameter_value().double_value
        face_tracking_conf = self.get_parameter('face_tracking_confidence').get_parameter_value().double_value
        
        self.holistic = self.mp_holistic.Holistic(
            min_detection_confidence=detection_conf,
            min_tracking_confidence=tracking_conf
        )
        
        # Face mesh for detailed face landmarks
        self.face_mesh = self.mp_face_mesh.FaceMesh(
            max_num_faces=1,
            refine_landmarks=True,
            min_detection_confidence=face_detection_conf,
            min_tracking_confidence=face_tracking_conf
        )
        
        # ROI state
        self.roi_enabled = self.get_parameter('roi_enabled').get_parameter_value().bool_value
        self.roi_x = self.get_parameter('roi_x').get_parameter_value().integer_value
        self.roi_y = self.get_parameter('roi_y').get_parameter_value().integer_value
        self.roi_width = self.get_parameter('roi_width').get_parameter_value().integer_value
        self.roi_height = self.get_parameter('roi_height').get_parameter_value().integer_value
        
        # ROI selection state
        self.selecting_roi = False
        self.roi_start_point = None
        self.roi_end_point = None
        
        # OpenCV setup
        self.setup_opencv_window()
        
        # Subscriber
        self.image_sub = self.create_subscription(
            Image, '/camera_01/color/image_raw', self.image_callback, 10)
        
        # Publishers
        self.annotated_pub = self.create_publisher(
            Image, '/front_camera/annotated_image', 10)
        self.pose_landmarks_pub = self.create_publisher(
            Float32MultiArray, '/front_camera/pose_landmarks', 10)
        self.face_landmarks_pub = self.create_publisher(
            Float32MultiArray, '/front_camera/face_landmarks', 10)
        self.left_hand_landmarks_pub = self.create_publisher(
            Float32MultiArray, '/front_camera/left_hand_landmarks', 10)
        self.right_hand_landmarks_pub = self.create_publisher(
            Float32MultiArray, '/front_camera/right_hand_landmarks', 10)
        
        self.get_logger().info('Front Camera Node initialized')

    def setup_opencv_window(self):
        """OpenCV window setup for ROI selection"""
        try:
            cv2.namedWindow('Front Camera - ROI Selection', cv2.WINDOW_AUTOSIZE)
            cv2.setMouseCallback('Front Camera - ROI Selection', self.mouse_callback)
            self.get_logger().info('OpenCV window setup for ROI selection')
        except Exception as e:
            self.get_logger().error(f'Failed to setup OpenCV window: {str(e)}')

    def mouse_callback(self, event, x, y, flags, param):
        """Mouse callback for ROI selection"""
        if event == cv2.EVENT_LBUTTONDOWN:
            self.selecting_roi = True
            self.roi_start_point = (x, y)
            self.get_logger().info(f'ROI start point: ({x}, {y})')
            
        elif event == cv2.EVENT_LBUTTONUP and self.selecting_roi:
            self.roi_end_point = (x, y)
            self.selecting_roi = False
            
            # Calculate ROI rectangle
            x1 = min(self.roi_start_point[0], self.roi_end_point[0])
            y1 = min(self.roi_start_point[1], self.roi_end_point[1])
            x2 = max(self.roi_start_point[0], self.roi_end_point[0])
            y2 = max(self.roi_start_point[1], self.roi_end_point[1])
            
            self.roi_x = x1
            self.roi_y = y1
            self.roi_width = x2 - x1
            self.roi_height = y2 - y1
            self.roi_enabled = True
            
            # Update parameters
            self.set_parameters([
                Parameter('roi_enabled', Parameter.Type.BOOL, True),
                Parameter('roi_x', Parameter.Type.INTEGER, self.roi_x),
                Parameter('roi_y', Parameter.Type.INTEGER, self.roi_y),
                Parameter('roi_width', Parameter.Type.INTEGER, self.roi_width),
                Parameter('roi_height', Parameter.Type.INTEGER, self.roi_height)
            ])
            
            self.get_logger().info(f'ROI set: x={self.roi_x}, y={self.roi_y}, width={self.roi_width}, height={self.roi_height}')
            
        elif event == cv2.EVENT_MOUSEMOVE and self.selecting_roi:
            self.roi_end_point = (x, y)

    def process_image(self, cv_image):
        """Process image with MediaPipe Holistic and Face Mesh"""
        height, width = cv_image.shape[:2]
        
        # Apply ROI if enabled
        if self.roi_enabled:
            # Ensure ROI is within image bounds
            x1 = max(0, min(self.roi_x, width - 1))
            y1 = max(0, min(self.roi_y, height - 1))
            x2 = max(x1 + 1, min(self.roi_x + self.roi_width, width))
            y2 = max(y1 + 1, min(self.roi_y + self.roi_height, height))
            
            processing_image = cv_image[y1:y2, x1:x2]
            roi_offset = (x1, y1)
        else:
            processing_image = cv_image
            roi_offset = (0, 0)
            x1 = y1 = 0
            x2 = width
            y2 = height
        
        # MediaPipe processing
        image_rgb = cv2.cvtColor(processing_image, cv2.COLOR_BGR2RGB)
        image_rgb.flags.writeable = False
        
        # Process with Holistic
        holistic_results = self.holistic.process(image_rgb)
        
        # Process with Face Mesh for detailed face landmarks
        face_mesh_results = self.face_mesh.process(image_rgb)
        
        image_rgb.flags.writeable = True
        annotated_roi = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR)
        
        # Draw Holistic landmarks on ROI
        if holistic_results.pose_landmarks:
            self.mp_drawing.draw_landmarks(
                annotated_roi, holistic_results.pose_landmarks, self.mp_holistic.POSE_CONNECTIONS,
                landmark_drawing_spec=self.mp_drawing_styles.get_default_pose_landmarks_style())
        
        if holistic_results.left_hand_landmarks:
            self.mp_drawing.draw_landmarks(
                annotated_roi, holistic_results.left_hand_landmarks, self.mp_holistic.HAND_CONNECTIONS,
                landmark_drawing_spec=self.mp_drawing_styles.get_default_hand_landmarks_style())
        
        if holistic_results.right_hand_landmarks:
            self.mp_drawing.draw_landmarks(
                annotated_roi, holistic_results.right_hand_landmarks, self.mp_holistic.HAND_CONNECTIONS,
                landmark_drawing_spec=self.mp_drawing_styles.get_default_hand_landmarks_style())
        
        # Draw detailed face mesh
        if face_mesh_results.multi_face_landmarks:
            for face_landmarks in face_mesh_results.multi_face_landmarks:
                self.mp_drawing.draw_landmarks(
                    annotated_roi, face_landmarks, self.mp_face_mesh.FACEMESH_CONTOURS,
                    landmark_drawing_spec=None,
                    connection_drawing_spec=self.mp_drawing_styles.get_default_face_mesh_contours_style())
                
                # Draw additional face mesh details
                self.mp_drawing.draw_landmarks(
                    annotated_roi, face_landmarks, self.mp_face_mesh.FACEMESH_IRISES,
                    landmark_drawing_spec=None,
                    connection_drawing_spec=self.mp_drawing_styles.get_default_face_mesh_iris_connections_style())
        
        # Create full annotated image
        annotated_image = cv_image.copy()
        if self.roi_enabled:
            annotated_image[y1:y2, x1:x2] = annotated_roi
        else:
            annotated_image = annotated_roi
        
        # Extract landmark data
        pose_landmarks = self.extract_pose_landmarks(holistic_results, width, height, roi_offset)
        face_landmarks = self.extract_face_landmarks(face_mesh_results, width, height, roi_offset)
        left_hand_landmarks = self.extract_hand_landmarks(holistic_results.left_hand_landmarks, width, height, roi_offset)
        right_hand_landmarks = self.extract_hand_landmarks(holistic_results.right_hand_landmarks, width, height, roi_offset)
        
        return annotated_image, pose_landmarks, face_landmarks, left_hand_landmarks, right_hand_landmarks

    def extract_pose_landmarks(self, results, width, height, roi_offset):
        """Extract pose landmarks"""
        landmarks = []
        if results.pose_landmarks:
            for landmark in results.pose_landmarks.landmark:
                # Convert to absolute coordinates and adjust for ROI offset
                x = landmark.x * self.roi_width + roi_offset[0] if self.roi_enabled else landmark.x * width
                y = landmark.y * self.roi_height + roi_offset[1] if self.roi_enabled else landmark.y * height
                landmarks.extend([x, y, landmark.z])
        return landmarks

    def extract_face_landmarks(self, results, width, height, roi_offset):
        """Extract detailed face landmarks from Face Mesh"""
        landmarks = []
        if results.multi_face_landmarks:
            for face_landmarks in results.multi_face_landmarks:
                for landmark in face_landmarks.landmark:
                    # Convert to absolute coordinates and adjust for ROI offset
                    x = landmark.x * self.roi_width + roi_offset[0] if self.roi_enabled else landmark.x * width
                    y = landmark.y * self.roi_height + roi_offset[1] if self.roi_enabled else landmark.y * height
                    landmarks.extend([x, y, landmark.z])
        return landmarks

    def extract_hand_landmarks(self, hand_landmarks, width, height, roi_offset):
        """Extract hand landmarks"""
        landmarks = []
        if hand_landmarks:
            for landmark in hand_landmarks.landmark:
                # Convert to absolute coordinates and adjust for ROI offset
                x = landmark.x * self.roi_width + roi_offset[0] if self.roi_enabled else landmark.x * width
                y = landmark.y * self.roi_height + roi_offset[1] if self.roi_enabled else landmark.y * height
                landmarks.extend([x, y, landmark.z])
        return landmarks

    def image_callback(self, msg):
        """Image callback"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Process image
            annotated_image, pose_landmarks, face_landmarks, left_hand_landmarks, right_hand_landmarks = self.process_image(cv_image)
            
            # Display image with ROI selection
            display_image = annotated_image.copy()
            
            # Draw ROI rectangle
            if self.roi_enabled:
                cv2.rectangle(display_image, 
                            (self.roi_x, self.roi_y), 
                            (self.roi_x + self.roi_width, self.roi_y + self.roi_height), 
                            (0, 255, 0), 2)
                cv2.putText(display_image, 'ROI', 
                          (self.roi_x, self.roi_y - 10), 
                          cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # Draw current selection
            if self.selecting_roi and self.roi_start_point and self.roi_end_point:
                cv2.rectangle(display_image, self.roi_start_point, self.roi_end_point, (255, 0, 0), 2)
            
            # Add instructions
            cv2.putText(display_image, 'Drag to select ROI, Press R to reset, Q to quit', 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            # Add landmark count info
            face_count = len(face_landmarks) // 3 if face_landmarks else 0
            cv2.putText(display_image, f'Face landmarks: {face_count}', 
                       (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            
            cv2.imshow('Front Camera - ROI Selection', display_image)
            
            # Handle key presses
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                cv2.destroyAllWindows()
                rclpy.shutdown()
            elif key == ord('r'):
                self.roi_enabled = False
                self.set_parameters([Parameter('roi_enabled', Parameter.Type.BOOL, False)])
                self.get_logger().info('ROI reset')
            
            # Publish results (always publish, even if empty)
            # Publish annotated image
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated_image, "bgr8")
            annotated_msg.header = msg.header
            self.annotated_pub.publish(annotated_msg)
            
            # Publish landmarks
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
                
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    
    node = FrontCameraNode()
    
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