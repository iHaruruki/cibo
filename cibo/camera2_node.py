#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import cv2
import mediapipe as mp
import numpy as np


class FrontCameraNode(Node):
    def __init__(self):
        super().__init__('front_camera')
        
        # Parameters
        self.declare_parameter('roi_x', 100)
        self.declare_parameter('roi_y', 100)
        self.declare_parameter('roi_width', 400)
        self.declare_parameter('roi_height', 300)
        self.declare_parameter('min_detection_confidence', 0.5)
        self.declare_parameter('min_tracking_confidence', 0.5)
        
        # Get parameters
        self.roi_x = self.get_parameter('roi_x').value
        self.roi_y = self.get_parameter('roi_y').value
        self.roi_width = self.get_parameter('roi_width').value
        self.roi_height = self.get_parameter('roi_height').value
        self.min_detection_confidence = self.get_parameter('min_detection_confidence').value
        self.min_tracking_confidence = self.get_parameter('min_tracking_confidence').value
        
        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # Initialize MediaPipe
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_drawing_styles = mp.solutions.drawing_styles
        
        # Initialize MediaPipe Holistic
        self.holistic = mp.solutions.holistic.Holistic(
            min_detection_confidence=self.min_detection_confidence,
            min_tracking_confidence=self.min_tracking_confidence
        )
        
        # ROI selection state
        self.roi_selected = False
        self.selecting_roi = True
        
        # Subscriber
        self.image_sub = self.create_subscription(
            Image, '/camera_02/color/image_raw', self.image_callback, 10)
        
        # Publishers
        self.annotated_pub = self.create_publisher(
            Image, '/front_camera/annotated_image', 10)
        self.pose_landmarks_pub = self.create_publisher(
            Float32MultiArray, '/front_camera/pose_landmarks', 10)
        self.left_hand_landmarks_pub = self.create_publisher(
            Float32MultiArray, '/front_camera/left_hand_landmarks', 10)
        self.right_hand_landmarks_pub = self.create_publisher(
            Float32MultiArray, '/front_camera/right_hand_landmarks', 10)
        
        # OpenCV window for ROI selection
        cv2.namedWindow('Front Camera - ROI Selection', cv2.WINDOW_AUTOSIZE)
        cv2.setMouseCallback('Front Camera - ROI Selection', self.mouse_callback)
        
        self.get_logger().info('Front Camera Node initialized')
        self.get_logger().info(f'Initial ROI: x={self.roi_x}, y={self.roi_y}, width={self.roi_width}, height={self.roi_height}')
        self.get_logger().info('Click and drag to select ROI, press SPACE to confirm, ESC to use parameters')

    def mouse_callback(self, event, x, y, flags, param):
        """Mouse callback for ROI selection"""
        if self.selecting_roi:
            if event == cv2.EVENT_LBUTTONDOWN:
                self.roi_start = (x, y)
                self.roi_end = (x, y)
                
            elif event == cv2.EVENT_MOUSEMOVE and flags == cv2.EVENT_FLAG_LBUTTON:
                self.roi_end = (x, y)
                
            elif event == cv2.EVENT_LBUTTONUP:
                self.roi_end = (x, y)
                # Update ROI parameters
                self.roi_x = min(self.roi_start[0], self.roi_end[0])
                self.roi_y = min(self.roi_start[1], self.roi_end[1])
                self.roi_width = abs(self.roi_end[0] - self.roi_start[0])
                self.roi_height = abs(self.roi_end[1] - self.roi_start[1])

    def image_callback(self, msg):
        """Image callback"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # ROI selection mode
            if self.selecting_roi:
                self.handle_roi_selection(cv_image)
                return
            
            # Process image with MediaPipe
            annotated_image = self.process_image(cv_image)
            
            # Publish annotated image
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated_image, "bgr8")
            annotated_msg.header = msg.header
            self.annotated_pub.publish(annotated_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

    def handle_roi_selection(self, image):
        """Handle ROI selection"""
        display_image = image.copy()
        
        # Draw current ROI
        cv2.rectangle(display_image, 
                     (self.roi_x, self.roi_y), 
                     (self.roi_x + self.roi_width, self.roi_y + self.roi_height), 
                     (0, 255, 0), 2)
        
        # Draw instructions
        cv2.putText(display_image, 'Click and drag to select ROI', 
                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(display_image, 'Press SPACE to confirm, ESC to use parameters', 
                   (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(display_image, f'ROI: ({self.roi_x}, {self.roi_y}, {self.roi_width}, {self.roi_height})', 
                   (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        # If dragging, show current selection
        if hasattr(self, 'roi_start') and hasattr(self, 'roi_end'):
            cv2.rectangle(display_image, self.roi_start, self.roi_end, (255, 0, 0), 2)
        
        cv2.imshow('Front Camera - ROI Selection', display_image)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord(' '):  # Space to confirm
            self.selecting_roi = False
            cv2.destroyWindow('Front Camera - ROI Selection')
            self.get_logger().info(f'ROI confirmed: x={self.roi_x}, y={self.roi_y}, width={self.roi_width}, height={self.roi_height}')
        elif key == 27:  # ESC to use parameters
            self.selecting_roi = False
            cv2.destroyWindow('Front Camera - ROI Selection')
            self.get_logger().info('Using ROI from parameters')

    def process_image(self, image):
        """Process image with MediaPipe"""
        height, width = image.shape[:2]
        
        # Extract ROI
        roi_x2 = min(self.roi_x + self.roi_width, width)
        roi_y2 = min(self.roi_y + self.roi_height, height)
        roi_image = image[self.roi_y:roi_y2, self.roi_x:roi_x2]
        
        if roi_image.size == 0:
            self.get_logger().warn('ROI is empty, using full image')
            roi_image = image
            roi_x_offset = 0
            roi_y_offset = 0
        else:
            roi_x_offset = self.roi_x
            roi_y_offset = self.roi_y
        
        # Convert BGR to RGB
        rgb_image = cv2.cvtColor(roi_image, cv2.COLOR_BGR2RGB)
        rgb_image.flags.writeable = False
        
        # Process with MediaPipe Holistic
        holistic_results = self.holistic.process(rgb_image)
        
        # Convert back to BGR
        rgb_image.flags.writeable = True
        annotated_roi = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)
        
        # Draw pose landmarks
        if holistic_results.pose_landmarks:
            self.mp_drawing.draw_landmarks(
                annotated_roi, 
                holistic_results.pose_landmarks,
                mp.solutions.holistic.POSE_CONNECTIONS,
                landmark_drawing_spec=self.mp_drawing_styles.get_default_pose_landmarks_style())
            
            # Publish pose landmarks
            self.publish_pose_landmarks(holistic_results.pose_landmarks, roi_x_offset, roi_y_offset)
        
        # Draw hand landmarks
        if holistic_results.left_hand_landmarks:
            self.mp_drawing.draw_landmarks(
                annotated_roi,
                holistic_results.left_hand_landmarks,
                mp.solutions.holistic.HAND_CONNECTIONS,
                landmark_drawing_spec=self.mp_drawing_styles.get_default_hand_landmarks_style(),
                connection_drawing_spec=self.mp_drawing_styles.get_default_hand_connections_style())
            
            # Publish left hand landmarks
            self.publish_hand_landmarks(holistic_results.left_hand_landmarks, roi_x_offset, roi_y_offset, 'left')
        
        if holistic_results.right_hand_landmarks:
            self.mp_drawing.draw_landmarks(
                annotated_roi,
                holistic_results.right_hand_landmarks,
                mp.solutions.holistic.HAND_CONNECTIONS,
                landmark_drawing_spec=self.mp_drawing_styles.get_default_hand_landmarks_style(),
                connection_drawing_spec=self.mp_drawing_styles.get_default_hand_connections_style())
            
            # Publish right hand landmarks
            self.publish_hand_landmarks(holistic_results.right_hand_landmarks, roi_x_offset, roi_y_offset, 'right')
        
        # Create full image with ROI overlay
        annotated_image = image.copy()
        annotated_image[self.roi_y:roi_y2, self.roi_x:roi_x2] = annotated_roi
        
        # Draw ROI rectangle on full image
        cv2.rectangle(annotated_image, (self.roi_x, self.roi_y), (roi_x2, roi_y2), (0, 255, 0), 2)
        cv2.putText(annotated_image, 'ROI', (self.roi_x, self.roi_y-10), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        return annotated_image

    def publish_pose_landmarks(self, landmarks, roi_x_offset, roi_y_offset):
        """Publish pose landmarks"""
        msg = Float32MultiArray()
        data = []
        
        for landmark in landmarks.landmark:
            # Convert normalized coordinates to image coordinates
            x = landmark.x * self.roi_width + roi_x_offset
            y = landmark.y * self.roi_height + roi_y_offset
            z = landmark.z
            visibility = landmark.visibility
            
            data.extend([x, y, z, visibility])
        
        msg.data = data
        self.pose_landmarks_pub.publish(msg)

    def publish_hand_landmarks(self, landmarks, roi_x_offset, roi_y_offset, hand_type):
        """Publish hand landmarks"""
        msg = Float32MultiArray()
        data = []
        
        for landmark in landmarks.landmark:
            # Convert normalized coordinates to image coordinates
            x = landmark.x * self.roi_width + roi_x_offset
            y = landmark.y * self.roi_height + roi_y_offset
            z = landmark.z
            
            data.extend([x, y, z])
        
        msg.data = data
        
        if hand_type == 'left':
            self.left_hand_landmarks_pub.publish(msg)
        else:
            self.right_hand_landmarks_pub.publish(msg)

    def destroy_node(self):
        """Clean up"""
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    node = FrontCameraNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()