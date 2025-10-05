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


class TopCameraNode(Node):
    def __init__(self):
        super().__init__('top_camera')
        
        # Parameters
        self.declare_parameter('roi_x', 0)
        self.declare_parameter('roi_y', 0)
        self.declare_parameter('roi_width', 640)
        self.declare_parameter('roi_height', 480)
        self.declare_parameter('min_detection_confidence', 0.5)
        self.declare_parameter('min_tracking_confidence', 0.5)
        self.declare_parameter('enable_roi', False)
        
        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # Initialize MediaPipe
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_drawing_styles = mp.solutions.drawing_styles
        self.mp_holistic = mp.solutions.holistic
        
        # Initialize MediaPipe Holistic
        self.holistic = self.mp_holistic.Holistic(
            min_detection_confidence=self.get_parameter('min_detection_confidence').value,
            min_tracking_confidence=self.get_parameter('min_tracking_confidence').value,
            enable_segmentation=False,
            refine_face_landmarks=False  # Top camera focuses on hands and pose
        )
        
        # ROI selection variables
        self.roi_selecting = False
        self.roi_start_point = None
        self.roi_end_point = None
        self.roi_x = self.get_parameter('roi_x').value
        self.roi_y = self.get_parameter('roi_y').value
        self.roi_width = self.get_parameter('roi_width').value
        self.roi_height = self.get_parameter('roi_height').value
        self.enable_roi = self.get_parameter('enable_roi').value
        
        # Setup OpenCV window for ROI selection
        self.setup_opencv_window()
        
        # Subscriber
        self.image_sub = self.create_subscription(
            Image,
            '/camera_02/color/image_raw',
            self.image_callback,
            10
        )
        
        # Publishers
        self.annotated_image_pub = self.create_publisher(
            Image,
            '/top_camera/annotated_image',
            10
        )
        self.pose_landmarks_pub = self.create_publisher(
            Float32MultiArray,
            '/top_camera/pose_landmarks',
            10
        )
        self.left_hand_landmarks_pub = self.create_publisher(
            Float32MultiArray,
            '/top_camera/left_hand_landmarks',
            10
        )
        self.right_hand_landmarks_pub = self.create_publisher(
            Float32MultiArray,
            '/top_camera/right_hand_landmarks',
            10
        )
        
        self.get_logger().info('Top Camera Node initialized')
    
    def setup_opencv_window(self):
        """Setup OpenCV window for ROI selection"""
        try:
            cv2.namedWindow('Top Camera ROI', cv2.WINDOW_AUTOSIZE)
            cv2.setMouseCallback('Top Camera ROI', self.mouse_callback)
            self.get_logger().info('OpenCV window setup for ROI selection')
        except Exception as e:
            self.get_logger().error(f'Failed to setup OpenCV window: {str(e)}')
    
    def mouse_callback(self, event, x, y, flags, param):
        """Mouse callback for ROI selection"""
        if event == cv2.EVENT_LBUTTONDOWN:
            self.roi_selecting = True
            self.roi_start_point = (x, y)
            self.get_logger().info(f'ROI start point: ({x}, {y})')
            
        elif event == cv2.EVENT_LBUTTONUP and self.roi_selecting:
            self.roi_selecting = False
            self.roi_end_point = (x, y)
            
            # Calculate ROI parameters
            x1, y1 = self.roi_start_point
            x2, y2 = self.roi_end_point
            
            self.roi_x = min(x1, x2)
            self.roi_y = min(y1, y2)
            self.roi_width = abs(x2 - x1)
            self.roi_height = abs(y2 - y1)
            self.enable_roi = True
            
            # Update parameters
            self.set_parameters([
                Parameter('roi_x', Parameter.Type.INTEGER, self.roi_x),
                Parameter('roi_y', Parameter.Type.INTEGER, self.roi_y),
                Parameter('roi_width', Parameter.Type.INTEGER, self.roi_width),
                Parameter('roi_height', Parameter.Type.INTEGER, self.roi_height),
                Parameter('enable_roi', Parameter.Type.BOOL, self.enable_roi)
            ])
            
            self.get_logger().info(
                f'ROI set: x={self.roi_x}, y={self.roi_y}, '
                f'width={self.roi_width}, height={self.roi_height}'
            )
    
    def extract_roi(self, image):
        """Extract ROI from image"""
        if not self.enable_roi:
            return image, 0, 0
        
        height, width = image.shape[:2]
        
        # Ensure ROI is within image bounds
        x = max(0, min(self.roi_x, width - 1))
        y = max(0, min(self.roi_y, height - 1))
        w = min(self.roi_width, width - x)
        h = min(self.roi_height, height - y)
        
        roi = image[y:y+h, x:x+w]
        return roi, x, y
    
    def process_landmarks(self, landmarks, roi_offset_x=0, roi_offset_y=0, image_width=1, image_height=1):
        """Process landmarks and convert to pixel coordinates"""
        if landmarks is None:
            return []
        
        landmark_array = []
        for landmark in landmarks.landmark:
            # Convert normalized coordinates to pixel coordinates
            x = landmark.x * image_width + roi_offset_x
            y = landmark.y * image_height + roi_offset_y
            landmark_array.extend([x, y, landmark.z if hasattr(landmark, 'z') else 0.0])
        
        return landmark_array
    
    def image_callback(self, msg):
        """Image callback function"""
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            original_height, original_width = cv_image.shape[:2]
            
            # Extract ROI
            roi_image, roi_x, roi_y = self.extract_roi(cv_image)
            roi_height, roi_width = roi_image.shape[:2]
            
            # Convert to RGB for MediaPipe
            rgb_image = cv2.cvtColor(roi_image, cv2.COLOR_BGR2RGB)
            rgb_image.flags.writeable = False
            
            # Process with MediaPipe Holistic
            results = self.holistic.process(rgb_image)
            
            # Convert back to BGR for OpenCV
            rgb_image.flags.writeable = True
            annotated_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)
            
            # Draw pose landmarks
            if results.pose_landmarks:
                self.mp_drawing.draw_landmarks(
                    annotated_image,
                    results.pose_landmarks,
                    self.mp_holistic.POSE_CONNECTIONS,
                    landmark_drawing_spec=self.mp_drawing_styles.get_default_pose_landmarks_style()
                )
            
            # Draw hand landmarks
            if results.left_hand_landmarks:
                self.mp_drawing.draw_landmarks(
                    annotated_image,
                    results.left_hand_landmarks,
                    self.mp_holistic.HAND_CONNECTIONS,
                    landmark_drawing_spec=self.mp_drawing_styles.get_default_hand_landmarks_style(),
                    connection_drawing_spec=self.mp_drawing_styles.get_default_hand_connections_style()
                )
            
            if results.right_hand_landmarks:
                self.mp_drawing.draw_landmarks(
                    annotated_image,
                    results.right_hand_landmarks,
                    self.mp_holistic.HAND_CONNECTIONS,
                    landmark_drawing_spec=self.mp_drawing_styles.get_default_hand_landmarks_style(),
                    connection_drawing_spec=self.mp_drawing_styles.get_default_hand_connections_style()
                )
            
            # Create full-size annotated image
            full_annotated_image = cv_image.copy()
            if self.enable_roi:
                full_annotated_image[roi_y:roi_y+roi_height, roi_x:roi_x+roi_width] = annotated_image
            else:
                full_annotated_image = annotated_image
            
            # Display for ROI selection
            display_image = full_annotated_image.copy()
            
            # Draw ROI rectangle
            if self.enable_roi:
                cv2.rectangle(
                    display_image,
                    (roi_x, roi_y),
                    (roi_x + roi_width, roi_y + roi_height),
                    (0, 255, 0), 2
                )
                cv2.putText(
                    display_image, 'ROI',
                    (roi_x, roi_y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2
                )
            
            # Add instruction text
            cv2.putText(
                display_image,
                'Drag to select ROI, Press R to reset, Q to quit',
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2
            )
            
            cv2.imshow('Top Camera ROI', display_image)
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord('q'):
                cv2.destroyAllWindows()
            elif key == ord('r'):
                self.enable_roi = False
                self.set_parameters([Parameter('enable_roi', Parameter.Type.BOOL, False)])
                self.get_logger().info('ROI reset')
            
            # Publish annotated image
            annotated_msg = self.bridge.cv2_to_imgmsg(full_annotated_image, 'bgr8')
            annotated_msg.header = msg.header
            self.annotated_image_pub.publish(annotated_msg)
            
            # Process and publish landmarks
            pose_landmarks = self.process_landmarks(
                results.pose_landmarks, roi_x, roi_y, roi_width, roi_height
            )
            
            left_hand_landmarks = self.process_landmarks(
                results.left_hand_landmarks, roi_x, roi_y, roi_width, roi_height
            )
            
            right_hand_landmarks = self.process_landmarks(
                results.right_hand_landmarks, roi_x, roi_y, roi_width, roi_height
            )
            
            # Publish landmarks
            pose_msg = Float32MultiArray()
            pose_msg.data = pose_landmarks
            self.pose_landmarks_pub.publish(pose_msg)
            
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
    
    node = TopCameraNode()
    
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