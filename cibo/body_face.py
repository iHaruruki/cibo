# Copyright (c) 2025 Haruki Isono
# This software is released under the MIT License, see LICENSE.

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import cv2
import mediapipe as mp
import numpy as np


class MediaPipeNode(Node):
    def __init__(self):
        super().__init__('mediapipe_processor')
        
        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # Initialize MediaPipe
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_pose = mp.solutions.pose
        self.mp_face_mesh = mp.solutions.face_mesh
        self.mp_drawing_styles = mp.solutions.drawing_styles
        
        # Initialize MediaPipe models
        self.face_mesh = mp.solutions.face_mesh.FaceMesh(
            max_num_faces=1,
            refine_landmarks=True,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )
        
        self.pose = mp.solutions.pose.Pose(
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )
        
        # Subscribers for camera feeds
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
        
        # Publishers for annotated images
        self.camera1_annotated_pub = self.create_publisher(
            Image,
            '/mp/camera_01/annotated_image',
            10
        )
        
        self.camera2_annotated_pub = self.create_publisher(
            Image,
            '/mp/camera_02/annotated_image',
            10
        )
        
        # Publishers for landmarks
        self.camera1_landmarks_pub = self.create_publisher(
            Float32MultiArray,
            '/mp/camera_01/landmarks',
            10
        )
        
        self.camera2_landmarks_pub = self.create_publisher(
            Float32MultiArray,
            '/mp/camera_02/landmarks',
            10
        )
        
        self.get_logger().info('MediaPipe ROS2 Node initialized')

    def process_image(self, cv_image, camera_name):
        """Process image with MediaPipe and return annotated image and landmarks"""
        
        # Convert BGR to RGB for MediaPipe
        image_rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        image_rgb.flags.writeable = False
        
        # Process with MediaPipe
        pose_results = self.pose.process(image_rgb)
        face_results = self.face_mesh.process(image_rgb)
        
        # Convert back to BGR for visualization
        image_rgb.flags.writeable = True
        annotated_image = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR)
        
        # Get image dimensions
        height, width = cv_image.shape[:2]
        
        # Draw pose landmarks
        if pose_results.pose_landmarks:
            self.mp_drawing.draw_landmarks(
                annotated_image,
                pose_results.pose_landmarks,
                self.mp_pose.POSE_CONNECTIONS
            )
        
        # Draw face landmarks
        if face_results.multi_face_landmarks:
            for face_landmarks in face_results.multi_face_landmarks:
                self.mp_drawing.draw_landmarks(
                    image=annotated_image,
                    landmark_list=face_landmarks,
                    connections=self.mp_face_mesh.FACEMESH_CONTOURS,
                    landmark_drawing_spec=None,
                    connection_drawing_spec=self.mp_drawing_styles
                    .get_default_face_mesh_contours_style()
                )
                
                self.mp_drawing.draw_landmarks(
                    image=annotated_image,
                    landmark_list=face_landmarks,
                    connections=self.mp_face_mesh.FACEMESH_IRISES,
                    landmark_drawing_spec=None,
                    connection_drawing_spec=self.mp_drawing_styles
                    .get_default_face_mesh_iris_connections_style()
                )
        
        # Extract landmarks data
        landmarks = []
        
        # Add pose landmarks
        if pose_results.pose_landmarks:
            for landmark in pose_results.pose_landmarks.landmark:
                landmarks.append(landmark.x * width)
                landmarks.append(landmark.y * height)
            self.get_logger().debug(f'{camera_name}: Pose landmarks count: {len(landmarks)}')
        else:
            self.get_logger().debug(f'{camera_name}: No pose data')
            landmarks.extend([0.0, 0.0])  # Default values
        
        # Add face landmarks
        if face_results.multi_face_landmarks:
            for face_landmarks in face_results.multi_face_landmarks:
                for landmark in face_landmarks.landmark:
                    landmarks.append(landmark.x * width)
                    landmarks.append(landmark.y * height)
            self.get_logger().debug(f'{camera_name}: Total landmarks count: {len(landmarks)}')
        else:
            self.get_logger().debug(f'{camera_name}: No face data')
            landmarks.extend([0.0, 0.0])  # Default values
        
        return annotated_image, landmarks

    def camera1_callback(self, msg):
        """Callback for camera 1"""
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Process image
            annotated_image, landmarks = self.process_image(cv_image, "camera_01")
            
            # Publish annotated image
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated_image, "bgr8")
            annotated_msg.header = msg.header
            self.camera1_annotated_pub.publish(annotated_msg)
            
            # Publish landmarks
            landmarks_msg = Float32MultiArray()
            landmarks_msg.data = landmarks
            self.camera1_landmarks_pub.publish(landmarks_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error processing camera 1: {str(e)}')

    def camera2_callback(self, msg):
        """Callback for camera 2"""
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Process image
            annotated_image, landmarks = self.process_image(cv_image, "camera_02")
            
            # Publish annotated image
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated_image, "bgr8")
            annotated_msg.header = msg.header
            self.camera2_annotated_pub.publish(annotated_msg)
            
            # Publish landmarks
            landmarks_msg = Float32MultiArray()
            landmarks_msg.data = landmarks
            self.camera2_landmarks_pub.publish(landmarks_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error processing camera 2: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    
    node = MediaPipeNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()