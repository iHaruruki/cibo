#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import cv2
import mediapipe as mp
import numpy as np


class IntegratedMediaPipeNode(Node):
    def __init__(self):
        super().__init__('integrated_mediapipe_processor')
        
        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # Initialize MediaPipe
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_pose = mp.solutions.pose
        self.mp_face_mesh = mp.solutions.face_mesh
        self.mp_hands = mp.solutions.hands
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
        
        self.hands = mp.solutions.hands.Hands(
            max_num_hands=2,
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
        
        # Publishers for annotated images (combined pose, face, and hands)
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
        
        # Publishers for hand landmarks
        self.camera1_hand_landmarks_pub = self.create_publisher(
            Float32MultiArray,
            '/mp/camera_01/hand_landmarks',
            10
        )
        
        self.camera2_hand_landmarks_pub = self.create_publisher(
            Float32MultiArray,
            '/mp/camera_02/hand_landmarks',
            10
        )
        
        # Publishers for face landmarks
        self.camera1_face_landmarks_pub = self.create_publisher(
            Float32MultiArray,
            '/mp/camera_01/face_landmarks',
            10
        )
        
        self.camera2_face_landmarks_pub = self.create_publisher(
            Float32MultiArray,
            '/mp/camera_02/face_landmarks',
            10
        )
        
        self.get_logger().info('Integrated MediaPipe ROS2 Node initialized')

    def process_image(self, cv_image, camera_name):
        """Process image with MediaPipe (pose, face, hands) and return annotated image and landmarks"""
        
        # Convert BGR to RGB for MediaPipe
        image_rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        image_rgb.flags.writeable = False
        
        # Process with MediaPipe
        pose_results = self.pose.process(image_rgb)
        face_results = self.face_mesh.process(image_rgb)
        hands_results = self.hands.process(image_rgb)
        
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
        
        # Draw hand landmarks
        if hands_results.multi_hand_landmarks:
            for hand_landmarks in hands_results.multi_hand_landmarks:
                self.mp_drawing.draw_landmarks(
                    annotated_image, 
                    hand_landmarks, 
                    self.mp_hands.HAND_CONNECTIONS
                )
        
        # Extract hand landmarks data
        hand_landmarks = []
        if hands_results.multi_hand_landmarks:
            for hand_landmarks_data in hands_results.multi_hand_landmarks:
                for landmark in hand_landmarks_data.landmark:
                    hand_landmarks.append(landmark.x * width)
                    hand_landmarks.append(landmark.y * height)
            self.get_logger().debug(f'{camera_name}: Hand landmarks count: {len(hand_landmarks)}')
        else:
            self.get_logger().debug(f'{camera_name}: No hand data')
            hand_landmarks = [0.0, 0.0]  # Default values
        
        # Extract face landmarks data (including pose)
        face_landmarks = []
        
        # Add pose landmarks
        if pose_results.pose_landmarks:
            for landmark in pose_results.pose_landmarks.landmark:
                face_landmarks.append(landmark.x * width)
                face_landmarks.append(landmark.y * height)
            self.get_logger().debug(f'{camera_name}: Pose landmarks count: {len(face_landmarks)}')
        else:
            self.get_logger().debug(f'{camera_name}: No pose data')
            face_landmarks.extend([0.0, 0.0])  # Default values
        
        # Add face landmarks
        if face_results.multi_face_landmarks:
            for face_landmarks_data in face_results.multi_face_landmarks:
                for landmark in face_landmarks_data.landmark:
                    face_landmarks.append(landmark.x * width)
                    face_landmarks.append(landmark.y * height)
            self.get_logger().debug(f'{camera_name}: Total face landmarks count: {len(face_landmarks)}')
        else:
            self.get_logger().debug(f'{camera_name}: No face data')
            face_landmarks.extend([0.0, 0.0])  # Default values
        
        return annotated_image, hand_landmarks, face_landmarks

    def camera1_callback(self, msg):
        """Callback for camera 1"""
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Process image
            annotated_image, hand_landmarks, face_landmarks = self.process_image(cv_image, "camera_01")
            
            # Publish annotated image
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated_image, "bgr8")
            annotated_msg.header = msg.header
            self.camera1_annotated_pub.publish(annotated_msg)
            
            # Publish hand landmarks
            hand_landmarks_msg = Float32MultiArray()
            hand_landmarks_msg.data = hand_landmarks
            self.camera1_hand_landmarks_pub.publish(hand_landmarks_msg)
            
            # Publish face landmarks
            face_landmarks_msg = Float32MultiArray()
            face_landmarks_msg.data = face_landmarks
            self.camera1_face_landmarks_pub.publish(face_landmarks_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error processing camera 1: {str(e)}')

    def camera2_callback(self, msg):
        """Callback for camera 2"""
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Process image
            annotated_image, hand_landmarks, face_landmarks = self.process_image(cv_image, "camera_02")
            
            # Publish annotated image
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated_image, "bgr8")
            annotated_msg.header = msg.header
            self.camera2_annotated_pub.publish(annotated_msg)
            
            # Publish hand landmarks
            hand_landmarks_msg = Float32MultiArray()
            hand_landmarks_msg.data = hand_landmarks
            self.camera2_hand_landmarks_pub.publish(hand_landmarks_msg)
            
            # Publish face landmarks
            face_landmarks_msg = Float32MultiArray()
            face_landmarks_msg.data = face_landmarks
            self.camera2_face_landmarks_pub.publish(face_landmarks_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error processing camera 2: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    
    node = IntegratedMediaPipeNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()