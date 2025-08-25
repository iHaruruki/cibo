#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import cv2
import mediapipe as mp
import numpy as np


class MediaPipeBodyFaceHandsNode(Node):
    """Combined MediaPipe processor for pose, face mesh and hands.

    Subscribes to two camera topics and publishes a single annotated image
    and a Float32MultiArray containing concatenated landmarks for each
    camera. The landmark array layout is: [pose(x,y)*N, face(x,y)*M, hand(x,y)*K]
    where missing sections are replaced by a single pair of zeros.
    """

    def __init__(self):
        super().__init__('mediapipe_body_face_hand_processor')

        # CV Bridge
        self.bridge = CvBridge()

        # MediaPipe utilities
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_drawing_styles = mp.solutions.drawing_styles
        self.mp_pose = mp.solutions.pose
        self.mp_face_mesh = mp.solutions.face_mesh
        self.mp_hands = mp.solutions.hands

        # Initialize models
        # Keep the default models reasonably performant for realtime
        self.pose = self.mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5)
        self.face_mesh = self.mp_face_mesh.FaceMesh(max_num_faces=1, refine_landmarks=True,
                                                   min_detection_confidence=0.5, min_tracking_confidence=0.5)
        self.hands = self.mp_hands.Hands(max_num_hands=2, min_detection_confidence=0.5, min_tracking_confidence=0.5)

        # Subscribers
        self.camera1_sub = self.create_subscription(Image, '/camera_01/color/image_raw', self.camera1_callback, 10)
        self.camera2_sub = self.create_subscription(Image, '/camera_02/color/image_raw', self.camera2_callback, 10)

        # Publishers
        self.camera1_annotated_pub = self.create_publisher(Image, '/mp/camera_01/annotated_image', 10)
        self.camera2_annotated_pub = self.create_publisher(Image, '/mp/camera_02/annotated_image', 10)

        self.camera1_landmarks_pub = self.create_publisher(Float32MultiArray, '/mp/camera_01/landmarks', 10)
        self.camera2_landmarks_pub = self.create_publisher(Float32MultiArray, '/mp/camera_02/landmarks', 10)

        self.get_logger().info('MediaPipe body+face+hand ROS2 node initialized')

    def process_image(self, cv_image, camera_name):
        """Run all MediaPipe models on the image, draw annotations and return
        annotated image plus concatenated landmarks.
        """
        # Convert to RGB for MediaPipe
        image_rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        image_rgb.flags.writeable = False

        pose_results = self.pose.process(image_rgb)
        face_results = self.face_mesh.process(image_rgb)
        hands_results = self.hands.process(image_rgb)

        # Back to BGR for drawing/visualization
        image_rgb.flags.writeable = True
        annotated_image = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR)

        height, width = cv_image.shape[:2]

        # Draw pose
        if pose_results.pose_landmarks:
            try:
                self.mp_drawing.draw_landmarks(
                    annotated_image,
                    pose_results.pose_landmarks,
                    self.mp_pose.POSE_CONNECTIONS,
                    landmark_drawing_spec=self.mp_drawing_styles.get_default_pose_landmarks_style()
                )
            except Exception:
                # drawing_styles may not have get_default_pose_landmarks_style on some versions
                self.mp_drawing.draw_landmarks(annotated_image, pose_results.pose_landmarks, self.mp_pose.POSE_CONNECTIONS)

        # Draw face mesh
        if face_results.multi_face_landmarks:
            for face_landmarks in face_results.multi_face_landmarks:
                self.mp_drawing.draw_landmarks(
                    image=annotated_image,
                    landmark_list=face_landmarks,
                    connections=self.mp_face_mesh.FACEMESH_TESSELATION,
                    landmark_drawing_spec=None,
                    connection_drawing_spec=self.mp_drawing_styles.get_default_face_mesh_tesselation_style()
                )
                # draw contours/irises if available
                self.mp_drawing.draw_landmarks(
                    image=annotated_image,
                    landmark_list=face_landmarks,
                    connections=self.mp_face_mesh.FACEMESH_CONTOURS,
                    landmark_drawing_spec=None,
                    connection_drawing_spec=self.mp_drawing_styles.get_default_face_mesh_contours_style()
                )

        # Draw hands
        if hands_results.multi_hand_landmarks:
            for hand_landmarks in hands_results.multi_hand_landmarks:
                self.mp_drawing.draw_landmarks(
                    annotated_image,
                    hand_landmarks,
                    self.mp_hands.HAND_CONNECTIONS
                )

        # Collect landmarks into flat list (x*width, y*height)
        landmarks = []

        # Pose landmarks
        if pose_results.pose_landmarks:
            for lm in pose_results.pose_landmarks.landmark:
                landmarks.append(lm.x * width)
                landmarks.append(lm.y * height)
        else:
            # keep a minimal placeholder so consumer can detect empty
            landmarks.extend([0.0, 0.0])

        # Face landmarks
        if face_results.multi_face_landmarks:
            for face_landmarks in face_results.multi_face_landmarks:
                for lm in face_landmarks.landmark:
                    landmarks.append(lm.x * width)
                    landmarks.append(lm.y * height)
        else:
            landmarks.extend([0.0, 0.0])

        # Hand landmarks (all hands concatenated)
        if hands_results.multi_hand_landmarks:
            for hand_landmarks in hands_results.multi_hand_landmarks:
                for lm in hand_landmarks.landmark:
                    landmarks.append(lm.x * width)
                    landmarks.append(lm.y * height)
        else:
            landmarks.extend([0.0, 0.0])

        self.get_logger().debug(f'{camera_name}: landmarks length {len(landmarks)}')

        return annotated_image, landmarks

    def _handle_camera(self, msg, annotated_pub, landmarks_pub, camera_name):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            annotated_image, landmarks = self.process_image(cv_image, camera_name)

            annotated_msg = self.bridge.cv2_to_imgmsg(annotated_image, 'bgr8')
            annotated_msg.header = msg.header
            annotated_pub.publish(annotated_msg)

            landmarks_msg = Float32MultiArray()
            landmarks_msg.data = landmarks
            landmarks_pub.publish(landmarks_msg)

        except Exception as e:
            self.get_logger().error(f'{camera_name} processing error: {e}')

    def camera1_callback(self, msg):
        self._handle_camera(msg, self.camera1_annotated_pub, self.camera1_landmarks_pub, 'camera_01')

    def camera2_callback(self, msg):
        self._handle_camera(msg, self.camera2_annotated_pub, self.camera2_landmarks_pub, 'camera_02')


def main(args=None):
    rclpy.init(args=args)
    node = MediaPipeBodyFaceHandsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
