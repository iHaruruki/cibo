#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Int32MultiArray
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2
import mediapipe as mp
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class MPProcessorNode(Node):
    def __init__(self):
        super().__init__('mp_processor')

        # パラメータ（必要なら起動時に変更可能）
        self.declare_parameter('camera_01_topic', '/camera_01/color/image_raw')
        self.declare_parameter('camera_02_topic', '/camera_02/color/image_raw')
        self.declare_parameter('use_compressed', False)
        self.declare_parameter('publish_annotated_image', True)

        cam1_topic = self.get_parameter('camera_01_topic').get_parameter_value().string_value
        cam2_topic = self.get_parameter('camera_02_topic').get_parameter_value().string_value
        self.use_compressed = self.get_parameter('use_compressed').get_parameter_value().bool_value
        self.publish_annotated = self.get_parameter('publish_annotated_image').get_parameter_value().bool_value

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # Publishers: ランドマークと（任意）アノテーション画像
        self.pub_landmarks_cam1 = self.create_publisher(Int32MultiArray, 'mp/camera_01/landmarks', 10)
        self.pub_landmarks_cam2 = self.create_publisher(Int32MultiArray, 'mp/camera_02/landmarks', 10)
        if self.publish_annotated:
            self.pub_annot_cam1 = self.create_publisher(Image, 'mp/camera_01/annotated_image', qos)
            self.pub_annot_cam2 = self.create_publisher(Image, 'mp/camera_02/annotated_image', qos)

        # Subscribers
        if self.use_compressed:
            self.sub_cam1 = self.create_subscription(CompressedImage, cam1_topic + '/compressed' if not cam1_topic.endswith('/compressed') else cam1_topic, lambda msg: self._image_callback(msg, 'camera_01'), qos)
            self.sub_cam2 = self.create_subscription(CompressedImage, cam2_topic + '/compressed' if not cam2_topic.endswith('/compressed') else cam2_topic, lambda msg: self._image_callback(msg, 'camera_02'), qos)
        else:
            self.sub_cam1 = self.create_subscription(Image, cam1_topic, lambda msg: self._image_callback(msg, 'camera_01'), qos)
            self.sub_cam2 = self.create_subscription(Image, cam2_topic, lambda msg: self._image_callback(msg, 'camera_02'), qos)

        self.bridge = CvBridge()

        # MediaPipe の初期化（Pose と FaceMesh）
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_pose = mp.solutions.pose
        self.mp_face_mesh = mp.solutions.face_mesh
        self.mp_styles = mp.solutions.drawing_styles

        self.pose = self.mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5)
        self.face_mesh = self.mp_face_mesh.FaceMesh(max_num_faces=1, refine_landmarks=True, min_detection_confidence=0.5, min_tracking_confidence=0.5)

        self.get_logger().info('mp_processor node started')

    def _decode_image(self, msg):
        # msg は sensor_msgs/Image または sensor_msgs/CompressedImage
        if isinstance(msg, CompressedImage):
            # JPEG/PNG bytes -> cv2 image
            np_arr = np.frombuffer(msg.data, np.uint8)
            image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            return image
        else:
            try:
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                return cv_image
            except CvBridgeError as e:
                self.get_logger().error(f'CvBridge error: {e}')
                return None

    def _image_callback(self, msg, camera_name):
        cv_img = self._decode_image(msg)
        if cv_img is None:
            return

        height, width = cv_img.shape[:2]

        # MediaPipe は RGB 入力を期待
        image_rgb = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        image_rgb.flags.writeable = False

        # Pose と FaceMesh を処理
        results_pose = self.pose.process(image_rgb)
        results_face = self.face_mesh.process(image_rgb)

        image_rgb.flags.writeable = True
        annotated = cv_img.copy()

        # Draw face landmarks
        if results_face.multi_face_landmarks:
            for face_landmarks in results_face.multi_face_landmarks:
                self.mp_drawing.draw_landmarks(
                    image=annotated,
                    landmark_list=face_landmarks,
                    connections=self.mp_face_mesh.FACEMESH_CONTOURS,
                    landmark_drawing_spec=None,
                    connection_drawing_spec=self.mp_styles.get_default_face_mesh_contours_style())
                self.mp_drawing.draw_landmarks(
                    image=annotated,
                    landmark_list=face_landmarks,
                    connections=self.mp_face_mesh.FACEMESH_IRISES,
                    landmark_drawing_spec=None,
                    connection_drawing_spec=self.mp_styles.get_default_face_mesh_iris_connections_style())

        # Pose landmarks
        if results_pose.pose_landmarks:
            self.mp_drawing.draw_landmarks(annotated, results_pose.pose_landmarks, self.mp_pose.POSE_CONNECTIONS)

        # ランドマーク配列を作成（x*width, y*height を int に変換）
        landmarks = []
        if results_pose.pose_landmarks:
            for lm in results_pose.pose_landmarks.landmark:
                landmarks.append(int(lm.x * width))
                landmarks.append(int(lm.y * height))
        else:
            # 空データのハンドリング（必要に応じて変更）
            landmarks.extend([0, 0])

        if results_face.multi_face_landmarks:
            for face_landmarks in results_face.multi_face_landmarks:
                for lm in face_landmarks.landmark:
                    landmarks.append(int(lm.x * width))
                    landmarks.append(int(lm.y * height))
        else:
            # 顔ランドマークが無い場合は 0 を付与（適宜調整）
            landmarks.append(0)
            landmarks.append(0)

        # Publish landmarks
        msg_lm = Int32MultiArray()
        msg_lm.data = landmarks
        if camera_name == 'camera_01':
            self.pub_landmarks_cam1.publish(msg_lm)
            if self.publish_annotated:
                try:
                    self.pub_annot_cam1.publish(self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8'))
                except CvBridgeError as e:
                    self.get_logger().error(f'CvBridge publish error: {e}')
        else:
            self.pub_landmarks_cam2.publish(msg_lm)
            if self.publish_annotated:
                try:
                    self.pub_annot_cam2.publish(self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8'))
                except CvBridgeError as e:
                    self.get_logger().error(f'CvBridge publish error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = MPProcessorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down mp_processor')
        node.pose.close()
        node.face_mesh.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()