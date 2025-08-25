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

        # QoS: landmarks は best_effort でも良いが、annotated image を購読する多くのノードは RELIABLE を期待する場合がある。
        qos_best_effort = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=5)
        # annotated image 用に RELIABLE を用意（互換性が必要ならこちらを使う）
        qos_reliable = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=5)

        # Publishers
        self.pub_landmarks_cam1 = self.create_publisher(Int32MultiArray, 'mp/camera_01/landmarks', 10)
        self.pub_landmarks_cam2 = self.create_publisher(Int32MultiArray, 'mp/camera_02/landmarks', 10)
        if self.publish_annotated:
            # ここで RELIABLE を使うことで、RELIABLE サブスクライバとも互換になります
            self.pub_annot_cam1 = self.create_publisher(Image, 'mp/camera_01/annotated_image', qos_reliable)
            self.pub_annot_cam2 = self.create_publisher(Image, 'mp/camera_02/annotated_image', qos_reliable)

        # Subscribers
        # Subscribers は既存コードのまま
        if self.use_compressed:
            self.sub_cam1 = self.create_subscription(CompressedImage, cam1_topic + '/compressed' if not cam1_topic.endswith('/compressed') else cam1_topic, lambda msg: self._image_callback(msg, 'camera_01'), qos_best_effort)
            self.sub_cam2 = self.create_subscription(CompressedImage, cam2_topic + '/compressed' if not cam2_topic.endswith('/compressed') else cam2_topic, lambda msg: self._image_callback(msg, 'camera_02'), qos_best_effort)
        else:
            self.sub_cam1 = self.create_subscription(Image, cam1_topic, lambda msg: self._image_callback(msg, 'camera_01'), qos_best_effort)
            self.sub_cam2 = self.create_subscription(Image, cam2_topic, lambda msg: self._image_callback(msg, 'camera_02'), qos_best_effort)

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
        # decode to cv2 BGR
        cv_img = None
        if isinstance(msg, CompressedImage):
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        else:
            try:
                cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            except CvBridgeError as e:
                self.get_logger().error(f'CvBridge error: {e}')
                return

        if cv_img is None:
            return

        orig_h, orig_w = cv_img.shape[:2]
        # make square by padding
        S = max(orig_h, orig_w)
        pad_top = (S - orig_h) // 2
        pad_bottom = S - orig_h - pad_top
        pad_left = (S - orig_w) // 2
        pad_right = S - orig_w - pad_left
        img_square = cv2.copyMakeBorder(cv_img, pad_top, pad_bottom, pad_left, pad_right,
                                        borderType=cv2.BORDER_CONSTANT, value=(0, 0, 0))

        # MediaPipe expects RGB
        img_rgb = cv2.cvtColor(img_square, cv2.COLOR_BGR2RGB)
        img_rgb.flags.writeable = False

        results_pose = self.pose.process(img_rgb)
        results_face = self.face_mesh.process(img_rgb)

        img_rgb.flags.writeable = True
        annotated = img_square.copy()
        # draw landmarks for debugging
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

        if results_pose.pose_landmarks:
            self.mp_drawing.draw_landmarks(annotated, results_pose.pose_landmarks, self.mp_pose.POSE_CONNECTIONS)

        # Convert normalized (relative to square S) coords back to original image pixel coords:
        landmarks = []
        if results_pose.pose_landmarks:
            for lm in results_pose.pose_landmarks.landmark:
                # lm.x, lm.y are normalized w.r.t width=S and height=S
                px = int(lm.x * S) - pad_left
                py = int(lm.y * S) - pad_top
                # clamp into original image bounds
                px = max(0, min(orig_w - 1, px))
                py = max(0, min(orig_h - 1, py))
                landmarks.append(px)
                landmarks.append(py)
        else:
            landmarks.extend([0, 0])

        if results_face.multi_face_landmarks:
            for face_landmarks in results_face.multi_face_landmarks:
                for lm in face_landmarks.landmark:
                    px = int(lm.x * S) - pad_left
                    py = int(lm.y * S) - pad_top
                    px = max(0, min(orig_w - 1, px))
                    py = max(0, min(orig_h - 1, py))
                    landmarks.append(px)
                    landmarks.append(py)
        else:
            landmarks.extend([0, 0])

        # publish landmarks
        msg_lm = Int32MultiArray()
        msg_lm.data = landmarks
        if camera_name == 'camera_01':
            self.pub_landmarks_cam1.publish(msg_lm)
            if self.publish_annotated:
                try:
                    # publish annotated image in original size (crop out padding)
                    crop = annotated[pad_top:pad_top+orig_h, pad_left:pad_left+orig_w]
                    self.pub_annot_cam1.publish(self.bridge.cv2_to_imgmsg(crop, encoding='bgr8'))
                except CvBridgeError as e:
                    self.get_logger().error(f'CvBridge publish error: {e}')
        else:
            self.pub_landmarks_cam2.publish(msg_lm)
            if self.publish_annotated:
                try:
                    crop = annotated[pad_top:pad_top+orig_h, pad_left:pad_left+orig_w]
                    self.pub_annot_cam2.publish(self.bridge.cv2_to_imgmsg(crop, encoding='bgr8'))
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