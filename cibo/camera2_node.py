#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import cv2
import mediapipe as mp
import numpy as np
import time
import os


class HolisticCamera02Node(Node):
    """
    Camera 02 専用 MediaPipe Holistic ノード（共通基底クラスなし版）
    - /camera_02/color/image_raw を購読
    - 推論結果を /mp/camera_02/... にパブリッシュ
    - OpenCV ウィンドウで ROI 操作
    """

    def __init__(self):
        super().__init__('holistic_camera_02')

        self.camera_name = 'camera_02'
        self.image_topic = '/camera_02/color/image_raw'
        self.publish_ns = 'mp'
        self.use_opencv_view = True
        self.enable_roi = True
        self.initial_roi_size = 320
        self.model_complexity = 1
        self.refine_face = True
        self.min_detection_confidence = 0.5
        self.min_tracking_confidence = 0.5
        self.best_effort_qos = True

        self.bridge = CvBridge()
        self.mp_holistic = mp.solutions.holistic
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_styles = mp.solutions.drawing_styles
        self.holistic = self.mp_holistic.Holistic(
            static_image_mode=False,
            model_complexity=self.model_complexity,
            smooth_landmarks=True,
            enable_segmentation=False,
            refine_face_landmarks=self.refine_face,
            min_detection_confidence=self.min_detection_confidence,
            min_tracking_confidence=self.min_tracking_confidence
        )

        self.roi_state = {
            'enabled': False,
            'cx': 200,
            'cy': 200,
            'size': int(self.initial_roi_size),
            'x1': 0, 'y1': 0, 'x2': 0, 'y2': 0
        }
        self.last_click_time = 0.0

        self.opencv_available = False
        if self.use_opencv_view:
            self.opencv_available = self._init_window()

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT if self.best_effort_qos else ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            durability=DurabilityPolicy.VOLATILE
        )

        self.sub = self.create_subscription(
            Image, self.image_topic, self.image_callback, qos
        )

        base = f'/{self.publish_ns}/{self.camera_name}'
        self.annotated_pub = self.create_publisher(Image, f'{base}/annotated_image', 10)
        self.pose_pub = self.create_publisher(Float32MultiArray, f'{base}/pose_landmarks', 10)
        self.face_pub = self.create_publisher(Float32MultiArray, f'{base}/face_landmarks', 10)
        self.left_hand_pub = self.create_publisher(Float32MultiArray, f'{base}/left_hand_landmarks', 10)
        self.right_hand_pub = self.create_publisher(Float32MultiArray, f'{base}/right_hand_landmarks', 10)

        self.get_logger().info('[holistic_camera_02] started')

    def _window_name(self):
        return f'Holistic {self.camera_name}'

    def _init_window(self):
        try:
            self.get_logger().info(f"OpenCV version: {cv2.__version__}")
            self.get_logger().info(f"DISPLAY: {os.environ.get('DISPLAY', 'Not set')}")
            win = self._window_name()
            cv2.namedWindow(win, cv2.WINDOW_AUTOSIZE)
            cv2.setMouseCallback(win, self._mouse_callback)
            return True
        except Exception as e:
            self.get_logger().warn(f"OpenCV window init failed: {e}")
            return False

    def _mouse_callback(self, event, x, y, flags, param=None):
        if event == cv2.EVENT_LBUTTONDOWN and self.enable_roi:
            now = time.time()
            if now - self.last_click_time < 0.05:
                return
            self.last_click_time = now
            roi = self.roi_state
            roi['cx'] = x
            roi['cy'] = y
            half = roi['size'] // 2
            roi['x1'] = max(0, x - half)
            roi['y1'] = max(0, y - half)
            roi['x2'] = x + half
            roi['y2'] = y + half
            roi['enabled'] = True
            self.get_logger().info(
                f"[{self.camera_name}] ROI center=({roi['cx']},{roi['cy']}), size={roi['size']} "
                f"-> ({roi['x1']},{roi['y1']})-({roi['x2']},{roi['y2']})"
            )

    def image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge failed: {e}')
            return

        h, w = frame.shape[:2]

        # ROI
        if self.enable_roi and self.roi_state['enabled']:
            half = self.roi_state['size'] // 2
            cx = int(np.clip(self.roi_state['cx'], 0, w - 1))
            cy = int(np.clip(self.roi_state['cy'], 0, h - 1))
            x1 = max(0, cx - half)
            y1 = max(0, cy - half)
            x2 = min(w, cx + half)
            y2 = min(h, cy + half)
            self.roi_state.update({'x1': x1, 'y1': y1, 'x2': x2, 'y2': y2})
            roi_img = frame[y1:y2, x1:x2]
        else:
            x1 = y1 = 0
            x2 = w
            y2 = h
            roi_img = frame

        roi_w = x2 - x1
        roi_h = y2 - y1

        # 推論
        rgb = cv2.cvtColor(roi_img, cv2.COLOR_BGR2RGB)
        rgb.flags.writeable = False
        results = self.holistic.process(rgb)
        rgb.flags.writeable = True
        annotated_roi = roi_img.copy()

        # 描画
        if results.pose_landmarks:
            self.mp_drawing.draw_landmarks(
                annotated_roi,
                results.pose_landmarks,
                self.mp_holistic.POSE_CONNECTIONS,
                landmark_drawing_spec=self.mp_styles.get_default_pose_landmarks_style()
            )
        if results.face_landmarks:
            self.mp_drawing.draw_landmarks(
                annotated_roi,
                results.face_landmarks,
                self.mp_holistic.FACEMESH_TESSELATION,
                landmark_drawing_spec=None,
                connection_drawing_spec=self.mp_styles.get_default_face_mesh_tesselation_style()
            )
        if results.left_hand_landmarks:
            self.mp_drawing.draw_landmarks(
                annotated_roi,
                results.left_hand_landmarks,
                self.mp_holistic.HAND_CONNECTIONS
            )
        if results.right_hand_landmarks:
            self.mp_drawing.draw_landmarks(
                annotated_roi,
                results.right_hand_landmarks,
                self.mp_holistic.HAND_CONNECTIONS
            )

        annotated_full = frame.copy()
        annotated_full[y1:y2, x1:x2] = annotated_roi

        # OpenCV
        if self.opencv_available:
            disp = annotated_full.copy()
            if self.enable_roi and self.roi_state['enabled']:
                cv2.rectangle(disp, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(disp, f'{self.camera_name}  q:close  +/- resize  r reset',
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (230, 230, 230), 1)
            win = self._window_name()
            try:
                cv2.imshow(win, disp)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    cv2.destroyWindow(win)
                    self.opencv_available = False
                elif key in (ord('+'), ord('=')) and self.enable_roi:
                    self.roi_state['size'] = min(self.roi_state['size'] + 40, max(w, h))
                elif key == ord('-') and self.enable_roi:
                    self.roi_state['size'] = max(self.roi_state['size'] - 40, 20)
                elif key == ord('r') and self.enable_roi:
                    self.roi_state['enabled'] = False
            except Exception:
                self.opencv_available = False

        def to_global(lmk):
            gx = x1 + lmk.x * roi_w
            gy = y1 + lmk.y * roi_h
            return gx, gy, lmk.z

        pose_msg = Float32MultiArray()
        if results.pose_landmarks:
            buf = []
            for l in results.pose_landmarks.landmark:
                gx, gy, gz = to_global(l)
                buf.extend([gx, gy, gz, l.visibility])
            pose_msg.data = buf

        face_msg = Float32MultiArray()
        if results.face_landmarks:
            buf = []
            for l in results.face_landmarks.landmark:
                gx, gy, gz = to_global(l)
                buf.extend([gx, gy, gz])
            face_msg.data = buf

        left_msg = Float32MultiArray()
        if results.left_hand_landmarks:
            buf = []
            for l in results.left_hand_landmarks.landmark:
                gx, gy, gz = to_global(l)
                buf.extend([gx, gy, gz])
            left_msg.data = buf

        right_msg = Float32MultiArray()
        if results.right_hand_landmarks:
            buf = []
            for l in results.right_hand_landmarks.landmark:
                gx, gy, gz = to_global(l)
                buf.extend([gx, gy, gz])
            right_msg.data = buf

        img_msg = self.bridge.cv2_to_imgmsg(annotated_full, 'bgr8')
        img_msg.header = msg.header
        self.annotated_pub.publish(img_msg)
        self.pose_pub.publish(pose_msg)
        self.face_pub.publish(face_msg)
        self.left_hand_pub.publish(left_msg)
        self.right_hand_pub.publish(right_msg)

    def destroy_node(self):
        try:
            self.holistic.close()
        except Exception:
            pass
        if self.opencv_available:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HolisticCamera02Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()