#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class ImageShowNode(Node):
    def __init__(self):
        super().__init__('image_show_node')

        # --- Parameters ---
        self.declare_parameter('window_scale', 1.0)  # 1.0で原寸
        self.window_scale = float(self.get_parameter('window_scale').value)

        # --- Bridges & Buffers ---
        self.bridge = CvBridge()
        self.front_img = None
        self.top_img = None
        self.front_camera_pose_img = None
        self.top_camera_pose_img = None

        # --- QoS for camera streams (sensor-data friendly) ---
        sensor_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )

        # --- Subscriptions (raw images) ---
        self.front_camera_sub = self.create_subscription(
            Image,
            '/camera_01/color/image_raw',
            self.front_camera_callback,
            sensor_qos
        )
        self.top_camera_sub = self.create_subscription(
            Image,
            '/camera_02/color/image_raw',
            self.top_camera_callback,
            sensor_qos
        )

        # --- Subscriptions (annotated images) ---
        self.front_camera_pose_sub = self.create_subscription(
            Image,
            '/front_camera/annotated_image',
            self.front_camera_pose_callback,
            sensor_qos
        )
        self.top_camera_pose_sub = self.create_subscription(
            Image,
            '/top_camera/annotated_image',
            self.top_camera_pose_callback,
            sensor_qos
        )

        # --- GUI timer (process imshow + window events) ---
        self.timer = self.create_timer(0.03, self.gui_loop)  # 30ms周期

        self.get_logger().info('Initialized image_show_node')

    def resize_if_needed(self, img):
        if img is None or self.window_scale == 1.0:
            return img
        h, w = img.shape[:2]
        nw, nh = int(w * self.window_scale), int(h * self.window_scale)
        return cv2.resize(img, (nw, nh), interpolation=cv2.INTER_AREA)

    # ---- Callbacks ----
    def front_camera_callback(self, msg: Image):
        try:
            self.front_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'Front image convert failed: {e}')

    def top_camera_callback(self, msg: Image):
        try:
            self.top_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'Top image convert failed: {e}')

    def front_camera_pose_callback(self, msg: Image):
        try:
            self.front_camera_pose_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'Front camera pose image convert failed: {e}')

    def top_camera_pose_callback(self, msg: Image):
        try:
            self.top_camera_pose_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'Top camera pose image convert failed: {e}')

    # ---- GUI loop ----
    def gui_loop(self):
        # Front
        if self.front_img is not None:
            cv2.imshow('Front Camera', self.resize_if_needed(self.front_img))
        # Top
        if self.top_img is not None:
            cv2.imshow('Top Camera', self.resize_if_needed(self.top_img))
        # Front Pose
        if self.front_camera_pose_img is not None:
            cv2.imshow('Front Camera Pose', self.resize_if_needed(self.front_camera_pose_img))
        # Top Pose
        if self.top_camera_pose_img is not None:
            cv2.imshow('Top Camera Pose', self.resize_if_needed(self.top_camera_pose_img))

        # 必須: GUIイベント処理
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            self.get_logger().info('Quit requested (q).')
            rclpy.shutdown()

    def destroy_node(self):
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ImageShowNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
