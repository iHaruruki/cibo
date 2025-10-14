#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import cv2
import mediapipe as mp
import numpy as np
import message_filters
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class TopCameraNode(Node):
    def __init__(self):
        super().__init__('top_camera')

        # === CV Bridge ===
        self.bridge = CvBridge()

        # === MediaPipe ===
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_drawing_styles = mp.solutions.drawing_styles
        self.mp_holistic = mp.solutions.holistic
        self.holistic = self.mp_holistic.Holistic(
            min_detection_confidence=self.declare_parameter('min_detection_confidence', 0.5).value,
            min_tracking_confidence=self.declare_parameter('min_tracking_confidence', 0.5).value
        )

        # === ROI params ===
        self.roi_enabled = self.declare_parameter('roi_enabled', False).value
        self.roi_x = self.declare_parameter('roi_x', 0).value
        self.roi_y = self.declare_parameter('roi_y', 0).value
        self.roi_width = self.declare_parameter('roi_width', 400).value
        self.roi_height = self.declare_parameter('roi_height', 300).value

        # === Topic / frame params ===
        self.color_topic = self.declare_parameter('color_topic', '/camera_02/color/image_raw').value
        self.depth_topic = self.declare_parameter('depth_topic', '/camera_02/depth/image_raw').value
        self.depth_info_topic = self.declare_parameter('depth_info_topic', '/camera_02/depth/camera_info').value
        self.camera_frame = self.declare_parameter('camera_frame', 'camera_02_depth_optical_frame').value
        self.tf_rate_hz = float(self.declare_parameter('tf_rate_hz', 20.0).value)

        # === GUI state ===
        self.dragging = False
        self.start_point = None
        self.end_point = None
        self.setup_opencv_window()

        # === Publishers ===
        self.annotated_pub = self.create_publisher(Image, '/top_camera/annotated_image', 10)
        self.pose_landmarks_pub = self.create_publisher(Float32MultiArray, '/top_camera/pose_landmarks', 10)
        self.left_hand_landmarks_pub = self.create_publisher(Float32MultiArray, '/top_camera/left_hand_landmarks', 10)
        self.right_hand_landmarks_pub = self.create_publisher(Float32MultiArray, '/top_camera/right_hand_landmarks', 10)

        # === TF ===
        self.tf_broadcaster = TransformBroadcaster(self)
        self.last_tf_time = self.get_clock().now()

        # === Subscribers (sync) ===
        color_sub = message_filters.Subscriber(self, Image, self.color_topic, qos_profile=10)
        depth_sub = message_filters.Subscriber(self, Image, self.depth_topic, qos_profile=10)
        depth_info_sub = message_filters.Subscriber(self, CameraInfo, self.depth_info_topic, qos_profile=10)

        ats = message_filters.ApproximateTimeSynchronizer(
            [color_sub, depth_sub, depth_info_sub], queue_size=10, slop=0.06
        )
        ats.registerCallback(self.synced_callback)

        self.get_logger().info('Top Camera Node initialized (depth→3D + tf broadcasting)')

    # ===== GUI =====
    def setup_opencv_window(self):
        try:
            cv2.namedWindow('Top Camera - ROI Selection', cv2.WINDOW_AUTOSIZE)
            cv2.setMouseCallback('Top Camera - ROI Selection', self.mouse_callback)
        except Exception as e:
            self.get_logger().error(f'Failed to setup OpenCV window: {str(e)}')

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.dragging = True
            self.start_point = (x, y)
            self.end_point = (x, y)
        elif event == cv2.EVENT_MOUSEMOVE and self.dragging:
            self.end_point = (x, y)
        elif event == cv2.EVENT_LBUTTONUP and self.dragging and self.start_point:
            self.dragging = False
            self.end_point = (x, y)
            x1, y1 = min(self.start_point[0], self.end_point[0]), min(self.start_point[1], self.end_point[1])
            x2, y2 = max(self.start_point[0], self.end_point[0]), max(self.start_point[1], self.end_point[1])
            self.roi_x, self.roi_y = x1, y1
            self.roi_width, self.roi_height = x2 - x1, y2 - y1
            self.roi_enabled = True
            self.set_parameters([
                Parameter('roi_enabled', Parameter.Type.BOOL, True),
                Parameter('roi_x', Parameter.Type.INTEGER, self.roi_x),
                Parameter('roi_y', Parameter.Type.INTEGER, self.roi_y),
                Parameter('roi_width', Parameter.Type.INTEGER, self.roi_width),
                Parameter('roi_height', Parameter.Type.INTEGER, self.roi_height),
            ])
            self.get_logger().info(f'ROI set: x={self.roi_x}, y={self.roi_y}, w={self.roi_width}, h={self.roi_height}')

    # ===== Core =====
    def synced_callback(self, color_msg: Image, depth_msg: Image, depth_info: CameraInfo):
        # color
        try:
            color = self.bridge.imgmsg_to_cv2(color_msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'color cv bridge error: {e}')
            return

        # depth → meters
        try:
            depth = self.bridge.imgmsg_to_cv2(depth_msg)
            if depth_msg.encoding in ('16UC1', 'mono16'):
                depth_m = depth.astype(np.float32) / 1000.0
            elif depth_msg.encoding in ('32FC1',):
                depth_m = depth.astype(np.float32)
            else:
                depth_m = depth.astype(np.float32)  # assume meters
        except Exception as e:
            self.get_logger().error(f'depth cv bridge error: {e}')
            return

        # MediaPipe & drawing
        annotated_image, pose_lm, lhand_lm, rhand_lm, roi_ctx = self.process_image(color)

        # publish annotated & 2D lists
        ann = self.bridge.cv2_to_imgmsg(annotated_image, "bgr8")
        ann.header = color_msg.header
        self.annotated_pub.publish(ann)
        self._publish_array(self.pose_landmarks_pub, pose_lm)
        self._publish_array(self.left_hand_landmarks_pub, lhand_lm)
        self._publish_array(self.right_hand_landmarks_pub, rhand_lm)

        # TF rate limit
        now = self.get_clock().now()
        if (now - self.last_tf_time).nanoseconds < (1e9 / self.tf_rate_hz):
            return
        self.last_tf_time = now

        # intrinsics from depth (registered to color assumed)
        fx = depth_info.k[0]; fy = depth_info.k[4]
        cx = depth_info.k[2]; cy = depth_info.k[5]

        # broadcast helper
        def broadcast_set(flat_xyz, prefix):
            n = len(flat_xyz) // 3
            for i in range(n):
                u = float(flat_xyz[3*i + 0])  # pixel x
                v = float(flat_xyz[3*i + 1])  # pixel y

                # clamp to image
                ui = int(np.clip(u, 0, depth_m.shape[1]-1))
                vi = int(np.clip(v, 0, depth_m.shape[0]-1))

                z = self._robust_depth(depth_m, vi, ui)  # meters
                if not np.isfinite(z) or z <= 0.0:
                    continue

                X = (u - cx) / fx * z
                Y = (v - cy) / fy * z

                t = TransformStamped()
                t.header.stamp = color_msg.header.stamp
                t.header.frame_id = self.camera_frame
                t.child_frame_id = f'{prefix}_{i}'
                t.transform.translation.x = float(X)
                t.transform.translation.y = float(Y)
                t.transform.translation.z = float(z)
                t.transform.rotation.x = 0.0
                t.transform.rotation.y = 0.0
                t.transform.rotation.z = 0.0
                t.transform.rotation.w = 1.0
                self.tf_broadcaster.sendTransform(t)

        if pose_lm:   broadcast_set(pose_lm,  'pose')
        if lhand_lm:  broadcast_set(lhand_lm,'left_hand')
        if rhand_lm:  broadcast_set(rhand_lm,'right_hand')

        # show ROI overlay / keys
        disp = annotated_image.copy()
        if self.roi_enabled and self.roi_width > 0 and self.roi_height > 0:
            cv2.rectangle(disp, (self.roi_x, self.roi_y),
                          (self.roi_x+self.roi_width, self.roi_y+self.roi_height),
                          (0,255,0), 2)
            cv2.putText(disp, 'ROI', (self.roi_x, self.roi_y-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
        if self.dragging and self.start_point and self.end_point:
            cv2.rectangle(disp, self.start_point, self.end_point, (255,0,0), 2)
            cv2.putText(disp, 'Selecting ROI...',
                        (self.start_point[0], self.start_point[1]-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,0,0), 2)
        cv2.putText(disp, 'Drag to select ROI', (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
        cv2.imshow('Top Camera - ROI Selection', disp)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            cv2.destroyAllWindows()
        elif key == ord('r'):
            self.roi_enabled = False
            self.set_parameters([Parameter('roi_enabled', Parameter.Type.BOOL, False)])
            self.get_logger().info('ROI reset')

    # ===== helpers =====
    def _publish_array(self, pub, flat_list):
        msg = Float32MultiArray()
        msg.data = flat_list
        pub.publish(msg)

    def _robust_depth(self, depth_m, v, u):
        """3x3近傍の中央値で欠損補完（m）"""
        h, w = depth_m.shape
        v0, v1 = max(0, v-1), min(h, v+2)
        u0, u1 = max(0, u-1), min(w, u+2)
        patch = depth_m[v0:v1, u0:u1].reshape(-1)
        vals = patch[np.isfinite(patch) & (patch > 0.0)]
        if vals.size == 0:
            return np.nan
        return float(np.median(vals))

    def process_image(self, cv_image):
        """MediaPipeをROI内で実行→2Dランドマーク抽出（フル画像座標）"""
        H, W = cv_image.shape[:2]
        if self.roi_enabled and self.roi_width > 0 and self.roi_height > 0:
            rx = int(np.clip(self.roi_x, 0, W-1))
            ry = int(np.clip(self.roi_y, 0, H-1))
            rx2 = int(np.clip(rx + self.roi_width, 0, W))
            ry2 = int(np.clip(ry + self.roi_height, 0, H))
            proc = cv_image[ry:ry2, rx:rx2]
            roi_offset = (rx, ry)
        else:
            proc = cv_image
            roi_offset = (0, 0)
            rx, ry, rx2, ry2 = 0, 0, W, H

        rgb = cv2.cvtColor(proc, cv2.COLOR_BGR2RGB)
        rgb.flags.writeable = False
        results = self.holistic.process(rgb)
        rgb.flags.writeable = True
        annotated = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

        if results.pose_landmarks:
            self.mp_drawing.draw_landmarks(
                annotated, results.pose_landmarks, self.mp_holistic.POSE_CONNECTIONS,
                landmark_drawing_spec=self.mp_drawing_styles.get_default_pose_landmarks_style())
        if results.left_hand_landmarks:
            self.mp_drawing.draw_landmarks(
                annotated, results.left_hand_landmarks, self.mp_holistic.HAND_CONNECTIONS,
                landmark_drawing_spec=self.mp_drawing_styles.get_default_hand_landmarks_style(),
                connection_drawing_spec=self.mp_drawing_styles.get_default_hand_connections_style())
        if results.right_hand_landmarks:
            self.mp_drawing.draw_landmarks(
                annotated, results.right_hand_landmarks, self.mp_holistic.HAND_CONNECTIONS,
                landmark_drawing_spec=self.mp_drawing_styles.get_default_hand_landmarks_style(),
                connection_drawing_spec=self.mp_drawing_styles.get_default_hand_connections_style())

        if self.roi_enabled and self.roi_width > 0 and self.roi_height > 0:
            full_annot = cv_image.copy()
            full_annot[ry:ry2, rx:rx2] = annotated
        else:
            full_annot = annotated

        pose_landmarks = self.extract_pose_landmarks(results, W, H, roi_offset, (rx, ry, rx2, ry2))
        left_hand_landmarks = self.extract_hand_landmarks(results.left_hand_landmarks, W, H, roi_offset, (rx, ry, rx2, ry2))
        right_hand_landmarks = self.extract_hand_landmarks(results.right_hand_landmarks, W, H, roi_offset, (rx, ry, rx2, ry2))

        roi_ctx = (self.roi_x, self.roi_y, self.roi_width, self.roi_height, self.roi_enabled)
        return full_annot, pose_landmarks, left_hand_landmarks, right_hand_landmarks, roi_ctx

    def extract_pose_landmarks(self, results, width, height, roi_offset, roi_bbox):
        out = []
        if results and results.pose_landmarks:
            W = roi_bbox[2] - roi_bbox[0]
            H = roi_bbox[3] - roi_bbox[1]
            for lm in results.pose_landmarks.landmark:
                x = lm.x * W + roi_offset[0]
                y = lm.y * H + roi_offset[1]
                z = lm.z
                out.extend([x, y, z])
        return out

    def extract_hand_landmarks(self, hand_lm, width, height, roi_offset, roi_bbox):
        out = []
        if hand_lm:
            W = roi_bbox[2] - roi_bbox[0]
            H = roi_bbox[3] - roi_bbox[1]
            for lm in hand_lm.landmark:
                x = lm.x * W + roi_offset[0]
                y = lm.y * H + roi_offset[1]
                z = lm.z
                out.extend([x, y, z])
        return out


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
