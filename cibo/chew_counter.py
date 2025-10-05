#!/usr/bin/env python3
import math
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from std_msgs.msg import Int32, Float32
from std_msgs.msg import Float32MultiArray
import cv2
import numpy as np

# MediaPipe FaceMesh の主な口唇インデックス
IDX_LEFT_MOUTH  = 61   # 左口角
IDX_RIGHT_MOUTH = 291  # 右口角
IDX_UPPER_INNER = 13   # 上内唇
IDX_LOWER_INNER = 14   # 下内唇

NUM_LANDMARKS = 468
STRIDE = 3  # x,y,z

def get_xy(flat, idx):
    """flat: [x1,y1,z1,x2,y2,z2,...] から idx番目の (x,y) を返す"""
    base = idx * STRIDE
    return float(flat[base]), float(flat[base + 1])

def safe_norm(a, b):
    return float(np.linalg.norm(np.array(a) - np.array(b))) + 1e-6

class ChewingFromLandmarksNode(Node):
    def __init__(self):
        super().__init__('chewing_from_landmarks')

        # -------- Parameters --------
        self.declare_parameter('landmarks_topic', '/front_camera/face_landmarks')
        self.declare_parameter('mar_high', 0.020)         # 開いた判定
        self.declare_parameter('mar_low',  0.012)         # 閉じた判定
        self.declare_parameter('ema_alpha', 0.25)         # MAR平滑化
        self.declare_parameter('min_interval_sec', 0.25) # 二重カウント防止
        self.declare_parameter('show_debug', True)      # デバッグテキスト表示

        self.topic         = self.get_parameter('landmarks_topic').value
        self.mar_high      = float(self.get_parameter('mar_high').value)
        self.mar_low       = float(self.get_parameter('mar_low').value)
        self.ema_alpha     = float(self.get_parameter('ema_alpha').value)
        self.min_interval  = float(self.get_parameter('min_interval_sec').value)
        self.show_debug    = bool(self.get_parameter('show_debug').value)

        # -------- ROS I/O --------
        qos = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=5,
                         reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.sub = self.create_subscription(Float32MultiArray, self.topic, self.landmarks_cb, qos)
        self.pub_count = self.create_publisher(Int32,  '/chewing/count', 10)
        self.pub_mar   = self.create_publisher(Float32, '/chewing/mar',   10)

        # -------- State --------
        self.chew_count = 0
        self.last_chew_ts = 0.0
        self.state_open = False
        self.mar_ema = None
        self.last_msg_ok = False

        # （任意）簡易デバッグウィンドウ
        self.timer = self.create_timer(0.03, self.debug_loop)

        self.get_logger().info(f'ChewingFromLandmarks started. Subscribing: {self.topic}')

    def compute_mar_from_flat(self, flat):
        # 入力検証
        if flat is None or len(flat) < NUM_LANDMARKS * STRIDE:
            return None

        try:
            left  = get_xy(flat, IDX_LEFT_MOUTH)
            right = get_xy(flat, IDX_RIGHT_MOUTH)
            upper = get_xy(flat, IDX_UPPER_INNER)
            lower = get_xy(flat, IDX_LOWER_INNER)

            horizontal = safe_norm(right, left)
            vertical   = safe_norm(upper, lower)
            mar = vertical / horizontal
            if math.isfinite(mar) and 0.0 < mar < 2.0:  # 適当なレンジチェック
                return float(mar)
        except Exception as e:
            self.get_logger().warn(f'MAR compute failed: {e}')
        return None

    def landmarks_cb(self, msg: Float32MultiArray):
        flat = msg.data
        mar = self.compute_mar_from_flat(flat)
        if mar is None:
            self.last_msg_ok = False
            return

        # EMAで平滑化
        if self.mar_ema is None:
            self.mar_ema = mar
        else:
            self.mar_ema = self.ema_alpha * mar + (1.0 - self.ema_alpha) * self.mar_ema

        self.pub_mar.publish(Float32(data=float(self.mar_ema)))

        # ヒステリシス＋最小間隔
        now = time.time()
        if not self.state_open and self.mar_ema >= self.mar_high:
            self.state_open = True
        if self.state_open and self.mar_ema <= self.mar_low:
            if now - self.last_chew_ts >= self.min_interval:
                self.chew_count += 1
                self.last_chew_ts = now
                self.pub_count.publish(Int32(data=int(self.chew_count)))
            self.state_open = False

        self.last_msg_ok = True

    def debug_loop(self):
        if not self.show_debug:
            return
        canvas = np.zeros((120, 400, 3), dtype=np.uint8)
        txt1 = f'CHEWS: {self.chew_count}'
        mar_txt = f'MAR: {self.mar_ema:.3f}' if self.mar_ema is not None else 'MAR: ---'
        ok_txt = 'LMK: OK' if self.last_msg_ok else 'LMK: ---'
        cv2.putText(canvas, txt1, (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0,255,0), 2, cv2.LINE_AA)
        cv2.putText(canvas, mar_txt, (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,255), 2, cv2.LINE_AA)
        cv2.putText(canvas, ok_txt, (280, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200,200,200), 1, cv2.LINE_AA)
        cv2.imshow('ChewingFromLandmarks (debug)', canvas)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('r'):
            self.chew_count = 0
            self.get_logger().info('Chew counter reset.')
        if key == ord('q'):
            rclpy.shutdown()

    def destroy_node(self):
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ChewingFromLandmarksNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
