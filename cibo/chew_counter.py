#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import time
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, UInt32, Empty, Float32

# MediaPipe Face Mesh landmark indices used
IDX_UPPER_INNER_LIP = 13
IDX_LOWER_INNER_LIP = 14
IDX_MOUTH_LEFT      = 61
IDX_MOUTH_RIGHT     = 291

NUM_LANDMARKS = 468
VALUES_PER_LMK = 3  # x, y, z


def _lmk_xy(flat: list, idx: int) -> Optional[Tuple[float, float]]:
    base = idx * VALUES_PER_LMK
    if base + 1 >= len(flat):
        return None
    return flat[base], flat[base + 1]


def _dist(p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
    return math.hypot(p1[0] - p2[0], p1[1] - p2[1])


class ChewCounterNode(Node):
    """
    Subscribes: /front_camera/face_landmarks (Float32MultiArray)
      - data length should be 468 * 3 = 1404 (x,y,z flattened)
      - x,y は送信側で画像座標にスケール済（本ノードは比で扱うためOK）

    Publishes:
      - /chew_count (UInt32): 累積カウント
      - /chew_event (Empty): 1咀嚼ごとに単発発行
      - /mouth_open_ratio (Float32): 可視化・デバッグ用
    """

    def __init__(self):
        super().__init__('chew_counter_node')

        # Parameters（必要に応じて ros2 param で上書き可）
        self.declare_parameter('topic_in', '/front_camera/face_landmarks')
        self.declare_parameter('open_threshold', 0.065)   # 開き判定（ratioがこれ以上）
        self.declare_parameter('close_threshold', 0.045)  # 閉じ判定（ratioがこれ未満）
        self.declare_parameter('min_open_time', 0.10)     # [s] 口を開いている最短時間
        self.declare_parameter('refractory_time', 0.30)   # [s] カウント後の再カウント待ち
        self.declare_parameter('min_mouth_width', 5.0)    # [px] 非常に小さい顔/誤検出の弾き

        topic_in = self.get_parameter('topic_in').get_parameter_value().string_value
        self.open_threshold = self.get_parameter('open_threshold').value
        self.close_threshold = self.get_parameter('close_threshold').value
        self.min_open_time = self.get_parameter('min_open_time').value
        self.refractory_time = self.get_parameter('refractory_time').value
        self.min_mouth_width = self.get_parameter('min_mouth_width').value

        # Subscribers / Publishers
        self.sub = self.create_subscription(
            Float32MultiArray, topic_in, self.cb_landmarks, 10
        )
        self.pub_count = self.create_publisher(UInt32, '/chew_count', 10)
        self.pub_event = self.create_publisher(Empty, '/chew_event', 10)
        self.pub_ratio = self.create_publisher(Float32, '/mouth_open_ratio', 10)

        # State
        self.chew_count = 0
        self.mouth_open = False
        self.last_open_ts: Optional[float] = None
        self.last_chew_ts: float = 0.0

        self.get_logger().info(
            f'ChewCounterNode started. Subscribing: {topic_in} | '
            f'open>= {self.open_threshold:.3f}, close< {self.close_threshold:.3f}, '
            f'min_open {self.min_open_time:.2f}s, refractory {self.refractory_time:.2f}s'
        )

    def cb_landmarks(self, msg: Float32MultiArray):
        data = msg.data
        if len(data) < NUM_LANDMARKS * VALUES_PER_LMK:
            # たまに顔未検出で空配列の可能性あり
            self._reset_open_state_if_needed()
            return

        p_up = _lmk_xy(data, IDX_UPPER_INNER_LIP)
        p_lo = _lmk_xy(data, IDX_LOWER_INNER_LIP)
        p_l  = _lmk_xy(data, IDX_MOUTH_LEFT)
        p_r  = _lmk_xy(data, IDX_MOUTH_RIGHT)

        if None in (p_up, p_lo, p_l, p_r):
            self._reset_open_state_if_needed()
            return

        mouth_height = _dist(p_up, p_lo)
        mouth_width  = _dist(p_l, p_r)

        # 幅が小さすぎる（顔が極小/ブレ/誤検出）ときはスキップ
        if mouth_width < self.min_mouth_width or mouth_width <= 0.0:
            self._reset_open_state_if_needed()
            return

        ratio = mouth_height / mouth_width

        # 可視化用に配信
        self.pub_ratio.publish(Float32(data=float(ratio)))

        now = time.monotonic()

        # 開→閉のヒステリシス判定
        if not self.mouth_open:
            # 「開いた」判定
            if ratio >= self.open_threshold:
                # 直前カウントからのリフラクトリを確認
                if now - self.last_chew_ts >= self.refractory_time:
                    self.mouth_open = True
                    self.last_open_ts = now
        else:
            # 「閉じた」判定
            if ratio < self.close_threshold:
                # 一定以上の開口継続時間でカウント
                if self.last_open_ts is not None and (now - self.last_open_ts) >= self.min_open_time:
                    self.chew_count += 1
                    self.last_chew_ts = now
                    self.pub_count.publish(UInt32(data=self.chew_count))
                    self.pub_event.publish(Empty())
                # 状態を閉口へ
                self.mouth_open = False
                self.last_open_ts = None

    def _reset_open_state_if_needed(self):
        # 顔が消えた/計算できないフレームは状態維持だが、
        # 長時間不在が続く場合に誤動作しにくいように軽く落とすならここで処理
        pass


def main():
    rclpy.init()
    node = ChewCounterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
