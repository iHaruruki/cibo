#!/usr/bin/env python3

import math
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult

from std_msgs.msg import Float32MultiArray, Int32, Float32, String


class ChewCounterNode(Node):
    def __init__(self):
        super().__init__('chew_counter')

        # ---- Parameters ----
        self.declare_parameter('face_landmarks_topic', '/front_camera/face_landmarks')
        self.declare_parameter('open_threshold', 0.060)       # 正規化値 (例: 0.05〜0.08 程度)
        self.declare_parameter('close_threshold', 0.040)      # ヒステリシス下限
        self.declare_parameter('smoothing_alpha', 0.25)       # 0.0~1.0
        self.declare_parameter('auto_calibrate', True)
        self.declare_parameter('calibration_frames', 300)
        self.declare_parameter('calib_open_sigma', 1.2)
        self.declare_parameter('calib_close_sigma', 0.6)
        self.declare_parameter('min_cycle_interval_sec', 0.35)
        self.declare_parameter('publish_debug', True)
        self.declare_parameter('chew_rate_window_sec', 60.0)
        self.declare_parameter('inactive_timeout_sec', 2.0)

        # Internal parameter cache
        self.params = {}
        self._load_params()

        # Dynamic parameter callback
        self.add_on_set_parameters_callback(self._on_param_change)

        # ---- State Variables ----
        self.chew_count = 0
        self.in_cycle = False           # 現在「開いている」→閉じるのを待っているフェーズか
        self.last_cycle_time = 0.0
        self.smoothed_opening = None    # 平滑後の正規化開き
        self.calibration_values = []    # 初期フレームの開き値蓄積
        self.calibrated = not self.params['auto_calibrate']
        self.calib_mean = 0.0
        self.calib_std = 0.0

        self.last_detection_time = 0.0  # 最終ランドマーク受信
        self.chew_timestamps = deque()  # 咀嚼完了時刻 (rate 計算用)

        # Face landmark indices (MediaPipe Face Mesh)
        self.idx_upper_lip = 13
        self.idx_lower_lip = 14
        self.idx_face_top = 10
        self.idx_face_bottom = 152

        # ---- Publishers ----
        self.pub_count = self.create_publisher(Int32, '/chew_counter/chew_count', 10)
        self.pub_rate = self.create_publisher(Float32, '/chew_counter/chew_rate', 10)
        self.pub_state = self.create_publisher(String, '/chew_counter/state', 10)
        if self.params['publish_debug']:
            self.pub_debug = self.create_publisher(Float32MultiArray, '/chew_counter/debug', 10)
        else:
            self.pub_debug = None

        # ---- Subscriber ----
        face_landmarks_topic = self.params['face_landmarks_topic']
        self.sub_landmarks = self.create_subscription(
            Float32MultiArray,
            face_landmarks_topic,
            self.face_landmarks_callback,
            10
        )

        # ---- Timers ----
        # 状態/レート更新 (1 Hz)
        self.timer_status = self.create_timer(1.0, self._publish_rate_and_state)

        self.get_logger().info('ChewCounterNode started. Subscribing: %s' % face_landmarks_topic)
        if self.params['auto_calibrate']:
            self.get_logger().info('Auto calibration enabled: collecting %d frames...' %
                                   self.params['calibration_frames'])
        else:
            self.get_logger().info('Manual thresholds: open=%.4f close=%.4f' %
                                   (self.params['open_threshold'], self.params['close_threshold']))

    # ---------------- Parameter Handling ----------------
    def _load_params(self):
        self.params['face_landmarks_topic'] = self.get_parameter('face_landmarks_topic').get_parameter_value().string_value
        self.params['open_threshold'] = self.get_parameter('open_threshold').get_parameter_value().double_value
        self.params['close_threshold'] = self.get_parameter('close_threshold').get_parameter_value().double_value
        self.params['smoothing_alpha'] = self.get_parameter('smoothing_alpha').get_parameter_value().double_value
        self.params['auto_calibrate'] = self.get_parameter('auto_calibrate').get_parameter_value().bool_value
        self.params['calibration_frames'] = self.get_parameter('calibration_frames').get_parameter_value().integer_value
        self.params['calib_open_sigma'] = self.get_parameter('calib_open_sigma').get_parameter_value().double_value
        self.params['calib_close_sigma'] = self.get_parameter('calib_close_sigma').get_parameter_value().double_value
        self.params['min_cycle_interval_sec'] = self.get_parameter('min_cycle_interval_sec').get_parameter_value().double_value
        self.params['publish_debug'] = self.get_parameter('publish_debug').get_parameter_value().bool_value
        self.params['chew_rate_window_sec'] = self.get_parameter('chew_rate_window_sec').get_parameter_value().double_value
        self.params['inactive_timeout_sec'] = self.get_parameter('inactive_timeout_sec').get_parameter_value().double_value

    def _on_param_change(self, changes):
        for p in changes:
            if p.name in self.params:
                self.params[p.name] = p.value
                if p.name in ('open_threshold', 'close_threshold'):
                    self.get_logger().info(f'Updated threshold param {p.name}={p.value}')
                if p.name == 'publish_debug':
                    if p.value and self.pub_debug is None:
                        self.pub_debug = self.create_publisher(Float32MultiArray, '/chew_counter/debug', 10)
                    elif not p.value and self.pub_debug is not None:
                        self.pub_debug = None
        return SetParametersResult(successful=True)

    # ---------------- Core Processing ----------------
    def face_landmarks_callback(self, msg: Float32MultiArray):
        """
        face_landmarks Float32MultiArray:
          data = [x0, y0, z0, x1, y1, z1, ...] (ピクセル座標 or 既存ノード仕様による)
          本ノードは x,y をピクセル座標として扱う想定。
        """
        now = self.get_clock().now().nanoseconds / 1e9
        self.last_detection_time = now

        data = msg.data
        n = len(data) // 3
        required_indices = [self.idx_upper_lip, self.idx_lower_lip, self.idx_face_top, self.idx_face_bottom]
        if any(idx >= n for idx in required_indices):
            # 顔が未検出 or 部分的
            return

        def get_point(i):
            base = i * 3
            return data[base], data[base + 1]  # (x,y)

        x_up, y_up = get_point(self.idx_upper_lip)
        x_low, y_low = get_point(self.idx_lower_lip)
        x_top, y_top = get_point(self.idx_face_top)
        x_bot, y_bot = get_point(self.idx_face_bottom)

        mouth_open_px = math.dist((x_up, y_up), (x_low, y_low))
        face_height_px = math.dist((x_top, y_top), (x_bot, y_bot))

        if face_height_px < 1e-6:
            return

        norm_open = mouth_open_px / face_height_px

        # Calibration
        if not self.calibrated and self.params['auto_calibrate']:
            self.calibration_values.append(norm_open)
            remaining = self.params['calibration_frames'] - len(self.calibration_values)
            if remaining % 30 == 0 and remaining > 0:
                self.get_logger().info(f'Calibrating... remaining frames: {remaining}')
            if len(self.calibration_values) >= self.params['calibration_frames']:
                self._finalize_calibration()
        # Smoothing
        alpha = max(0.0, min(1.0, self.params['smoothing_alpha']))
        if self.smoothed_opening is None:
            self.smoothed_opening = norm_open
        else:
            self.smoothed_opening = alpha * norm_open + (1 - alpha) * self.smoothed_opening

        open_th, close_th = self._current_thresholds()

        # Chew cycle detection with hysteresis
        chew_completed = False
        if not self.in_cycle:
            # look for opening
            if self.smoothed_opening >= open_th:
                self.in_cycle = True
        else:
            # waiting to close
            if self.smoothed_opening <= close_th:
                # check min interval
                if now - self.last_cycle_time >= self.params['min_cycle_interval_sec']:
                    self.chew_count += 1
                    self.last_cycle_time = now
                    self.chew_timestamps.append(now)
                    chew_completed = True
                self.in_cycle = False

        # Trim chew timestamps older than rate window
        window = self.params['chew_rate_window_sec']
        cutoff = now - window
        while self.chew_timestamps and self.chew_timestamps[0] < cutoff:
            self.chew_timestamps.popleft()

        # Publish core outputs (count increment event)
        if chew_completed:
            self._publish_count()

        # Debug publish each frame
        if self.pub_debug:
            dbg = Float32MultiArray()
            dbg.data = [
                float(mouth_open_px),
                float(face_height_px),
                float(norm_open),
                float(self.smoothed_opening),
                float(open_th),
                float(close_th),
                float(self.calib_mean),
                float(self.calib_std),
                1.0 if self.in_cycle else 0.0
            ]
            self.pub_debug.publish(dbg)

    def _finalize_calibration(self):
        import statistics
        if len(self.calibration_values) < 2:
            self.calib_mean = self.calibration_values[0] if self.calibration_values else 0.0
            self.calib_std = 0.0
        else:
            self.calib_mean = statistics.mean(self.calibration_values)
            self.calib_std = statistics.pstdev(self.calibration_values)
        # derive thresholds
        self.params['open_threshold'] = self.calib_mean + self.params['calib_open_sigma'] * self.calib_std
        self.params['close_threshold'] = self.calib_mean + self.params['calib_close_sigma'] * self.calib_std
        # Ensure hysteresis
        if self.params['close_threshold'] >= self.params['open_threshold']:
            self.params['close_threshold'] = self.params['open_threshold'] * 0.75
        self.calibrated = True
        self.get_logger().info(
            f'Calibration done: mean={self.calib_mean:.5f} std={self.calib_std:.5f} '
            f'-> open={self.params["open_threshold"]:.5f} close={self.params["close_threshold"]:.5f}'
        )

    def _current_thresholds(self):
        return self.params['open_threshold'], self.params['close_threshold']

    # ---------------- Publishers ----------------
    def _publish_count(self):
        msg = Int32()
        msg.data = self.chew_count
        self.pub_count.publish(msg)

    def _publish_rate_and_state(self):
        now = self.get_clock().now().nanoseconds / 1e9

        # State
        if not self.calibrated and self.params['auto_calibrate']:
            state = 'calibrating'
        else:
            if now - self.last_detection_time > self.params['inactive_timeout_sec']:
                state = 'inactive'
            else:
                state = 'active'
        state_msg = String()
        state_msg.data = state
        self.pub_state.publish(state_msg)

        # Chew rate (chews / minute)
        window = self.params['chew_rate_window_sec']
        # Timestamps deque already trimmed in callback
        if window > 0:
            rate_per_min = (len(self.chew_timestamps) / window) * 60.0
        else:
            rate_per_min = 0.0
        rate_msg = Float32()
        rate_msg.data = float(rate_per_min)
        self.pub_rate.publish(rate_msg)

        # Also publish cumulative count periodically (in case no new chews)
        self._publish_count()

    # ---------------- Shutdown ----------------
    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
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