#!/usr/bin/env python3
import time, math
from collections import deque
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from std_msgs.msg import Int32, Float32, Float32MultiArray

# MediaPipe FaceMesh indices (inner lips + mouth corners)
IDX_LEFT_MOUTH  = 61
IDX_RIGHT_MOUTH = 291
IDX_UPPER_INNER = 13
IDX_LOWER_INNER = 14

NUM_LANDMARKS = 468
STRIDE = 3  # x,y,z

def get_xy(flat, idx):
    base = idx * STRIDE
    return float(flat[base]), float(flat[base + 1])

def safe_norm(a, b):
    return float(np.linalg.norm(np.array(a) - np.array(b))) + 1e-6

class ChewingFromLandmarksAdaptive(Node):
    def __init__(self):
        super().__init__('chewing_from_landmarks_adaptive')

        # ---------- Parameters ----------
        self.declare_parameter('landmarks_topic', '/front_camera/face_landmarks')
        self.declare_parameter('ema_alpha', 0.1)          # MARの指数平滑
        self.declare_parameter('use_adaptive', False)       # True: 自己適応しきい値 / False: 固定値
        # 固定しきい値（このログに合う推奨値）
        self.declare_parameter('mar_high', 0.008)
        self.declare_parameter('mar_low',  0.005)

        # 自己適応の窓とゲイン
        self.declare_parameter('window_sec', 3.0)          # 移動窓（秒）
        self.declare_parameter('fs_guess', 30.0)           # 想定FPS（しきい値の更新窓サイズ用）
        self.declare_parameter('k_high', 4.0)              # high = median + k_high*MAD
        self.declare_parameter('k_low',  2.0)              # low  = median + k_low *MAD
        self.declare_parameter('amp_min', 0.004)           # 最低振幅（low⇄high差がこれ未満なら捨てる）
        self.declare_parameter('ignore_amp_ge', 0.08)      # これ以上の巨大開口は咀嚼としてカウントしない（嚥下/あくび除外）
        self.declare_parameter('min_interval_sec', 0.30)   # 二重カウント防止
        self.declare_parameter('min_open_frames', 2)       # high超えが連続NフレでOPEN確定
        self.declare_parameter('min_close_frames', 1)      # low未満が連続NフレでCLOSE確定

        # ---------- Read params ----------
        self.topic            = self.get_parameter('landmarks_topic').value
        self.ema_alpha        = float(self.get_parameter('ema_alpha').value)
        self.use_adaptive     = bool(self.get_parameter('use_adaptive').value)
        self.mar_high_fixed   = float(self.get_parameter('mar_high').value)
        self.mar_low_fixed    = float(self.get_parameter('mar_low').value)
        self.window_sec       = float(self.get_parameter('window_sec').value)
        self.fs_guess         = float(self.get_parameter('fs_guess').value)
        self.k_high           = float(self.get_parameter('k_high').value)
        self.k_low            = float(self.get_parameter('k_low').value)
        self.amp_min          = float(self.get_parameter('amp_min').value)
        self.ignore_amp_ge    = float(self.get_parameter('ignore_amp_ge').value)
        self.min_interval     = float(self.get_parameter('min_interval_sec').value)
        self.min_open_frames  = int(self.get_parameter('min_open_frames').value)
        self.min_close_frames = int(self.get_parameter('min_close_frames').value)

        # ---------- ROS I/O ----------
        qos = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=5,
                         reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.sub = self.create_subscription(Float32MultiArray, self.topic, self.landmarks_cb, qos)
        self.pub_count = self.create_publisher(Int32,  '/chewing/count',  10)
        self.pub_mar   = self.create_publisher(Float32, '/chewing/mar',    10)
        self.pub_hi    = self.create_publisher(Float32, '/chewing/mar_high', 10)
        self.pub_lo    = self.create_publisher(Float32, '/chewing/mar_low',  10)

        # ---------- State ----------
        self.mar_ema = None
        self.hist = deque(maxlen=max(15, int(self.window_sec * self.fs_guess)))
        self.chew_count = 0
        self.last_chew_ts = 0.0

        # ヒステリシス状態
        self.state_open = False
        self.open_streak = 0
        self.close_streak = 0

        # サイクルの最大・最小（振幅評価用）
        self.cycle_max = -1e9
        self.cycle_min = +1e9

        self.get_logger().info(f'Chewing adaptive started. Subscribing: {self.topic}')

    # ---- MAR from landmarks ----
    def compute_mar_from_flat(self, flat):
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
            if math.isfinite(mar) and 0.0 < mar < 2.0:
                return float(mar)
        except Exception as e:
            self.get_logger().warn(f'MAR compute failed: {e}')
        return None

    # ---- Adaptive thresholds (median + MAD) ----
    def dyn_thresholds(self):
        if len(self.hist) < 15:
            # サンプル不足時は固定値/フォールバック
            return self.mar_high_fixed, self.mar_low_fixed
        arr = np.asarray(self.hist, dtype=np.float32)

        # Winsorize（しきい値計算用に極端値を軽く抑える：上位5%を95%点に丸める）
        p95 = float(np.percentile(arr, 95.0))
        arr_clip = np.minimum(arr, p95)

        med = float(np.median(arr_clip))
        mad = float(np.median(np.abs(arr_clip - med))) + 1e-9

        high = med + self.k_high * mad
        low  = med + self.k_low  * mad

        # 最小振幅確保
        if high - low < self.amp_min:
            pad = 0.5 * (self.amp_min - (high - low))
            high += pad; low -= pad

        return high, low

    def landmarks_cb(self, msg: Float32MultiArray):
        mar = self.compute_mar_from_flat(msg.data)
        if mar is None:
            return

        # EMA 平滑
        if self.mar_ema is None:
            self.mar_ema = mar
        else:
            self.mar_ema = self.ema_alpha * mar + (1.0 - self.ema_alpha) * self.mar_ema

        # 履歴にpush（しきい値算出用）
        self.hist.append(self.mar_ema)

        # しきい値
        if self.use_adaptive:
            high, low = self.dyn_thresholds()
        else:
            high, low = self.mar_high_fixed, self.mar_low_fixed

        # 可視化用Publish
        self.pub_mar.publish(Float32(data=float(self.mar_ema)))
        self.pub_hi.publish(Float32(data=float(high)))
        self.pub_lo.publish(Float32(data=float(low)))

        # --- Cycle logic with streaks & amplitude gate ---
        now = time.time()

        # OPEN 確定（high超えを連続min_open_frames）
        if self.mar_ema >= high:
            self.open_streak += 1
        else:
            self.open_streak = 0

        # CLOSE 確定（low未満を連続min_close_frames）
        if self.mar_ema <= low:
            self.close_streak += 1
        else:
            self.close_streak = 0

        if not self.state_open and self.open_streak >= self.min_open_frames:
            # サイクル開始
            self.state_open = True
            self.cycle_max = self.mar_ema
            self.cycle_min = self.mar_ema
        elif self.state_open:
            # サイクル中はmax/min更新
            self.cycle_max = max(self.cycle_max, self.mar_ema)
            self.cycle_min = min(self.cycle_min, self.mar_ema)

            if self.close_streak >= self.min_close_frames:
                # サイクル終了 → 振幅とリフラクトリで判定
                amp = self.cycle_max - self.cycle_min

                # 巨大開口（嚥下/あくび/誤検出）除外
                if self.cycle_max >= self.ignore_amp_ge:
                    pass  # カウントしない
                elif amp >= self.amp_min and (now - self.last_chew_ts) >= self.min_interval:
                    self.chew_count += 1
                    self.last_chew_ts = now
                    self.pub_count.publish(Int32(data=int(self.chew_count)))

                # リセット
                self.state_open = False
                self.open_streak = 0
                self.close_streak = 0
                self.cycle_max = -1e9
                self.cycle_min = +1e9


def main(args=None):
    rclpy.init(args=args)
    node = ChewingFromLandmarksAdaptive()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
