#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import numpy as np
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
from scipy.spatial.transform import Rotation as R
import json
import os
import threading


class CameraCalibrationROS2(Node):
    def __init__(self):
        super().__init__('camera_calibration_node')
        
        # CV Bridge初期化
        self.bridge = CvBridge()
        
        # TF関連初期化
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # カメラパラメータ（AstraStereo相当）
        self.camera_params = {
            'width': 640,
            'height': 480,  # 一般的な高さに変更
            'thh': np.radians(33.95),  # 水平視野角
            'tvh': np.radians(22.65),  # 垂直視野角
            'tdh': np.radians(39.0),   # 対角視野角
            'hh': 240  # 高さの半分
        }
        
        # チェスボードパラメータ
        self.chessboard_size = (8, 6)  # 内部コーナー数 (横, 縦)
        self.square_size = 25.0  # チェスボードの1マスのサイズ (mm)
        
        # チェスボード検出用パラメータ
        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        
        # 3D座標生成（チェスボードパターン）
        self.objp = np.zeros((self.chessboard_size[0] * self.chessboard_size[1], 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:self.chessboard_size[0], 0:self.chessboard_size[1]].T.reshape(-1, 2)
        self.objp *= self.square_size
        
        # 検出された点を保存
        self.camera1_object_points = []  # 3D点
        self.camera1_image_points = []   # 2D点
        self.camera2_object_points = []  # 3D点
        self.camera2_image_points = []   # 2D点
        
        # 現在の検出状態
        self.current_corners_cam1 = None
        self.current_corners_cam2 = None
        self.corners_detected_cam1 = False
        self.corners_detected_cam2 = False
        
        # 個別サブスクライバー（メッセージフィルターを使わない）
        self.camera1_rgb_sub = self.create_subscription(
            Image, '/camera_01/color/image_raw', self.camera1_rgb_callback, 10)
        self.camera1_depth_sub = self.create_subscription(
            Image, '/camera_01/depth/image_raw', self.camera1_depth_callback, 10)
        self.camera2_rgb_sub = self.create_subscription(
            Image, '/camera_02/color/image_raw', self.camera2_rgb_callback, 10)
        self.camera2_depth_sub = self.create_subscription(
            Image, '/camera_02/depth/image_raw', self.camera2_depth_callback, 10)
        
        # パブリッシャー
        self.calibration_status_pub = self.create_publisher(Bool, '/calibration/status', 10)
        
        # 画像データ保存用
        self.current_rgb1 = None
        self.current_depth1 = None
        self.current_rgb2 = None
        self.current_depth2 = None
        
        # 画像受信フラグ
        self.rgb1_received = False
        self.depth1_received = False
        self.rgb2_received = False
        self.depth2_received = False
        
        # キャリブレーション結果
        self.transformation_matrix = None
        self.camera_matrix1 = None
        self.camera_matrix2 = None
        self.dist_coeffs1 = None
        self.dist_coeffs2 = None
        self.rotation_matrix = None
        self.translation_vector = None
        self.calibration_completed = False
        
        # カメラ間距離
        self.camera_distance = 0.0
        
        # OpenCVウィンドウの初期化
        self.setup_opencv_windows()
        
        # 表示更新タイマー（30Hz）
        self.display_timer = self.create_timer(1.0/30.0, self.update_display)
        
        # キー入力処理用タイマー（60Hz）
        self.key_timer = self.create_timer(1.0/60.0, self.process_key_input)
        
        # チェスボード検出タイマー（10Hz）
        self.detection_timer = self.create_timer(1.0/10.0, self.detect_chessboard)
        
        # TF配信タイマー（10Hz）- キャリブレーション完了後に開始
        self.tf_timer = None
        
        # 起動時に保存されたキャリブレーション結果を読み込み
        self.load_calibration()
        
        self.get_logger().info('チェスボードキャリブレーションノードが初期化されました')
        self.get_logger().info('使用方法：')
        self.get_logger().info(f'- チェスボード({self.chessboard_size[0]}x{self.chessboard_size[1]})を両カメラに表示')
        self.get_logger().info('- スペースキー：現在の検出結果を保存')
        self.get_logger().info('- cキー：キャリブレーション実行')
        self.get_logger().info('- rキー：リセット')
        self.get_logger().info('- sキー：結果保存')
        self.get_logger().info('- lキー：保存済み結果読み込み')
        self.get_logger().info('- qキー：終了')

    def setup_opencv_windows(self):
        """OpenCVウィンドウの初期設定"""
        # ウィンドウを作成して位置を設定
        cv2.namedWindow('Camera1_RGB', cv2.WINDOW_NORMAL)
        cv2.namedWindow('Camera1_Depth', cv2.WINDOW_NORMAL)
        cv2.namedWindow('Camera2_RGB', cv2.WINDOW_NORMAL)
        cv2.namedWindow('Camera2_Depth', cv2.WINDOW_NORMAL)
        
        # ウィンドウサイズ設定
        cv2.resizeWindow('Camera1_RGB', 640, 480)
        cv2.resizeWindow('Camera1_Depth', 640, 480)
        cv2.resizeWindow('Camera2_RGB', 640, 480)
        cv2.resizeWindow('Camera2_Depth', 640, 480)
        
        # ウィンドウの位置を設定
        cv2.moveWindow('Camera1_RGB', 0, 0)
        cv2.moveWindow('Camera1_Depth', 650, 0)
        cv2.moveWindow('Camera2_RGB', 0, 520)
        cv2.moveWindow('Camera2_Depth', 650, 520)
        
        # 初期画像表示
        self.show_waiting_message()

    def show_waiting_message(self):
        """待機中メッセージを表示"""
        black_image = np.zeros((480, 640, 3), dtype=np.uint8)
        texts = [
            "Waiting for camera data...",
            f"Camera1 RGB: {'OK' if self.rgb1_received else 'Waiting'}",
            f"Camera1 Depth: {'OK' if self.depth1_received else 'Waiting'}",
            f"Camera2 RGB: {'OK' if self.rgb2_received else 'Waiting'}",
            f"Camera2 Depth: {'OK' if self.depth2_received else 'Waiting'}",
            "",
            f"Chessboard: {self.chessboard_size[0]}x{self.chessboard_size[1]} corners",
            f"Square size: {self.square_size}mm",
            "",
            "Controls:",
            "SPACE = Capture, C = Calibrate, R = Reset, S = Save, Q = Quit",
            "",
            f"Captured pairs: {len(self.camera1_image_points)}",
            f"Cam1 detected: {'YES' if self.corners_detected_cam1 else 'NO'}",
            f"Cam2 detected: {'YES' if self.corners_detected_cam2 else 'NO'}",
            f"Calibrated: {'YES' if self.calibration_completed else 'NO'}",
            f"Camera distance: {self.camera_distance:.1f}mm" if self.calibration_completed else ""
        ]
        
        for i, text in enumerate(texts):
            if text:
                cv2.putText(black_image, text, (10, 30 + i * 25), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        cv2.imshow('Camera1_RGB', black_image.copy())
        cv2.imshow('Camera2_RGB', black_image.copy())
        cv2.imshow('Camera1_Depth', np.zeros((480, 640), dtype=np.uint8))
        cv2.imshow('Camera2_Depth', np.zeros((480, 640), dtype=np.uint8))

    def camera1_rgb_callback(self, msg):
        """Camera1 RGB画像コールバック"""
        try:
            self.current_rgb1 = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.rgb1_received = True
            self.get_logger().debug('Camera1 RGB受信', once=True)
        except Exception as e:
            self.get_logger().error(f'Camera1 RGB変換エラー: {str(e)}')

    def camera1_depth_callback(self, msg):
        """Camera1 Depth画像コールバック"""
        try:
            self.current_depth1 = self.bridge.imgmsg_to_cv2(msg, "16UC1")
            self.depth1_received = True
            self.get_logger().debug('Camera1 Depth受信', once=True)
        except Exception as e:
            self.get_logger().error(f'Camera1 Depth変換エラー: {str(e)}')

    def camera2_rgb_callback(self, msg):
        """Camera2 RGB画像コールバック"""
        try:
            self.current_rgb2 = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.rgb2_received = True
            self.get_logger().debug('Camera2 RGB受信', once=True)
        except Exception as e:
            self.get_logger().error(f'Camera2 RGB変換エラー: {str(e)}')

    def camera2_depth_callback(self, msg):
        """Camera2 Depth画像コールバック"""
        try:
            self.current_depth2 = self.bridge.imgmsg_to_cv2(msg, "16UC1")
            self.depth2_received = True
            self.get_logger().debug('Camera2 Depth受信', once=True)
        except Exception as e:
            self.get_logger().error(f'Camera2 Depth変換エラー: {str(e)}')

    def process_key_input(self):
        """キー入力処理"""
        try:
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord(' '):  # スペースキー
                self.get_logger().info('スペースキー: チェスボード検出結果を保存')
                self.capture_chessboard()
            elif key == ord('c'):
                self.get_logger().info('Cキー: キャリブレーション開始')
                self.calibrate_cameras()
            elif key == ord('r'):
                self.get_logger().info('Rキー: リセット')
                self.reset_calibration()
            elif key == ord('s'):
                self.get_logger().info('Sキー: 保存')
                self.save_calibration()
            elif key == ord('l'):
                self.get_logger().info('Lキー: 保存済み結果読み込み')
                self.load_calibration()
            elif key == ord('q'):
                self.get_logger().info('Qキー: 終了')
                rclpy.shutdown()
            elif key == 27:  # ESCキー
                self.get_logger().info('ESCキー: 終了')
                rclpy.shutdown()
                
        except Exception as e:
            self.get_logger().error(f'キー入力処理エラー: {str(e)}')

    def update_display(self):
        """表示更新"""
        try:
            # 全ての画像が受信されていない場合は待機メッセージを表示
            if not all([self.rgb1_received, self.depth1_received, 
                       self.rgb2_received, self.depth2_received]):
                self.show_waiting_message()
                return
            
            # 画像が None でないかチェック
            if (self.current_rgb1 is None or self.current_depth1 is None or 
                self.current_rgb2 is None or self.current_depth2 is None):
                return
            
            # 深度画像の可視化
            depth1_vis = self.create_depth_visualization(self.current_depth1)
            depth2_vis = self.create_depth_visualization(self.current_depth2)
            
            # RGB画像のコピーを作成
            rgb1_display = self.current_rgb1.copy()
            rgb2_display = self.current_rgb2.copy()
            
            # チェスボードコーナーを描画
            self.draw_chessboard_corners(rgb1_display, self.current_corners_cam1, self.corners_detected_cam1)
            self.draw_chessboard_corners(rgb2_display, self.current_corners_cam2, self.corners_detected_cam2)
            
            # ステータス情報をオーバーレイ
            self.draw_status_overlay(rgb1_display, "Camera 1")
            self.draw_status_overlay(rgb2_display, "Camera 2")
            
            # 画像表示
            cv2.imshow('Camera1_RGB', rgb1_display)
            cv2.imshow('Camera1_Depth', depth1_vis)
            cv2.imshow('Camera2_RGB', rgb2_display)
            cv2.imshow('Camera2_Depth', depth2_vis)
            
        except Exception as e:
            self.get_logger().error(f'表示更新エラー: {str(e)}')

    def draw_chessboard_corners(self, image, corners, detected):
        """チェスボードコーナーを描画"""
        if detected and corners is not None:
            # コーナーを描画
            cv2.drawChessboardCorners(image, self.chessboard_size, corners, detected)
            
            # 検出成功を示す枠を描画
            cv2.rectangle(image, (5, 5), (image.shape[1]-5, image.shape[0]-5), (0, 255, 0), 3)
        else:
            # 検出失敗を示す枠を描画
            cv2.rectangle(image, (5, 5), (image.shape[1]-5, image.shape[0]-5), (0, 0, 255), 3)

    def draw_status_overlay(self, image, camera_name):
        """ステータス情報をオーバーレイ"""
        captured_count = len(self.camera1_image_points)
        
        if camera_name == "Camera 1":
            detected = self.corners_detected_cam1
        else:
            detected = self.corners_detected_cam2
        
        status_texts = [
            f"{camera_name}: {'DETECTED' if detected else 'NO PATTERN'}",
            f"Captured: {captured_count}/10+ pairs",
            f"Calibrated: {'YES' if self.calibration_completed else 'NO'}",
            f"Distance: {self.camera_distance:.1f}mm" if self.calibration_completed else "",
            "SPACE=Capture C=Calibrate R=Reset S=Save L=Load Q=Quit"
        ]
        
        # 背景を描画
        overlay = image.copy()
        cv2.rectangle(overlay, (5, 5), (500, 80), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.7, image, 0.3, 0, image)
        
        # テキストを描画
        for i, text in enumerate(status_texts):
            color = (0, 255, 0) if detected and i == 0 else (255, 255, 255)
            cv2.putText(image, text, (10, 25 + i * 20), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

    def detect_chessboard(self):
        """チェスボード検出"""
        try:
            # 両方のRGB画像が利用可能かチェック
            if self.current_rgb1 is None or self.current_rgb2 is None:
                return
            
            # Camera1でチェスボード検出
            gray1 = cv2.cvtColor(self.current_rgb1, cv2.COLOR_BGR2GRAY)
            ret1, corners1 = cv2.findChessboardCorners(gray1, self.chessboard_size, None)
            
            if ret1:
                # サブピクセル精度でコーナーを改良
                corners1 = cv2.cornerSubPix(gray1, corners1, (11, 11), (-1, -1), self.criteria)
                self.current_corners_cam1 = corners1
                self.corners_detected_cam1 = True
            else:
                self.current_corners_cam1 = None
                self.corners_detected_cam1 = False
            
            # Camera2でチェスボード検出
            gray2 = cv2.cvtColor(self.current_rgb2, cv2.COLOR_BGR2GRAY)
            ret2, corners2 = cv2.findChessboardCorners(gray2, self.chessboard_size, None)
            
            if ret2:
                # サブピクセル精度でコーナーを改良
                corners2 = cv2.cornerSubPix(gray2, corners2, (11, 11), (-1, -1), self.criteria)
                self.current_corners_cam2 = corners2
                self.corners_detected_cam2 = True
            else:
                self.current_corners_cam2 = None
                self.corners_detected_cam2 = False
                
        except Exception as e:
            self.get_logger().error(f'チェスボード検出エラー: {str(e)}')

    def capture_chessboard(self):
        """現在のチェスボード検出結果を保存"""
        if not self.corners_detected_cam1 or not self.corners_detected_cam2:
            self.get_logger().warn('両方のカメラでチェスボードが検出されていません')
            return
        
        # 3D点を追加
        self.camera1_object_points.append(self.objp)
        self.camera2_object_points.append(self.objp)
        
        # 2D点を追加
        self.camera1_image_points.append(self.current_corners_cam1)
        self.camera2_image_points.append(self.current_corners_cam2)
        
        self.get_logger().info(f'チェスボードペア {len(self.camera1_image_points)} を保存しました')

    def calibrate_cameras(self):
        """キャリブレーション実行"""
        if len(self.camera1_image_points) < 5:
            self.get_logger().warn('キャリブレーションには最低5つのチェスボードペアが必要です')
            return
        
        try:
            # カメラ内部パラメータの初期推定
            image_size = (640, 480)
            
            self.get_logger().info('キャリブレーション開始...')
            
            # Camera1のキャリブレーション
            ret1, mtx1, dist1, rvecs1, tvecs1 = cv2.calibrateCamera(
                self.camera1_object_points, self.camera1_image_points, 
                image_size, None, None,
                flags=cv2.CALIB_RATIONAL_MODEL)
            
            # Camera2のキャリブレーション
            ret2, mtx2, dist2, rvecs2, tvecs2 = cv2.calibrateCamera(
                self.camera2_object_points, self.camera2_image_points, 
                image_size, None, None,
                flags=cv2.CALIB_RATIONAL_MODEL)
            
            self.get_logger().info(f'Camera1 キャリブレーション誤差: {ret1:.4f}')
            self.get_logger().info(f'Camera2 キャリブレーション誤差: {ret2:.4f}')
            
            # ステレオキャリブレーション（より高精度なフラグを使用）
            ret_stereo, mtx1, dist1, mtx2, dist2, R, T, E, F = cv2.stereoCalibrate(
                self.camera1_object_points, self.camera1_image_points, self.camera2_image_points,
                mtx1, dist1, mtx2, dist2, image_size, 
                criteria=self.criteria, 
                flags=cv2.CALIB_FIX_INTRINSIC + cv2.CALIB_RATIONAL_MODEL)
            
            self.get_logger().info(f'ステレオキャリブレーション誤差: {ret_stereo:.4f}')
            
            # 結果を保存
            self.camera_matrix1 = mtx1
            self.camera_matrix2 = mtx2
            self.dist_coeffs1 = dist1
            self.dist_coeffs2 = dist2
            self.rotation_matrix = R
            self.translation_vector = T
            
            # 変換行列を作成（4x4）
            self.transformation_matrix = np.eye(4)
            self.transformation_matrix[:3, :3] = R
            self.transformation_matrix[:3, 3] = T.flatten()
            
            # カメラ間距離を計算（ミリメートル単位）
            self.camera_distance = np.linalg.norm(T) * 1000  # mからmmに変換
            
            # キャリブレーション完了フラグ
            self.calibration_completed = True
            
            self.get_logger().info('キャリブレーション完了！')
            self.get_logger().info(f'回転行列 R:\n{R}')
            self.get_logger().info(f'平行移動ベクトル T: {T.flatten()}')
            self.get_logger().info(f'カメラ間距離: {self.camera_distance:.1f}mm')
            
            # 継続的なTF配信を開始
            if self.tf_timer is None:
                self.tf_timer = self.create_timer(1.0/10.0, self.publish_transform_continuous)
            
            # 初回TF配信
            self.publish_transform()
            
            # ステータス配信
            status_msg = Bool()
            status_msg.data = True
            self.calibration_status_pub.publish(status_msg)
            
        except Exception as e:
            self.get_logger().error(f'キャリブレーションエラー: {str(e)}')

    def create_depth_visualization(self, depth_image):
        """深度画像の可視化"""
        if depth_image is None:
            return np.zeros((480, 640), dtype=np.uint8)
        
        threshold_up = 3000  # mm
        threshold_low = 300  # mm
        
        # 深度値を0-255の範囲に正規化
        depth_vis = np.zeros_like(depth_image, dtype=np.uint8)
        
        # 有効範囲内の深度値を正規化
        valid_mask = (depth_image >= threshold_low) & (depth_image <= threshold_up)
        if np.any(valid_mask):
            depth_vis[valid_mask] = 255 - 255 * (depth_image[valid_mask] - threshold_low) / (threshold_up - threshold_low)
        
        return depth_vis

    def compute_transformation_matrix(self, points_a, points_b):
        """変換行列計算（SVDを使用）- 削除予定"""
        # この関数は現在使用されていません
        pass

    def reset_calibration(self):
        """キャリブレーションデータをリセット"""
        self.camera1_object_points.clear()
        self.camera1_image_points.clear()
        self.camera2_object_points.clear()
        self.camera2_image_points.clear()
        
        self.transformation_matrix = None
        self.camera_matrix1 = None
        self.camera_matrix2 = None
        self.dist_coeffs1 = None
        self.dist_coeffs2 = None
        self.rotation_matrix = None
        self.translation_vector = None
        self.calibration_completed = False
        self.camera_distance = 0.0
        
        self.current_corners_cam1 = None
        self.current_corners_cam2 = None
        self.corners_detected_cam1 = False
        self.corners_detected_cam2 = False
        
        # TF配信を停止
        if self.tf_timer is not None:
            self.tf_timer.destroy()
            self.tf_timer = None
        
        self.get_logger().info('キャリブレーションデータがリセットされました')

    def verify_calibration(self):
        """キャリブレーション精度確認 - 削除予定"""
        # この関数は現在使用されていません
        pass

    def publish_transform(self):
        """TFとして変換を配信"""
        if not self.calibration_completed or self.transformation_matrix is None:
            return
        
        # 回転行列から四元数に変換
        rotation_matrix = self.transformation_matrix[:3, :3]
        r = R.from_matrix(rotation_matrix)
        quaternion = r.as_quat()  # [x, y, z, w]
        
        # 平行移動ベクトル
        translation = self.transformation_matrix[:3, 3]
        
        # TransformStamped作成
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'camera_01_link'
        transform.child_frame_id = 'camera_02_link'
        
        # mmをmに変換（チェスボードの場合は既にm単位）
        transform.transform.translation.x = float(translation[0])
        transform.transform.translation.y = float(translation[1])
        transform.transform.translation.z = float(translation[2])
        
        transform.transform.rotation.x = float(quaternion[0])
        transform.transform.rotation.y = float(quaternion[1])
        transform.transform.rotation.z = float(quaternion[2])
        transform.transform.rotation.w = float(quaternion[3])
        
        # TF配信
        self.tf_broadcaster.sendTransform(transform)
        
    def publish_transform_continuous(self):
        """継続的なTF配信"""
        self.publish_transform()

    def reset_points(self):
        """古い関数 - reset_calibrationに置き換え"""
        self.reset_calibration()

    def save_calibration(self):
        """キャリブレーション結果をファイルに保存"""
        if not self.calibration_completed or self.transformation_matrix is None:
            self.get_logger().warn('保存するキャリブレーション結果がありません')
            return
        
        # ホームディレクトリに保存
        home_dir = os.path.expanduser('~')
        filename = os.path.join(home_dir, 'chessboard_calibration_result.json')
        
        calibration_data = {
            'transformation_matrix': self.transformation_matrix.tolist(),
            'camera_matrix1': self.camera_matrix1.tolist() if self.camera_matrix1 is not None else None,
            'camera_matrix2': self.camera_matrix2.tolist() if self.camera_matrix2 is not None else None,
            'dist_coeffs1': self.dist_coeffs1.tolist() if self.dist_coeffs1 is not None else None,
            'dist_coeffs2': self.dist_coeffs2.tolist() if self.dist_coeffs2 is not None else None,
            'rotation_matrix': self.rotation_matrix.tolist() if self.rotation_matrix is not None else None,
            'translation_vector': self.translation_vector.tolist() if self.translation_vector is not None else None,
            'camera_distance': self.camera_distance,
            'chessboard_size': self.chessboard_size,
            'square_size': self.square_size,
            'num_captured_pairs': len(self.camera1_image_points),
            'calibration_completed': self.calibration_completed,
            'timestamp': self.get_clock().now().to_msg().sec
        }
        
        try:
            with open(filename, 'w') as f:
                json.dump(calibration_data, f, indent=2)
            
            self.get_logger().info(f'チェスボードキャリブレーション結果を保存しました: {filename}')
            self.get_logger().info(f'カメラ間距離: {self.camera_distance:.1f}mm')
            
        except Exception as e:
            self.get_logger().error(f'保存エラー: {str(e)}')

    def load_calibration(self):
        """保存されたキャリブレーション結果を読み込み"""
        home_dir = os.path.expanduser('~')
        filename = os.path.join(home_dir, 'chessboard_calibration_result.json')
        
        if not os.path.exists(filename):
            self.get_logger().info('保存されたキャリブレーション結果が見つかりません')
            return
        
        try:
            with open(filename, 'r') as f:
                calibration_data = json.load(f)
            
            # データを復元
            self.transformation_matrix = np.array(calibration_data['transformation_matrix'])
            
            if calibration_data.get('camera_matrix1'):
                self.camera_matrix1 = np.array(calibration_data['camera_matrix1'])
            if calibration_data.get('camera_matrix2'):
                self.camera_matrix2 = np.array(calibration_data['camera_matrix2'])
            if calibration_data.get('dist_coeffs1'):
                self.dist_coeffs1 = np.array(calibration_data['dist_coeffs1'])
            if calibration_data.get('dist_coeffs2'):
                self.dist_coeffs2 = np.array(calibration_data['dist_coeffs2'])
            if calibration_data.get('rotation_matrix'):
                self.rotation_matrix = np.array(calibration_data['rotation_matrix'])
            if calibration_data.get('translation_vector'):
                self.translation_vector = np.array(calibration_data['translation_vector'])
            
            self.camera_distance = calibration_data.get('camera_distance', 0.0)
            self.calibration_completed = calibration_data.get('calibration_completed', False)
            
            if self.calibration_completed:
                # 継続的なTF配信を開始
                if self.tf_timer is None:
                    self.tf_timer = self.create_timer(1.0/10.0, self.publish_transform_continuous)
                
                self.get_logger().info('保存されたキャリブレーション結果を読み込みました')
                self.get_logger().info(f'カメラ間距離: {self.camera_distance:.1f}mm')
                self.get_logger().info('TF配信を開始しました')
            
        except Exception as e:
            self.get_logger().error(f'読み込みエラー: {str(e)}')

    def destroy_node(self):
        """ノード終了時の処理"""
        # TF配信を停止
        if self.tf_timer is not None:
            self.tf_timer.destroy()
        
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    node = CameraCalibrationROS2()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()