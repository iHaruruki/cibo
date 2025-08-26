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
        
        # クリック位置保存用
        self.clicked_points_cam1_2d = []
        self.clicked_points_cam1_3d = []
        self.clicked_points_cam2_2d = []
        self.clicked_points_cam2_3d = []
        
        # マーカー座標（実世界座標系、mm単位）
        self.marker_positions_front = [
            np.array([30, 180, 0]),
            np.array([180, 180, 0]),
            np.array([30, 30, 0]),
            np.array([180, 30, 0]),
            np.array([90, 120, 0]),
            np.array([120, 90, 0]),
            np.array([120, 120, 0]),
            np.array([90, 90, 0])
        ]
        
        self.marker_positions_top = [
            np.array([30, 0, 30]),
            np.array([180, 0, 30]),
            np.array([30, 0, 180]),
            np.array([180, 0, 180]),
            np.array([90, 0, 90]),
            np.array([120, 0, 120]),
            np.array([120, 0, 90]),
            np.array([90, 0, 120])
        ]
        
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
        
        # OpenCVウィンドウの初期化
        self.setup_opencv_windows()
        
        # 表示更新タイマー（30Hz）
        self.display_timer = self.create_timer(1.0/30.0, self.update_display)
        
        # キー入力処理用タイマー（60Hz）
        self.key_timer = self.create_timer(1.0/60.0, self.process_key_input)
        
        self.get_logger().info('カメラキャリブレーションノードが初期化されました')
        self.get_logger().info('使用方法：')
        self.get_logger().info('- 画像ウィンドウでマーカーをクリック')
        self.get_logger().info('- cキー：キャリブレーション実行')
        self.get_logger().info('- rキー：リセット')
        self.get_logger().info('- sキー：結果保存')
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
        
        # マウスコールバック設定
        cv2.setMouseCallback('Camera1_RGB', self.mouse_callback, 'Camera1_RGB')
        cv2.setMouseCallback('Camera2_RGB', self.mouse_callback, 'Camera2_RGB')
        
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
            "Controls:",
            "C = Calibrate, R = Reset, S = Save, Q = Quit",
            "",
            f"Points: Cam1={len(self.clicked_points_cam1_2d)}, Cam2={len(self.clicked_points_cam2_2d)}"
        ]
        
        for i, text in enumerate(texts):
            if text:
                cv2.putText(black_image, text, (10, 50 + i * 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        
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
            
            if key == ord('c'):
                self.get_logger().info('Cキー: キャリブレーション開始')
                self.calibrate_cameras()
            elif key == ord('r'):
                self.get_logger().info('Rキー: リセット')
                self.reset_points()
            elif key == ord('s'):
                self.get_logger().info('Sキー: 保存')
                self.save_calibration()
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
            
            # クリック点を描画
            self.draw_clicked_points(rgb1_display, self.clicked_points_cam1_2d, self.clicked_points_cam1_3d)
            self.draw_clicked_points(rgb2_display, self.clicked_points_cam2_2d, self.clicked_points_cam2_3d)
            
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

    def draw_clicked_points(self, image, points_2d, points_3d):
        """クリック点を描画"""
        for i, (point_2d, point_3d) in enumerate(zip(points_2d, points_3d)):
            if point_3d[2] == 0:
                cv2.circle(image, point_2d, 8, (255, 0, 0), -1)  # 青（3Dデータなし）
            else:
                cv2.circle(image, point_2d, 6, (255, 0, 255), 2)  # 紫（3Dデータあり）
            
            # 番号を表示
            cv2.putText(image, str(i), (point_2d[0]+10, point_2d[1]-10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

    def draw_status_overlay(self, image, camera_name):
        """ステータス情報をオーバーレイ"""
        if camera_name == "Camera 1":
            points_count = len(self.clicked_points_cam1_2d)
        else:
            points_count = len(self.clicked_points_cam2_2d)
        
        status_texts = [
            f"{camera_name}: {points_count}/8 points",
            "C=Calibrate R=Reset S=Save Q=Quit"
        ]
        
        # 背景を描画
        overlay = image.copy()
        cv2.rectangle(overlay, (5, 5), (400, 60), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.7, image, 0.3, 0, image)
        
        # テキストを描画
        for i, text in enumerate(status_texts):
            cv2.putText(image, text, (10, 25 + i * 20), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)

    def conv3d(self, depth_image, x, y):
        """2D座標を3D座標に変換"""
        if y >= depth_image.shape[0] or x >= depth_image.shape[1]:
            return np.array([0, 0, 0])
        
        # 深度値を取得（mm単位）
        depth_mm = depth_image[y, x]
        if depth_mm == 0:
            return np.array([0, 0, 0])
        
        # 画像中心からの相対座標
        xc = x - 320  # 画像幅の半分
        yc = y - self.camera_params['hh']  # 画像高さの半分
        
        # 角度計算
        thh = self.camera_params['thh']
        tvh = self.camera_params['tvh']
        tdh = self.camera_params['tdh']
        
        thx = np.tan(thh) * xc / 320
        thy = np.tan(tvh) * yc / self.camera_params['hh']
        thz = np.tan(tdh) * np.sqrt(xc**2 + yc**2) / np.sqrt(320**2 + self.camera_params['hh']**2)
        
        # 3D座標計算
        X = -depth_mm * thx / np.sqrt(1 + thx**2)
        Y = -depth_mm * thy / np.sqrt(1 + thy**2)
        Z = depth_mm / np.sqrt(1 + thz**2)
        
        return np.array([X, Y, Z])

    def mouse_callback(self, event, x, y, flags, param):
        """マウスクリックイベント処理"""
        if event == cv2.EVENT_LBUTTONDOWN:
            window_name = param
            
            if window_name == 'Camera1_RGB' and self.current_depth1 is not None:
                xyz = self.conv3d(self.current_depth1, x, y)
                self.clicked_points_cam1_2d.append((x, y))
                self.clicked_points_cam1_3d.append(xyz)
                self.get_logger().info(f'Camera1 点{len(self.clicked_points_cam1_2d)}: 2D({x}, {y}) -> 3D({xyz[0]:.1f}, {xyz[1]:.1f}, {xyz[2]:.1f})')
                
            elif window_name == 'Camera2_RGB' and self.current_depth2 is not None:
                xyz = self.conv3d(self.current_depth2, x, y)
                self.clicked_points_cam2_2d.append((x, y))
                self.clicked_points_cam2_3d.append(xyz)
                self.get_logger().info(f'Camera2 点{len(self.clicked_points_cam2_2d)}: 2D({x}, {y}) -> 3D({xyz[0]:.1f}, {xyz[1]:.1f}, {xyz[2]:.1f})')

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
        """変換行列計算（SVDを使用）"""
        points_a = np.array(points_a)
        points_b = np.array(points_b)
        
        # 重心計算
        centroid_a = np.mean(points_a, axis=0)
        centroid_b = np.mean(points_b, axis=0)
        
        # 中心化
        points_a_centered = points_a - centroid_a
        points_b_centered = points_b - centroid_b
        
        # 共分散行列
        H = points_a_centered.T @ points_b_centered
        
        # SVD
        U, S, Vt = np.linalg.svd(H)
        
        # 回転行列
        R = Vt.T @ U.T
        
        # 反射を修正
        if np.linalg.det(R) < 0:
            Vt[-1, :] *= -1
            R = Vt.T @ U.T
        
        # 平行移動ベクトル
        t = centroid_b - R @ centroid_a
        
        # 4x4変換行列
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = t
        
        return T

    def calibrate_cameras(self):
        """キャリブレーション実行"""
        if len(self.clicked_points_cam1_3d) < 4 or len(self.clicked_points_cam2_3d) < 4:
            self.get_logger().warn('キャリブレーションには各カメラで最低4点必要です')
            return
        
        if len(self.clicked_points_cam1_3d) != len(self.clicked_points_cam2_3d):
            self.get_logger().warn('両カメラの点数が一致しません')
            return
        
        try:
            # 各カメラからマーカー座標系への変換行列
            T_cam1_to_marker = self.compute_transformation_matrix(
                self.clicked_points_cam1_3d[:len(self.clicked_points_cam1_3d)], 
                self.marker_positions_front[:len(self.clicked_points_cam1_3d)]
            )
            T_cam2_to_marker = self.compute_transformation_matrix(
                self.clicked_points_cam2_3d[:len(self.clicked_points_cam2_3d)], 
                self.marker_positions_top[:len(self.clicked_points_cam2_3d)]
            )
            
            # Camera2からCamera1への変換行列
            T_marker_to_cam1 = np.linalg.inv(T_cam1_to_marker)
            self.transformation_matrix = T_marker_to_cam1 @ T_cam2_to_marker
            
            self.get_logger().info('キャリブレーション完了！')
            self.get_logger().info(f'変換行列:\n{self.transformation_matrix}')
            
            # 変換精度確認
            self.verify_calibration()
            
            # TFとして配信
            self.publish_transform()
            
            # ステータス配信
            status_msg = Bool()
            status_msg.data = True
            self.calibration_status_pub.publish(status_msg)
            
        except Exception as e:
            self.get_logger().error(f'キャリブレーションエラー: {str(e)}')

    def verify_calibration(self):
        """キャリブレーション精度確認"""
        if self.transformation_matrix is None:
            return
        
        errors = []
        for i, point_cam2 in enumerate(self.clicked_points_cam2_3d):
            if point_cam2[2] == 0:  # 無効点をスキップ
                continue
                
            # Camera2の点をCamera1座標系に変換
            point_cam2_homogeneous = np.append(point_cam2, 1.0)
            transformed_point = self.transformation_matrix @ point_cam2_homogeneous
            transformed_point = transformed_point[:3]
            
            # 対応するCamera1の点との誤差計算
            if i < len(self.clicked_points_cam1_3d):
                error = np.linalg.norm(transformed_point - self.clicked_points_cam1_3d[i])
                errors.append(error)
                
                self.get_logger().info(f'点{i}: Camera2({point_cam2}) -> Camera1推定({transformed_point}) 誤差: {error:.2f}mm')
        
        if errors:
            mean_error = np.mean(errors)
            self.get_logger().info(f'平均誤差: {mean_error:.2f}mm')

    def publish_transform(self):
        """TFとして変換を配信"""
        if self.transformation_matrix is None:
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
        
        # mmをmに変換
        transform.transform.translation.x = translation[0] / 1000.0
        transform.transform.translation.y = translation[1] / 1000.0
        transform.transform.translation.z = translation[2] / 1000.0
        
        transform.transform.rotation.x = quaternion[0]
        transform.transform.rotation.y = quaternion[1]
        transform.transform.rotation.z = quaternion[2]
        transform.transform.rotation.w = quaternion[3]
        
        # TF配信
        self.tf_broadcaster.sendTransform(transform)
        
        self.get_logger().info('TFが配信されました')

    def reset_points(self):
        """クリック点をリセット"""
        self.clicked_points_cam1_2d.clear()
        self.clicked_points_cam1_3d.clear()
        self.clicked_points_cam2_2d.clear()
        self.clicked_points_cam2_3d.clear()
        self.transformation_matrix = None
        
        self.get_logger().info('クリック点がリセットされました')

    def save_calibration(self):
        """キャリブレーション結果をファイルに保存"""
        if self.transformation_matrix is None:
            self.get_logger().warn('保存するキャリブレーション結果がありません')
            return
        
        # ホームディレクトリに保存
        home_dir = os.path.expanduser('~')
        filename = os.path.join(home_dir, 'camera_calibration_result.json')
        
        calibration_data = {
            'transformation_matrix': self.transformation_matrix.tolist(),
            'camera1_points_2d': self.clicked_points_cam1_2d,
            'camera1_points_3d': [point.tolist() for point in self.clicked_points_cam1_3d],
            'camera2_points_2d': self.clicked_points_cam2_2d,
            'camera2_points_3d': [point.tolist() for point in self.clicked_points_cam2_3d],
            'marker_positions_front': [pos.tolist() for pos in self.marker_positions_front],
            'marker_positions_top': [pos.tolist() for pos in self.marker_positions_top],
            'timestamp': self.get_clock().now().to_msg().sec
        }
        
        try:
            with open(filename, 'w') as f:
                json.dump(calibration_data, f, indent=2)
            
            self.get_logger().info(f'キャリブレーション結果を保存しました: {filename}')
            
        except Exception as e:
            self.get_logger().error(f'保存エラー: {str(e)}')

    def destroy_node(self):
        """ノード終了時の処理"""
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