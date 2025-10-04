#!/home/ubuntu/ros2_ws/src/cibo/cibo_ws/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import cv2
import mediapipe as mp
import numpy as np
import os
import sys


class IntegratedMediaPipeNode(Node):
    def __init__(self):
        super().__init__('integrated_mediapipe_processor')
        
        print("Python executable:", sys.executable)
        
        # OpenCV環境チェック
        self.check_opencv_environment()
        
        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # Initialize MediaPipe
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_pose = mp.solutions.pose
        self.mp_face_mesh = mp.solutions.face_mesh
        self.mp_hands = mp.solutions.hands
        self.mp_drawing_styles = mp.solutions.drawing_styles
        
        # Initialize MediaPipe models
        self.face_mesh = mp.solutions.face_mesh.FaceMesh(
            max_num_faces=1,
            refine_landmarks=True,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )
        
        self.pose = mp.solutions.pose.Pose(
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )
        
        self.hands = mp.solutions.hands.Hands(
            max_num_hands=2,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )
        
        # ROI state
        # 'size' を追加（矩形は中心(cx,cy) と size から計算）
        # デフォルトサイズを大きめに（例: 400）に設定しています。必要であれば数値を変更してください。
        self.roi_states = {
            'camera_01': {'enabled': False, 'cx': 200, 'cy': 200, 'size': 300, 'x1': 100, 'y1': 100, 'x2': 500, 'y2': 400},
            'camera_02': {'enabled': False, 'cx': 200, 'cy': 200, 'size': 300, 'x1': 100, 'y1': 100, 'x2': 500, 'y2': 400}
        }
        
        # 最後に操作した（クリックした）カメラ名。サイズ変更はこのカメラに作用します。
        self.last_interacted_camera = None
        
        # OpenCV windows setup
        self.opencv_enabled = self.setup_opencv_windows()
        
        # Subscribers
        self.camera1_sub = self.create_subscription(
            Image, '/camera_01/color/image_raw', self.camera1_callback, 10)
        self.camera2_sub = self.create_subscription(
            Image, '/camera_02/color/image_raw', self.camera2_callback, 10)
        
        # Publishers
        self.camera1_annotated_pub = self.create_publisher(
            Image, '/mp/camera_01/annotated_image', 10)
        self.camera2_annotated_pub = self.create_publisher(
            Image, '/mp/camera_02/annotated_image', 10)
        self.camera1_hand_landmarks_pub = self.create_publisher(
            Float32MultiArray, '/mp/camera_01/hand_landmarks', 10)
        self.camera2_hand_landmarks_pub = self.create_publisher(
            Float32MultiArray, '/mp/camera_02/hand_landmarks', 10)
        self.camera1_face_landmarks_pub = self.create_publisher(
            Float32MultiArray, '/mp/camera_01/face_landmarks', 10)
        self.camera2_face_landmarks_pub = self.create_publisher(
            Float32MultiArray, '/mp/camera_02/face_landmarks', 10)
        
        self.get_logger().info('Integrated MediaPipe ROS2 Node initialized')

    def check_opencv_environment(self):
        """OpenCV環境をチェック"""
        self.get_logger().info(f"OpenCV version: {cv2.__version__}")
        self.get_logger().info(f"DISPLAY: {os.environ.get('DISPLAY', 'Not set')}")
        
        # 簡単なウィンドウテスト
        try:
            test_img = np.zeros((100, 300, 3), dtype=np.uint8)
            cv2.putText(test_img, 'Test Window', (10, 50), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            cv2.namedWindow('Test', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('Test', test_img)
            cv2.waitKey(1000)  # 1秒待機
            cv2.destroyWindow('Test')
            
            self.get_logger().info("OpenCV window test: SUCCESS")
            return True
            
        except Exception as e:
            self.get_logger().error(f"OpenCV window test failed: {str(e)}")
            return False

    def setup_opencv_windows(self):
        """OpenCVウィンドウのセットアップ"""
        try:
            cv2.namedWindow('Camera 01', cv2.WINDOW_AUTOSIZE)
            cv2.namedWindow('Camera 02', cv2.WINDOW_AUTOSIZE)
            
            # マウスコールバック設定
            cv2.setMouseCallback('Camera 01', 
                               lambda event, x, y, flags, param: self.mouse_callback(event, x, y, flags, 'camera_01'))
            cv2.setMouseCallback('Camera 02', 
                               lambda event, x, y, flags, param: self.mouse_callback(event, x, y, flags, 'camera_02'))
            
            self.get_logger().info("OpenCV windows setup successful")
            return True
            
        except Exception as e:
            self.get_logger().error(f"Failed to setup OpenCV windows: {str(e)}")
            return False

    def mouse_callback(self, event, x, y, flags, camera_name):
        """マウスイベントハンドラ"""
        if event == cv2.EVENT_LBUTTONDOWN:
            self.get_logger().info(f"Mouse clicked on {camera_name} at ({x}, {y})")
            # クリック位置を中心にサイズからROIを設定
            roi = self.roi_states[camera_name]
            roi['cx'] = x
            roi['cy'] = y
            half = roi['size'] // 2
            roi['x1'] = max(0, x - half)
            roi['y1'] = max(0, y - half)
            roi['x2'] = x + half
            roi['y2'] = y + half
            roi['enabled'] = True
            self.last_interacted_camera = camera_name
            self.get_logger().info(f"ROI set for {camera_name}: center=({roi['cx']},{roi['cy']}), size={roi['size']} -> ({roi['x1']}, {roi['y1']}) to ({roi['x2']}, {roi['y2']})")

    def process_image(self, cv_image, camera_name):
        """画像処理（MediaPipe）"""
        height, width = cv_image.shape[:2]
        roi_state = self.roi_states[camera_name]
        
        # ROI抽出
        if roi_state['enabled']:
            # size と中心 (cx, cy) から矩形を再計算（サイズ変更後も中心を保持）
            half = int(roi_state.get('size', 200) // 2)
            cx = int(roi_state.get('cx', width//2))
            cy = int(roi_state.get('cy', height//2))
            x1 = max(0, cx - half)
            y1 = max(0, cy - half)
            x2 = min(width, cx + half)
            y2 = min(height, cy + half)

            # 保存
            roi_state['x1'], roi_state['y1'], roi_state['x2'], roi_state['y2'] = x1, y1, x2, y2
            
            processing_image = cv_image[y1:y2, x1:x2]
        else:
            processing_image = cv_image
            x1 = y1 = 0
            x2 = width
            y2 = height
        
        # MediaPipe処理
        image_rgb = cv2.cvtColor(processing_image, cv2.COLOR_BGR2RGB)
        image_rgb.flags.writeable = False
        
        pose_results = self.pose.process(image_rgb)
        face_results = self.face_mesh.process(image_rgb)
        hands_results = self.hands.process(image_rgb)
        
        image_rgb.flags.writeable = True
        annotated_roi = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR)
        
        # ランドマーク描画（ROI内の画像に描画）
        if pose_results.pose_landmarks:
            self.mp_drawing.draw_landmarks(
                annotated_roi, pose_results.pose_landmarks, self.mp_pose.POSE_CONNECTIONS)
        
        if face_results.multi_face_landmarks:
            for face_landmarks in face_results.multi_face_landmarks:
                self.mp_drawing.draw_landmarks(
                    annotated_roi, face_landmarks, self.mp_face_mesh.FACEMESH_CONTOURS,
                    landmark_drawing_spec=None,
                    connection_drawing_spec=self.mp_drawing_styles.get_default_face_mesh_contours_style())
        
        if hands_results.multi_hand_landmarks:
            for hand_landmarks in hands_results.multi_hand_landmarks:
                self.mp_drawing.draw_landmarks(
                    annotated_roi, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)
        
        # -- パブリッシュ用画像（バウンディングボックスは入れない） --
        # 元画像のコピーにROI内のアノテーションを埋め込む（ただし矩形フレームは描かない）
        published_image = cv_image.copy()
        published_image[y1:y2, x1:x2] = annotated_roi
        
        # OpenCVで表示用の画像（ここでだけ矩形とラベルを描画）
        display_image = published_image.copy()
        if roi_state['enabled']:
            cv2.rectangle(display_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(display_image, 'ROI', (x1, y1-10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        # OpenCV表示処理
        if self.opencv_enabled:
            try:
                # 操作説明を追加
                disp = display_image.copy()
                cv2.putText(disp, f'{camera_name} - Click to set ROI', 
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                cv2.putText(disp, 'Press Q to quit OpenCV', 
                           (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                cv2.putText(disp, 'After clicking: +/- to change ROI size, r to reset', 
                           (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)
                
                cv2.imshow(camera_name.replace('_', ' ').title(), disp)
                
                # キー入力チェック
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    self.get_logger().info("Closing OpenCV windows")
                    cv2.destroyAllWindows()
                    self.opencv_enabled = False
                elif key == ord('r'):
                    # ROIリセット
                    self.roi_states[camera_name]['enabled'] = False
                    self.get_logger().info(f"ROI reset for {camera_name}")
                # サイズ増減（最後にクリックしたカメラに対してのみ反映）
                elif key in (ord('+'), ord('='), ord('-')):
                    if self.last_interacted_camera == camera_name:
                        roi = self.roi_states[camera_name]
                        current_size = int(roi.get('size', 200))
                        if key in (ord('+'), ord('=')):
                            new_size = current_size + 40  # 増分は 40px
                        else:
                            new_size = max(20, current_size - 40)  # 最小 20px
                        # 上限は画像サイズに依存させる
                        max_allowed = max(width, height)
                        new_size = min(new_size, max_allowed)
                        roi['size'] = new_size
                        # 中心が未設定なら画像中心を使う
                        cx = int(roi.get('cx', width//2))
                        cy = int(roi.get('cy', height//2))
                        half = new_size // 2
                        roi['x1'] = max(0, cx - half)
                        roi['y1'] = max(0, cy - half)
                        roi['x2'] = min(width, cx + half)
                        roi['y2'] = min(height, cy + half)
                        roi['enabled'] = True
                        self.get_logger().info(f"ROI size changed for {camera_name}: {current_size} -> {new_size}")
                        
            except Exception as e:
                self.get_logger().debug(f"OpenCV display error: {str(e)}")
        
        # ランドマークデータ抽出（※既存のスケーリングロジックは維持）
        hand_landmarks = []
        if hands_results.multi_hand_landmarks:
            for hand_landmarks_data in hands_results.multi_hand_landmarks:
                for landmark in hand_landmarks_data.landmark:
                    hand_landmarks.append(landmark.x * width)
                    hand_landmarks.append(landmark.y * height)
        else:
            hand_landmarks = [0.0, 0.0]
        
        face_landmarks = []
        if pose_results.pose_landmarks:
            for landmark in pose_results.pose_landmarks.landmark:
                face_landmarks.append(landmark.x * width)
                face_landmarks.append(landmark.y * height)
        else:
            face_landmarks.extend([0.0, 0.0])
        
        if face_results.multi_face_landmarks:
            for face_landmarks_data in face_results.multi_face_landmarks:
                for landmark in face_landmarks_data.landmark:
                    face_landmarks.append(landmark.x * width)
                    face_landmarks.append(landmark.y * height)
        else:
            face_landmarks.extend([0.0, 0.0])
        
        return published_image, hand_landmarks, face_landmarks

    def camera1_callback(self, msg):
        """Camera 1 callback"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            annotated_image, hand_landmarks, face_landmarks = self.process_image(cv_image, "camera_01")
            
            # Publish results (ここではバウンディングボックスは描かれていない画像を送る)
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated_image, "bgr8")
            annotated_msg.header = msg.header
            self.camera1_annotated_pub.publish(annotated_msg)
            
            hand_landmarks_msg = Float32MultiArray()
            hand_landmarks_msg.data = hand_landmarks
            self.camera1_hand_landmarks_pub.publish(hand_landmarks_msg)
            
            face_landmarks_msg = Float32MultiArray()
            face_landmarks_msg.data = face_landmarks
            self.camera1_face_landmarks_pub.publish(face_landmarks_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error processing camera 1: {str(e)}')

    def camera2_callback(self, msg):
        """Camera 2 callback"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            annotated_image, hand_landmarks, face_landmarks = self.process_image(cv_image, "camera_02")
            
            # Publish results (ここではバウンディングボックスは描かれていない画像を送る)
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated_image, "bgr8")
            annotated_msg.header = msg.header
            self.camera2_annotated_pub.publish(annotated_msg)
            
            hand_landmarks_msg = Float32MultiArray()
            hand_landmarks_msg.data = hand_landmarks
            self.camera2_hand_landmarks_pub.publish(hand_landmarks_msg)
            
            face_landmarks_msg = Float32MultiArray()
            face_landmarks_msg.data = face_landmarks
            self.camera2_face_landmarks_pub.publish(face_landmarks_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error processing camera 2: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    
    node = IntegratedMediaPipeNode()
    
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