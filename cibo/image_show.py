import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class CameraImageDisplay(Node):
    def __init__(self):
        super().__init__('camera_image_display')

        # CvBridgeのインスタンスを作成
        self.bridge = CvBridge()

        # camera_01の画像トピックをサブスクライブ
        self.color_sub_01 = self.create_subscription(Image, '/camera_01/color/image_raw', self.color_callback_01, 10)
        self.depth_sub_01 = self.create_subscription(Image, '/camera_01/depth/image_raw', self.depth_callback_01, 10)  # 赤外線代わりに深度画像を使用
        # camera_02の画像トピックをサブスクライブ
        self.color_sub_02 = self.create_subscription(Image, '/camera_02/color/image_raw', self.color_callback_02, 10)
        self.depth_sub_02 = self.create_subscription(Image, '/camera_02/depth/image_raw', self.depth_callback_02, 10)  # 赤外線代わりに深度画像を使用

        # ウィンドウ名を設定
        cv2.namedWindow("Camera 01 Color", cv2.WINDOW_NORMAL)
        cv2.namedWindow("Camera 01 Depth", cv2.WINDOW_NORMAL)
        cv2.namedWindow("Camera 02 Color", cv2.WINDOW_NORMAL)
        cv2.namedWindow("Camera 02 Depth", cv2.WINDOW_NORMAL)

    def color_callback_01(self, msg):
        """camera_01 カラー画像のコールバック"""
        try:
            color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv2.imshow("Camera 01 Color", color_image)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Error processing camera 01 color image: {e}")

    def depth_callback_01(self, msg):
        """camera_01 深度画像のコールバック（赤外線代わり）"""
        try:
            # 深度画像を8ビット画像に変換（通常は16ビットなので正規化）
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            depth_image_normalized = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX)  # 0〜255の範囲に正規化
            depth_image_8bit = cv2.convertScaleAbs(depth_image_normalized)  # 8ビットに変換

            # カラーマッピングを適用してカラーに変換
            depth_image_colored = cv2.applyColorMap(depth_image_8bit, cv2.COLORMAP_JET)  # JETカラーマップ（赤から青に変化）

            cv2.imshow("Camera 01 Depth", depth_image_8bit)  # カラー化した深度画像を表示
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Error processing camera 01 depth image: {e}")

    def color_callback_02(self, msg):
        """camera_02 カラー画像のコールバック"""
        try:
            color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv2.imshow("Camera 02 Color", color_image)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Error processing camera 02 color image: {e}")

    def depth_callback_02(self, msg):
        """camera_02 深度画像のコールバック（赤外線代わり）"""
        try:
            # 深度画像を8ビット画像に変換（通常は16ビットなので正規化）
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            depth_image_normalized = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX)  # 0〜255の範囲に正規化
            depth_image_8bit = cv2.convertScaleAbs(depth_image_normalized)  # 8ビットに変換

            # カラーマッピングを適用してカラーに変換
            depth_image_colored = cv2.applyColorMap(depth_image_8bit, cv2.COLORMAP_JET)  # JETカラーマップ（赤から青に変化）

            cv2.imshow("Camera 02 Depth", depth_image_8bit)  # カラー化した深度画像を表示
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Error processing camera 02 depth image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = CameraImageDisplay()

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
