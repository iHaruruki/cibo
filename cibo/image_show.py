import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraImageDisplay(Node):
    def __init__(self):
        super().__init__('camera_image_display')

        # CvBridgeのインスタンスを作成
        self.bridge = CvBridge()

        # camera_01の画像トピックをサブスクライブ
        self.color_sub_01 = self.create_subscription(Image, '/camera_01/color/image_raw', self.color_callback_01, 10)
        self.depth_sub_01 = self.create_subscription(Image, '/camera_01/depth/image_raw', self.depth_callback_01, 10)
        self.ir_sub_01 = self.create_subscription(Image, '/camera_01/ir/image_raw', self.ir_callback_01, 10)

        # camera_02の画像トピックをサブスクライブ
        self.color_sub_02 = self.create_subscription(Image, '/camera_02/color/image_raw', self.color_callback_02, 10)
        self.depth_sub_02 = self.create_subscription(Image, '/camera_02/depth/image_raw', self.depth_callback_02, 10)
        self.ir_sub_02 = self.create_subscription(Image, '/camera_02/ir/image_raw', self.ir_callback_02, 10)

        # ウィンドウ名を設定
        cv2.namedWindow("Camera 01 Color", cv2.WINDOW_NORMAL)
        cv2.namedWindow("Camera 01 Depth", cv2.WINDOW_NORMAL)
        cv2.namedWindow("Camera 01 IR", cv2.WINDOW_NORMAL)

        cv2.namedWindow("Camera 02 Color", cv2.WINDOW_NORMAL)
        cv2.namedWindow("Camera 02 Depth", cv2.WINDOW_NORMAL)
        cv2.namedWindow("Camera 02 IR", cv2.WINDOW_NORMAL)

    def color_callback_01(self, msg):
        """camera_01 カラー画像のコールバック"""
        try:
            color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv2.imshow("Camera 01 Color", color_image)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Error processing camera 01 color image: {e}")

    def depth_callback_01(self, msg):
        """camera_01 深度画像のコールバック"""
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            depth_image = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX)
            depth_image = cv2.convertScaleAbs(depth_image)
            cv2.imshow("Camera 01 Depth", depth_image)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Error processing camera 01 depth image: {e}")

    def ir_callback_01(self, msg):
        """camera_01 赤外線画像のコールバック"""
        try:
            # もしエンコーディングが'mono16'の場合、'mono8'に変更
            ir_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono16')  # 'mono16'に変更してみてください
            ir_image = cv2.normalize(ir_image, None, 0, 255, cv2.NORM_MINMAX)
            ir_image = cv2.convertScaleAbs(ir_image)
            cv2.imshow("Camera 01 IR", ir_image)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Error processing camera 01 IR image: {e}")

    def color_callback_02(self, msg):
        """camera_02 カラー画像のコールバック"""
        try:
            color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv2.imshow("Camera 02 Color", color_image)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Error processing camera 02 color image: {e}")

    def depth_callback_02(self, msg):
        """camera_02 深度画像のコールバック"""
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            depth_image = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX)
            depth_image = cv2.convertScaleAbs(depth_image)
            cv2.imshow("Camera 02 Depth", depth_image)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Error processing camera 02 depth image: {e}")

    def ir_callback_02(self, msg):
        """camera_02 赤外線画像のコールバック"""
        try:
            # 赤外線画像のエンコーディング形式を'mono8'または'mono16'で試します
            ir_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono16')  # 'mono16'に変更してみてください
            ir_image = cv2.normalize(ir_image, None, 0, 255, cv2.NORM_MINMAX)
            ir_image = cv2.convertScaleAbs(ir_image)
            cv2.imshow("Camera 02 IR", ir_image)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Error processing camera 02 IR image: {e}")

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
