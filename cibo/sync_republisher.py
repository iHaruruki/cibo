import copy
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from builtin_interfaces.msg import Time
from sensor_msgs.msg import Image, CameraInfo
from message_filters import Subscriber, ApproximateTimeSynchronizer


class DualCameraSyncRepublisher(Node):
    def __init__(self):
        super().__init__('dual_camera_sync_node')

        # Parameters
        self.declare_parameter('queue_size', 20)
        self.declare_parameter('slop_sec', 0.08)  # 80ms
        self.declare_parameter('restamp', False)  # 原則 False（計測時刻保持）

        q = int(self.get_parameter('queue_size').value)
        slop = float(self.get_parameter('slop_sec').value)
        self.restamp = bool(self.get_parameter('restamp').value)

        # Subscribers (raw topics)
        self.c1_color_sub = Subscriber(self, Image, '/camera_01/color/image_raw', qos_profile=qos_profile_sensor_data)
        self.c1_depth_sub = Subscriber(self, Image, '/camera_01/depth/image_raw', qos_profile=qos_profile_sensor_data)
        self.c1_info_sub  = Subscriber(self, CameraInfo, '/camera_01/depth/camera_info', qos_profile=qos_profile_sensor_data)

        self.c2_color_sub = Subscriber(self, Image, '/camera_02/color/image_raw', qos_profile=qos_profile_sensor_data)
        self.c2_depth_sub = Subscriber(self, Image, '/camera_02/depth/image_raw', qos_profile=qos_profile_sensor_data)
        self.c2_info_sub  = Subscriber(self, CameraInfo, '/camera_02/depth/camera_info', qos_profile=qos_profile_sensor_data)

        # Approximate synchronizer across 6 topics
        self.sync = ApproximateTimeSynchronizer(
            [self.c1_color_sub, self.c1_depth_sub, self.c1_info_sub,
             self.c2_color_sub, self.c2_depth_sub, self.c2_info_sub],
            queue_size=q,
            slop=slop
        )
        self.sync.registerCallback(self.synced_cb)

        # Publishers (/sync namespace)
        self.pub_c1_color = self.create_publisher(Image,     '/sync/camera_01/color/image_raw', qos_profile_sensor_data)
        self.pub_c1_depth = self.create_publisher(Image,     '/sync/camera_01/depth/image_raw', qos_profile_sensor_data)
        self.pub_c1_info  = self.create_publisher(CameraInfo,'/sync/camera_01/depth/camera_info', qos_profile_sensor_data)

        self.pub_c2_color = self.create_publisher(Image,     '/sync/camera_02/color/image_raw', qos_profile_sensor_data)
        self.pub_c2_depth = self.create_publisher(Image,     '/sync/camera_02/depth/image_raw', qos_profile_sensor_data)
        self.pub_c2_info  = self.create_publisher(CameraInfo,'/sync/camera_02/depth/camera_info', qos_profile_sensor_data)

        self.get_logger().info(f'DualCameraSyncRepublisher started: queue_size={q}, slop={slop}s, restamp={self.restamp}')

    def synced_cb(self, c1_color, c1_depth, c1_info, c2_color, c2_depth, c2_info):
        msgs = [c1_color, c1_depth, c1_info, c2_color, c2_depth, c2_info]

        if self.restamp:
            # 同一時刻に押し揃える（推奨しません。必要時のみ使用）
            # ここでは6枚のうち最新時刻を採用
            latest = max([m.header.stamp.sec + m.header.stamp.nanosec*1e-9 for m in msgs])
            secs = int(latest)
            nsecs = int((latest - secs) * 1e9)
            common_stamp = Time(sec=secs, nanosec=nsecs)
            out = []
            for m in msgs:
                mc = copy.deepcopy(m)
                mc.header.stamp = common_stamp
                out.append(mc)
            c1_color, c1_depth, c1_info, c2_color, c2_depth, c2_info = out

        # Republish
        self.pub_c1_color.publish(c1_color)
        self.pub_c1_depth.publish(c1_depth)
        self.pub_c1_info.publish(c1_info)

        self.pub_c2_color.publish(c2_color)
        self.pub_c2_depth.publish(c2_depth)
        self.pub_c2_info.publish(c2_info)

    def destroy_node(self):
        super().destroy_node()


def main():
    rclpy.init()
    node = DualCameraSyncRepublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()