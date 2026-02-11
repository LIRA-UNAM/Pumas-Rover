import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped

from cv_bridge import CvBridge
import cv2
import numpy as np

from ultralytics import YOLO
from message_filters import Subscriber, ApproximateTimeSynchronizer


class YoloDepthNode(Node):

    def __init__(self):
        super().__init__('yolo_depth_node')

        self.bridge = CvBridge()
        self.model = YOLO('yolov8n.pt')

        # Intrínsecos
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        # CameraInfo
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera/color/camera_info',
            self.camera_info_callback,
            10
        )

        # Publicador
        self.point_pub = self.create_publisher(
            PointStamped,
            '/yolo/object_point_camera',
            10
        )

        # Subscribers sincronizados
        self.rgb_sub = Subscriber(
            self,
            Image,
            '/camera/camera/color/image_raw'
        )

        self.depth_sub = Subscriber(
            self,
            Image,
            '/camera/camera/depth/image_rect_raw'
        )

        self.sync = ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub],
            queue_size=10,
            slop=0.1
        )
        self.sync.registerCallback(self.synced_callback)

        self.get_logger().info('YOLO + RS')


    def camera_info_callback(self, msg: CameraInfo):
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]

        self.get_logger().info(
            f'Camera intrinsics loaded: '
            f'fx={self.fx:.2f}, fy={self.fy:.2f}, '
            f'cx={self.cx:.2f}, cy={self.cy:.2f}'
        )

        self.destroy_subscription(self.camera_info_sub)


    def synced_callback(self, rgb_msg: Image, depth_msg: Image):

        if self.fx is None:
            return

        rgb = self.bridge.imgmsg_to_cv2(rgb_msg, 'bgr8')
        depth = self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough')

        results = self.model(rgb, verbose=False)

        if len(results[0].boxes) == 0:
            return

        box = results[0].boxes[0]
        x_center, y_center, w, h = box.xywh[0].cpu().numpy()

        u = int(x_center)
        v = int(y_center)

        if u < 2 or v < 2 or u >= depth.shape[1] - 2 or v >= depth.shape[0] - 2:
            return

        # Promedio de profundidad (5x5)
        kernel = depth[v-2:v+3, u-2:u+3]
        kernel = kernel[kernel > 0]

        if len(kernel) == 0:
            return

        if depth.dtype == np.uint16:
            Z = np.mean(kernel) / 1000.0  # mm → m
        else:
            Z = float(np.mean(kernel))

        X = (u - self.cx) * Z / self.fx
        Y = (v - self.cy) * Z / self.fy

        pt = PointStamped()
        pt.header.stamp = rgb_msg.header.stamp
        pt.header.frame_id = 'camera_color_optical_frame'
        pt.point.x = float(X)
        pt.point.y = float(Y)
        pt.point.z = float(Z)

        self.point_pub.publish(pt)

        self.get_logger().info(
            f'[CAMERA] X={X:.3f}, Y={Y:.3f}, Z={Z:.3f} m'
        )

        annotated = results[0].plot()
        cv2.circle(annotated, (u, v), 5, (0, 0, 255), -1)
        cv2.imshow('YOLO + Depth', annotated)
        cv2.waitKey(1)


def main():
    rclpy.init()
    node = YoloDepthNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
