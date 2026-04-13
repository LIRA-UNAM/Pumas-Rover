import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
from ultralytics import YOLO
from message_filters import Subscriber, ApproximateTimeSynchronizer

class YoloStoneDetector(Node):
    def __init__(self):
        super().__init__('yolo_stone_detector')
        self.bridge = CvBridge()

        weights_path = os.path.expanduser('~/Pumas-Rover/ros2_ws/weights/best.pt')
        self.model = YOLO(weights_path)
        self.fx = self.fy = self.cx = self.cy = None
        
        self.point_pub = self.create_publisher(PointStamped, '/yolo/object_point_camera', 10)
        self.confirmed_pub = self.create_publisher(PointStamped, '/yolo/confirmed_object', 10)

        self.detection_count = 0
        self.threshold = 5 

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.camera_info_sub = self.create_subscription(CameraInfo, '/camera/camera/color/camera_info', self.info_cb, qos_profile)
        self.rgb_sub = Subscriber(self, Image, '/camera/camera/color/image_raw', qos_profile=qos_profile)
        self.depth_sub = Subscriber(self, Image, '/camera/camera/depth/image_rect_raw', qos_profile=qos_profile)

        self.sync = ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub], queue_size=10, slop=0.2)
        self.sync.registerCallback(self.synced_callback)
        
        self.get_logger().info('YOLO Detector iniciado. Siguiendo el objeto más cercano.')

    def info_cb(self, msg):
        self.fx, self.fy, self.cx, self.cy = msg.k[0], msg.k[4], msg.k[2], msg.k[5]
        self.destroy_subscription(self.camera_info_sub)

    def synced_callback(self, rgb_msg, depth_msg):
        if self.fx is None: return
        try:
            frame = self.bridge.imgmsg_to_cv2(rgb_msg, 'bgr8')
            depth_frame = self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough')
        except Exception as e:
            self.get_logger().error(f'Error CV Bridge: {e}')
            return

        results = self.model(frame, conf=0.6, verbose=False)
        closest_point = None
        min_z = float('inf')
        found_in_frame = False

        for result in results:
            for box in result.boxes:
                found_in_frame = True
                x_c, y_c, _, _ = box.xywh[0].cpu().numpy()
                u, v = int(x_c), int(y_c)
                
                # ROI de profundidad
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                depth_roi = depth_frame[max(0, y1):min(depth_frame.shape[0], y2), max(0, x1):min(depth_frame.shape[1], x2)]
                valid_depths = depth_roi[depth_roi > 0]
                
                if len(valid_depths) == 0: continue
                Z_current = np.median(valid_depths) / 1000.0 

                if Z_current < min_z:
                    min_z = Z_current
                    X_current = (u - self.cx) * Z_current / self.fx
                    Y_current = (v - self.cy) * Z_current / self.fy
                    closest_point = (X_current, Y_current, Z_current)

        if found_in_frame: self.detection_count += 1
        else: self.detection_count = 0

        if closest_point:
            X, Y, Z = closest_point
            pt = PointStamped()
            pt.header = rgb_msg.header
            pt.point.x, pt.point.y, pt.point.z = X, Y, Z
            self.point_pub.publish(pt)
            
            # LOG DE OBJETIVO ELEGIDO
            self.get_logger().info(f'OBJETIVO ELEGIDO -> Distancia Z: {Z:.2f}m | X Cam: {X:.2f}', once=False)

            if self.detection_count >= self.threshold:
                self.confirmed_pub.publish(pt)

        cv2.imshow('YOLO + RS', results[0].plot())
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = YoloStoneDetector()
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