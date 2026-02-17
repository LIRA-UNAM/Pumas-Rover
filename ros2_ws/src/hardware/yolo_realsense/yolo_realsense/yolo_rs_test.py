import rclpy
from rclpy.node import Node
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

        # Ruta de modelo entrenado
        weights_path = os.path.expanduser('~/Proyectos/Brazo_repo/dev_ws/weights/best.pt')
        self.model = YOLO(weights_path)

        self.fx = self.fy = self.cx = self.cy = None
        self.point_pub = self.create_publisher(PointStamped, '/yolo/object_point_camera', 10)

        # Suscripciones
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera/color/camera_info',
            self.info_cb,
            10
        )

        self.rgb_sub = Subscriber(self, Image, '/camera/camera/color/image_raw')
        self.depth_sub = Subscriber(self, Image, '/camera/camera/depth/image_rect_raw')  

        self.sync = ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub],
            queue_size=10,
            slop=0.1
        )
        self.sync.registerCallback(self.synced_callback)

        self.get_logger().info('YOLO + RS')

    def info_cb(self, msg):
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]
        self.destroy_subscription(self.camera_info_sub)

    def synced_callback(self, rgb_msg, depth_msg):

        if self.fx is None:
            return

        frame = self.bridge.imgmsg_to_cv2(rgb_msg, 'bgr8')
        depth_frame = self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough')

        results = self.model(frame, conf=0.6, verbose=False)

        closest_point = None
        min_z = float('inf')
        target_label = ""

        
        for result in results:
            for box in result.boxes:

                
                x_c, y_c, _, _ = box.xywh[0].cpu().numpy()
                u, v = int(x_c), int(y_c)

                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                depth_roi = depth_frame[y1:y2, x1:x2]
                valid_depths = depth_roi[depth_roi > 0]

                if len(valid_depths) == 0:
                    continue

                # Calcular profundidad Z
                if depth_frame.dtype == np.uint16:
                    Z_current = np.median(valid_depths) / 1000.0
                else:
                    Z_current = np.median(valid_depths)

                
                if Z_current < min_z:
                    min_z = Z_current

                    X_current = (u - self.cx) * Z_current / self.fx
                    Y_current = (v - self.cy) * Z_current / self.fy

                    closest_point = (X_current, Y_current, Z_current)
                    target_label = self.model.names[int(box.cls[0])]

       
        if closest_point:
            X, Y, Z = closest_point

            self.get_logger().info(
                f'[CAMERA] X={X:.3f}, Y={Y:.3f}, Z={Z:.3f} m'
            )

            pt = PointStamped()
            pt.header = rgb_msg.header
            pt.header.frame_id = 'camera_color_optical_frame'
            pt.point.x = X
            pt.point.y = Y
            pt.point.z = Z

            self.point_pub.publish(pt)

            
            annotated = results[0].plot()
            cv2.putText(
                annotated,
                f"OBJETIVO: {target_label} ({Z:.2f}m)",
                (50, 50),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (0, 255, 0),
                2
            )

            cv2.imshow('YOLO + Depth', annotated)
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
