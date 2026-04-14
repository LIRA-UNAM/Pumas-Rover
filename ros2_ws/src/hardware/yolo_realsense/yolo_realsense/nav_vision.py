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

class NavVisionNode(Node):
    def __init__(self):
        super().__init__('nav_vision')
        self.bridge = CvBridge()
        
        
        weights_path = os.path.expanduser('~/Pumas-Rover/ros2_ws/weights/fin.pt')
        self.model = YOLO(weights_path) 
        
        self.distance_pub = self.create_publisher(PointStamped, '/yolo/end_distance', 10)
        
        self.fx = self.fy = self.cx = self.cy = None
        
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Suscripción a la info de cámara
        self.camera_info_sub = self.create_subscription(
            CameraInfo, 
            '/camera/camera/color/camera_info', 
            self.info_cb, 
            qos_profile
        )

        # Suscriptores sincronizados con el perfil correcto
        self.rgb_sub = Subscriber(self, Image, '/camera/camera/color/image_raw', qos_profile=qos_profile)
        self.depth_sub = Subscriber(self, Image, '/camera/camera/aligned_depth_to_color/image_raw', qos_profile=qos_profile)
        
       
        self.sync = ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub], 
            queue_size=10, 
            slop=0.1
        )
        self.sync.registerCallback(self.callback)

        self.processing = False
        cv2.namedWindow("Navegación - YOLO", cv2.WINDOW_NORMAL)
        
        self.get_logger().info('Nodo NavVision activo. Esperando flujo de cámara...')

    def info_cb(self, msg):
        # Guardamos los parámetros intrínsecos de la cámara
        self.fx, self.fy, self.cx, self.cy = msg.k[0], msg.k[4], msg.k[2], msg.k[5]
        self.get_logger().info('¡Calibración recibida! Iniciando visor...')
        self.destroy_subscription(self.camera_info_sub)

    def callback(self, rgb_msg, depth_msg):

        if self.processing:
            return
        self.processing = True
        try:
            if self.fx is None:
                return

            frame = self.bridge.imgmsg_to_cv2(rgb_msg, 'bgr8')
            depth_frame = self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough')

            results = self.model(frame, conf=0.5, verbose=False)
            annotated_frame = results[0].plot()

            for result in results:
                for box in result.boxes:
                   x_c, y_c, _, _ = box.xywh[0].cpu().numpy()
                   u, v = int(x_c), int(y_c)

                   if 0 <= v < depth_frame.shape[0] and 0 <= u < depth_frame.shape[1]:
                        z_raw = depth_frame[v, u]
                        if z_raw > 0:
                            Z = float(z_raw) / 1000.0
                            X = (u - self.cx) * Z / self.fx
                            Y = (v - self.cy) * Z / self.fy

                            dist_msg = PointStamped()
                            dist_msg.header = rgb_msg.header
                            dist_msg.point.x = X
                            dist_msg.point.y = Y
                            dist_msg.point.z = Z
                            self.distance_pub.publish(dist_msg)
                            # Dibujar datos en el frame
                            x1, y1, _, _ = box.xyxy[0].cpu().numpy().astype(int)
                            cv2.putText(annotated_frame, f"Z:{Z:.2f}m X:{X:.2f}m", (x1, y1 - 10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            cv2.imshow("Navegación - YOLO", annotated_frame)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f'Error: {e}')

        finally:
            self.processing = False
        

                        

      

def main(args=None):
    rclpy.init(args=args)
    node = NavVisionNode()
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