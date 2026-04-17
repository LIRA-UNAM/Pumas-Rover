import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String
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
        
        # MODELOS
        path_banderin = os.path.expanduser('~/Pumas-Rover/ros2_ws/weights/fin.pt')
        path_rocas = os.path.expanduser('~/Pumas-Rover/ros2_ws/weights/best.pt')
        self.model_banderin = YOLO(path_banderin) 
        self.model_rocas = YOLO(path_rocas)
        
        # ESTADOS Y CONTROL 
        self.current_state = 'SM_BUSCANDO_BANDERIN' 
        self.fx = self.fy = self.cx = self.cy = None
        self.processing = False 
        
        # FILTRO ANTIRUIDO (THRESHOLD)
        self.detection_count = 0
        self.threshold = 5  
        
        # TOPICOS 
        self.pub_banderin = self.create_publisher(PointStamped, '/vision/target_flag', 10)
        self.pub_roca = self.create_publisher(PointStamped, '/vision/target_rock', 10)
        self.pub_roca_color = self.create_publisher(String, '/vision/target_rock_color', 10) # <--- NUEVO
        self.pub_inicio = self.create_publisher(PointStamped, '/vision/start_flag', 10)
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # SUSCRIPTORES
        self.state_sub = self.create_subscription(String, '/mission/current_state', self.state_cb, 10)
        self.camera_info_sub = self.create_subscription(CameraInfo, '/camera/camera/color/camera_info', self.info_cb, qos_profile)

        # RGB + Depth
        self.rgb_sub = Subscriber(self, Image, '/camera/camera/color/image_raw', qos_profile=qos_profile)
        self.depth_sub = Subscriber(self, Image, '/camera/camera/aligned_depth_to_color/image_raw', qos_profile=qos_profile)
        
        self.sync = ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub], queue_size=10, slop=0.1)
        self.sync.registerCallback(self.vision_cb)

        cv2.namedWindow("Pumas-Rover Vision", cv2.WINDOW_NORMAL)
        self.get_logger().info('VISION ACTIVADA. Listo para publicar coordenadas y colores.')

    def state_cb(self, msg):
        if msg.data in ['SM_BUSCANDO_BANDERIN', 'SM_BUSCANDO_ROCAS', 'SM_REGRESO']:
            if self.current_state != msg.data:
                self.current_state = msg.data
                self.detection_count = 0 
                self.get_logger().info(f'🔄 ESTADO DE MISIÓN ACTUALIZADO: {self.current_state} (Filtro reiniciado)')

    def info_cb(self, msg):
        self.fx, self.fy, self.cx, self.cy = msg.k[0], msg.k[4], msg.k[2], msg.k[5]
        self.get_logger().info('Cámara calibrada correctamente.')
        self.destroy_subscription(self.camera_info_sub)

    def process_inference(self, frame, depth_frame, model, conf_val):
        results = model(frame, conf=conf_val, verbose=False)
        closest_pt = None
        min_z = float('inf')
        detected_name = ""

        for result in results:
            for box in result.boxes:
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                u, v = int((x1 + x2) / 2), int((y1 + y2) / 2)
                
                depth_roi = depth_frame[max(0, y1):min(depth_frame.shape[0], y2), max(0, x1):min(depth_frame.shape[1], x2)]
                valid_depths = depth_roi[depth_roi > 0]
                
                if len(valid_depths) == 0: continue
                
                z_median = np.median(valid_depths) / 1000.0 

                if z_median < min_z:
                    min_z = z_median
                    x_coord = (u - self.cx) * z_median / self.fx
                    y_coord = (v - self.cy) * z_median / self.fy
                    closest_pt = (x_coord, y_coord, z_median)
                    
                    cls_id = int(box.cls[0].item())
                    detected_name = result.names[cls_id]

        return closest_pt, detected_name, results[0].plot()

    def vision_cb(self, rgb_msg, depth_msg):
        if self.processing or self.fx is None: return
        self.processing = True

        try:
            frame = self.bridge.imgmsg_to_cv2(rgb_msg, 'bgr8')
            depth_frame = self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough')
            
            target_pt = None
            name = ""
            debug_frame = frame.copy()
        
            # SELECCIÓN DE MODELO SEGÚN EL ESTADO
            if self.current_state == 'SM_BUSCANDO_BANDERIN':
                target_pt, name, debug_frame = self.process_inference(frame, depth_frame, self.model_banderin, 0.6)
            elif self.current_state == 'SM_BUSCANDO_ROCAS':
                target_pt, name, debug_frame = self.process_inference(frame, depth_frame, self.model_rocas, 0.5)
            elif self.current_state == 'SM_REGRESO':
                target_pt, name, debug_frame = self.process_inference(frame, depth_frame, self.model_banderin, 0.6)

            if target_pt:
                self.detection_count += 1
                
                # Solo publica si superó el umbral de confianza
                if self.detection_count >= self.threshold:
                    
                    # Preparamos el mensaje de coordenadas (usado para todo)
                    pt_msg = PointStamped()
                    pt_msg.header = rgb_msg.header
                    pt_msg.point.x, pt_msg.point.y, pt_msg.point.z = target_pt
                    
                    if self.current_state == 'SM_BUSCANDO_BANDERIN':
                        self.pub_banderin.publish(pt_msg)
                        
                    elif self.current_state == 'SM_BUSCANDO_ROCAS':
                        # Publicamos Coordenadas
                        self.pub_roca.publish(pt_msg)
                        
                        # Publicamos el Color en el nuevo tópico
                        color_msg = String()
                        color_msg.data = name
                        self.pub_roca_color.publish(color_msg)
                        
                        self.get_logger().info(f'ROCA CONFIRMADA: {name} a {target_pt[2]:.2f}m', once=True)
                        
                    elif self.current_state == 'SM_REGRESO':
                        self.pub_inicio.publish(pt_msg)
                        self.get_logger().info(f'INICIO CONFIRMADO: A {target_pt[2]:.2f}m', once=True)
            else:
                self.detection_count = 0
            
            cv2.imshow("Pumas-Rover Vision", debug_frame)
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