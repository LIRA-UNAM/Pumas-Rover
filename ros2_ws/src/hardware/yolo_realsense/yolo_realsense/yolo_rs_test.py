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

        #Modelo
        weights_path = os.path.expanduser('~/Proyectos/Brazo_repo/dev_ws/weights/best.pt')
        self.model = YOLO(weights_path)

        self.fx = self.fy = self.cx = self.cy = None
        self.point_pub = self.create_publisher(PointStamped, '/yolo/object_point_camera', 10)

        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        
        rgb_topic = '/camera/camera/color/image_raw'
        
        depth_topic = '/camera/camera/depth/image_rect_raw' 
        info_topic = '/camera/camera/color/camera_info'

        self.get_logger().info(f'Suscrito a RGB: {rgb_topic}')
        self.get_logger().info(f'Suscrito a DEPTH: {depth_topic}')

        
        self.camera_info_sub = self.create_subscription(
            CameraInfo, 
            info_topic, 
            self.info_cb, 
            qos_profile
        )

        
        self.rgb_sub = Subscriber(self, Image, rgb_topic, qos_profile=qos_profile)
        self.depth_sub = Subscriber(self, Image, depth_topic, qos_profile=qos_profile)

        
        self.sync = ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub], 
            queue_size=10, 
            slop=0.2
        )
        self.sync.registerCallback(self.synced_callback)
        
        self.get_logger().info('YOLO + RS')

    def info_cb(self, msg):
        self.fx, self.fy, self.cx, self.cy = msg.k[0], msg.k[4], msg.k[2], msg.k[5]
        self.get_logger().info(f'Calibración fx={self.fx:.1f}, fy={self.fy:.1f}')
        
        self.destroy_subscription(self.camera_info_sub)

    def synced_callback(self, rgb_msg, depth_msg):
        
        if self.fx is None: 
            return

        try:
            
            frame = self.bridge.imgmsg_to_cv2(rgb_msg, 'bgr8')
            depth_frame = self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough') # mm
        except Exception as e:
            self.get_logger().error(f'Error: {e}')
            return

        
        results = self.model(frame, conf=0.6, verbose=False)
        
        closest_point = None
        min_z = float('inf')
        
        
        for result in results:
            for box in result.boxes:
                x_c, y_c, _, _ = box.xywh[0].cpu().numpy()
                u, v = int(x_c), int(y_c)
                
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                
                h, w = depth_frame.shape
                x1, y1 = max(0, x1), max(0, y1)
                x2, y2 = min(w, x2), min(h, y2)
                depth_roi = depth_frame[y1:y2, x1:x2]
                
                
                valid_depths = depth_roi[depth_roi > 0]
                if len(valid_depths) == 0: continue
                
                
                Z_current = np.median(valid_depths) / 1000.0 

                
                if Z_current < min_z:
                    min_z = Z_current
                    
                    
                    X_current = (u - self.cx) * Z_current / self.fx
                    Y_current = (v - self.cy) * Z_current / self.fy
                    
                    closest_point = (X_current, Y_current, Z_current)

        
        annotated = results[0].plot()

        if closest_point:
            X, Y, Z = closest_point
            
           
            self.get_logger().info(f'[CAMERA] X={X:.3f}, Y={Y:.3f}, Z={Z:.3f} m')

           
            text_coord = f"X:{X:.2f} Y:{Y:.2f} Z:{Z:.2f}m"
            cv2.putText(annotated, text_coord, (u, v - 10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
            
            pt = PointStamped()
            pt.header = rgb_msg.header
            
            pt.header.frame_id = rgb_msg.header.frame_id 
            pt.point.x, pt.point.y, pt.point.z = X, Y, Z
            self.point_pub.publish(pt)

        cv2.imshow('YOLO + RS', annotated)
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