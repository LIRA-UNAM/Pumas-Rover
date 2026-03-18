import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, Twist
import time


SM_WAITING = 0      
SM_APPROACHING = 1  
SM_ARRIVED = 2      

class PathPlanner(Node):
    def __init__(self):
        super().__init__('path_planner')
        
        
        self.subscription = self.create_subscription(
            PointStamped,
            '/yolo/end_distance',
            self.target_callback,
            10)
            
        
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        
        self.state = SM_WAITING
        self.last_msg_time = time.time()
        self.target_x = 0.0
        self.target_z = 0.0
        
        
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info('Path Planner para Rover Lunar iniciado.')

    def target_callback(self, msg):
        
        self.target_x = msg.point.x
        self.target_z = msg.point.z
        self.last_msg_time = time.time()

    def control_loop(self):
        now = time.time()
        msg = Twist()

        # 1. VERIFICACIÓN DE TIMEOUT 
        if (now - self.last_msg_time) > 1.0:
            self.state = SM_WAITING
            self.stop_robot()
            return

        # 2. MÁQUINA DE ESTADOS
        if self.target_z > 0.35: 
            self.state = SM_APPROACHING
        elif self.target_z <= 0.30 and self.target_z > 0:
            self.state = SM_ARRIVED

        # 3. ACCIONES POR ESTADO
        if self.state == SM_APPROACHING:
            msg.linear.x = min(0.2, (self.target_z - 0.3) * 0.5)
            
            
            msg.angular.z = -self.target_x * 0.8 
            
            self.get_logger().info(f'Acercándose... Z: {self.target_z:.2f}m', throttle_duration_sec=1.0)
            self.publisher.publish(msg)

        elif self.state == SM_ARRIVED:
            self.get_logger().info('FIN', throttle_duration_sec=2.0)
            self.stop_robot()

    def stop_robot(self):
        self.publisher.publish(Twist())

def main(args=None):
    rclpy.init(args=args)
    node = PathPlanner()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()