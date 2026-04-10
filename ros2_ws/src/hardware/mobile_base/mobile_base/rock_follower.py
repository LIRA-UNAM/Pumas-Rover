import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, Twist
from std_msgs.msg import Bool
import time
import math


SM_WAITING = 0      
SM_APPROACHING = 1  
SM_ARRIVED = 2      

class PathPlanner(Node):
    def __init__(self):
        super().__init__('path_planner')
        
        
        self.subscription = self.create_subscription(
            PointStamped,
            '/yolo/object_point_camera',
            self.target_callback,
            10)
        
        self.subscription_move = self.create_subscription(
            Bool,
            '/follow_rock',
            self.move_callback,
            10)
            
        
        self.publisher_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        
        
        
        self.state = SM_WAITING
        self.last_msg_time = time.time()
        self.target_x = 0.0
        self.target_z = 0.0

        
        self.linear_max = 0.5
        self.angular_max = 0.5
        #Speed profile parameters
        self.des_accel_distance = 1.2
        self.acel = 0.05
        self.current_speed = 0.0
        self.goal_tolerance = 0.8
        self.Kp = 0.4  #Max 1.5 for 0.6 m/s
        self.move = False
        
        
        self.timer = self.create_timer(0.05, self.control_loop)
        

    def target_callback(self, msg):
        
        self.target_x = msg.point.x
        self.target_z = msg.point.z
        self.last_msg_time = time.time()


    def move_callback(self, msg):
        self.move = True
          

    def control_loop(self):
        now = time.time()
        msg = Twist()

        if (now - self.last_msg_time) > 1.0:
                self.state = SM_WAITING
                self.number_point = 0
                #self.get_logger().info('Esperando nuevo objetivo...')
        elif self.move == True:
                #Speed profile
                self.state = SM_APPROACHING
                
                if self.target_z < self.des_accel_distance:
                    if self.target_z <= self.goal_tolerance:
                        self.current_speed = 0.0
                        self.state = SM_ARRIVED
                    else:
                        self.current_speed = self.Kp * self.target_z 
                else:
                    if self.current_speed < self.linear_max:
                        self.current_speed = self.current_speed + self.acel
                        
                    else:
                         self.current_speed = self.linear_max

        else: 
              self.state = SM_ARRIVED

        if self.state == SM_WAITING:
                msg.linear.x = 0.0
                msg.angular.z = 0.0
                #self.publisher_vel.publish(msg)
            
        elif self.state == SM_APPROACHING:
                
                        msg.linear.x = self.current_speed * math.exp(-(self.target_z**2)/0.7) 
                        msg.angular.z = self.angular_max*((2/(1+math.exp(self.target_x/0.2)))-1) 
            
                        self.publisher_vel.publish(msg)
                        #self.get_logger().info(f"Distancia al objetivo: {distance_to_target:.2f}, Posición del robot: {self.robot_x:.2f}, {self.robot_y:.2f}")
                    #self.get_logger().info(self.move)

        elif self.state == SM_ARRIVED:
                msg.linear.x = 0.0
                msg.angular.z = 0.0
                self.publisher_vel.publish(msg)
                self.move = False
                #self.get_logger().info('FIN', throttle_duration_sec=2.0)
                self.stop_robot()
                #self.get_logger().info(f"Ruta completa: {self.path_traveled}")
                

    def stop_robot(self):
        self.publisher_vel.publish(Twist())

def main(args=None):
    rclpy.init(args=args)
    node = PathPlanner()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()