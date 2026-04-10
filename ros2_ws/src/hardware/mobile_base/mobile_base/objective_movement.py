import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import PointStamped, Twist, Quaternion
from std_msgs.msg import Float32, Bool
from rclpy.duration import Duration
import time
import math


SM_WAITING = 0      
SM_APPROACHING = 1  
SM_ROTATING = 2
SM_ARRIVED = 3  

class PathPlanner(Node):
    def __init__(self):
        super().__init__('path_planner')
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(0.05, self.read_tf)  #0.1 anteriormente

       
        #ros2 topic pub --once /distance_movement std_msgs/msg/Float32 "{data: 1.0}"
        #ros2 topic pub --once /angle_movement std_msgs/msg/Float32 "{data: 1.57}"


        self.subscription_distance = self.create_subscription(
            Float32,
            '/distance_movement',
            self.distance_callback,
            10)
        
        self.subscription_angle = self.create_subscription(
            Float32,
            '/angle_movement',
            self.angle_callback,
            10
        )
        self.subscription_stop = self.create_subscription(
            Bool,
            '/stop_movement',
            self.stop_callback,
            10
        )
            
        
        self.publisher_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.publisher_finish = self.create_publisher(Bool, '/movement_finished',10)
        
        
        self.state = SM_WAITING
        self.last_msg_time = time.time()
        self.target_x = 0.0
        self.target_y = 0.0
        self.prev_target_x = 0.0
        self.prev_target_y = 0.0
        self.move = False

        #For position
        self.resolution = 0.1

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        self.linear_max = 0.5
        self.angular_max = 0.5
        #Speed profile parameters
        self.des_accel_distance = 0.4
        self.acel = 0.05
        self.current_speed = 0.0
        self.goal_tolerance = 0.05
        self.target_tolerance = 0.15
        self.Kp = 1.2  #Max 1.2 for 0.5 m/s
        self.alpha_vl = 0.8
        self.beta_w = 0.2


        
        #Parael path



        self.prev_x = 0.0
        self.prev_y = 0.0
        self.prev_theta = 0.0

        
        self.path_traveled = []
        
        
        
        
        self.timer = self.create_timer(0.05, self.control_loop)
       



    def distance_callback(self, msg):
        
        self.distance_movement = msg.data
        self.target_x = self.robot_x + self.distance_movement * math.cos(self.robot_theta)
        self.target_y = self.robot_y + self.distance_movement * math.sin(self.robot_theta)
        self.state = SM_APPROACHING
        self.last_msg_time = time.time()

        self.move = True
        
        

    def angle_callback(self, msg):
        
        self.distance_movement = msg.data
        self.state = SM_ROTATING
        self.last_msg_time = time.time()

        self.move = True

    def stop_callback (self,msg):
        self.move = False
        self.state = SM_ARRIVED
        self.last_msg_time = time.time()
        
        
        


    def read_tf(self):
        try:
            t = self.tf_buffer.lookup_transform(
                "odom",       # frame_id
                "base_link",  # frame_id child
                rclpy.time.Time()
            )

            self.robot_x = t.transform.translation.x
            self.robot_y = t.transform.translation.y

            q = t.transform.rotation

            # quathernion to euler (yaw)
            self.robot_theta = 2 * math.atan2(q.z, q.w)

            #print (f"x: {self.robot_x:.2}, y: {self.robot_y:.2}, theta: {self.robot_theta:.2}")

            if self.robot_x-self.prev_x > 0.1 or self.robot_y-self.prev_y > 0.1:
                
                self.path_traveled.append((self.robot_x, self.robot_y))
                self.prev_x = self.robot_x
                self.prev_y = self.robot_y
                


                

        except Exception as e:
            self.get_logger().warn(f"No TF available: {str(e)}")





    def control_loop(self):
            
            msg = Twist()
            msg_finish = Bool()
            
            # MÁQUINA DE ESTADOS
            if (self.target_x == self.prev_target_x and self.target_y == self.prev_target_y) or self.prev_theta == self.robot_theta or time.time() - self.last_msg_time > 10.0:
                self.state = SM_WAITING
                #self.get_logger().info('Esperando nuevo objetivo...')
            elif self.move == True:
                
                if self.state == SM_APPROACHING:

                    distance_to_goal = math.sqrt((self.target_x - self.robot_x) ** 2 + (self.target_y - self.robot_y) ** 2)
                    if distance_to_goal < self.des_accel_distance:
                        if distance_to_goal < self.goal_tolerance:
                            self.current_speed = 0.0
                            self.state = SM_ARRIVED
                        else:
                            self.current_speed = self.Kp * distance_to_goal
                    
                    else:
                        if self.current_speed < self.linear_max:
                            self.current_speed = self.current_speed + self.acel
                        else:
                            self.current_speed = self.linear_max

                    v,w = self.calculate_control(self.robot_x, self.robot_y, self.robot_theta, self.target_x, self.target_y, self.alpha_vl, self.beta_w, self.linear_max, self.angular_max)
                    msg.linear.x = v
                    msg.angular.z = w
                    self.publisher_vel.publish(msg)

                elif self.state == SM_ROTATING:
                    error_angle = math.atan2(self.target_y - self.robot_y, self.target_x - self.robot_x) - self.robot_theta
                    error_angle = (error_angle + math.pi)%(2*math.pi) - math.pi
                    if abs(error_angle) > self.goal_tolerance:
                        msg.linear.x = 0.0
                        msg.angular.z = self.angular_max*(2/(1 + math.exp(-error_angle/self.beta_w)) - 1)
                    else:
                        self.state = SM_ARRIVED
                
            else:
                self.state = SM_ARRIVED
            

            # ACCIONES POR ESTADO
            if self.state == SM_WAITING:
                msg.linear.x = 0.0
                msg.angular.z = 0.0
                
            


            elif self.state == SM_ARRIVED:
                msg.linear.x = 0.0
                msg.angular.z = 0.0
                msg_finish.data = True
                self.publisher_finish.publish(msg_finish)
                self.publisher_vel.publish(msg)
                self.move = False
                #self.get_logger().info('FIN', throttle_duration_sec=2.0)
                self.stop_robot()
                #self.get_logger().info(f"Ruta completa: {self.path_traveled}")
                self.get_logger().info(
                    f"Fin de la ruta x: {self.robot_x:.2f}, y: {self.robot_y:.2f}, theta: {self.robot_theta:.2f}"
                    )
                
                self.prev_target_x = self.target_x
                self.prev_target_y = self.target_y
                self.prev_theta = self.robot_theta
                #self.get_logger().info(f"Mapa de la ruta: {self.map_data}")
        
            #self.get_logger().info('Esperando nuevo objetivo...')

    def calculate_control(self, robot_x, robot_y, robot_angle, goal_x, goal_y, alpha, beta, v_max, w_max):
        v,w = 0,0
        error_angle = math.atan2(goal_y - robot_y, goal_x - robot_x) - robot_angle
        error_angle = (error_angle + math.pi)%(2*math.pi) - math.pi
        v = v_max*math.exp(-error_angle*error_angle/alpha)
        w = w_max*(2/(1 + math.exp(-error_angle/beta)) - 1)
                
        return [v,w]

    def stop_robot(self):
        self.publisher_vel.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = PathPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()