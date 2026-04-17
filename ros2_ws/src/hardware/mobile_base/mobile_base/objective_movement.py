import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import PointStamped, Twist, Point, PoseStamped, Pose
from nav_msgs.msg import Path
from rclpy.duration import Duration
import time
import math


SM_WAITING = 0      
SM_APPROACHING = 1  
SM_ARRIVED = 2      

class PathPlanner(Node):
    def __init__(self):
        super().__init__('path_planner')
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(0.05, self.read_tf)  #0.1 anteriormente

        #ros2 topic pub --once /objective_point geometry_msgs/msg/Point "{x: 1.0, y: 0.0, z: 0.0}"


        self.subscription = self.create_subscription(
            Point,
            'objective_point',
            self.target_callback,
            10)
        
        self.subscription = self.create_subscription(Path,'/path_planning/path', self.path_callback, 10)
            
        
        self.publisher_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.publisher_goal = self.create_publisher(Point, 'goal', 10)
        self.publisher_start = self.create_publisher(Point, 'start', 10)
        
        self.msg_start = Point()
        self.msg_goal = Point()
        
        self.state = SM_WAITING
        self.last_msg_time = time.time()
        self.target_x = 0.0
        self.target_y = 0.0
        self.prev_target_x = 0.0
        self.prev_target_y = 0.0
        self.move = False

        #Para el mapa
        self.resolution = 0.1
        # self.width = 2 #5 metros de prueba
        # self.height = 2 #5 metros de prueba

        # self.origin_x = -self.width* self.resolution / 2.0
        # self.origin_y = -self.height * self.resolution / 2.0

        # self.map_data = [-1] * (int(self.width/self.resolution) * int(self.height/self.resolution))  # Quien sabe, -1 es desconocido, 0 es libre, 100 es ocupado
        # # Marcar el centro como libre para el inicio
        # self.map_data[int(self.height/2) * int(self.width/self.resolution) + int(self.width/2)] = 0


        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        self.linear_max = 0.7
        self.angular_max = 0.7
        #Speed profile parameters
        self.des_accel_distance = 0.4
        self.acel = 0.05
        self.current_speed = 0.0
        self.goal_tolerance = 0.05
        self.target_tolerance = 0.15
        self.Kp = 1.2  #Max 1.2 for 0.5 m/s
        self.alpha_vl = 0.8
        self.beta_w = 0.5


        
        #Parael path



        self.prev_x = 0.0
        self.prev_y = 0.0
        self.prev_theta = 0.0

        self.path = []
        self.path_traveled = []
        self.number_point = 0
        self.path_map = []
        
        
        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info('Path Planner para Rover Lunar iniciado.')



    def target_callback(self, msg):
        #Provisionalmente hacer la devolución de ruta con tópicos, actualizar a un servicio o action
        self.target_x = msg.x
        self.target_y = msg.y
        self.last_msg_time = time.time()
        self.msg_goal.x = self.target_x
        self.msg_goal.y = self.target_y
        self.publisher_goal.publish(self.msg_goal)
        self.msg_start.x = self.robot_x
        self.msg_start.y = self.robot_y
        self.publisher_start.publish(self.msg_start)
        self.get_logger().info(f"Nuevo objetivo recibido: x={self.target_x}, y={self.target_y}")

    def path_callback(self, msg):
        self.path = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]
        self.get_logger().info(f"Nuevo path recibido con {self.path}")
        self.move = True
        #self.control_loop()  # Llamar al control loop para procesar el nuevo path
        



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
            
        
            
            # MÁQUINA DE ESTADOS
            if self.target_x == self.prev_target_x and self.target_y == self.prev_target_y:
                self.state = SM_WAITING
                self.number_point = 0
                #self.get_logger().info('Esperando nuevo objetivo...')
            elif self.move == True:
                #Speed profile
                self.state = SM_APPROACHING
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
            else:
                self.state = SM_ARRIVED

            msg = Twist()            
            
        
            # ACCIONES POR ESTADO
            if self.state == SM_WAITING:
                msg.linear.x = 0.0
                msg.angular.z = 0.0
                #self.publisher_vel.publish(msg)
            
            elif self.state == SM_APPROACHING:

                msg.linear.x, msg.angular.z = self.calculate_control(self.robot_x,self.robot_y,self.robot_theta,self.target_x,self.target_y,self.alpha_vl,self.beta_w,self.current_speed,self.angular_max)
                self.publisher_vel.publish(msg)
                self.get_logger().info(f"Vl={msg.linear.x:.2f}, W={msg.angular.z:.2f} Dist: {distance_to_goal:.2f}, Pos: {self.robot_x:.2f}, {self.robot_y:.2f}")
   

            elif self.state == SM_ARRIVED:
                msg.linear.x = 0.0
                msg.angular.z = 0.0
                self.publisher_vel.publish(msg)
                msg.linear.x = 0.0
                msg.angular.z = 0.0
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
                

    def calculate_control(self, robot_x, robot_y, robot_angle, goal_x, goal_y, alpha, beta, v_max, w_max):
        v,w = 0,0
        #
        # TODO:
        # Implement the control law given by:
        #
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