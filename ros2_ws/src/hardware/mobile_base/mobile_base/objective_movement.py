import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import PointStamped, Twist, Point
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
            
        
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        
        self.state = SM_WAITING
        self.last_msg_time = time.time()
        self.target_x = 0.0
        self.target_y = 0.0
        self.move = False

        #Para el mapa
        self.resolution = 0.1
        self.width = 2 #5 metros de prueba
        self.height = 2 #5 metros de prueba

        self.origin_x = -self.width* self.resolution / 2.0
        self.origin_y = -self.height * self.resolution / 2.0

        self.map_data = [-1] * (int(self.width/self.resolution) * int(self.height/self.resolution))  # Quien sabe, -1 es desconocido, 0 es libre, 100 es ocupado

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0

        # Marcar el centro como libre para el inicio
        self.map_data[int(self.height/2) * int(self.width/self.resolution) + int(self.width/2)] = 0
        #Parael path



        self.prev_x = 0.0
        self.prev_y = 0.0
        self.prev_theta = 0.0

        self.path = []
        self.path_map = []
        
        
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info('Path Planner para Rover Lunar iniciado.')



    def target_callback(self, msg):
        
        self.target_x = msg.x
        self.target_y = msg.y
        self.last_msg_time = time.time()
        self.move = True
        self.get_logger().info(f"Nuevo objetivo recibido: x={self.target_x}, y={self.target_y}")




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

            if self.robot_x-self.prev_x > 0.1 or self.robot_y-self.prev_y > 0.1:
                
                self.path.append((self.robot_x, self.robot_y))
                mx, my = self.world_to_map(self.robot_x, self.robot_y)
                self.map_data[my * self.width + mx] = 0  # Marcar como libre en el mapa
                self.path_map.append((mx, my))
                self.prev_x = self.robot_x
                self.prev_y = self.robot_y
                self.prev_theta = self.robot_theta


                

        except Exception as e:
            self.get_logger().warn(f"No TF available: {str(e)}")





    def control_loop(self):
        if self.move:
            # now = time.time()
            msg = Twist()

        # 1. VERIFICACIÓN DE TIMEOUT 
            # if (now - self.last_msg_time) > 1.0:
            #     self.state = SM_WAITING
            #     self.stop_robot()
            #     self.get_logger().info('Esperando nuevo objetivo...')
            #     return

            distance_to_target = math.sqrt((self.target_x - self.robot_x) ** 2 + (self.target_y - self.robot_y) ** 2)
            error_angle = math.atan2(self.target_y - self.robot_y, self.target_x - self.robot_x) - self.robot_theta
            error_angle = (error_angle + math.pi) % (2 * math.pi) - math.pi  # Normalizar a [-pi, pi]
        # 2. MÁQUINA DE ESTADOS
            if distance_to_target > 0.1: 
                self.state = SM_APPROACHING
                
            elif distance_to_target <= 0.1:
                self.state = SM_ARRIVED

        # 3. ACCIONES POR ESTADO

            
            if self.state == SM_APPROACHING:


                #Falta perfil de velocidad para disminuir velocidad al acercarse.
                msg.linear.x = 1.2 * math.exp(-(error_angle**2)/0.8)  # Velocidad proporcional a la distancia (máx 0.8 m/s)
                msg.angular.z = 0.8*((2/(1+math.exp(-error_angle/0.4)))-1) #Los dynamixels tambalean mucho tratando de llegar a una velocidad, limitarlos en el mobile base
            
            
                self.publisher.publish(msg)
                self.get_logger().info(f"Velocidad publicada: linear.x={msg.linear.x:.2f}, angular.z={msg.angular.z:.2f}")
                self.get_logger().info(f"Distancia al objetivo: {distance_to_target:.2f}, Posición del robot: {self.robot_x:.2f}, {self.robot_y:.2f}")
                #self.get_logger().info(self.move)

            elif self.state == SM_ARRIVED:
                msg.linear.x = 0.0
                msg.angular.z = 0.0
                self.publisher.publish(msg)
                self.move = False
                self.get_logger().info('FIN', throttle_duration_sec=2.0)
                self.stop_robot()
                self.get_logger().info(f"Ruta completa: {self.path}")
                self.get_logger().info(
                    f"Fin de la ruta x: {self.robot_x:.2f}, y: {self.robot_y:.2f}, theta: {self.robot_theta:.2f}"
                    )
                #self.get_logger().info(f"Mapa de la ruta: {self.map_data}")
        else:
            self.stop_robot()
            #self.get_logger().info('Esperando nuevo objetivo...')

    def stop_robot(self):
        self.publisher.publish(Twist())

    def world_to_map(self, x, y):
        mx = int((x - self.origin_x) / self.resolution)
        my = int((y - self.origin_y) / self.resolution)
        return mx, my


def main(args=None):
    rclpy.init(args=args)
    node = PathPlanner()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()