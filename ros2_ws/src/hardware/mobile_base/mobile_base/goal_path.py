import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import PointStamped, Twist, Point, PoseStamped, Pose
from std_msgs.msg import Float32
from nav_msgs.msg import Path
from rclpy.duration import Duration
import time
import math
import numpy
import heapq

   

class PathPlanner(Node):
    def __init__(self):
        super().__init__('path_planner')
        

        #ros2 topic pub --once /goal geometry_msgs/msg/Point "{x: 4.0, y: 0.0, z: 0.0}"

        #ros2 topic pub --once /start geometry_msgs/msg/Point "{x: 0.0, y: 0.0, z: 0.0}"


        self.subscription = self.create_subscription(
            Point,
            'goal',
            self.goal_callback,
            10)
        
        self.subscription = self.create_subscription(
            Point,
            'start',
            self.start_callback,
            10)
        # self.subscription_yaw = self.create_subscription(
        #     Float32,
        #     'yaw',
        #     self.yaw_callback,
        #     10
        # )
        self.pub_path = self.create_publisher(Path, '/path_planning/path', 10)
        
            
        
        
        
        self.goal_x = 0.0
        self.goal_y = 0.0
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.goal = False
        self.start = False

        #Para el mapa
        self.resolution = 0.1
        self.width = 2 #5 metros de prueba
        self.height = 2 #5 metros de prueba

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        
        #Parael path



        self.prev_x = 0.0
        self.prev_y = 0.0
        self.prev_theta = 0.0

        self.path = []
        self.path_map = []

        self.msg_path = Path()

        self.w1 = 2.0
        self.w2 = 0.07
        self.steps = 10000

        self.get_logger().info(f"Path Planner node initialized")
        self.timer = self.create_timer(0.05, self.control_loop)
        
        
        



    def goal_callback(self, msg):
        
        self.target_x = msg.x
        self.target_y = msg.y
        self.last_msg_time = time.time()
        self.goal = True
        self.get_logger().info(f"Meta recibida: x={self.target_x}, y={self.target_y}")

        
    def start_callback(self, msg):
        
        self.robot_x = msg.x
        self.robot_y = msg.y
        self.last_msg_time = time.time()
        self.start = True
        self.get_logger().info(f"Inicio recibido: x={self.robot_x}, y={self.robot_y}")
        
    # def yaw_callback(self,msg):
    #     self.yaw_angle = msg.data
    #     self.get_logger().info(f"Angulo recibido: yaw={self.yaw_angle}")


    def a_star(self, start_r, start_c, goal_r, goal_c, use_diagonals):

        height = abs(goal_r - start_r)+1
        width = abs(goal_c - start_c)+1

        in_open_list   = numpy.full((height, width), False)
        in_closed_list = numpy.full((height, width), False)
        g_values       = numpy.full((height, width), float("inf"))
        f_values       = numpy.full((height, width), float("inf"))
        parent_nodes   = numpy.full((height, width, 2), -1)
        open_list = []
        if use_diagonals: #Every adjacent node has: [row_offset, col_offset, cost]
            adjacents = [[1,0,1],[0,1,1],[-1,0,1],[0,-1,1], [1,1,1.414], [-1,1,1.414], [-1,-1,1.414],[1,-1,1.414]]
        else:
            adjacents = [[1,0,1],[0,1,1],[-1,0,1],[0,-1,1]]

        heapq.heappush(open_list, (0, [start_r, start_c]))
        in_open_list[start_r, start_c] = True
        g_values    [start_r, start_c] = 0
        [row, col]= [start_r, start_c]   #Current node

        while len(open_list) > 0 and [row,col] != [goal_r,goal_c]:
            #agarra el nodo con más f de la OL
            current_node = heapq.heappop(open_list)[1]
            row,col = current_node
            in_closed_list[row, col] = True
            for r,c,cost in adjacents:
                #print (r,c,cost)
                neighbour_r,neighbour_c = row+r,col+c
                if neighbour_r < 0 or neighbour_c < 0 or neighbour_r >= height or neighbour_c>=width or in_closed_list[neighbour_r,neighbour_c]:
                    continue
                g_new_value = g_values[row,col] + cost + 0 #cost_map[neighbour_r,neighbour_c]
                if use_diagonals:
                    heuristic = math.sqrt(((goal_r-neighbour_r)**2)+((goal_c-neighbour_c)**2))
                    #Distancia euclidiana
                else:
                    heuristic = abs(goal_r-neighbour_r)+abs(goal_c-neighbour_c)
                    #Distancia de Manhattan
                f_new_value = g_new_value + heuristic
                if g_new_value < g_values[neighbour_r,neighbour_c]:
                    g_values[neighbour_r,neighbour_c] = g_new_value
                    f_values[neighbour_r,neighbour_c] = f_new_value
                    parent_nodes [neighbour_r,neighbour_c] = [row,col]

                    if in_open_list[neighbour_r,neighbour_c] == False:
                        in_open_list[neighbour_r,neighbour_c] == True
                        heapq.heappush(open_list, (f_values[neighbour_r,neighbour_c], [neighbour_r, neighbour_c]))
                    
        
        path = []
        while parent_nodes[goal_r, goal_c][0] != -1:
            path.insert(0, [goal_r, goal_c])
            [goal_r, goal_c] = parent_nodes[goal_r, goal_c]
        return path #Esta devolviendo casillas, multiplicar por resolución para obtener coordenadas reales

#Restar la posición actual para el algoritmo y luego sumarsela

#Restar la posición actual a ambos, eso los desplazara en el mapa y dejara la posicon actual en el 0,0 del mapa, si el objetivo tiene un negativo, pasalo a positivo y luego recuerdalo

#La ruta seran puro puntos positivos, multiplica a negativo aquellos que antes pasaste a positivo, luego vuelve a sumar la posición de inicio

    def stop_robot(self):
        self.publisher.publish(Twist())
    
    def control_loop(self):
      
            if self.goal and self.start:
                #self.get_logger().info(f"Iniciando planificación de ruta")
                goal_x, goal_y = self.get_absolute_points()
                self.path = self.a_star(0, 0, int(goal_y/self.resolution), int(goal_x/self.resolution),use_diagonals=False)
                coordinates_path = self.get_path_coordinates()
                self.get_logger().info(f"Ruta calculada: {self.path}, Ruta normal: {coordinates_path}")
                smooth_path = self.smooth_path (numpy.asarray([[positionx, positiony] for positionx,positiony in coordinates_path]),self.w1,self.w2,self.steps)
                self.get_logger().info (f"Ruta suavizada:")
                for x,y in smooth_path:
                    self.get_logger().info(f"x:{x:.2f}, y:{y:.2f}")
                self.goal = False
                self.start = False
                self.publish_path(smooth_path)
                #self.pub_path.publish(self.msg_path)
                



    def get_absolute_points (self):
        goal_x = abs(self.target_x - self.robot_x)
        goal_y = abs(self.target_y - self.robot_y)
        return goal_x, goal_y
    
    def get_path_coordinates(self):
        path_coordinates = []
        diference_x = self.target_x-self.robot_x
        diference_y = self.target_y-self.robot_y
        x_transform = 1
        y_transform = 1
        if diference_x != 0:
            x_transform = (diference_x)/abs(diference_x)
        if diference_y != 0:
            y_transform = (diference_y)/abs(diference_y)
        for cell in self.path:
            x = cell[1] * self.resolution*x_transform + self.robot_x
            y = cell[0] * self.resolution*y_transform + self.robot_y
            path_coordinates.append((x, y))
        return path_coordinates
    
    def smooth_path(self, Q, w1, w2, max_steps):
        P = numpy.copy(Q)
        tol     = 0.00001                   
        nabla   = numpy.full(Q.shape, float("inf"))
        epsilon = 0.1
        steps=0
        nabla[0], nabla[-1] = 0,0
        while numpy.linalg.norm(nabla)> tol*len(P) and max_steps >0:
            for i in range(1, len(Q)-1):
                nabla[i]=w1*(2*P[i] - P[i-1] - P[i+1]) + w2*(P[i] - Q[i])
            P = P - epsilon*nabla
            max_steps -=1                                            
        
        return P
    
    def publish_path(self, path_coordinates):
        self.msg_path.header.frame_id = "map"
        self.msg_path.header.stamp = self.get_clock().now().to_msg()
        self.msg_path.poses = []
        for x, y in path_coordinates:
            pose = PoseStamped()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            self.msg_path.poses.append(pose)
        self.pub_path.publish(self.msg_path)
        
            


def main(args=None):
    rclpy.init(args=args)
    node = PathPlanner()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()