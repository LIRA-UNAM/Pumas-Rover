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

        #ros2 topic pub --once /goal geometry_msgs/msg/Point "{x: 1.0, y: 0.0, z: 0.0}"


        self.subscription = self.create_subscription(
            Point,
            'goal',
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
        self.get_logger().info(f"Meta recibida: x={self.target_x}, y={self.target_y}")
        self.get_logger().info("Planificando ruta...")
        path = self.a_star(int((self.robot_y-self.origin_y)/self.resolution), int((self.robot_x-self.origin_x)/self.resolution), int((self.target_y-self.origin_y)/self.resolution), int((self.target_x-self.origin_x)/self.resolution), numpy.reshape(numpy.asarray(self.map_data), (int(self.height/self.resolution), int(self.width/self.resolution))), numpy.zeros((int(self.height/self.resolution), int(self.width/self.resolution))), False)



    def a_star(self, start_r, start_c, goal_r, goal_c, grid_map, cost_map, use_diagonals):
        [height, width] = grid_map.shape
        in_open_list   = numpy.full(grid_map.shape, False)
        in_closed_list = numpy.full(grid_map.shape, False)
        g_values       = numpy.full(grid_map.shape, float("inf"))
        f_values       = numpy.full(grid_map.shape, float("inf"))
        parent_nodes   = numpy.full((grid_map.shape[0],grid_map.shape[1],2),-1)
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
                if neighbour_r < 0 or neighbour_c < 0 or neighbour_r >= height or neighbour_c>=width or in_closed_list[neighbour_r,neighbour_c] or grid_map[neighbour_r, neighbour_c] > 50 or grid_map[neighbour_r, neighbour_c] == -1:
                    continue
                g_new_value = g_values[row,col] + cost + cost_map[neighbour_r,neighbour_c]
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
                    
            #print (row,col,goal_r,goal_c)



        #
        # TODO:
        # Implement the A* algorithm for path planning
        # Map is considered to be a 2D array and start and goal positions
        # are given as row-col pairs. You can follow these steps:
        #
        # WHILE open list is not empty and current is different from goal:
        #     Get current node [row,col] from open list (see heapq.heappop function)
        #     Mark current node as 'in_closed_list'
        #     For [r,c,cost] in adjacent nodes:
        #         Get r,c indices of neighbours of current node (check content of adjacents)
        #         Discard if r,c is out of map, occupied, unknonw or in closed list, and continue
        #         get a g-value g as: g-value of current node + dist + cost of neighbour r,c
        #         Calculate heuristic 
        #         Calculate f-value
        #         IF g < g_vaprint (row,col,goal_r,goal_c)lue of neighbour r,c:
        #             set g as g_value of neighbour r,c
        #             set f as f_value of neighbour r,c
        #             SET current node row,col as parent of neighbour r,c
        #         If neighbour r,c is not in open list:
        #             mark r,c as 'in_open_list'
        #             add r,c to open list (check heapq.heappush)
        #
        
        #
        # END OF WHILE
        #
        
        path = []
        while parent_nodes[goal_r, goal_c][0] != -1:
            path.insert(0, [goal_r, goal_c])
            [goal_r, goal_c] = parent_nodes[goal_r, goal_c]
        return path
    

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