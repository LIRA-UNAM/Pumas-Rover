import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import PointStamped, Twist, Quaternion, Point
from std_msgs.msg import Float32, Bool, Int16
from rclpy.duration import Duration
from xarm_msgs.srv import SetInt16, SetInt16ById
import time
import math


SM_WAIT = 0      
SM_GO_FORWARD = 1  
SM_ROTATE = 2
SM_SEARCH_GOAL = 3
SM_GOAL_PURSUIT = 4
SM_GOAL_LEAVE = 5
SM_SAND_SEARCH = 6
SM_ROCK_SEARCH = 7
SM_ROCK = 8
SM_GO_START = 9
SM_START_PURSUIT = 10




class PathPlanner(Node):
    def __init__(self):
        super().__init__('path_planner')

        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.confirmed_sub = self.create_subscription(PointStamped, '/yolo/confirmed_object', self.detection_callback, 10)
        self.confirmed_goal = self.create_subscription(
             PointStamped,
             '/yolo/end_distance',
             self.confirmed_goal_callback,
             10)
        self.registered_rock_sub = self.create_subscription(PointStamped, '/registered_rock', self.registered_rock_callback, 10)

        #ros2 topic pub --once /sm_start std_msgs/msg/Bool "{data: true}"

        self.sm_start = self.create_subscription(Bool, '/sm_start', self.sm_start_callback, 10)


        # self.publisher_distance = self.create_publisher(Float32, '/distance_movement', 10)
        # self.publisher_angle = self.create_publisher(Float32, '/angle_movement',10)
        self.publisher_rockfollower = self.create_publisher(Bool, '/follow_rock', 10)
        self.publisher_goalfollower = self.create_publisher(Bool, '/follow_goal', 10)
        self.publisher_objective_movement = self.create_publisher (Point, '/objective_point', 10)
        self.publisher_objective_start = self.create_publisher (Point, '/objective_start', 10)
        

        self.publisher_stop_movement = self.create_publisher (Bool, '/stop_movement', 10)

        self.publisher_state = self.create_publisher(Int16, '/rover_state',10)
    
        #Tunning movement parameters
        self.foward_advance = 4.0 #meters
        self.angle_rotation = math.pi/2 #radians
        self.found_rocks = 0
        self.goal_reached = False

        self.init_time = time.time ()
        

        self.state = SM_WAIT
        self.movement_finished = False

        self.timer = self.create_timer(0.05, self.machine_loop)

    def sm_start_callback(self,msg):
        if msg.data == True:
            self.state = SM_GO_FORWARD
            self.movement_finished = True
            self.get_logger().info("State Machine Started")
        

    def cmd_vel_callback(self, msg):
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        if linear_x == 0.0 and angular_z == 0.0:
            #time.sleep(0.3)
            self.movement_finished = True #It will check when the robot finished any of the movement nodes


    def detection_callback (self,msg):
        #Forget everything and go for that rock #<#)/
            if self.goal_reached:
                self.stop_movement()
                self.state = SM_ROCK

    def confirmed_goal_callback (self,msg):
            self.stop_movement()
            self.state = SM_GOAL_PURSUIT

    def registered_rock_callback (self,msg):
        self.found_rocks += 1

    def machine_loop(self):
        if self.state == SM_WAIT or not self.movement_finished:
            #self.get_logger().info('Wating')
            return

        elif self.state == SM_GO_FORWARD:
            self.first_alaign = False
            self.movement(5.0, 0.0) #We just want to publish the distance once by how the objective movement node is designed
            self.get_logger().info('GO FOWARD ! #<#)/')
            self.state = SM_ROTATE
        
        elif self.state == SM_ROTATE:
            self.movement(1.0, -3.0) #Negative is used for right desplacement
            self.get_logger().info('ROTATING RIGHT c:<')
            self.state = SM_SEARCH_GOAL

        elif self.state == SM_SEARCH_GOAL:
            self.movement(3.0, -3.0)
            self.get_logger().info('GO DIAGONAL FOR THAT GOAL >~<)')
            self.state = SM_SEARCH_GOAL

        elif self.state == SM_GOAL_PURSUIT:
            self.get_logger().info("GOAL PURSUIT, GO TRHOUGH HELL FOR IT ._.)")
            self.state = SM_GOAL_LEAVE

        elif self.state == SM_GOAL_LEAVE:
            self.movement(-3.0,3.0) #We just want to publish the angle once by how the objective movement node is designed
            self.get_logger().info('NOW GET OUT OF HERE -_-)')
            self.goal_reached = True
            self.state = SM_SAND_SEARCH

        elif self.state == SM_SAND_SEARCH:
            self.movement(-1.0,2.0) #We just want to publish the distance once by how the objective movement node is designed
            self.get_logger().info('MMMM ROCKS')
            self.state = SM_ROCK_SEARCH

        elif self.state == SM_ROCK_SEARCH:
            self.movement(-2.0,1.0) #We just want to publish the distance once by how the objective movement node is designed
            self.get_logger().info('WHERE ARE THOSE FREAKING ROCKS 0_o)?')
            if self.found_rocks < 3 or time.time()-self.init_time < 480:
                self.state = SM_SAND_SEARCH
            else:
                self.state = SM_GO_START

        elif self.state == SM_GO_START:
            self.get_logger().info('MISSION IS OVER, LETS GO HOME BUDY U_U)/')
            self.state = SM_GO_START


        elif self.state == SM_ROCK:
            self.get_logger().info("ROCK PURSUIT, GO TRHOUGH HEAVEN FOR IT ._.)")
            self.rock_movement()
            self.state = SM_ROCK_SEARCH


        elif self.state == SM_START_PURSUIT:
            self.get_logger().info('GOAL DETECTED, EXIT IS OUR HANDS #<#)/')
            self.stop_movement()

    def stop_movement(self):
        self.publisher_stop_movement.publish(Bool(data = True))
        self.movement_finished = True


    def movement (self, x,y):
        point = Point()
        point.x = x
        point.y = y
        self.publisher_objective_movement.publish(point)
        self.movement_finished = False

    def rock_movement(self):
        time.sleep(0.2)
        self.publisher_rockfollower.publish(Bool(data=True))
        self.movement_finished = False

    def start_movement (self):
        point = Point()
        point.x = 0.0
        point.y = 0.0
        self.publisher_objective_start.publish(point)
        self.movement_finished = False



def main(args=None):
    rclpy.init(args=args)
    node = PathPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()