import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import PointStamped, Twist, Quaternion
from std_msgs.msg import Float32, Bool
from rclpy.duration import Duration
from xarm_msgs.srv import SetInt16, SetInt16ById
import time
import math


SM_WAIT = 0      
SM_GO_FOWARD = 1  
SM_ROTATE = 2
SM_ROCK = 3
SM_SEARCHING = 4
SM_GOAL = 5

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
        self.arm_stop = self.create_subscription(Bool, '/arm_stop', self.arm_stop_callback, 10)

        #ros2 topic pub --once /sm_start std_msgs/msg/Bool "{data: true}"

        self.sm_start = self.create_subscription(Bool, '/sm_start', self.sm_start_callback, 10)



        self.publisher_distance = self.create_publisher(Float32, '/distance_movement', 10)
        self.publisher_angle = self.create_publisher(Float32, '/angle_movement',10)
        self.publisher_rockfollower = self.create_publisher(Bool, '/follow_rock', 10)
        self.publisher_goalfollower = self.create_publisher(Bool, '/follow_goal', 10)
        self.publisher_arm_searcher = self.create_publisher(Bool, '/arm_searcher', 10)

        self.publisher_stop_movement = self.create_publisher (Bool, '/stop_movement', 10)

    
        #Tunning movement parameters
        self.foward_advance = 1.0 #meters
        self.angle_rotation = math.pi/2 #radians
        self.found_rocks = 0

        self.state = SM_WAIT
        self.movement_finished = False

        self.timer = self.create_timer(0.05, self.machine_loop)

    def sm_start_callback(self,msg):
        if msg.data == True:
            self.state = SM_GO_FOWARD
            self.movement_finished = True
            self.get_logger().info("State Machine Started")
        

    def cmd_vel_callback(self, msg):
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        if linear_x == 0.0 and angular_z == 0.0:
            #time.sleep(0.3)
            self.movement_finished = True #It will check when the robot finished any of the movement nodes
            
    def arm_stop_callback(self,msg):
        self.movement_finished = True


    def detection_callback (self,msg):
        #Forget everything and go for that rock #<#)/

            self.found_rocks += 1
            self.stop_movement()
            self.state = SM_ROCK

    def machine_loop(self):
        if self.state == SM_WAIT or not self.movement_finished:
            #self.get_logger().info('Wating')
            return

        elif self.state == SM_GO_FOWARD:
            self.go_foward() #We just want to publish the distance once by how the objective movement node is designed
            self.get_logger().info('GO FOWARD! #<#)/')
            self.state = SM_ROTATE

        elif self.state == SM_ROTATE:
            self.rotate() #We just want to publish the angle once by how the objective movement node is designed
            self.get_logger().info('ROTATING c:<')
            self.state = SM_SEARCHING

        elif self.state == SM_ROCK:
            self.get_logger().info("ROCK PURSUIT GO TRHOUGH HEAVEN FOR IT ._.)")
            self.rock_movement()
            self.state = SM_SEARCHING

        elif self.state == SM_SEARCHING:
            self.get_logger().info('Looking for rocks -_-)')
            self.arm_search()
            self.state = SM_GO_FOWARD

        elif self.state == SM_GOAL:
            self.get_logger().info('Goal reached! Stopping movement.')
            self.stop_movement()

    def stop_movement(self):
        self.publisher_stop_movement.publish(Bool(data = True))
        self.movement_finished = True

    def go_foward(self):
        
        msg = Float32()
        msg.data = self.foward_advance
        self.publisher_distance.publish(msg)
        self.movement_finished = False
        #self.get_logger().info('Waiting in peace n_n)')

    def rotate (self):
        msg = Float32()
        msg.data = self.angle_rotation
        self.publisher_angle.publish(msg)
        self.movement_finished = False
        #self.get_logger().info('Waiting in peace n_n)')

    def arm_search (self):
        # self.enable_motion()
        # self.set_mode()
        # self.set_state()
        msg = Bool()
        msg.data = True
        self.publisher_arm_searcher.publish(msg)
        self.movement_finished = False
        #self.get_logger().info('Waiting in peace n_n)')

    def rock_movement(self):
        time.sleep(0.2)
        self.publisher_rockfollower.publish(Bool(data=True))
        self.movement_finished = False
        
        



    


def main(args=None):
    rclpy.init(args=args)
    node = PathPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()