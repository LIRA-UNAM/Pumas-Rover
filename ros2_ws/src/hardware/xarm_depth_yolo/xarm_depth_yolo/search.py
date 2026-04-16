import rclpy
from rclpy.node import Node
from xarm_msgs.srv import MoveJoint, SetInt16, Call, SetInt16ById
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Bool
import time

class SearchNode(Node):
    def __init__(self):
        super().__init__('search_node')
        
        self.joint_client = self.create_client(MoveJoint, '/xarm/set_servo_angle')
        self.state_client = self.create_client(SetInt16, '/xarm/set_state')
        self.mode_client = self.create_client(SetInt16, '/xarm/set_mode')
        self.enable_client = self.create_client(SetInt16ById, '/xarm/motion_enable')
        self.error_client = self.create_client(Call, '/xarm/clean_error')
        
        self.create_subscription(JointState, '/xarm/joint_states', self.initial_pos_callback, 10)
        self.create_subscription(PointStamped, '/yolo/confirmed_object', self.detection_callback, 10)
        self.create_subscription(Bool, '/arm_searcher', self.start_callback, 10)
        self.create_subscription(Bool, '/search/resume', self.resume_callback, 10)

        self.follower_pub = self.create_publisher(Bool, '/follower/start', 10)

        self.sm_start = False
        self.state = "IDLE"
        self.idx = 0
        self.busy = False
        self.initial_check_done = False 
        
        self.HOME_JOINTS = [-1.5747, -0.2777, -0.0971, -0.0056, 0.1502, 0.0002]
        self.j5_normal, self.j5_look = -1.0594, -0.7461
        self.VEL_CADERA, self.VEL_LOOK = 0.35, 0.15 
        
        self.puntos_base = [
            [0.0, -0.2809, -0.1047, -0.0052, self.j5_normal, 0.0017],
            [-1.5707, -0.2809, -0.1047, -0.0052, self.j5_normal, 0.0017],
            [-3.1415, -0.2809, -0.1047, -0.0052, self.j5_normal, 0.0017]
        ]
        
        self.get_logger().info('SearchNode V4 Final Listo.')
        self.timer = self.create_timer(0.1, self.fsm_loop)

    def initial_pos_callback(self, msg):
        self.initial_check_done = True

    def hard_reset_sequence(self):
        self.error_client.call_async(Call.Request())
        req_en = SetInt16ById.Request(); req_en.id = 8; req_en.data = 1
        self.enable_client.call_async(req_en)
        req_mode = SetInt16.Request(); req_mode.data = 0
        self.mode_client.call_async(req_mode)
        req_state = SetInt16.Request(); req_state.data = 0
        self.state_client.call_async(req_state)

    def start_callback(self, msg):
        if msg.data and self.state == "IDLE":
            self.hard_reset_sequence()
            self.delay_start = time.time()
            self.state = "INIT_DELAY"

    def resume_callback(self, msg):
        if msg.data:
            self.get_logger().info('Follower terminó. Reanudando búsqueda...')
            self.sm_start = True
            self.state = "INIT_DELAY"
            self.delay_start = time.time()
            self.hard_reset_sequence()

    def detection_callback(self, msg):
        searching_states = ["MOVING", "LOOK", "RETRACT"]
        if self.sm_start and self.state in searching_states:
            self.get_logger().warn('¡ROCA DETECTADA! Suspendiendo búsqueda...')
            self.sm_start = False
            self.busy = False
            self.stop_arm()
            self.state = "GOING_HOME"

    def stop_arm(self):
        req = SetInt16.Request(); req.data = 4 
        self.state_client.call_async(req)

    def send_angles(self, angles, speed, is_home=False):
        self.busy = True
        req = MoveJoint.Request()
        req.angles, req.speed, req.acc = angles, speed, 2.0
        req.wait = False 
        self.joint_client.call_async(req)
        self.move_timeout = time.time()
        self.is_moving_to_home = is_home

    def fsm_loop(self):
        if not self.initial_check_done: return
        
        if self.busy:
            if time.time() - self.move_timeout > 5.0:
                self.busy = False
                if self.is_moving_to_home:
                    self.get_logger().info('En HOME. Avisando al Follower...')
                    self.follower_pub.publish(Bool(data=True))
                    self.state = "WAITING_FOR_FOLLOWER" 
            return

        if self.state == "INIT_DELAY":
            if time.time() - self.delay_start > 2.0:
                self.sm_start, self.state = True, "MOVING"
            return

        if self.state == "GOING_HOME":
            self.hard_reset_sequence()
            self.delay_start = time.time()
            self.state = "DELAY_HOME"
            return
            
        if self.state == "DELAY_HOME":
            if time.time() - self.delay_start > 1.0:
                self.send_angles(self.HOME_JOINTS, self.VEL_CADERA, is_home=True)
            return

        elif self.sm_start:
            if self.state == "MOVING":
                self.send_angles(self.puntos_base[self.idx], self.VEL_CADERA)
                self.state = "LOOK"
            elif self.state == "LOOK":
                pose = list(self.puntos_base[self.idx]); pose[4] = self.j5_look
                self.send_angles(pose, self.VEL_LOOK)
                self.state = "RETRACT"
            elif self.state == "RETRACT":
                self.send_angles(self.puntos_base[self.idx], self.VEL_LOOK)
                self.idx = (self.idx + 1) % len(self.puntos_base)
                self.state = "MOVING"

def main():
    rclpy.init(); node = SearchNode(); rclpy.spin(node); rclpy.shutdown()

if __name__ == '__main__':
    main()