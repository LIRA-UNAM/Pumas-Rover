import rclpy 
from rclpy.node import Node 
from std_msgs.msg import Int32 
from std_srvs.srv import Trigger 
import time 

class GripperSoloTest(Node): 
    def __init__(self): 
        super().__init__('gripper_node') 
        self.adc_val = 0 
        self.last_adc_val = 0 
        self.current_angle = 0 
        self.is_active = False 
        self.adc_sub = self.create_subscription(Int32, '/gripper_adc', self.adc_cb, 10) 
        self.cmd_pub = self.create_publisher(Int32, '/gripper_cmd', 10) 
        self.create_service(Trigger, '/gripper/smart_grab', self.grab_srv_cb) 
        self.create_service(Trigger, '/gripper/open', self.open_srv_cb) 
        self.timer = self.create_timer(0.1, self.control_loop) 
        self.get_logger().info("NODO GRIPPER LISTO") 

    def adc_cb(self, msg): self.adc_val = msg.data 

    def open_srv_cb(self, request, response): 
        self.is_active = False 
        self.current_angle = 0 
        msg = Int32(); msg.data = 0 
        self.cmd_pub.publish(msg) 
        response.success = True; return response 

    def grab_srv_cb(self, request, response): 
        time.sleep(5) 
        self.current_angle = 0; self.is_active = True 
        response.success = True; return response 

    def control_loop(self): 
        if not self.is_active: return 
        self.current_angle += 2 
        if self.current_angle > 180: 
            self.is_active = False; return 
        msg = Int32(); msg.data = self.current_angle 
        self.cmd_pub.publish(msg) 
        if self.current_angle > 20: 
            diff = abs(self.adc_val - self.last_adc_val) 
            if diff < 5: 
                msg.data = self.current_angle - 3 
                self.cmd_pub.publish(msg) 
                self.is_active = False 
        self.last_adc_val = self.adc_val 

def main(): 
    rclpy.init(); node = GripperSoloTest(); rclpy.spin(node); rclpy.shutdown()