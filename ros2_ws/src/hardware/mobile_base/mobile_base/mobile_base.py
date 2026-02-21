import rclpy
import time
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from tf2_ros.buffer import Buffer #tf2 = transformation for odometry and base_link
#from Rosmaster_Lib import Rosmaster
from mobile_base import roboclaw_3
from dynamixel_sdk import *
#import roboclaw_3
#from roboclaw_3 import Roboclaw

#Adresses for dynamixel protocol 1.0
ADDR_TORQUE_ENABLE = 24
ADDR_GOAL_POSITION = 30
ADDR_PRESENT_POSITION = 36
LEN_GOAL_POSITION = 2 
PROTOCOL_VERSION = 1.0
BAUDRATE = 57600  
DEVICE_NAME = '/dev/ttyUSB0' 
TORQUE_ENABLE = 1  
TORQUE_DISABLE = 0 
NUM_SERVOS = 2

class MobileBaseNode(Node): 
    def __init__(self):
        super().__init__('mobile_base')

        # self.bot = Rosmaster('/dev/ttyUSB0')
        # self.bot.create_receive_threading()
        #
        self.ADDRESS = 0x80
        self.roboclaw_front = roboclaw_3.Roboclaw("/dev/ttyACM0", 115200)
        self.roboclaw_center = roboclaw_3.Roboclaw("/dev/ttyACM1", 115200)
        self.roboclaw_rear = roboclaw_3.Roboclaw("/dev/ttyACM2", 115200)
        self.roboclaw_front.Open()
        self.roboclaw_center.Open()
        self.roboclaw_rear.Open()

        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.tf_broadcaster = TransformBroadcaster(self)

        self.data = False
        self.notdata = 0
        self.limit = 5

        self.accel_max = 8000
        self.max_pwm = 60
        self.linear = 0.0
        self.angular = 0.0

        self.diameter = 0.107
        self.radius = self.diameter / 2.0
        self.width = 0.45 #Rover measure of left wheels to right wheels
        self.height = 0.35 #Rover measure of center wheels to front wheels
        self.ppr = 3500

        #3509

        self.meters_per_tick = (math.pi * self.diameter) / self.ppr

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        #For dynamixels
        self.goal_position = [2048,2048,2048,2048] #Initial position for each dynamixel 
        self.DXL_ID = [1,3,4,2] #ID for all 4 motors #1&3 = left wheels, 4&2 = right wheels

        #Preparation of dynamixels using the SDK Dynamixel
        self.port_handler = PortHandler(DEVICE_NAME)
        self.packet_handler = PacketHandler(PROTOCOL_VERSION)
        if not self.port_handler.openPort():
            self.get_logger().error('Failed to open the port!')
            return
        
        self.get_logger().info('Succeeded to open the port.')

        if not self.port_handler.setBaudRate(BAUDRATE):
            self.get_logger().error('Failed to set the baudrate!')
            return
        self.get_logger().info('Succeeded to set the baudrate.')

        self.groupSyncWrite = GroupSyncWrite(self.port_handler,self.packet_handler,ADDR_GOAL_POSITION,LEN_GOAL_POSITION)
        #Use of GroupSyncWrite for controlling 4 dynamixels using a single package

        self.setup_dynamixel(self.DXL_ID)

        #For the roboclaws
        self.roboclaw_front.ResetEncoders(self.ADDRESS)
        self.roboclaw_center.ResetEncoders(self.ADDRESS)
        self.roboclaw_rear.ResetEncoders(self.ADDRESS)

        #self.prev_enc = self.bot.get_motor_encoder()
        self.prev_enc1 = self.roboclaw_front.ReadEncM1(self.ADDRESS)
        self.prev_enc2 = self.roboclaw_front.ReadEncM2(self.ADDRESS)
        self.prev_enc1 = self.roboclaw_center.ReadEncM1(self.ADDRESS)
        self.prev_enc2 = self.roboclaw_center.ReadEncM2(self.ADDRESS)
        self.prev_enc1 = self.roboclaw_rear.ReadEncM1(self.ADDRESS)
        self.prev_enc2 = self.roboclaw_rear.ReadEncM2(self.ADDRESS)

        self.timer = self.create_timer(0.1, self.update_odometry) #0.05 anteriormente

        self.get_logger().info("Odometria con TF")

    def cmd_vel_callback(self, msg):
        self.linear = msg.linear.x
        self.angular = msg.angular.z
        self.data = True
        self.notdata = 0
        self.drive()

    def setup_dynamixel(self, dxl_id):

        
        for c in range(NUM_SERVOS):
            self.packet_handler.write1ByteTxRx(self.port_handler, dxl_id[c], ADDR_TORQUE_ENABLE, TORQUE_ENABLE)


        # self.packet_handler.write1ByteTxRx(self.port_handler, dxl_id[0], ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        # self.packet_handler.write1ByteTxRx(self.port_handler, dxl_id[1], ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        self.get_logger().info('Succeeded to enable torque.')
        self.get_logger().info(f'Current angle: {self.goal_position[0]*(360/4096)}')
        for c in range(NUM_SERVOS):
            
            param = [DXL_LOBYTE(int(round(self.goal_position[c]))),DXL_HIBYTE(int(round(self.goal_position[c])))]
            self.groupSyncWrite.addParam(self.DXL_ID[c], param)

        self.groupSyncWrite.txPacket()
        self.groupSyncWrite.clearParam()

    def drive(self):
        # pwm_linear = int(self.linear * self.max_pwm)
        # pwm_angular = int(self.angular * self.max_pwm)

        # left = pwm_linear - pwm_angular
        # right = (pwm_linear + pwm_angular) * -1

        # pwm_linear = int(self.linear * self.max_pwm)
        # pwm_angular = int(self.angular * self.max_pwm)
        # left = pwm_linear - pwm_angular
        # right = (pwm_linear + pwm_angular) * -1

        #Wheels positions and speeds with dynamixels
        if self.angular > 0.05 or self.angular < -0.05: #Poner un umbral
            
            wheel_information = self.get_wheel_configuration (self.linear,self.angular)   
        else :
            wheel_information = [[self.linear,self.linear,self.linear,self.linear,self.linear,self.linear],[0,0,0,0,0,0]]

        wheel_angles = self.radian_to_dynamixel (wheel_information[1])
        wheel_speeds = self.meters_to_ticks_ps (wheel_information[0])

        self.goal_position[0] = wheel_angles[2]
        self.goal_position[1] = wheel_angles[0]
        self.goal_position[2] = wheel_angles[5]
        self.goal_position[3] = wheel_angles[3]

        for c in range(NUM_SERVOS):
            
            param = [DXL_LOBYTE(int(round(self.goal_position[c]))),DXL_HIBYTE(int(round(self.goal_position[c])))]
            self.groupSyncWrite.addParam(self.DXL_ID[c], param)

        self.groupSyncWrite.txPacket()
        self.groupSyncWrite.clearParam()


        #Printing speeds

        left = max(min(left, 127), -127)
        right = max(min(right, 127), -127)

        self.roboclaw_front.SpeedAccelM1(self.ADDRESS,self.accel_max,int(round(wheel_speeds[0])))
        self.roboclaw_center.SpeedAccelM1(self.ADDRESS,self.accel_max,int(round(wheel_speeds[1])))
        self.roboclaw_rear.SpeedAccelM1(self.ADDRESS,self.accel_max,int(round(wheel_speeds[2])))
        self.roboclaw_front.SpeedAccelM2(self.ADDRESS,self.accel_max,int(round(wheel_speeds[3])))
        self.roboclaw_center.SpeedAccelM2(self.ADDRESS,self.accel_max,int(round(wheel_speeds[4])))
        self.roboclaw_rear.SpeedAccelM2(self.ADDRESS,self.accel_max,int(round(wheel_speeds[5])))
        # if (self.linear > 0):
        #     self.roboclaw_front.ForwardM1(self.ADDRESS, wheel_information[0][0] * self.max_pwm)
        #     self.roboclaw_center.ForwardM1(self.ADDRESS, wheel_information[0][1] * self.max_pwm)
        #     self.roboclaw_rear.ForwardM1(self.ADDRESS, wheel_information[0][2] * self.max_pwm)
        #     self.roboclaw_front.ForwardM2(self.ADDRESS, wheel_information[0][3] * self.max_pwm)
        #     self.roboclaw_center.ForwardM2(self.ADDRESS, wheel_information[0][4] * self.max_pwm)
        #     self.roboclaw_rear.ForwardM2(self.ADDRESS, wheel_information[0][5] * self.max_pwm)
        # else:
        #     self.roboclaw_front.BackwardM1(self.ADDRESS, wheel_information[0][0] * self.max_pwm)
        #     self.roboclaw_center.BackwardM1(self.ADDRESS, wheel_information[0][1] * self.max_pwm)
        #     self.roboclaw_rear.BackwardM1(self.ADDRESS, wheel_information[0][2] * self.max_pwm)
        #     self.roboclaw_front.BackwardM2(self.ADDRESS, wheel_information[0][3] * self.max_pwm)
        #     self.roboclaw_center.BackwardM2(self.ADDRESS, wheel_information[0][4] * self.max_pwm)
        #     self.roboclaw_rear.BackwardM2(self.ADDRESS, wheel_information[0][5] * self.max_pwm)

        
        # if (speed_left_edge >= 0):
        #     self.roboclaw_front.ForwardM1(self.ADDRESS, speed_left_edge * self.max_pwm)
        # else:
        #     self.roboclaw_front.BackwardM1(self.ADDRESS, -speed_left_edge * self.max_pwm)
        # if (right >= 0):
        #     self.roboclaw_front.ForwardM2(self.ADDRESS, speed_right_edge * self.max_pwm)
        # else:
        #     self.roboclaw_front.BackwardM2(self.ADDRESS, -speed_right_edge * self.max_pwm)


    def get_wheel_configuration (self,linear,angular): #FUNCTION TO OBTAIN WHEEL INFORMATION
        
        radius = linear/angular
            
        radius_left_center = radius - (self.width/2)
        radius_right_center = radius + (self.width/2)

        if linear != 0:
            sign = radius/abs(radius)
            sign2 = linear/abs(linear)
        else:
            sign = angular/abs(angular)
            sign2 = 1

        radius_left_frontal = math.sqrt(self.height**2+radius_left_center**2) * (sign)
        radius_right_frontal = math.sqrt(self.height**2+radius_right_center**2) * (sign)

        angle_left_front = math.atan2(self.height,  radius_left_center* (sign)) * (sign)#Radians
        angle_left_rear = math.atan2(-self.height,radius_left_center* (sign))* (sign)
        angle_right_front = math.atan2(self.height,radius_right_center* (sign)) * (sign)
        angle_right_rear = math.atan2(-self.height,radius_right_center* (sign))* (sign)
        
        # print (angle_left_front*(360/(2*math.pi)))
        # print (angle_left_rear*(360/(2*math.pi)))
        # print (angle_right_front*(360/(2*math.pi)))
        # print (angle_right_rear*(360/(2*math.pi)))

        #speeds:

        v_lf = abs(radius_left_frontal * angular) * sign2
        v_lc = abs(radius_left_center * angular) * sign2
        v_lr = abs(radius_right_frontal * angular)* sign2
        v_rf = abs(radius_right_frontal * angular)* sign2
        v_rc = abs(radius_right_center * angular)* sign2
        v_rr = abs(radius_right_frontal * angular)* sign2

        return [[v_lf,v_lc,v_lr,v_rf,v_rc,v_rr],[angle_left_front,0,angle_left_rear,angle_right_front,0,angle_right_rear]]

    def radian_to_dynamixel (self,angles): #Angles in radian to angles in bits for each dynamixel

        angles[0] = 2048 + (4096/(2*math.pi))* angles[0]
        angles[2] = 2048 + (4096/(2*math.pi))* angles[2]
        angles[3] = 2048 + (4096/(2*math.pi))* angles[3]
        angles[5] = 2048 + (4096/(2*math.pi))* angles[5]

        return angles
    
    def meters_to_ticks_ps (self,speeds):
        c=0
        for c in range(6):
            speeds[c] = speeds [c]/self.meters_per_tick
        return speeds


    def stop(self):
        self.roboclaw_front.ForwardM1(self.ADDRESS, 0)
        self.roboclaw_front.ForwardM2(self.ADDRESS, 0)

    def update_odometry(self):
        if not self.data:
            self.notdata += 1
        else:
            self.data = False

        if self.notdata >= self.limit:
            self.stop()
            return

        enc1 = self.roboclaw_front.ReadEncM1(self.ADDRESS)
        enc2 = self.roboclaw_front.ReadEncM2(self.ADDRESS)

        if enc1 is None or self.prev_enc1 is None:
            return

        d_left = (enc1[1] - self.prev_enc1[1]) * self.meters_per_tick *-1
        d_right = (enc2[1] - self.prev_enc2[1]) * self.meters_per_tick

        # self.get_logger().info(
         #   f"enc1={enc1[1]:.4f} enc2={enc2[1]:.4f} ")

        self.prev_enc1 = enc1
        self.prev_enc2 = enc2

        d_s = (d_right + d_left) / 2.0
        d_theta = (d_right - d_left) / self.width

        self.theta += d_theta
        self.x += d_s * math.cos(self.theta)
        self.y += d_s * math.sin(self.theta)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0

        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(self.theta / 2.0)
        t.transform.rotation.w = math.cos(self.theta / 2.0)

        self.tf_broadcaster.sendTransform(t)

        #self.get_logger().info(
            #f"x={self.x:.4f} y={self.y:.4f} theta={self.theta:.4f}")

    def __del__(self):
        
        #Desabilitar torque al terminar el nodo para evitar que se mueva
        for c in range(NUM_SERVOS):
            self.packet_handler.write1ByteTxOnly(self.port_handler,
                                           self.DXL_ID[c],
                                           ADDR_TORQUE_ENABLE,
                                           TORQUE_DISABLE)
        
        self.port_handler.closePort()
        self.get_logger().info('Shutting down read_write_node')

def main(args=None):
    rclpy.init(args=args)
    node = MobileBaseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()