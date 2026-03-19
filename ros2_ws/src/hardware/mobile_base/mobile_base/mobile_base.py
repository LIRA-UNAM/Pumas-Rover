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
NUM_SERVOS = 4

class MobileBaseNode(Node): 
    def __init__(self):
        super().__init__('mobile_base')

        
        #Configuration needed for 3 roboclaws:
        self.ADDRESS = 0x80 #Adress to send instructions to the roboclaw
        self.roboclaw_front = roboclaw_3.Roboclaw("/dev/ttyACM2", 115200) #Create the roboclaw object with the device of the rare roboclaw
        self.roboclaw_center = roboclaw_3.Roboclaw("/dev/ttyACM1", 115200) #Create the roboclaw object with the device of the center roboclaw
        self.roboclaw_rear = roboclaw_3.Roboclaw("/dev/ttyACM0", 115200) #Create the roboclaw object with the device of the frontal roboclaw
        
        #Open comunication with the 3 roboclaws
        self.roboclaw_front.Open()
        if not self.roboclaw_front.Open():
            self.get_logger().fatal("Failed to open the front roboclaw")
            return
        self.roboclaw_center.Open()
        if not self.roboclaw_center.Open():
            self.get_logger().fatal("Failed to open the center roboclaw")
            return
        self.roboclaw_rear.Open()
        if not self.roboclaw_rear.Open():
            self.get_logger().fatal("Failed to open the rear roboclaw")
            rclpy.shutdown()
            return

        

        #Subscription to cmd_vel topic, give us the robot speed in form of a Twist
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.stop_driver = False #Variable to control if the robot should stop or not when it doesn't receive data

        #Broadcaster for odometry transformation
        self.tf_broadcaster = TransformBroadcaster(self)

        #Variables for odometry calculations
        self.data = False
        self.notdata = 0
        self.limit = 5

        #VARIABLES AND PARAMETERS FOR SPEED WHEELS
        
        #100 500 y 100 for rear and frontal
        #1000 200 y 1000 for center
        #Values above are decent functional

        self.k_p = 1000
        self.k_i = 800
        self.k_d = 100
        self.k_p_center = 10000
        self.k_i_center = 500
        self.k_d_center = 1000
        self.qpps = 11000 #341918 #12500
        self.max_accel = 50000

        self.roboclaw_front.SetM1VelocityPID(self.ADDRESS,self.k_p,self.k_i,self.k_d,self.qpps)
        self.roboclaw_front.SetM2VelocityPID(self.ADDRESS,self.k_p,self.k_i,self.k_d,self.qpps)
        self.roboclaw_center.SetM1VelocityPID(self.ADDRESS,self.k_p_center,self.k_i_center,self.k_d_center,self.qpps)
        self.roboclaw_center.SetM2VelocityPID(self.ADDRESS,self.k_p_center,self.k_i_center,self.k_d_center,self.qpps)
        self.roboclaw_rear.SetM1VelocityPID(self.ADDRESS,self.k_p,self.k_i,self.k_d,self.qpps)
        self.roboclaw_rear.SetM2VelocityPID(self.ADDRESS,self.k_p,self.k_i,self.k_d,self.qpps)
        
        
        self.accel_max = 8000
        self.max_pwm = 60
        self.linear = 0.0
        self.angular = 0.0

        #Physic parameters to calculate odometry and speeds
        self.diameter = 0.107
        self.radius = self.diameter / 2.0
        self.width = 0.32 #Rover measure of left wheels to right wheels
        self.height = 0.29 #Rover measure of center wheels to front wheels
        self.ppr = 4400
       
        self.meters_per_tick = (math.pi * self.diameter) / self.ppr #For encoders
        self.servo_odometry_angles = [0,0,0,0]

        #Values for publish odometry
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        #Variable to control dynamixels
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
        #Use of GroupSyncWrite for controlling 4 dynamixels using a single package
        self.groupSyncWrite = GroupSyncWrite(self.port_handler,self.packet_handler,ADDR_GOAL_POSITION,LEN_GOAL_POSITION)
        
        #Initial configurartion of dynamixels
        self.setup_dynamixel(self.DXL_ID)

        #FOR ROBOCLAWS

        #Set values of roboclaws to 0
        self.roboclaw_front.ResetEncoders(self.ADDRESS)
        self.roboclaw_center.ResetEncoders(self.ADDRESS)
        self.roboclaw_rear.ResetEncoders(self.ADDRESS)

        #Get a first measure of 6 encoders in the robot start
        self.prev_enc1_front = self.roboclaw_front.ReadEncM1(self.ADDRESS) [1]
        self.prev_enc2_front = self.roboclaw_front.ReadEncM2(self.ADDRESS) [1]
        self.prev_enc1_center = self.roboclaw_center.ReadEncM1(self.ADDRESS) [1]
        self.prev_enc2_center = self.roboclaw_center.ReadEncM2(self.ADDRESS) [1]
        self.prev_enc1_rear = self.roboclaw_rear.ReadEncM1(self.ADDRESS) [1]
        self.prev_enc2_rear = self.roboclaw_rear.ReadEncM2(self.ADDRESS) [1]
        
        #Calculate odometry every 0.1 seconds
        self.timer = self.create_timer(0.1, self.update_odometry) #0.05 anteriormente

        self.get_logger().info("Odometria con TF")

    def cmd_vel_callback(self, msg):
        self.linear = msg.linear.x
        self.angular = msg.angular.z
        self.data = True
        self.notdata = 0
        if self.stop_driver == False:
            self.drive()
        

    def setup_dynamixel(self, dxl_id):

        
        for c in range(NUM_SERVOS):
            self.packet_handler.write1ByteTxRx(self.port_handler, dxl_id[c], ADDR_TORQUE_ENABLE, TORQUE_ENABLE)

        self.get_logger().info('Succeeded to enable torque.')
        self.get_logger().info(f'Current angle: {self.goal_position[0]*(360/4096)}')
        for c in range(NUM_SERVOS):
            
            param = [DXL_LOBYTE(int(round(self.goal_position[c]))),DXL_HIBYTE(int(round(self.goal_position[c])))]
            dxl_addparam_result = self.groupSyncWrite.addParam(self.DXL_ID[c], param)
            if dxl_addparam_result != True:
                self.get_logger().error(f'Failed to addparam for ID {self.DXL_ID[c]}')
                return

        dxl_comm_result = self.groupSyncWrite.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().error(f'Failed to set initial position: {self.packet_handler.getTxRxResult(dxl_comm_result)}')
            return
        else:
            self.get_logger().info('Succeeded to set initial position.')
        self.groupSyncWrite.clearParam()

    def drive(self):
        
        #Wheels positions and speeds with dynamixels
        if self.angular > 0.005 or self.angular < -0.005: #Poner un umbral
            
            wheel_information = self.get_wheel_configuration (self.linear,self.angular)   
        else :
            wheel_information = [[self.linear,self.linear,self.linear,self.linear,self.linear,self.linear],[0,0,0,0,0,0]]

        #Transform either speeds and angles to ticks per second and bits
        wheel_angles = self.radian_to_dynamixel (wheel_information[1])
        wheel_speeds = self.meters_to_ticks_ps (wheel_information[0])

        #Update wish angles
        self.goal_position[0] = wheel_angles[0]
        self.goal_position[1] = wheel_angles[2]
        self.goal_position[2] = wheel_angles[3]
        self.goal_position[3] = wheel_angles[5]

        #Prepare and save new angles in dynamixel servos
        for c in range(NUM_SERVOS):
            param = [DXL_LOBYTE(int(round(self.goal_position[c]))),DXL_HIBYTE(int(round(self.goal_position[c])))]
            dxl_addparam_result = self.groupSyncWrite.addParam(self.DXL_ID[c], param)
            if dxl_addparam_result != True:
                self.get_logger().error(f'Failed to addparam for ID {self.DXL_ID[c]}')
                return

        #Send and move dynamixel servos with the new save angles
        dxl_comm_result = self.groupSyncWrite.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().error(f'Failed to set wheel positions: {self.packet_handler.getTxRxResult(dxl_comm_result)}')
            self.stop_driver = True
            return
        else:
            self.get_logger().info('Succeeded to set wheel positions.')
        self.groupSyncWrite.clearParam()


        #Printing speeds using roboclaws

        self.roboclaw_front.SpeedAccelM1(self.ADDRESS,self.accel_max,int(round(wheel_speeds[0])))
        self.roboclaw_center.SpeedAccelM1(self.ADDRESS,self.accel_max,int(round(wheel_speeds[1])))
        self.roboclaw_rear.SpeedAccelM1(self.ADDRESS,self.accel_max,int(round(wheel_speeds[2])))
        self.roboclaw_front.SpeedAccelM2(self.ADDRESS,self.accel_max,int(round(wheel_speeds[3])))
        self.roboclaw_center.SpeedAccelM2(self.ADDRESS,self.accel_max,int(round(wheel_speeds[4])))
        self.roboclaw_rear.SpeedAccelM2(self.ADDRESS,self.accel_max,int(round(wheel_speeds[5])))
        
       



    def get_wheel_configuration (self,linear,angular): #FUNCTION TO OBTAIN WHEEL INFORMATION
        
        radius = linear/angular
            
        radius_left_center = radius - (self.width/2)
        radius_right_center = radius + (self.width/2)

        if self.linear > 0.005 or self.linear < -0.005: #if linear!=0
            sign = radius/abs(radius)
            sign2 = linear/abs(linear)
            sign3 = 1
            
        else:
            sign = angular/abs(angular)
            sign2 = 1
            sign3 = (angular/abs(angular))
            

        radius_left_frontal = math.sqrt(self.height**2+radius_left_center**2) * (sign)
        radius_right_frontal = math.sqrt(self.height**2+radius_right_center**2) * (sign)

        #Wheel angles
        angle_left_front = math.atan2(self.height,  radius_left_center* (sign)) * (sign)#Radians
        angle_left_rear = math.atan2(-self.height,radius_left_center* (sign))* (sign)
        angle_right_front = math.atan2(self.height,radius_right_center* (sign)) * (sign)
        angle_right_rear = math.atan2(-self.height,radius_right_center* (sign))* (sign)
        

        #Wheel speeds:
        v_lf = abs(radius_left_frontal * angular) * sign2
        v_lc = abs(radius_left_center * angular) * sign2 * sign3
        v_lr = abs(radius_right_frontal * angular)* sign2
        v_rf = abs(radius_right_frontal * angular)* sign2
        v_rc = abs(radius_right_center * angular)* sign2 * -sign3
        v_rr = abs(radius_right_frontal * angular)* sign2

        return [[v_lf,v_lc,v_lr,v_rf,v_rc,v_rr],[angle_left_front,0,angle_left_rear,angle_right_front,0,angle_right_rear]]

    def radian_to_dynamixel (self,angles): #Angles in radian to angles in bits for each dynamixel

        angles[0] = 2048 - (4096/(2*math.pi))* angles[0]
        angles[2] = 2048 - (4096/(2*math.pi))* angles[2]
        angles[3] = 2048 - (4096/(2*math.pi))* angles[3]
        angles[5] = 2048 - (4096/(2*math.pi))* angles[5]

        return angles
    
    def meters_to_ticks_ps (self,speeds): #SpeedAccelM1() needs speed in ticks per second
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

        #Encoders 1 = left, Encoders 2 = right
        enc1_front = self.roboclaw_front.ReadEncM1(self.ADDRESS) [1]
        enc2_front = self.roboclaw_front.ReadEncM2(self.ADDRESS) [1]
        enc1_center = self.roboclaw_front.ReadEncM1(self.ADDRESS) [1]
        enc2_center = self.roboclaw_front.ReadEncM2(self.ADDRESS) [1]
        enc1_rear = self.roboclaw_rear.ReadEncM1(self.ADDRESS) [1]
        enc2_rear = self.roboclaw_rear.ReadEncM2(self.ADDRESS) [1]

        #Calculate the angle change between current wheel postion and idle position (makes robot goes foward)
        self.servo_odometry_angles[0] = (2048 - self.goal_position [0]) * (2*math.pi/4096)
        self.servo_odometry_angles[1] = (2048 - self.goal_position [1]) * (2*math.pi/4096)
        self.servo_odometry_angles[2] = (2048 - self.goal_position [2]) * (2*math.pi/4096)
        self.servo_odometry_angles[3] = (2048 - self.goal_position [3]) * (2*math.pi/4096)

        if enc1_front is None or self.prev_enc1_front is None:
            return

        #Use angle wheel, ppr and advance to calculate position advance in meters
        dx_front_left = (enc1_front - self.prev_enc1_front) * math.cos(self.servo_odometry_angles[0])*self.meters_per_tick
        dx_rear_left = (enc1_rear - self.prev_enc1_rear) * math.cos(self.servo_odometry_angles[1])*self.meters_per_tick
        dx_front_right = (enc2_front - self.prev_enc2_front) * math.cos(self.servo_odometry_angles[2])*self.meters_per_tick
        dx_rear_right = (enc2_rear - self.prev_enc2_rear) * math.cos(self.servo_odometry_angles[3])*self.meters_per_tick

        #Average value of right and left for imitate a differential robot
        dx_right = (dx_front_right + dx_rear_right) / 2.0
        dx_left = (dx_front_left + dx_rear_left) / 2.0
        
        #Update old encoder value for the next iteration
        self.prev_enc1_front = enc1_front
        self.prev_enc2_front = enc2_front
        self.prev_enc1_center = enc1_center
        self.prev_enc2_center = enc2_center
        self.prev_enc1_rear = enc1_rear
        self.prev_enc2_rear = enc2_rear

        #Calculate final advance and angle postion in this iteration
        d_theta = (dx_right - dx_left) / self.width 
        d_s = (dx_right + dx_left) / 2.0

        #Split position in 2 axes and update total final odometry position
        self.theta += d_theta
        self.x += d_s * math.cos(self.theta)
        self.y += d_s * math.sin(self.theta)

        #Publish odometry
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

        self.get_logger().info(
             f"x={self.x:.4f} y={self.y:.4f} theta={self.theta:.4f}")

    def __del__(self):
        
        #Disable movement in dynamixel servos
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