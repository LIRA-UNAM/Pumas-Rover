import rclpy
import time
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer #tf2 = transformation for odometry and base_link
#from Rosmaster_Lib import Rosmaster
from mobile_base import roboclaw_3
#import roboclaw_3
#from roboclaw_3 import Roboclaw

class MobileBaseNode(Node): 
    def __init__(self):
        super().__init__('mobile_base')

        # self.bot = Rosmaster('/dev/ttyUSB0')
        # self.bot.create_receive_threading()
        self.ADDRESS = 0x80
        self.roboclaw = roboclaw_3.Roboclaw("/dev/ttyACM0", 115200)
        self.roboclaw.Open()

        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.max_pwm = 60
        self.linear = 0.0
        self.angular = 0.0

        self.diameter = 0.107
        self.radius = self.diameter / 2.0
        self.wheel_dist = 0.45 #SE DEBE MEDIR LA DISTANCIA ENTRE RUEDA A RUEDA
        self.ppr = 3600

        self.meters_per_tick = (math.pi * self.diameter) / self.ppr

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        #self.prev_enc = self.bot.get_motor_encoder()
        self.prev_enc1 = self.roboclaw.ReadEncM1(self.ADDRESS)
        self.prev_enc2 = self.roboclaw.ReadEncM2(self.ADDRESS)

        self.timer = self.create_timer(0.05, self.update_odometry)

        self.get_logger().info("Odometría")

    def cmd_vel_callback(self, msg):
        self.linear = msg.linear.x
        self.angular = msg.angular.z
        self.drive()

    def drive(self):
        pwm_linear = int(self.linear * self.max_pwm)
        pwm_angular = int(self.angular * self.max_pwm)

        left = pwm_linear - pwm_angular
        right = (pwm_linear + pwm_angular)*-1

        left = max(min(left, 127), -127)
        right = max(min(right, 127), -127)

        if (left >= 0):
            self.roboclaw.ForwardM1(self.ADDRESS, left)
        else:
            self.roboclaw.BackwardM1(self.ADDRESS, -left)
        if (right >= 0):
            self.roboclaw.ForwardM2(self.ADDRESS, right)
        else:
            self.roboclaw.BackwardM2(self.ADDRESS, -right)

    def update_odometry(self):
        #enc=self.bot.get_motor_encoder()
        enc1=self.roboclaw.ReadEncM1(self.ADDRESS)
        enc2=self.roboclaw.ReadEncM2(self.ADDRESS)
        if enc1 is None or self.prev_enc1 is None:
            return

        d_left = (enc1[1]-self.prev_enc1[1])* self.meters_per_tick

        d_right = (enc2[1]-self.prev_enc2[1])* self.meters_per_tick

        self.prev_enc1 = enc1
        self.prev_enc2 = enc2

        d_s = (d_right + d_left) / 2.0
        d_theta = (d_right - d_left) / self.wheel_dist

        self.theta += d_theta
        self.x += d_s * math.cos(self.theta)
        self.y += d_s * math.sin(self.theta)

        self.get_logger().info(
            f"x={self.x:.4f} y={self.y:.4f} theta={self.theta:.4f}"
        )
        

def main(args=None):
    rclpy.init(args=args)
    node = MobileBaseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
