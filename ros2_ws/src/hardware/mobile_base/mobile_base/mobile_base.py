import rclpy
import time
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer #tf2 = transformation for odometry and base_link
from Rosmaster_Lib import Rosmaster

class MobileBaseNode(Node): 
    def __init__(self):
        super().__init__('mobile_base')

        self.bot = Rosmaster('/dev/ttyUSB0')
        self.bot.create_receive_threading()

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

        self.prev_enc = self.bot.get_motor_encoder()

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
        right = pwm_linear + pwm_angular

        left = max(min(left, 100), -100)
        right = max(min(right, 100), -100)

        self.bot.set_motor(left, left, right, right)

    def update_odometry(self):
        enc = self.bot.get_motor_encoder()
        if enc is None or self.prev_enc is None:
            return

        d_left = (
            (enc[0] - self.prev_enc[0]) +
            (enc[1] - self.prev_enc[1])
        ) / 2.0 * self.meters_per_tick

        d_right = (
            (enc[2] - self.prev_enc[2]) +
            (enc[3] - self.prev_enc[3])
        ) / 2.0 * self.meters_per_tick

        self.prev_enc = enc

        d_s = (d_right + d_left) / 2.0
        d_theta = (d_right - d_left) / self.wheel_dist

        self.theta += d_theta
        self.x += d_s * math.cos(self.theta)
        self.y += d_s * math.sin(self.theta)

        self.get_logger().info(
            f"x={self.x:.3f} y={self.y:.3f} theta={self.theta:.3f}"
        )
        

def main(args=None):
    rclpy.init(args=args)
    node = MobileBaseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
