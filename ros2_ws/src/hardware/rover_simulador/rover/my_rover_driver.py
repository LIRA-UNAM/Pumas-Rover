import rclpy
from geometry_msgs.msg import Twist

HALF_DISTANCE_BETWEEN_WHEELS = 0.42
WHEEL_RADIUS = 0.2

class MyRoverDriver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot
        # Configuración de motores (ajusta según tu robot)
        self.__left_motor_1 = self.__robot.getDevice('left_motor_1')
        self.__left_motor_2 = self.__robot.getDevice('left_motor_2')
        self.__left_motor_3 = self.__robot.getDevice('left_motor_3')
        self.__right_motor_1 = self.__robot.getDevice('right_motor_1')
        self.__right_motor_2 = self.__robot.getDevice('right_motor_2')
        self.__right_motor_3 = self.__robot.getDevice('right_motor_3')
        
        self.__left_motor_1.setPosition(float('inf'))
        self.__left_motor_1.setVelocity(0)
        self.__left_motor_2.setPosition(float('inf'))
        self.__left_motor_2.setVelocity(0)
        self.__left_motor_3.setPosition(float('inf'))
        self.__left_motor_3.setVelocity(0)

        self.__right_motor_1.setPosition(float('inf'))
        self.__right_motor_1.setVelocity(0)
        self.__right_motor_2.setPosition(float('inf'))
        self.__right_motor_2.setVelocity(0)
        self.__right_motor_3.setPosition(float('inf'))
        self.__right_motor_3.setVelocity(0)
        
        self.__target_twist = Twist()

        rclpy.init(args=None)
        self.__node = rclpy.create_node('my_rover_driver')
        self.__node.create_subscription(Twist, 'cmd_vel', self.__cmd_vel_callback, 1)
        
    def __cmd_vel_callback(self, twist):
        self.__target_twist = twist

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)
        # Invierte el signo de forward_speed y angular_speed
        forward_speed = -self.__target_twist.linear.x
        angular_speed = -self.__target_twist.angular.z

        command_motor_left = (forward_speed + angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS
        command_motor_right = (forward_speed - angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS

        self.__left_motor_1.setVelocity(command_motor_left)
        self.__left_motor_2.setVelocity(command_motor_left)
        self.__left_motor_3.setVelocity(command_motor_left)
        self.__right_motor_1.setVelocity(command_motor_right)
        self.__right_motor_2.setVelocity(command_motor_right)
        self.__right_motor_3.setVelocity(command_motor_right)
