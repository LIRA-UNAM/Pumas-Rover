
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
import sys, select, termios, tty


class RemoteControl(Node):

    def __init__(self):
        super().__init__('control_publisher')
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('Mobile Base trough a remote control')

        self.current_linear_speed = 0.0
        self.current_angular_speed = 0.0

        self.buttons = [0,0,0,0,0,0,0,0,0,0,0,0,0,0]
        self.axes = [0,0,0,0,0,0]
        
            

        

    def joy_callback(self,msg):
        #self.buttons_info = msg #There's no data function in msg of Joy 
        #Information for Pro Controller Nintendo Swtich 1
        self.axes = msg.axes 
        #[0] = Left joystick = left
        #[1] = Left joystick = up
        #[2] = Right joystick = left
        #[3] = Right joystick = up
        #[4] = D pad = left
        #[5] = D pad joystick = up

        self.buttons = msg.buttons
        #[0] = B
        #[1] = A
        #[2] = X
        #[3] = Y
        #[4] = CAMERA
        #[5] = L
        #[6] = R
        #[7] = ZL
        #[8] = ZR
        #[9] = -
        #[10] = +
        #[11] = HOME
        #[12] = LSB
        #[13] = RSB

        #self.get_logger().info(f"Received information:  {self.axes}") 
    
    def timer_callback(self):
        
            
        if self.buttons[2] == 1:
            self.current_linear_speed = 0.0
            self.current_angular_speed = 0.0
            self.get_logger().info(f"Stop")

        if self.axes[0] > 0.2:
                
            self.current_linear_speed = 0.0
            self.current_angular_speed = 2*self.axes[0]
            self.get_logger().info(f"Left_turn")

        elif self.axes[0] < -0.2:

            self.current_linear_speed = 0.0
            self.current_angular_speed = 2*self.axes[0]
            self.get_logger().info(f"Right_turn")

        elif self.axes[1] > 0.2:
                
            self.current_linear_speed = 2*self.axes[1]
            self.current_angular_speed = 0.0
            self.get_logger().info(f"Foward")

        elif self.axes[1] < -0.2:

            self.current_linear_speed = 2*self.axes[1]
            self.current_angular_speed = 0.0
            self.get_logger().info(f"Back")

        else:
            self.current_linear_speed = 0.0
            self.current_angular_speed = 0.0

        msg = Twist()
        msg.linear.x = self.current_linear_speed
        msg.angular.z = self.current_angular_speed
        self.publisher_.publish(msg)
        self.get_logger().info(f'Speed: "{self.current_linear_speed}" "{self.current_angular_speed}"')
        


def main(args=None):
    rclpy.init(args=args)
    node = RemoteControl()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():   
            rclpy.shutdown()


if __name__ == '__main__':
    main()
    