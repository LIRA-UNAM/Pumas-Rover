
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, select, termios, tty


class KeyboardSpeed(Node):

    def __init__(self):
        super().__init__('comandos_publisher')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.get_logger().info('Linear and angular speed')
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.settings = termios.tcgetattr(sys.stdin)
        self.i = 0
        self.current_linear_speed = 0.0
        self.current_angular_speed = 0.0

    def timer_callback(self):
        key = self.get_key()

        if key:  

            
            if key == "q" or key == "Q":
            
                self.current_linear_speed = 0.0
                self.current_angular_speed = 0.0
                self.get_logger().info(f"Stop")

            elif key == "w" or key == "W":

                self.current_linear_speed = self.current_linear_speed + 0.1
                self.current_angular_speed = 0.0
                self.get_logger().info(f"Foward")

            elif key == "s" or key == "S":
                self.current_linear_speed = self.current_linear_speed - 0.1
                self.current_angular_speed = 0.0
                self.get_logger().info(f"Back")

            elif key == "a" or key == "A":
                self.current_linear_speed = 0.0
                self.current_angular_speed = self.current_angular_speed + 0.1
                self.get_logger().info(f"Left_turn")
              

            elif key == "d" or key== "D":
                self.current_linear_speed = 0.0
                self.current_angular_speed = self.current_angular_speed - 0.1
                self.get_logger().info(f"Right_turn")

            elif key == "h" or key== "H":
                self.current_linear_speed = 0.0
                self.current_angular_speed = -5.0
                self.get_logger().info(f"Right_turn")

            elif key == "f" or key== "F":
                self.current_linear_speed = 0.0
                self.current_angular_speed = 5.0
                self.get_logger().info(f"Left_turn")

            elif key == "t" or key== "T":
                self.current_linear_speed = 5.0
                self.current_angular_speed = 0.0
                self.get_logger().info(f"Foward")

            elif key == "g" or key== "G":
                self.current_linear_speed = -5.0
                self.current_angular_speed = 0.0
                self.get_logger().info(f"Backward")
              

            else:
                self.get_logger().info(f"Invalid command")

                


            msg = Twist()
            msg.linear.x = self.current_linear_speed
            msg.angular.z = self.current_angular_speed
            self.publisher_.publish(msg)
            self.get_logger().info(f'Speed: "{self.current_linear_speed}" "{self.current_angular_speed}"')

    def get_key(self):

        tty.setcbreak(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1) 
        
        if rlist: key = sys.stdin.read(1)
        else: key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

        


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardSpeed()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, node.settings)
        node.destroy_node()
        if rclpy.ok():   
            rclpy.shutdown()


if __name__ == '__main__':
    main()
    