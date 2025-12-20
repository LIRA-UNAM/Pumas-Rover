import rclpy
import time
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer #tf2 = transformation for odometry and base_link
from Rosmaster_Lib import Rosmaster

class MobileBaseNode(Node): 
    def __init__(self):
        super().__init__('mobile_base')

        self.bot = Rosmaster() #yahboom_board
        self.bot.create_receive_threading()
        self.dist = [0,0,0,0]
        self.diametro = 0.107 #m
        self.ppr = 3600 #aproximated pulses for a revolution of each encoder
        #robot_x,robot_y,robot_a
        self.perimeter = 10.7*3.14159
        self.current_speed = 40
        print (self.bot.get_version())
        
    def spin(self):
        while rclpy.ok():
            
            
            #print (self.bot.get_motor_encoder())
            self.encoder_sensor()
            print(self.dist)
            #self.motors_foward()
            rclpy.spin_once(self, timeout_sec=0)
            time.sleep(0.05)

    def motors_foward (self):
        self.bot.set_motor(
                self.current_speed,
                self.current_speed,
                self.current_speed,
                self.current_speed)
           
        #self.get_logger().info(f"avanzando")
        self.get_logger().info(f"Velocidad actual: {self.actual_speed}")

    def encoder_sensor (self): #get_encoders #promedio izquierdos y de derechos
        #print(self.bot.get_motor_encoder())
        self.dist[0] = -1*(self.bot.get_motor_encoder()[0]) * (self.perimetro/self.ppr)
        self.dist[1] = -1*(self.bot.get_motor_encoder()[1]) * (self.perimetro/self.ppr)
        self.dist[2] = -1*(self.bot.get_motor_encoder()[2]) * (self.perimetro/self.ppr)
        self.dist[3] = -1*(self.bot.get_motor_encoder()[3]) * (self.perimetro/self.ppr)
    






def main(args=None):
    
    rclpy.init(args=args)
    n = MobileBaseNode()
    n.spin()
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
