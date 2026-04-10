import rclpy
from rclpy.node import Node
from xarm_msgs.msg import RobotMsg
import time

class PrintPositionNode(Node):
    def __init__(self):
        super().__init__('print_position_node')
        
        
        self.state_sub = self.create_subscription(
            RobotMsg, 
            '/xarm/robot_states', 
            self.state_callback, 
            10
        )
        
        
        self.last_print_time = time.time()
        
        self.get_logger().info('Leyendo posición...')

    def state_callback(self, msg):
        current_time = time.time()
        
        
        if (current_time - self.last_print_time) >= 0.5:
            
            x = msg.pose[0]
            y = msg.pose[1]
            z = msg.pose[2]
            roll = msg.pose[3]
            pitch = msg.pose[4]
            yaw = msg.pose[5]

            
            cmd = (f'ros2 service call /xarm/set_position xarm_msgs/srv/MoveCartesian '
                   f'"{{pose: [{x:.1f}, {y:.1f}, {z:.1f}, {roll:.4f}, {pitch:.4f}, {yaw:.4f}], '
                   f'speed: 50, acc: 500, mvtime: 0}}"')
            
            
            print(cmd)
            
            self.last_print_time = current_time

def main(args=None):
    rclpy.init(args=args)
    node = PrintPositionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nLectura detenida.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
