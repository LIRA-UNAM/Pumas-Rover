import rclpy
from rclpy.node import Node
import math
from geometry_msgs.msg import PointStamped
from xarm_msgs.msg import RobotMsg


class FollowerTest(Node):
    def __init__(self):
        super().__init__('follower_test_node')

        self.current_x = None
        self.current_y = None
        self.current_z = None
        self.robot_ready = False

        
        self.state_sub = self.create_subscription(RobotMsg, '/xarm/robot_states', self.state_callback, 10)
       
        self.target_sub = self.create_subscription(PointStamped, '/stone/target_pose_base', self.calc_loop, 10)

        self.get_logger().info('Calculando trayectorias:')

    def state_callback(self, msg):
        # xArm reporta en mm
        self.current_x = msg.pose[0]
        self.current_y = msg.pose[1]
        self.current_z = msg.pose[2]
        self.robot_ready = True

    def calc_loop(self, msg):
        if not self.robot_ready: return

        #mm
        target_x_mm = msg.point.x * 1000.0
        target_y_mm = msg.point.y * 1000.0

        # ERROR
        dx = target_x_mm - self.current_x
        dy = target_y_mm - self.current_y
        dist_error = math.sqrt(dx**2 + dy**2)

        # SIGUIENTE POSICIÓN
        Kp = 0.5
        next_x = self.current_x + (dx * Kp)
        next_y = self.current_y + (dy * Kp)
        safe_z = 450.0 # Altura de seguridad fija

        
        print("-" * 50)
        print(f"POSE ACTUAL:  X={self.current_x:.1f}, Y={self.current_y:.1f} , Z={self.current_z:.1f} (mm)")
        print(f"TARGET: X={target_x_mm:.1f}, Y={target_y_mm:.1f} (mm)")
        print(f"ERROR:         {dist_error:.1f} mm")
        
        if dist_error < 15.0:
            print("ESTATUS:       CENTRADO")
        else:
            print(f"PREDICCIÓN:    El robot se moverá a -> X={next_x:.1f}, Y={next_y:.1f}, Z={safe_z}")
        print("-" * 50)

def main():
    rclpy.init()
    node = FollowerTest()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()