import rclpy
from rclpy.node import Node
import math
from geometry_msgs.msg import PointStamped
from xarm_msgs.msg import RobotMsg

class FollowerTest(Node):
    def __init__(self):
        super().__init__('Follower')

        self.current_x = None
        self.current_y = None
        self.current_z = None
        
        # ps actual
        self.current_roll = None
        self.current_pitch = None
        self.current_yaw = None
        
        self.robot_ready = False
        self.ruta_calculada = False 

        self.state_sub = self.create_subscription(RobotMsg, '/xarm/robot_states', self.state_callback, 10)
        self.target_sub = self.create_subscription(PointStamped, '/stone/target_pose_base', self.calc_loop, 10)

        self.get_logger().info('Ejecutando RRT')

    def state_callback(self, msg):
        self.current_x = msg.pose[0]
        self.current_y = msg.pose[1]
        self.current_z = msg.pose[2]
        
        self.current_roll = msg.pose[3]
        self.current_pitch = msg.pose[4]
        self.current_yaw = msg.pose[5]
        
        self.robot_ready = True

    def calc_loop(self, msg):
        
        if not self.robot_ready or self.ruta_calculada: 
            return

        #mm
        target_x_mm = msg.point.x * 1000.0
        target_y_mm = msg.point.y * 1000.0

        # ERROR
        dx_inicial = target_x_mm - self.current_x
        dy_inicial = target_y_mm - self.current_y
        dist_inicial = math.sqrt(dx_inicial**2 + dy_inicial**2)

        safe_z = 450.0
        epsilon = 50.0 # PASOS DE 5 CM 

        print("\n" + "=" * 70)
        print(f"GENERANDO RUTA RRT")
        print(f"INICIO: X={self.current_x:.1f}, Y={self.current_y:.1f}")
        print(f"META:   X={target_x_mm:.1f}, Y={target_y_mm:.1f}")
        print(f"DISTANCIA TOTAL: {dist_inicial:.1f} mm")
        print("=" * 80)

        
        sim_x = self.current_x
        sim_y = self.current_y
        paso_num = 1

        
        while True:
            
            dx = target_x_mm - sim_x
            dy = target_y_mm - sim_y
            dist = math.sqrt(dx**2 + dy**2)

            # CONDICIÓN final de llegada
            if dist < 15.0:
                print(f"\n PASO {paso_num} [ALINEACIÓN FINAL]")
                print("   Objetivo: Centrado (X,Y). Girando muñeca, mirando hacia abajo")
                cmd = f'ros2 service call /xarm/set_position xarm_msgs/srv/MoveCartesian "{{pose: [{sim_x:.1f}, {sim_y:.1f}, {safe_z}, 3.1416, 0.0, 0.0], speed: 50, acc: 500, mvtime: 0}}"'
                print(f"   Comando: {cmd}")
                break

            # CÁLCULO DEL NUEVO NODO 
            step_dist = min(dist, epsilon)
            sim_x = sim_x + step_dist * (dx / dist)
            sim_y = sim_y + step_dist * (dy / dist)

            print(f"\n🔹 PASO {paso_num} [ACERCAMIENTO]")
            print(f"   Coordenada: X={sim_x:.1f}, Y={sim_y:.1f} | Distancia restante: {(dist - step_dist):.1f} mm")
            print(f"   (ángulo original (para no perder de vista la roca))")
            cmd = f'ros2 service call /xarm/set_position xarm_msgs/srv/MoveCartesian "{{pose: [{sim_x:.1f}, {sim_y:.1f}, {safe_z}, {self.current_roll:.4f}, {self.current_pitch:.4f}, {self.current_yaw:.4f}], speed: 50, acc: 500, mvtime: 0}}"'
            print(f"   Comando: {cmd}")
            
            paso_num += 1

        print("=" * 70)
        print("FIN DE LA RUTA.")
        print("=" * 70 + "\n")

        
        self.ruta_calculada = True

def main():
    rclpy.init()
    node = FollowerTest()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()