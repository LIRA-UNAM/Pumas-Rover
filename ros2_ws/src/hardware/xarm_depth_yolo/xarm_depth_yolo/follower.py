import rclpy
from rclpy.node import Node
import math
from geometry_msgs.msg import PointStamped
from xarm_msgs.msg import RobotMsg

class FollowerTest(Node):
    def __init__(self):
        super().__init__('Follower')

        # --- PARÁMETROS CALIBRADOS FINAL ---
        self.EPSILON = 50.0        
        self.Z_FLOOR = -219.2      
        self.Z_APPROACH = -169.2   
        
        # --- AJUSTE DE OFFSETS (LA CLAVE) ---
        self.GRIPPER_OFFSET_Y = 16.5  # Adelante/Atrás (Ya calibrado por ti)
        self.GRIPPER_OFFSET_X = 8.0   # Izquierda/Derecha (Ajustado)
        
        # POSICIONES CARTESIANAS (A -> B -> C)
        self.HOME_CARTESIAN = [1.4, -292.1, 239.5, -3.1352, -0.6782, -1.5780]
        
        # POSICIONES ARTICULARES ACTUALIZADAS (Basadas en tu echo /joint_states)
        # J1: -1.5747, J2: -0.2777, J3: -0.0971, J4: -0.0056, J5: 0.1502, J6: 0.0002
        self.HOME_JOINTS = [-1.5747, -0.2777, -0.0971, -0.0056, 0.1502, 0.0002]
        self.DEPOSIT_JOINTS = [-1.5752, -0.2315, -0.1048, -2.2752, -0.2931, 0.0008]

        self.robot_ready = False
        self.ruta_calculada = False 

        self.state_sub = self.create_subscription(RobotMsg, '/xarm/robot_states', self.state_callback, 10)
        self.target_sub = self.create_subscription(PointStamped, '/stone/target_pose_base', self.calc_loop, 10)

        self.get_logger().info('Follower CALIBRADO (Joints Actualizados) Activo')

    def state_callback(self, msg):
        self.robot_ready = True

    def generate_cmd_cartesian(self, x, y, z, r, p, yw, label=""):
        z = max(z, self.Z_FLOOR)
        return (f'# {label}\n'
                f'ros2 service call /xarm/set_position xarm_msgs/srv/MoveCartesian '
                f'"{{pose: [{x:.1f}, {y:.1f}, {z:.1f}, {r:.4f}, {p:.4f}, {yw:.4f}], '
                f'speed: 50, acc: 500, mvtime: 0}}"')

    def generate_cmd_joint(self, angles, label=""):
        angs_str = ", ".join([f"{a:.4f}" for a in angles])
        return (f'# {label}\n'
                f'ros2 service call /xarm/set_servo_angle xarm_msgs/srv/MoveJoint '
                f'"{{angles: [{angs_str}], speed: 0.35, acc: 2.0, mvtime: 0}}"')

    def calc_loop(self, msg):
        if not self.robot_ready or self.ruta_calculada: 
            return

        target_x = msg.point.x * 1000.0
        target_y = msg.point.y * 1000.0
        
        grip_r, grip_p, grip_yw = -3.1416, 0.0, -1.5708

        print("\n" + "="*75)
        print(f" MISIÓN FINAL | ROCA DETECTADA EN: X={target_x:.1f}, Y={target_y:.1f} ")
        print("="*75)

        # --- FASE 1: DESCENSO VISUAL ---
        goal_b = [target_x, target_y, self.Z_APPROACH]
        dx = goal_b[0] - self.HOME_CARTESIAN[0]
        dy = goal_b[1] - self.HOME_CARTESIAN[1]
        dz = goal_b[2] - self.HOME_CARTESIAN[2]
        dist_total = math.sqrt(dx**2 + dy**2 + dz**2)
        num_pasos = math.ceil(dist_total / self.EPSILON)
        
        print(f"\n[FASE 1] DESCENSO EN {num_pasos} PASOS")
        for i in range(1, num_pasos + 1):
            ratio = i / num_pasos
            w_x = self.HOME_CARTESIAN[0] + (dx * ratio)
            w_y = self.HOME_CARTESIAN[1] + (dy * ratio)
            w_z = self.HOME_CARTESIAN[2] + (dz * ratio)
            print(self.generate_cmd_cartesian(w_x, w_y, w_z, self.HOME_CARTESIAN[3], self.HOME_CARTESIAN[4], self.HOME_CARTESIAN[5], f"Waypoint {i}"))

        # --- FASE 2: AGARRE (OFFSET X e Y) ---
        print("\n[FASE 2] AGARRE")
        final_x = target_x + self.GRIPPER_OFFSET_X
        final_y = target_y + self.GRIPPER_OFFSET_Y 
        
        print(self.generate_cmd_cartesian(final_x, final_y, self.Z_FLOOR, grip_r, grip_p, grip_yw, "Punto C (Centro Gripper)"))
        print(">> ACCIÓN: ros2 service call /gripper/smart_grab std_srvs/srv/Trigger {}")

        # --- FASE 3: RETORNO (HÍBRIDO) ---
        print("\n[FASE 3] REGRESO SEGURO")
        print(self.generate_cmd_cartesian(final_x, final_y, self.Z_APPROACH, grip_r, grip_p, grip_yw, "Escape Vertical"))
        print(self.generate_cmd_joint(self.HOME_JOINTS, "Arco a Home"))
        print(self.generate_cmd_joint(self.DEPOSIT_JOINTS, "Arco a Depósito"))
        print(">> ACCIÓN: ros2 service call /gripper/open std_srvs/srv/Trigger {}")
        print(self.generate_cmd_joint(self.HOME_JOINTS, "Regreso a Operación"))

        self.ruta_calculada = True

def main():
    rclpy.init()
    node = FollowerTest()
    rclpy.spin(node)
    rclpy.shutdown()