import rclpy
from rclpy.node import Node
import math
from geometry_msgs.msg import PointStamped
from xarm_msgs.msg import RobotMsg

class FollowerTest(Node):
    def __init__(self):
        super().__init__('Follower')

        # --- LÍMITES FÍSICOS REALES (Tus pruebas) ---
        self.Z_FLOOR = -192.0      # Límite absoluto del suelo (Punto C)
        self.Z_APPROACH = -74.0   # Altura de alineación (Punto B)
        
        # PUNTO A (HOME DE OPERACIÓN) - Proporcionado por ti
        self.HOME_POSE = [1.4, -292.1, 239.5, -3.1352, -0.6782, -1.5780]

        self.robot_ready = False
        self.ruta_calculada = False 

        self.state_sub = self.create_subscription(RobotMsg, '/xarm/robot_states', self.state_callback, 10)
        self.target_sub = self.create_subscription(PointStamped, '/stone/target_pose_base', self.calc_loop, 10)

        self.get_logger().info('Follower Optimizado (Trayectoria en L) Activo')

    def state_callback(self, msg):
        self.robot_ready = True

    def generate_cmd(self, x, y, z, r, p, yw, label=""):
        # CLAMPING DE SEGURIDAD
        z = max(z, self.Z_FLOOR)
        return (f'ros2 service call /xarm/set_position xarm_msgs/srv/MoveCartesian '
                f'"{{pose: [{x:.1f}, {y:.1f}, {z:.1f}, {r:.4f}, {p:.4f}, {yw:.4f}], '
                f'speed: 50, acc: 500, mvtime: 0}}"')

    def calc_loop(self, msg):
        if not self.robot_ready or self.ruta_calculada: 
            return

        # 1. COORDENADAS DE LA ROCA (TRANSFORMADA)
        target_x = msg.point.x * 1000.0
        target_y = msg.point.y * 1000.0
        
        # OFFSETS CALCULADOS (B -> C)
        target_c_y = target_y - 44.5 

        # ORIENTACIÓN DE AGARRE (Basada en tu punto "Centrado")
        # Roll: ~ -3.14, Pitch: ~ 0.0, Yaw: ~ -1.57
        grip_r, grip_p, grip_yw = -3.1416, 0.0, -1.5708

        print("\n" + "="*50)
        print(f"INICIANDO DESDE PUNTO A: {self.HOME_POSE[:3]}")
        print(f"OBJETIVO FINAL (PUNTO C): X={target_x:.1f}, Y={target_c_y:.1f}, Z={self.Z_FLOOR}")
        print("="*50)

        # --- FASE 1: MOVIMIENTO HORIZONTAL (ALTURA SEGURA) ---
        # Nos movemos en XY a la altura de Home (239.5) para no chocar con nada
        print("\n--- FASE 1: POSICIONAMIENTO XY (SOBRE EL OBJETIVO) ---")
        print(self.generate_cmd(target_x, target_y, self.HOME_POSE[2], grip_r, grip_p, grip_yw, "SOBRE PIEDRA"))

        # --- FASE 2: DESCENSO VERTICAL (PUNTO B) ---
        # Bajamos en línea recta hasta la altura de la cámara
        print("\n--- FASE 2: DESCENSO A PUNTO B (CÁMARA) ---")
        print(self.generate_cmd(target_x, target_y, self.Z_APPROACH, grip_r, grip_p, grip_yw, "PUNTO B"))

        # --- FASE 3: AJUSTE FINAL Y AGARRE (PUNTO C) ---
        # Movimiento corto para el offset del gripper
        print("\n--- FASE 3: POSICIÓN DE AGARRE (PUNTO C) ---")
        print(self.generate_cmd(target_x, target_c_y, self.Z_FLOOR, grip_r, grip_p, grip_yw, "PUNTO C"))

        # --- FASE 4: RETORNO SEGURO ---
        print("\n--- FASE 4: RETORNO SEGURO A PUNTO A ---")
        print("1. Subir Vertical:")
        print(self.generate_cmd(target_x, target_c_y, self.HOME_POSE[2], grip_r, grip_p, grip_yw))
        print("2. Regresar a Home:")
        print(self.generate_cmd(self.HOME_POSE[0], self.HOME_POSE[1], self.HOME_POSE[2], self.HOME_POSE[3], self.HOME_POSE[4], self.HOME_POSE[5]))

        self.ruta_calculada = True

def main():
    rclpy.init()
    node = FollowerTest()
    rclpy.spin(node)
    rclpy.shutdown()