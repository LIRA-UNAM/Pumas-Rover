from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool, Int16
import serial
import time
import math

# Opcional: Importa aquí los mensajes a los que te suscribas (ej. Odometry)
# from nav_msgs.msg import Odometry

class RoverSerialWriter(Node):
    def __init__(self):
        super().__init__('rover_serial_writer')
        
        # 1. Configurar el puerto serie (el USB del ESP32 Emisor)
        try:
            self.ser = serial.Serial('/dev/ttyUSB1', 115200, timeout=0.1)
            self.get_logger().info("✅ Puerto Serie Emisor conectado exitosamente.")
            time.sleep(2) # Pausa para que el ESP32 reinicie tranquilamente al abrir el serial
        except serial.SerialException as e:
            self.get_logger().error(f"❌ Error al abrir el puerto: {e}")
            raise SystemExit

        # 2. Aquí irían tus suscriptores reales (Odometría, Visión, etc.)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(Int16, '/rover_state', self.rover_state, 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(Bool, '/follow_rock', self.move_callback, 10)
        self.create_subscription(Int16, '/rock_state', self.rock_callback, 10)

        # 3. Timer para enviar datos constantemente a la Base (ej. 10 Hz / 0.1s)
        self.timer = self.create_timer(0.1, self.enviar_datos_al_esp32)

        # Variables internas simulando el estado del rover
        self.rover_x = 0.0
        self.rover_y = 0.0
        self.rover_theta = 0.0
        self.terreno = 0
        self.roca = 0
        self.fin = 0

    def enviar_datos_al_esp32(self):
        # Simulamos que el rover se está moviendo hacia adelante
        self.rover_x += 0.01

        # Construimos la cadena EXACTA separada por comas, y MUY IMPORTANTE: añadir \n al final
        cadena = f"{self.rover_x:.2f},{self.rover_y:.2f},{self.rover_theta:.2f},{self.terreno},{self.roca},{self.fin}\n"
        
        try:
            # Escribimos los bytes en el cable USB
            self.ser.write(cadena.encode('utf-8'))
            # self.get_logger().info(f"Enviado al USB: {cadena.strip()}")
        except Exception as e:
            self.get_logger().error(f"Error escribiendo en serial: {e}")

    def odom_callback(self, msg):
        self.rover_x = msg.pose.pose.position.x
        self.rover_y = msg.pose.pose.position.y
        self.rover_theta = 0.0
        if msg.pose.pose.orientation.w != 0.0:
            self.rover_theta = 2.0 * math.atan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)

        self.get_logger().info(
            f"[odom_callback] x={self.rover_x:.2f}, y={self.rover_y:.2f}, theta={self.rover_theta:.2f}"
        )

    def rover_state(self, msg):
        self.fin = int(msg.data)
        self.get_logger().info(f"[rover_state] rover state={self.fin}")

    def cmd_vel_callback(self, msg):
        self.get_logger().info(
            f"[cmd_vel] linear={msg.linear.x:.2f}, angular={msg.angular.z:.2f}"
        )

    def move_callback(self, msg):
        self.get_logger().info(f"[follow_rock] follow_rock={msg.data}")

    def rock_callback(self, msg):
        self.roca = int(msg.data)
        self.get_logger().info(f"[rock_state] rock_state={self.roca}")


def main(args=None):
    rclpy.init(args=args)
    nodo = RoverSerialWriter()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.ser.close()
        nodo.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()