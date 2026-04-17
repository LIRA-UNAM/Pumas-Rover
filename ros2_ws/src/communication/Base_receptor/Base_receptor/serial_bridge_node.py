import rclpy
from rclpy.node import Node
import serial
from nav_msgs.msg import Odometry
import math

class SerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge_node')
        
        # 1. Configurar el puerto serie (Asegúrate de que sea el correcto, ej. /dev/ttyUSB0)
        try:
            self.ser = serial.Serial('/dev/ttyUSB1', 115200, timeout=0.1)
            if self.ser.is_open:
                self.get_logger().info("✅ Puerto Serie /dev/ttyUSB1 conectado exitosamente.")
            else:
                self.get_logger().error("❌ Puerto serie no se abrió correctamente.")
                raise SystemExit
        except serial.SerialException as e:
            self.get_logger().error(f"❌ Error al abrir el puerto /dev/ttyUSB1: {e}")
            self.get_logger().info("💡 Verifica que el Puerto es correcto y el dispositivo está conectado.")
            raise SystemExit

        # 2. Crear el publicador de ROS 2 (Usaremos Odometry para la posición del Rover)
        self.odom_pub = self.create_publisher(Odometry, '/rover_odom', 10)
        
        # 3. Leer el puerto serie constantemente a 20Hz (cada 0.05s)
        self.timer = self.create_timer(0.05, self.leer_datos_esp32)
        self.get_logger().info("🔄 Serial Bridge listo para leer datos del ESP32...")

    def leer_datos_esp32(self):
        if self.ser.in_waiting > 0:
            try:
                # Leer la línea del ESP32 y quitar espacios/saltos de línea
                linea = self.ser.readline().decode('utf-8').strip()
                
                # Ignorar líneas vacías o incompletas
                if not linea or linea.startswith(','):
                    self.get_logger().debug(f"⚠️  Línea incompleta ignorada: '{linea}'")
                    return
                
                datos = linea.split(',')
                self.get_logger().info(f"📥 Recibido ({len(datos)} elementos): {linea}")

                # Verificamos que llegaron los 7 datos completos que programamos
                if len(datos) == 6:
                    x = float(datos[0])
                    y = float(datos[1])
                    theta_grados = float(datos[2])
                    tipo_terreno = int(datos[3])
                    tipo_roca = int(datos[4])
                    letrero_fin = int(datos[5])
                    letrero_inicio = int(datos[6])

                    # Imprimimos en consola para depurar
                    self.get_logger().info(f"📍 Pos: ({x}, {y}, {theta_grados}°) | 🏔️ Terreno: {tipo_terreno} | 🪨 Roca: {tipo_roca} | 📌 Fin: {letrero_fin} | 📌 Inicio: {letrero_inicio}")

                    # --- EJEMPLO: Publicar Posición en ROS 2 ---
                    odom_msg = Odometry()
                    odom_msg.header.stamp = self.get_clock().now().to_msg()
                    odom_msg.header.frame_id = "odom"
                    odom_msg.child_frame_id = "base_link"
                    
                    # Posición X, Y
                    odom_msg.pose.pose.position.x = x
                    odom_msg.pose.pose.position.y = y
                    
                    # Convertir el ángulo (theta) a Radianes para ROS 2
                    theta_rad = math.radians(theta_grados)
                    # Aquí iría la conversión de Euler a Cuaternión para el eje Z...
                    odom_msg.pose.pose.orientation.z = math.sin(theta_rad / 2.0)
                    odom_msg.pose.pose.orientation.w = math.cos(theta_rad / 2.0)

                    self.odom_pub.publish(odom_msg)
                    
                    # TODO: Aquí puedes agregar más publicadores para Markers (Las rocas)

            except Exception as e:
                # Ignoramos si llega basura en el serial ocasionalmente
                self.get_logger().error(f"❌ Error al procesar datos: {e}")
 

def main(args=None):
    rclpy.init(args=args)
    nodo = SerialBridge()
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