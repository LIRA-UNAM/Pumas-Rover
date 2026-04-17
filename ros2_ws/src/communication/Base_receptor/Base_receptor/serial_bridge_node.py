import rclpy
from rclpy.node import Node
import serial
import math

from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

class SerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge_node')
        
        try:
            self.ser = serial.Serial('/dev/ttyUSB1', 115200, timeout=0.1)
            self.get_logger().info("✅ Puerto Serie conectado exitosamente.")
        except serial.SerialException as e:
            self.get_logger().error(f"❌ Error al abrir el puerto: {e}")
            raise SystemExit

        # Publicadores
        self.odom_pub = self.create_publisher(Odometry, '/rover_odom', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/mapa_lunar_markers', 10)
        
        self.timer = self.create_timer(0.05, self.leer_datos_esp32)

        # Variables para el manejo del mapa
        self.marker_array = MarkerArray()
        self.marker_id_counter = 0
        
        # Para evitar publicar 100 veces la misma roca si el rover se queda quieto viéndola
        self.ultima_pos_roca = (-100.0, -100.0)
        self.ultimo_terreno_marcado = 0

    def crear_marcador(self, x, y, tipo, subtipo):
        marker = Marker()
        marker.header.frame_id = "odom" # El marco de referencia base
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.id = self.marker_id_counter
        self.marker_id_counter += 1
        
        marker.action = Marker.ADD
        marker.pose.position.x = float(x)
        marker.pose.position.y = float(y)
        marker.pose.position.z = 0.0
        
        # Rotación neutral
        marker.pose.orientation.w = 1.0

        # --- LÓGICA DE ROCAS ---
        if tipo == "ROCA":
            marker.type = Marker.SPHERE
            marker.scale.x = 0.15 # 15 cm de tamaño
            marker.scale.y = 0.15
            marker.scale.z = 0.15
            marker.color.a = 1.0 # Opacidad
            
            if subtipo == 1:   # Roja
                marker.color.r, marker.color.g, marker.color.b = 1.0, 0.0, 0.0
            elif subtipo == 2: # Azul
                marker.color.r, marker.color.g, marker.color.b = 0.0, 0.0, 1.0
            elif subtipo == 3: # Verde
                marker.color.r, marker.color.g, marker.color.b = 0.0, 1.0, 0.0

        # --- LÓGICA DE TERRENOS ---
        elif tipo == "TERRENO":
            marker.type = Marker.CUBE
            marker.scale.x = 0.5 # Área de medio metro
            marker.scale.y = 0.5
            marker.scale.z = 0.01 # Plano pegado al piso
            marker.color.a = 0.6 # Semi-transparente
            
            if subtipo == 1:   # Valle (Azul claro)
                marker.color.r, marker.color.g, marker.color.b = 0.0, 1.0, 1.0
            elif subtipo == 2: # Surco (Naranja)
                marker.color.r, marker.color.g, marker.color.b = 1.0, 0.5, 0.0
            elif subtipo == 3: # Pendiente (Morado)
                marker.color.r, marker.color.g, marker.color.b = 0.5, 0.0, 0.5

        # --- LÓGICA DE LETREROS (BANDERINES) ---
        elif tipo == "LETRERO":
            marker.type = Marker.TEXT_VIEW_FACING
            marker.scale.z = 0.3 # Tamaño de la letra
            marker.color.a = 1.0
            marker.color.r, marker.color.g, marker.color.b = 1.0, 1.0, 1.0 # Blanco
            marker.pose.position.z = 0.5 # Flotando medio metro arriba
            
            if subtipo == 1:
                marker.text = "🚩 INICIO"
            elif subtipo == 2:
                marker.text = "🏁 FIN"

        self.marker_array.markers.append(marker)
        self.marker_pub.publish(self.marker_array)

    def leer_datos_esp32(self):
        if self.ser.in_waiting > 0:
            try:
                linea = self.ser.readline().decode('utf-8').strip()
                datos = linea.split(',')

                if len(datos) == 7:
                    x = float(datos[0])
                    y = float(datos[1])
                    theta_grados = float(datos[2])
                    tipo_terreno = int(datos[3])
                    tipo_roca = int(datos[4])
                    letrero_fin = int(datos[5])
                    letrero_inicio = int(datos[6])

                    # 1. PUBLICAR ODOMETRÍA (Posición del Rover)
                    odom_msg = Odometry()
                    odom_msg.header.stamp = self.get_clock().now().to_msg()
                    odom_msg.header.frame_id = "odom"
                    odom_msg.child_frame_id = "base_link"
                    odom_msg.pose.pose.position.x = x
                    odom_msg.pose.pose.position.y = y
                    
                    theta_rad = math.radians(theta_grados)
                    odom_msg.pose.pose.orientation.z = math.sin(theta_rad / 2.0)
                    odom_msg.pose.pose.orientation.w = math.cos(theta_rad / 2.0)
                    self.odom_pub.publish(odom_msg)

                    # 2. LÓGICA DE MAPEADO (Filtro espacial simple)
                    # Si detectamos una roca y nos hemos movido al menos 10cm desde la última roca guardada
                    distancia_ultima_roca = math.sqrt((x - self.ultima_pos_roca[0])**2 + (y - self.ultima_pos_roca[1])**2)
                    
                    if tipo_roca > 0 and distancia_ultima_roca > 0.1:
                        self.crear_marcador(x, y, "ROCA", tipo_roca)
                        self.ultima_pos_roca = (x, y)
                        self.get_logger().info(f"🟢 Roca {tipo_roca} mapeada en X:{x:.2f}, Y:{y:.2f}")

                    # Si el terreno cambia, marcamos un cuadro en el mapa
                    if tipo_terreno > 0 and tipo_terreno != self.ultimo_terreno_marcado:
                        self.crear_marcador(x, y, "TERRENO", tipo_terreno)
                        self.ultimo_terreno_marcado = tipo_terreno
                        self.get_logger().info(f"🏔️ Terreno {tipo_terreno} mapeado.")
                    elif tipo_terreno == 0:
                        self.ultimo_terreno_marcado = 0 # Reseteamos si vuelve a terreno normal

                    # Banderines (Normalmente solo se verán una vez al inicio y al final)
                    if letrero_inicio == 1:
                        self.crear_marcador(x, y, "LETRERO", 1)
                    if letrero_fin == 1:
                        self.crear_marcador(x, y, "LETRERO", 2)

            except Exception as e:
                pass 

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