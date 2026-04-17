import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import serial


class GyroNode(Node):

    def __init__(self):
        super().__init__('gyro_node')

        # Publisher
        self.publisher_ = self.create_publisher(Int32, 'gyro', 10)

        # Serial (ajusta si cambia el puerto)
        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0)

        # Timer rápido
        self.timer = self.create_timer(0.005, self.read_serial)  # ~200 Hz


    def read_serial(self):
        # Leer TODO lo disponible en el buffer
        while self.ser.in_waiting > 0:
            try:
                line = self.ser.readline().decode('utf-8').strip()

                # Buscar Pitch directamente (más eficiente)
                if "Pitch:" in line:
                    pitch_str = line.split("Pitch:")[1].split()[0]
                    pitch = float(pitch_str)

                    # Clasificación
                    if -20 <= pitch <= 20:
                        estado = 1
                    elif pitch < -20:
                        estado = 0
                    else:
                        estado = 2

                    # Publicar
                    msg = Int32()
                    msg.data = estado
                    self.publisher_.publish(msg)

            except:
                # Ignorar errores de lectura/parsing
                pass


def main(args=None):
    rclpy.init(args=args)

    node = GyroNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()