import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
import math
import time

class MapNode(Node):

    def __init__(self):
        super().__init__('map_node')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(0.1, self.read_tf)  # 10 Hz


        #Para el mapa
        self.resolution = 0.1
        self.width = 5 #5 metros de prueba
        self.height = 5 #5 metros de prueba

        self.origin_x = -self.width* self.resolution / 2.0
        self.origin_y = -self.height * self.resolution / 2.0

        self.map_data = [-1] * (self.width * self.height)  # Quien sabe -1 es desconocido, 0 es libre, 100 es ocupado

        
        # Marcar el centro como libre para el inicio
        self.map_data[int(self.height/2) * self.width + int(self.width/2)] = 0
        #Parael path

        self.prev_x = 0.0
        self.prev_y = 0.0
        self.prev_theta = 0.0

        self.path = []
        self.path_map = []

    def read_tf(self):
        try:
            t = self.tf_buffer.lookup_transform(
                "odom",       # frame_id
                "base_link",  # frame_id child
                rclpy.time.Time()
            )

            x = t.transform.translation.x
            y = t.transform.translation.y

            q = t.transform.rotation

            # quathernion to euler (yaw)
            theta = 2 * math.atan2(q.z, q.w)

            if self.prev_x-x > 0.1 or self.prev_y - y > 0.1:
                self.path.append((x, y))
                mx, my = self.world_to_map(x, y)
                self.map_data[my * self.width + mx] = 0  # Marcar como libre en el mapa
                self.path_map.append((mx, my))
                self.prev_x = x
                self.prev_y = y
                self.prev_theta = theta

            if x == self.prev_x and y == self.prev_y and (self.prev_x!= 0.0 and self.prev_y != 0.0):
                
                self.get_logger().info(f"Ruta completa: {self.path}")
                self.get_logger().info(
                f"Fin de la ruta x: {x:.2f}, y: {y:.2f}, theta: {theta:.2f}"
                )
                self.get_logger().info(f"Mapa de la ruta: {self.map_data}")
                return
            else:

                self.get_logger().info(
                f"x: {x:.2f}, y: {y:.2f}, theta: {theta:.2f}"
                )

        except Exception as e:
            self.get_logger().warn(f"No TF available: {str(e)}")

    def world_to_map(self, x, y):
        mx = int((x - self.origin_x) / self.resolution)
        my = int((y - self.origin_y) / self.resolution)
        return mx, my


def main():
    rclpy.init()
    node = MapNode()
    rclpy.spin(node)
    rclpy.shutdown()