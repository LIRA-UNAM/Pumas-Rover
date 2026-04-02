import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import Point
from rclpy.time import Time
import math
from std_msgs.msg import Bool

class SearchGoal(Node):
    def __init__(self):
        super().__init__('search_goal_node')

        self.objective_pub = self.create_publisher(Point, 'objective_point', 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(0.2, self.timer_callback)

        self.enable_search = False
        self.enable_sub = self.create_subscription(Bool, 'enable_search', self.enable_callback, 10)

        self.origin_set = False
        self.origin_x = 0.0
        self.origin_y = 0.0

        self.area_size = 10.0
        self.step = 1.0
        self.grid_points = []
        self.point_index = 0
        self.current_target = None
        self.target_sent = False

        self.get_logger().info('SearchGoal iniciado, esperando TF para fijar origen y comenzar exploración 10x10.')

    def get_robot_position(self):
        try:
            t = self.tf_buffer.lookup_transform('odom', 'base_link', Time())
            return (t.transform.translation.x, t.transform.translation.y)
        except Exception as e:
            self.get_logger().debug(f'TF no disponible aún: {e}')
            return None

    def enable_callback(self, msg):
        self.enable_search = msg.data
        if self.enable_search:
            self.get_logger().info('Búsqueda habilitada.')
        else:
            self.get_logger().info('Búsqueda deshabilitada.')

    def generate_grid_points(self):
        half = self.area_size / 2.0
        steps = int(self.area_size / self.step) + 1

        for row in range(steps):
            y = self.origin_y - half + row * self.step
            if row % 2 == 0:
                cols = range(steps)
            else:
                cols = reversed(range(steps))
            for col in cols:
                x = self.origin_x - half + col * self.step
                self.grid_points.append((x, y))

        self.get_logger().info(f'Generados {len(self.grid_points)} puntos de exploración (paso {self.step} m).')

    def publish_next_target(self):
        tx, ty = self.grid_points[self.point_index]
        self.objective_pub.publish(Point(x=tx, y=ty, z=0.0))
        self.current_target = (tx, ty)
        self.target_sent = True
        self.get_logger().info(f'Objetivo publicado [{self.point_index + 1}/{len(self.grid_points)}] -> x={tx:.2f}, y={ty:.2f}')

    def timer_callback(self):
        if not self.enable_search:
            return

        position = self.get_robot_position()
        if position is None:
            return

        if not self.origin_set:
            self.origin_x, self.origin_y = position
            self.origin_set = True
            self.generate_grid_points()
            self.get_logger().info(f'Origen de exploración fijado en odom (x={self.origin_x:.2f}, y={self.origin_y:.2f}).')
            return

        if self.point_index >= len(self.grid_points):
            self.get_logger().info('Exploración completa de área 10x10 finalizada.')
            self.destroy_node()
            return

        if not self.target_sent:
            self.publish_next_target()
            return

        tx, ty = self.current_target
        dist = math.hypot(position[0] - tx, position[1] - ty)
        if dist < 0.25:
            self.get_logger().info(f'Punto alcanzado (dist={dist:.2f}): x={tx:.2f}, y={ty:.2f}')
            self.point_index += 1
            self.target_sent = False


def main(args=None):
    rclpy.init(args=args)
    node = SearchGoal()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()