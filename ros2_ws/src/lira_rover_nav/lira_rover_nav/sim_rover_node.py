#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Twist
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster

class LiraSimRoverNode(Node):
    """
    Simulador cinematico y de sensor LiDAR 2D para el Rover del LIRA.
    1. Integra la odometria en SE(2) a partir de /cmd_vel (o del bus CAN).
    2. Publica el topico /odom y la transformacion dinamica TF2: odom -> base_link.
    3. Genera un LaserScan sincronizado en /scan simulando la RectangleArena y obstaculos fijos.
    """
    def __init__(self):
        super().__init__('lira_sim_rover_node')

        # Parametros de configuracion
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('laser_frame', 'laser_frame')
        self.declare_parameter('enable_internal_lidar', True)

        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.laser_frame = self.get_parameter('laser_frame').value
        self.enable_internal_lidar = self.get_parameter('enable_internal_lidar').value

        # Publicadores y Broadcaster TF2
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Suscriptor a comandos de velocidad (/cmd_vel)
        self.cmd_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10
        )

        # Estado cinematico del Rover (x, y, yaw) en el marco odom
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.vx = 0.0
        self.wz = 0.0
        self.last_time = self.get_clock().now()

        # Obstaculos cilindricos fijos en el mundo: (x_obs, y_obs, radio)
        self.obstacles = [
            (1.6, 0.0, 0.20),   # Obstaculo frontal a 1.6 m
            (1.2, 1.2, 0.25),   # Obstaculo diagonal izquierdo
            (-1.2, -1.0, 0.25)  # Obstaculo trasero derecho
        ]
        self.arena_half_size = 4.0  # Paredes de la RectangleArena de 8x8 m

        # Timers: Odometria a 30 Hz y LiDAR a 10 Hz
        self.update_odometry()
        self.odom_timer = self.create_timer(1.0 / 30.0, self.update_odometry)
        if self.enable_internal_lidar:
            self.scan_timer = self.create_timer(1.0 / 10.0, self.publish_simulated_scan)

        self.get_logger().info('Simulador del Rover LIRA (Odometria + LiDAR Virtual) ACTIVO.')

    def cmd_vel_callback(self, msg: Twist):
        self.vx = msg.linear.x
        self.wz = msg.angular.z

    def update_odometry(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        # Integracion de Runge-Kutta de 2do orden (Punto medio) en SE(2)
        delta_yaw = self.wz * dt
        mid_yaw = self.yaw + 0.5 * delta_yaw
        self.x += self.vx * math.cos(mid_yaw) * dt
        self.y += self.vx * math.sin(mid_yaw) * dt
        self.yaw = math.atan2(math.sin(self.yaw + delta_yaw), math.cos(self.yaw + delta_yaw))

        # Cuaternion a partir de yaw
        qz = math.sin(self.yaw * 0.5)
        qw = math.cos(self.yaw * 0.5)

        # 1. Broadcast TF2: odom -> base_link
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)

        # 2. Publicar mensaje /odom
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.angular.z = self.wz
        self.odom_pub.publish(odom)

    def publish_simulated_scan(self):
        now = self.get_clock().now()
        scan = LaserScan()
        scan.header.stamp = now.to_msg()
        scan.header.frame_id = self.laser_frame

        num_readings = 180
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = (2.0 * math.pi) / num_readings
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = 0.12
        scan.range_max = 6.0

        # Posicion global del LiDAR (montado 0.28 m adelante de base_link)
        laser_x = self.x + 0.28 * math.cos(self.yaw)
        laser_y = self.y + 0.28 * math.sin(self.yaw)

        ranges = []
        for i in range(num_readings):
            local_angle = scan.angle_min + i * scan.angle_increment
            global_angle = self.yaw + local_angle
            dx = math.cos(global_angle)
            dy = math.sin(global_angle)

            min_dist = scan.range_max

            # 1. Interseccion con las 4 paredes de la RectangleArena (-4 a +4 m)
            if abs(dx) > 1e-6:
                t_wall_x = ((self.arena_half_size if dx > 0 else -self.arena_half_size) - laser_x) / dx
                if 0.0 < t_wall_x < min_dist:
                    min_dist = t_wall_x
            if abs(dy) > 1e-6:
                t_wall_y = ((self.arena_half_size if dy > 0 else -self.arena_half_size) - laser_y) / dy
                if 0.0 < t_wall_y < min_dist:
                    min_dist = t_wall_y

            # 2. Interseccion exacta rayo-circulo con los obstaculos fijos
            for (ox, oy, rad) in self.obstacles:
                fx = laser_x - ox
                fy = laser_y - oy
                b = 2.0 * (fx * dx + fy * dy)
                c = (fx * fx + fy * fy) - (rad * rad)
                discriminant = b * b - 4.0 * c
                if discriminant >= 0.0:
                    t_hit = (-b - math.sqrt(discriminant)) / 2.0
                    if scan.range_min < t_hit < min_dist:
                        min_dist = t_hit

            ranges.append(float(min_dist))

        scan.ranges = ranges
        self.scan_pub.publish(scan)

def main(args=None):
    rclpy.init(args=args)
    node = LiraSimRoverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()

