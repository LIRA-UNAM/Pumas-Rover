#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class RosmasterImu(Node):

    def __init__(self):
        super().__init__('rosmaster_imu')

        self.subscription = self.create_subscription(
            Imu,
            '/imu/data_raw',
            self.imu_callback,
            10
        )

        self.timer = self.create_timer(0.5, self.timer_callback)
        self.last_msg = None

        self.get_logger().info('Nodo rosmaster_imu iniciado')


    def imu_callback(self, msg):
        self.last_msg = msg


    def timer_callback(self):
        if self.last_msg is None:
            return

        g = self.last_msg.angular_velocity
        a = self.last_msg.linear_acceleration

        self.get_logger().info(
            f'Gyro [rad/s] X:{g.x:.3f} Y:{g.y:.3f} Z:{g.z:.3f} | '
            f'Accel [m/s²] X:{a.x:.2f} Y:{a.y:.2f} Z:{a.z:.2f}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = RosmasterImu()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

