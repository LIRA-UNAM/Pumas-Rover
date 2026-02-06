#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
import math


class AttitudeMinimal(Node):

    def __init__(self):
        super().__init__('attitude_minimal')

        self.create_subscription(Imu, '/imu/data_raw', self.imu_cb, 10)
        self.create_subscription(MagneticField, '/imu/mag', self.mag_cb, 10)

        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        self.alpha_rp = 0.05
        self.alpha_yaw = 0.1

        self.yaw_offset = 0.0
        self.yaw_ref_set = False

        self.get_logger().info(
            'Nodo iniciado.\n'
            'Deja el IMU quieto mirando al frente durante 2 segundos.'
        )

    # ---------------- ACCEL ----------------
    def imu_cb(self, msg):
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z

        pitch = math.atan2(-ax, math.sqrt(ay**2 + az**2))
        roll  = -math.atan2(ay, az)

        self.pitch = self.alpha_rp * pitch + (1 - self.alpha_rp) * self.pitch
        self.roll  = self.alpha_rp * roll  + (1 - self.alpha_rp) * self.roll

        self.print_angles()

    # ---------------- MAG ----------------
    def mag_cb(self, msg):
        mx = msg.magnetic_field.x
        my = msg.magnetic_field.y

        yaw_raw = -math.atan2(my, mx)

        if not self.yaw_ref_set:
            self.yaw_offset = yaw_raw
            self.yaw_ref_set = True
            self.get_logger().info('Yaw cero fijado al frente inicial')
            return

        yaw = yaw_raw - self.yaw_offset
        yaw = math.atan2(math.sin(yaw), math.cos(yaw))

        # ---------- LIMITAR A ±90° ----------
        yaw_deg = math.degrees(yaw)
        yaw_deg = self.fold_yaw_90(yaw_deg)
        yaw = math.radians(yaw_deg)

        self.yaw = self.alpha_yaw * yaw + (1 - self.alpha_yaw) * self.yaw

    # ---------- YAW FOLD ----------
    def fold_yaw_90(self, deg):
        if deg > 90:
            deg = 180 - deg
        elif deg < -90:
            deg = -180 - deg
        return deg

    # ---------------- PRINT ----------------
    def print_angles(self):
        self.get_logger().info(
            f'Roll:{math.degrees(self.roll):+6.1f}°  '
            f'Pitch:{math.degrees(self.pitch):+6.1f}°  '
            f'Yaw:{math.degrees(self.yaw):+6.1f}°'
        )


def main(args=None):
    rclpy.init(args=args)
    node = AttitudeMinimal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
