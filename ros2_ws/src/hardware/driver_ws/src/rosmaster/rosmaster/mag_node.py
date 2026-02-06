#!/usr/bin/env python3

import rclpy
from rclpy.node import Node


class RosmasterMag(Node):

    def __init__(self):
        super().__init__('rosmaster_mag')
        self.get_logger().info('Nodo rosmaster_mag listo (pendiente de tópico)')


def main(args=None):
    rclpy.init(args=args)
    node = RosmasterMag()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
