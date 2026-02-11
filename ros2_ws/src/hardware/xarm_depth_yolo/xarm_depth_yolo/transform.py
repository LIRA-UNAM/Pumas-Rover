import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import PointStamped
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs


class TransformPointNode(Node):

    def __init__(self):
        super().__init__('transform_point_node')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.sub = self.create_subscription(
            PointStamped,
            '/yolo/object_point_camera',
            self.point_callback,
            10
        )

        self.get_logger().info('TF point transform node started')


    def point_callback(self, msg: PointStamped):

        if not self.tf_buffer.can_transform(
            'link_base',
            msg.header.frame_id,
            msg.header.stamp,
            timeout=Duration(seconds=0.2)
        ):
            self.get_logger().warn('TF not available yet')
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                'link_base',
                msg.header.frame_id,
                msg.header.stamp,
                timeout=Duration(seconds=0.5)
            )

            point_base = tf2_geometry_msgs.do_transform_point(
                msg,
                transform
            )

            self.get_logger().info(
                f'[BASE_LINK] '
                f'X={point_base.point.x:.3f}, '
                f'Y={point_base.point.y:.3f}, '
                f'Z={point_base.point.z:.3f}'
            )

        except Exception as e:
            self.get_logger().warn(f'TF failed: {e}')


def main():
    rclpy.init()
    node = TransformPointNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()