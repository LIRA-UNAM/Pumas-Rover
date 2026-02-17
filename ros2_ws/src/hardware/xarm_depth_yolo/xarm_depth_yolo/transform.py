import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time

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

        self.get_logger().info('TRANSFORMADA')

    def point_callback(self, msg: PointStamped):

        
        try:
           
            transform = self.tf_buffer.lookup_transform(
                'link_base',          
                msg.header.frame_id,  
                Time(),               
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
            self.get_logger().warn(
                f'Esperando TF... {e}'
            )


def main():
    rclpy.init()
    node = TransformPointNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
