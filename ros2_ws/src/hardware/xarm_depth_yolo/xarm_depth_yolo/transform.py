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

        self.sub = self.create_subscription(PointStamped, '/yolo/object_point_camera', self.point_callback, 10)
        self.target_pub = self.create_publisher(PointStamped, '/stone/target_pose_base', 10)
        
        self.get_logger().info('Transformada')

    def point_callback(self, msg: PointStamped):
        try:
            
            transform = self.tf_buffer.lookup_transform(
                'link_base', 
                msg.header.frame_id, 
                Time(), 
                timeout=Duration(seconds=0.1)
            )

            point_base = tf2_geometry_msgs.do_transform_point(msg, transform)

            #TRANSFORMADA DE LA ROCA RESPECTO A LA BASE
            self.get_logger().info(
                f'[LINK_BASE] '
                f'X={point_base.point.x:.3f}, '
                f'Y={point_base.point.y:.3f}, '
                f'Z={point_base.point.z:.3f}'
            )

            self.target_pub.publish(point_base)

        except Exception as e:
            self.get_logger().warn(f'Esperando TF... {e}')

def main():
    rclpy.init()
    node = TransformPointNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()