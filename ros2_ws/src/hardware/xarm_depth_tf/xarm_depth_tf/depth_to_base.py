import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped
import sensor_msgs_py.point_cloud2 as pc2
from rclpy.duration import Duration
from rclpy.time import Time


class DepthToBase(Node):

    def __init__(self):
        super().__init__('depth_to_base')

        self.subscription = self.create_subscription(
            PointCloud2,
            '/camera/camera/depth/color/points',
            self.pc_callback,
            10
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.get_logger().info('Nodo depth_to_base iniciado.')

    def pc_callback(self, msg):

        
        for point in pc2.read_points(msg, skip_nans=True):
            x, y, z = point[0], point[1], point[2]
            break
        else:
            return

        point_cam = PointStamped()
        point_cam.header.frame_id = msg.header.frame_id
        point_cam.header.stamp = Time().to_msg()  
        point_cam.point.x = x
        point_cam.point.y = y
        point_cam.point.z = z

        try:
            point_base = self.tf_buffer.transform(
                point_cam,
                'link_base',
                timeout=Duration(seconds=0.5)
            )

            self.get_logger().info(
                f'Camera: x={x:.3f}, y={y:.3f}, z={z:.3f}  |  '
                f'Base: x={point_base.point.x:.3f}, '
                f'y={point_base.point.y:.3f}, '
                f'z={point_base.point.z:.3f}'
            )

        except Exception as e:
            self.get_logger().warn(f'TF failed: {str(e)}')


def main():
    rclpy.init()
    node = DepthToBase()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
