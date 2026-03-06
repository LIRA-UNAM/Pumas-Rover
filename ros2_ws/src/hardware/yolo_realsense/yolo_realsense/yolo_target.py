import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import CameraInfo


class YoloTarget(Node):

    def __init__(self):
        super().__init__('yolo_target')

        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        self.selected_id = None

        self.create_subscription(
            CameraInfo,
            '/camera/camera/color/camera_info',
            self.camera_info_callback,
            10
        )

        self.create_subscription(
            String,
            '/yolo/selected_id',
            self.id_callback,
            10
        )

        self.create_subscription(
            Detection2DArray,
            '/yolo/detections',
            self.detection_callback,
            10
        )

        self.pub = self.create_publisher(
            PointStamped,
            '/yolo/object_point_camera',
            10
        )

        self.get_logger().info("Target node started")


    def camera_info_callback(self, msg):
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]


    def id_callback(self, msg):
        self.selected_id = msg.data


    def detection_callback(self, msg):

        if self.selected_id is None or self.fx is None:
            return

        for det in msg.detections:

            if det.id == self.selected_id:

                u = det.bbox.center.position.x
                v = det.bbox.center.position.y
                z = float(det.header.frame_id)

                X = (u - self.cx) * z / self.fx
                Y = (v - self.cy) * z / self.fy

                pt = PointStamped()
                pt.header = msg.header
                pt.header.frame_id = 'camera_color_optical_frame'
                pt.point.x = float(X)
                pt.point.y = float(Y)
                pt.point.z = float(z)

                self.pub.publish(pt)

                self.get_logger().info(
                    f"[CAMERA] X={X:.3f} Y={Y:.3f} Z={z:.3f}"
                )


def main():
    rclpy.init()
    node = YoloTarget()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
