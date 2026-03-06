
import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
from std_msgs.msg import String


class YoloSelector(Node):

    def __init__(self):
        super().__init__('yolo_selector')

        self.sub = self.create_subscription(
            Detection2DArray,
            '/yolo/detections',
            self.callback,
            10
        )

        self.pub = self.create_publisher(
            String,
            '/yolo/selected_id',
            10
        )

        self.get_logger().info("YOLO IDs")


    def callback(self, msg):

        if len(msg.detections) == 0:
            return

        closest = None
        min_z = 999.0

        for det in msg.detections:
            z = float(det.header.frame_id)

            if z < min_z:
                min_z = z
                closest = det.id

        selected = String()
        selected.data = closest
        self.pub.publish(selected)

        self.get_logger().info(f"Selected ID: {closest} Z={min_z:.3f}")


def main():
    rclpy.init()
    node = YoloSelector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
