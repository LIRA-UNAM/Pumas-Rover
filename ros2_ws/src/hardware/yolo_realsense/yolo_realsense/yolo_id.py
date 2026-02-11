import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Int32

class YoloID(Node):

    def __init__(self):
        super().__init__('yolo_id')

        self.sub_pt = self.create_subscription(
            PointStamped,
            '/yolo/objects',
            self.pt_cb,
            50
        )

        self.sub_id = self.create_subscription(
            Int32,
            '/yolo/object_id',
            self.id_cb,
            50
        )

        self.pub = self.create_publisher(
            Int32,
            '/yolo/target_id',
            10
        )

        self.current_id = None
        self.objects = {}

    def id_cb(self, msg):
        self.current_id = msg.data

    def pt_cb(self, msg):
        if self.current_id is None:
            return

        self.objects[self.current_id] = msg.point.z

        if len(self.objects) == 0:
            return

        target = min(self.objects, key=self.objects.get)

        out = Int32()
        out.data = target
        self.pub.publish(out)

def main():
    rclpy.init()
    rclpy.spin(YoloID())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
