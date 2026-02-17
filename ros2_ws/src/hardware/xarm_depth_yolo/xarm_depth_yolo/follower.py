import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import TwistStamped

class VisualServoNode(Node):
    def __init__(self):
        super().__init__('visual_servo_node')
        self.Kp = 0.0009        # Ganancia 
        self.MAX_SPEED = 0.2    # Velocidad límite 
        self.DEADBAND = 8.0    
        self.sub_error = self.create_subscription(
            Float32, 
            '/yolo/pixel_error_x', 
            self.error_callback, 
            10
        )

        
        self.pub_vel = self.create_publisher(
            TwistStamped, 
            '/servo_server/delta_twist_cmds', 
            10
        )
        
        
        self.last_msg_time = self.get_clock().now()
        self.timer = self.create_timer(0.1, self.safety_check)

        self.get_logger().info('FOllower')

    def error_callback(self, msg):
        self.last_msg_time = self.get_clock().now()
        error_pixels = msg.data

        
        if abs(error_pixels) < self.DEADBAND:
            self.stop_robot()
            return  

        
        vel_y = self.Kp * error_pixels 

       
        vel_y = max(min(vel_y, self.MAX_SPEED), -self.MAX_SPEED)

        
        self.publish_velocity(vel_y)

    def publish_velocity(self, vy):
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = 'link_base'  

        
        twist.twist.linear.y = float(vy)
        
        
        twist.twist.linear.x = 0.0
        twist.twist.linear.z = 0.0
        twist.twist.angular.x = 0.0
        twist.twist.angular.y = 0.0
        twist.twist.angular.z = 0.0

        self.pub_vel.publish(twist)
        
        
        self.get_logger().info(f'Error: {vy/self.Kp:.1f}px -> Vel Y: {vy:.4f} m/s')

    def stop_robot(self):
        # Manda velocidad 0 para frenar suavemente
        self.publish_velocity(0.0)

    def safety_check(self):
        
        time_diff = (self.get_clock().now() - self.last_msg_time).nanoseconds / 1e9
        if time_diff > 0.5:
            self.stop_robot()

def main(args=None):
    rclpy.init(args=args)
    node = VisualServoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_robot() 
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()