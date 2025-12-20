"""Rover_Node controller."""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

TIME_STEP = 16
MAX_SPEED = 7.0
TURN_COEFFICIENT = 4.0
OBSTACLE_MIN = 0.75   # metros
OBSTACLE_MAX = 1.8    # metros

class RoverRos2(Node):
    """Rover ROS2 controller for Webots."""
    def __init__(self):
        """Initialize the Rover ROS2 controller for Webots."""
        super().__init__("rover_ros2")
        
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.lidar_subscriber = self.create_subscription(LaserScan, '/robot_1_/lidar', self.lidar_callback, 10)
        
        self.obstacle_detected = False
        self.turn_left = True

    def lidar_callback(self, msg):
        """Process lidar data and detect obstacles."""
        num_points = len(msg.ranges)
        front_width = int(num_points * 0.2)
        start = (num_points - front_width) // 2
        end = start + front_width
        front_ranges = msg.ranges[start:end]

        # Filter obstacles in the desired range
        obstacles_in_range = [d for d in front_ranges if OBSTACLE_MIN <= d <= OBSTACLE_MAX]
        self.obstacle_detected = bool(obstacles_in_range)

        if self.obstacle_detected:
            left_ranges = msg.ranges[:num_points // 3]
            right_ranges = msg.ranges[-num_points // 3:]
            avg_left = sum(left_ranges) / len(left_ranges)
            avg_right = sum(right_ranges) / len(right_ranges)
            self.turn_left = avg_left > avg_right

        command_msg = Twist()
        if self.obstacle_detected:
            if self.turn_left:
                command_msg.linear.x = -MAX_SPEED * 0.1
                command_msg.angular.z = 0.0
            else:
                command_msg.linear.x = MAX_SPEED * 0.1
                command_msg.angular.z = 0.0
        else:
            command_msg.linear.x = 0.0
            command_msg.angular.z = MAX_SPEED * 0.1
        self.publisher.publish(command_msg)


def main():
    """Main entry point for the node."""
    rclpy.init()
    rover = RoverRos2()
    rclpy.spin(rover)
    rover.node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()