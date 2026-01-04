#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class Navigator(Node):
    def __init__(self):
        super().__init__('navigator')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        self.current_pose = None
        self.get_logger().info('Navigator initialized')

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def move_to(self, x, y):
        if self.current_pose is None:
            self.get_logger().warn('No odometry data')
            return

        # Check field boundaries (2362mm x 1143mm, approx 2.36m x 1.14m, half field)
        field_width = 1.18
        field_height = 0.57
        if abs(x) > field_width or abs(y) > field_height:
            self.get_logger().warn('Target out of field boundaries')
            return

        # Simple proportional control to move to (x,y)
        dx = x - self.current_pose.position.x
        dy = y - self.current_pose.position.y
        distance = math.sqrt(dx**2 + dy**2)

        if distance < 0.1:  # close enough
            twist = Twist()
            self.publisher.publish(twist)
            return

        # Compute angle
        angle = math.atan2(dy, dx)
        current_angle = self.get_yaw(self.current_pose.orientation)
        angle_diff = angle - current_angle

        # Normalize angle
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi

        twist = Twist()
        twist.linear.x = 0.2 if abs(angle_diff) < 0.1 else 0.0
        twist.angular.z = 0.5 * angle_diff
        self.publisher.publish(twist)

    def get_yaw(self, orientation):
        # Convert quaternion to yaw
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
        return math.atan2(siny_cosp, cosy_cosp)

def main(args=None):
    rclpy.init(args=args)
    node = Navigator()
    # For testing, move to (1,0)
    node.move_to(1.0, 0.0)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()