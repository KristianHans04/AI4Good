#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
import math

class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry')
        self.publisher = self.create_publisher(Odometry, '/odom', 10)
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()
        self.wheel_radius = 0.05  # example
        self.wheel_base = 0.3     # example
        self.get_logger().info('Odometry initialized')

    def joint_callback(self, msg):
        # Assume joint_states has left_wheel and right_wheel velocities
        left_vel = 0.0
        right_vel = 0.0
        for i, name in enumerate(msg.name):
            if name == 'left_wheel':
                left_vel = msg.velocity[i]
            elif name == 'right_wheel':
                right_vel = msg.velocity[i]

        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        v = (left_vel + right_vel) * self.wheel_radius / 2
        omega = (right_vel - left_vel) * self.wheel_radius / self.wheel_base

        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt
        self.theta += omega * dt

        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.theta / 2)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2)
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = omega
        self.publisher.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()