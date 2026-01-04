#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class FruitPicker(Node):
    def __init__(self):
        super().__init__('fruit_picker')
        self.publisher = self.create_publisher(String, '/fruit_pick', 10)
        self.get_logger().info('Fruit picker initialized')

    def pick_fruit(self, fruit_type):
        # Simulate picking by publishing the fruit type
        msg = String()
        msg.data = fruit_type
        self.publisher.publish(msg)
        self.get_logger().info(f'Picked {fruit_type} fruit')

def main(args=None):
    rclpy.init(args=args)
    node = FruitPicker()
    # For testing
    node.pick_fruit('red')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()