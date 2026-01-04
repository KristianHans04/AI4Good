#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class SeedPlanter(Node):
    def __init__(self):
        super().__init__('seed_planter')
        self.publisher = self.create_publisher(Bool, '/seed_release', 10)
        self.get_logger().info('Seed planter initialized')

    def plant_seed(self):
        # Simulate planting by publishing True to release seed
        msg = Bool()
        msg.data = True
        self.publisher.publish(msg)
        self.get_logger().info('Seed planted')

def main(args=None):
    rclpy.init(args=args)
    node = SeedPlanter()
    # For testing
    node.plant_seed()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()