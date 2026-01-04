#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import time

class MissionController(Node):
    def __init__(self):
        super().__init__('mission_controller')
        self.state = 'INIT'
        self.start_time = time.time()
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.publisher = self.create_publisher(String, '/mission_state', 10)
        self.seed_publisher = self.create_publisher(Bool, '/seed_release', 10)
        self.fruit_publisher = self.create_publisher(String, '/fruit_pick', 10)
        self.fruit_sub = self.create_subscription(String, '/fruit_detections', self.fruit_callback, 10)
        self.plot_sub = self.create_subscription(String, '/plot_detections', self.plot_callback, 10)
        self.current_fruits = {}
        self.active_plots = []  # Assume referee signals or detect
        self.get_logger().info('Mission controller initialized')

    def fruit_callback(self, msg):
        # Parse "red:count,black:count,green:count"
        parts = msg.data.split(',')
        self.current_fruits = {p.split(':')[0]: int(p.split(':')[1]) for p in parts}

    def plot_callback(self, msg):
        self.active_plots = msg.data.split(',')  # Assume list

    def timer_callback(self):
        elapsed = time.time() - self.start_time
        if elapsed >= 120:
            self.state = 'FINISH'
        self.publish_state()
        # State logic
        if self.state == 'INIT':
            self.state = 'SOWING'  # Start sowing
        elif self.state == 'SOWING':
            # Plant seeds based on active plots (simplified)
            if 'Orange' in self.active_plots:
                self.seed_publisher.publish(Bool(data=True))  # Release large seed
            if elapsed > 30:  # After 30s, switch to irrigation
                self.state = 'IRRIGATION'
        elif self.state == 'IRRIGATION':
            # Trigger irrigation for seeded plots
            # Assume logic here
            if elapsed > 60:
                self.state = 'HARVESTING'
        elif self.state == 'HARVESTING':
            # Harvest black first, then red, avoid green
            if self.current_fruits.get('black', 0) > 0:
                self.fruit_publisher.publish(String(data='black'))
            elif self.current_fruits.get('red', 0) > 0:
                self.fruit_publisher.publish(String(data='red'))
            # If no more, finish
            if elapsed > 90:
                self.state = 'FINISH'
        elif self.state == 'FINISH':
            pass

    def publish_state(self):
        msg = String()
        msg.data = self.state
        self.publisher.publish(msg)
        self.get_logger().info(f'Current state: {self.state}')

def main(args=None):
    rclpy.init(args=args)
    node = MissionController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()