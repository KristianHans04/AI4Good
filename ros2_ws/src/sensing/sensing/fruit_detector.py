#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np

from std_msgs.msg import String

class FruitDetector(Node):
    def __init__(self):
        super().__init__('fruit_detector')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(String, '/fruit_detections', 10)
        self.get_logger().info('Fruit detector initialized')

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Define color ranges for fruits (adjusted for accuracy)
        red_lower = np.array([0, 50, 50])
        red_upper = np.array([10, 255, 255])
        black_lower = np.array([0, 0, 0])
        black_upper = np.array([180, 255, 30])
        green_lower = np.array([40, 50, 50])
        green_upper = np.array([80, 255, 255])

        # Create masks
        red_mask = cv2.inRange(hsv, red_lower, red_upper)
        black_mask = cv2.inRange(hsv, black_lower, black_upper)
        green_mask = cv2.inRange(hsv, green_lower, green_upper)

        # Find contours
        red_contours = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
        black_contours = cv2.findContours(black_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
        green_contours = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]

        detections = f"red:{len(red_contours)},black:{len(black_contours)},green:{len(green_contours)}"
        msg = String()
        msg.data = detections
        self.publisher.publish(msg)

        self.get_logger().info(f'Fruit detections: {detections}')

def main(args=None):
    rclpy.init(args=args)
    node = FruitDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()