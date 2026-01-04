#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np

class ColorSensor(Node):
    def __init__(self):
        super().__init__('color_sensor')
        self.subscription = self.create_subscription(
            Image,
            '/color_sensor/image_raw',
            self.image_callback,
            10)
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(String, '/plot_detections', 10)
        self.get_logger().info('Color sensor initialized')

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Define plot colors: Orange, Gray, Green
        orange_lower = np.array([5, 50, 50])
        orange_upper = np.array([15, 255, 255])
        gray_lower = np.array([0, 0, 50])
        gray_upper = np.array([180, 30, 200])
        green_lower = np.array([40, 50, 50])
        green_upper = np.array([80, 255, 255])

        # Create masks
        orange_mask = cv2.inRange(hsv, orange_lower, orange_upper)
        gray_mask = cv2.inRange(hsv, gray_lower, gray_upper)
        green_mask = cv2.inRange(hsv, green_lower, green_upper)

        # Determine dominant color
        orange_pixels = cv2.countNonZero(orange_mask)
        gray_pixels = cv2.countNonZero(gray_mask)
        green_pixels = cv2.countNonZero(green_mask)

        max_pixels = max(orange_pixels, gray_pixels, green_pixels)
        if max_pixels == orange_pixels:
            color = 'Orange'
        elif max_pixels == gray_pixels:
            color = 'Gray'
        else:
            color = 'Green'

        msg = String()
        msg.data = color
        self.publisher.publish(msg)

        self.get_logger().info(f'Detected plot color: {color}')

def main(args=None):
    rclpy.init(args=args)
    node = ColorSensor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()