#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from cv_bridge import CvBridge
import cv2

class QRDetector(Node):
    def __init__(self):
        super().__init__('qr_detector')
        self.subscription = self.create_subscription(Image, 'front_cam', self.image_callback, 10)
        self.publisher = self.create_publisher(Int32, 'qr_info', 10)
        self.bridge = CvBridge()
        self.detector = cv2.QRCodeDetector()

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        data, points, _ = self.detector.detectAndDecode(frame)
        
        try:
            if data:
                qr_value = int(data)
                self.publisher.publish(Int32(data=qr_value))
                # self.get_logger().info(f"Published QR Code: {qr_value}")
            else:
                self.publisher.publish(Int32(data=100))
                # self.get_logger().info("No QR Code detected, published 100")
        except ValueError:
            self.publisher.publish(Int32(data=100))
            self.get_logger().warn(f"Non-numeric QR Code data: {data}, published 100")

def main(args=None):
    rclpy.init(args=args)
    node = QRDetector()
    try:
        rclpy.spin(node)
    except Exception as e:
        print(f"Error: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()
