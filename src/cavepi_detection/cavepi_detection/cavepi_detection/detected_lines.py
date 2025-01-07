#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class DownwardCamVisualizer(Node):
    def __init__(self):
        super().__init__('detected_lines')
        self.bridge = CvBridge()
        
        # Subscriber to the downward_cam topic
        self.subscription = self.create_subscription(
            Image, 'downward_cam', self.image_callback, 10)
        self.subscription  # prevent unused variable warning

        # Initialize OpenCV window
        cv2.namedWindow('Downward Camera - Detected Lines', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Downward Camera - Detected Lines', 640, 480)

        self.get_logger().info('Downward Camera Visualizer Node has been started.')

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f"CV Bridge Error: {e}")
            return

        # Process the image to detect lines
        annotated_frame = self.process_image(frame)

        # Display the annotated image
        cv2.imshow('Downward Camera - Detected Lines', annotated_frame)
        cv2.waitKey(1)  # Necessary for OpenCV to update the window

    def process_image(self, frame):
        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Apply Gaussian Blur
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        # Perform Canny Edge Detection
        edges = cv2.Canny(blurred, 50, 150)

        # Detect lines using Probabilistic Hough Transform
        lines = cv2.HoughLinesP(
            edges,
            rho=1,
            theta=np.pi / 180,
            threshold=50,
            minLineLength=100,
            maxLineGap=20
        )

        # Draw detected lines in green
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

        return frame

    def destroy_node(self):
        # Close OpenCV windows upon shutdown
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = DownwardCamVisualizer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT) received. Shutting down node.')
    except Exception as e:
        node.get_logger().error(f'Unhandled Exception: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
