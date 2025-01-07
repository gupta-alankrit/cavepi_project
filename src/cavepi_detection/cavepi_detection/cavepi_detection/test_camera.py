#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
import cv2
from cv_bridge import CvBridge
import time

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('test_camera')  # Ensure node name matches launch file
        self.bridge = CvBridge()
        
        # Initialize cameras with retry mechanism
        self.front_camera = self.initialize_camera(0, "front_camera")
        self.downward_camera = self.initialize_camera(2, "downward_camera")

        self.front_publisher = self.create_publisher(Image, 'front_cam', 10)
        self.downward_publisher = self.create_publisher(Image, 'downward_cam', 10)

        self.front_working_pub = self.create_publisher(Bool, '/front_working', 10)
        self.down_working_pub = self.create_publisher(Bool, '/down_working', 10)

        # Timer to publish images at 20 Hz
        self.timer = self.create_timer(0.05, self.publish_images)

        self.is_front_working = True
        self.is_down_working = True

        # Throttle logs: track last log time
        self.last_log_time_front = self.get_clock().now().seconds_nanoseconds()[0]
        self.last_log_time_down = self.get_clock().now().seconds_nanoseconds()[0]

        self.log_interval = 2  # seconds

    def initialize_camera(self, index, camera_name):
        camera = cv2.VideoCapture(index)
        # Set explicit camera properties to stabilize frame rates and resolutions
        camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)    # HIGHLIGHT: Set frame width
        camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)   # HIGHLIGHT: Set frame height
        # camera.set(cv2.CAP_PROP_FPS, 30)             # HIGHLIGHT: Set FPS

        attempt = 0
        max_attempts = 10
        while not camera.isOpened() and attempt < max_attempts:
            self.get_logger().warn(
                f"Cannot open {camera_name} at index {index}. "
                f"Retrying ({attempt + 1}/{max_attempts})..."
            )
            time.sleep(1)
            camera = cv2.VideoCapture(index)
            camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            # camera.set(cv2.CAP_PROP_FPS, 30)
            attempt += 1

        if not camera.isOpened():
            self.get_logger().error(f"Failed to open {camera_name} after {max_attempts} attempts.")
        else:
            self.get_logger().info(f"{camera_name.capitalize()} opened successfully.")
        return camera

    def publish_images(self):
        current_time = self.get_clock().now().seconds_nanoseconds()[0]
        try:
            # Front camera
            if self.front_camera.isOpened():
                self.is_front_working = True
                ret_front, frame_front = self.front_camera.read()
                if ret_front:
                    try:
                        front_msg = self.bridge.cv2_to_imgmsg(frame_front, encoding='bgr8')
                        front_msg.header.stamp = self.get_clock().now().to_msg()
                        self.front_publisher.publish(front_msg)
                        # Throttle INFO logs to once every 2 seconds
                        if current_time - self.last_log_time_front >= self.log_interval:
                            self.get_logger().info('Publishing front camera images.')
                            self.last_log_time_front = current_time
                    except Exception as e:
                        self.get_logger().error(f"Failed to publish front_cam image: {e}")
                else:
                    self.get_logger().warn("Failed to read frame from front_cam.")
            else:
                self.get_logger().error("Front camera is not opened.")
                self.is_front_working = False
                self.front_camera = self.reinitialize_camera(0, "front_camera")

            # Downward camera
            if self.downward_camera.isOpened():
                self.is_down_working = True
                ret_downward, frame_downward = self.downward_camera.read()
                if ret_downward:
                    try:
                        downward_msg = self.bridge.cv2_to_imgmsg(frame_downward, encoding='bgr8')
                        downward_msg.header.stamp = self.get_clock().now().to_msg()
                        self.downward_publisher.publish(downward_msg)
                        # Throttle INFO logs to once every 2 seconds
                        if current_time - self.last_log_time_down >= self.log_interval:
                            self.get_logger().info('Publishing downward camera images.')
                            self.last_log_time_down = current_time
                    except Exception as e:
                        self.get_logger().error(f"Failed to publish downward_cam image: {e}")
                else:
                    self.get_logger().warn("Failed to read frame from downward_cam.")
            else:
                self.get_logger().error("Downward camera is not opened.")
                self.is_down_working = False
                self.downward_camera = self.reinitialize_camera(2, "downward_camera")

            # Publish working status
            front_working_state = Bool()
            front_working_state.data = self.is_front_working
            self.front_working_pub.publish(front_working_state)

            down_working_state = Bool()
            down_working_state.data = self.is_down_working
            self.down_working_pub.publish(down_working_state)

        except Exception as e:
            self.get_logger().error(f"Exception in publish_images: {e}")

    def reinitialize_camera(self, index, camera_name):
        self.get_logger().info(f"Reinitializing {camera_name}...")
        camera = cv2.VideoCapture(index)
        # Set explicit camera properties
        camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        # camera.set(cv2.CAP_PROP_FPS, 30)

        attempt = 0
        max_attempts = 5
        while not camera.isOpened() and attempt < max_attempts:
            self.get_logger().warn(
                f"Failed to reconnect {camera_name}. "
                f"Retrying ({attempt + 1}/{max_attempts})..."
            )
            time.sleep(1)
            camera = cv2.VideoCapture(index)
            camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            # camera.set(cv2.CAP_PROP_FPS, 30)
            attempt += 1

        if camera.isOpened():
            self.get_logger().info(f"{camera_name.capitalize()} reconnected successfully.")
            return camera
        else:
            self.get_logger().error(f"Failed to reconnect {camera_name} after {max_attempts} attempts.")
            return self.__dict__.get(camera_name + '_camera')

    def destroy_node(self):
        if self.front_camera:
            self.front_camera.release()
        if self.downward_camera:
            self.downward_camera.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received, shutting down.")
    except Exception as e:
        node.get_logger().error(f"Unhandled exception: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
