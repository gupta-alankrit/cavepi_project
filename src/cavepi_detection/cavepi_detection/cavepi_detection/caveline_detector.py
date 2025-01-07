import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float32MultiArray
from cv_bridge import CvBridge
import cv2
import numpy as np


class CaveLineDetector(Node):
    def __init__(self):
        super().__init__('caveline_detector')

        self.subscription = self.create_subscription(
            Image, 'downward_cam', self.image_callback, 10)
        self.pose_publisher = self.create_publisher(String, '/pose', 10)
        self.waypoints_publisher = self.create_publisher(Float32MultiArray, '/waypoints', 10)

        self.bridge = CvBridge()
        self.prev_offset_x = 0  
        self.direction_history = []  

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        frame_height, frame_width = frame.shape[:2]
        screen_center_x = frame_width // 2
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blurred, 50, 150)
        lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=50, minLineLength=100, maxLineGap=20)

        direction = "lost" 
        offset_x = 0
        detected_caveline = False
        rope_lines = []

        if lines is not None:
            rope_center_x = 0
            count = 0
            horizontal_count = 0  
            turning_count = 0  

            for line in lines:
                x1, y1, x2, y2 = line[0]
                length = np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

                if 100 < length < 300:
                    rope_lines.append(((x1, y1), (x2, y2)))
                    rope_center_x += (x1 + x2) // 2
                    count += 1

                    angle = abs(np.arctan2(y2 - y1, x2 - x1) * 180 / np.pi)
                    if 80 < angle < 100:  
                        turning_count += 1
                    elif angle < 10 or angle > 170:  
                        horizontal_count += 1

            if count > 0:
                rope_center_x = rope_center_x // count
                offset_x = rope_center_x - screen_center_x
                detected_caveline = True

                if turning_count > horizontal_count:
                    direction = "turn"
                else:  
                    direction = "straight"

        if not detected_caveline:
            direction = "lost"

        direction = self.smooth_direction(direction)
        offset_x = self.smooth_offset(offset_x) 
        self.publish_pose_and_waypoints(direction, offset_x)


    def smooth_offset(self, offset_x):
        alpha = 0.8  
        smoothed_offset = int(alpha * self.prev_offset_x + (1 - alpha) * offset_x)
        self.prev_offset_x = smoothed_offset
        return smoothed_offset


    def smooth_direction(self, new_direction, threshold=3):
        self.direction_history.append(new_direction)
        if len(self.direction_history) > threshold:
            self.direction_history.pop(0)
        return max(set(self.direction_history), key=self.direction_history.count)


    def publish_pose_and_waypoints(self, direction, offset_x):
        pose_msg = String()
        pose_msg.data = direction
        self.pose_publisher.publish(pose_msg)
        # self.get_logger().info(f"Published /pose: {direction}")
         
        waypoints_msg = Float32MultiArray()
        waypoints_msg.data = [float(offset_x)]
        self.waypoints_publisher.publish(waypoints_msg)
        # self.get_logger().info(f"Published /waypoints: {waypoints_msg.data}")


def main(args=None):
    rclpy.init(args=args)
    node = CaveLineDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()