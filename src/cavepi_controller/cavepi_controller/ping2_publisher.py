import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
from brping import Ping1D
import time


class Ping2Publisher(Node):
    def __init__(self):
        super().__init__('ping2_publisher')

        self.distance_publisher = self.create_publisher(Float32, '/distance', 10)
        self.accuracy_publisher = self.create_publisher(Bool, '/measurement_accuracy', 10)

        self.device = '/dev/ttyUSB0'  
        self.baudrate = 115200
        self.ping = Ping1D()

        if not self.ping.connect_serial(self.device, self.baudrate):
            self.get_logger().error(f"Failed to connect to Ping device on {self.device}")
            rclpy.shutdown()
            return

        if not self.ping.initialize():
            self.get_logger().error("Failed to initialize Ping!")
            rclpy.shutdown()
            return

        self.get_logger().info("Ping device initialized successfully.")

        self.last_valid_distance = None
        self.low_confidence_start_time = None

        self.timer = self.create_timer(0.1, self.get_ping_data)

    def get_ping_data(self):
        data = self.ping.get_distance()
        if data:
            confidence = data["confidence"]
            distance = data["distance"]

            if confidence >= 90:
                self.last_valid_distance = distance
                self.low_confidence_start_time = None

                distance_msg = Float32()
                distance_msg.data = distance
                self.distance_publisher.publish(distance_msg)

                self.get_logger().info(f"Published distance: {distance} mm (Confidence: {confidence}%)")
            else:
                self.get_logger().warn(f"Low confidence: {confidence}%. Retaining last valid distance.")

                if self.low_confidence_start_time is None:
                    self.low_confidence_start_time = self.get_clock().now()

                elapsed_time = (self.get_clock().now() - self.low_confidence_start_time).nanoseconds / 1e9

                if elapsed_time >= 10:
                    accuracy_msg = Bool()
                    accuracy_msg.data = False
                    self.accuracy_publisher.publish(accuracy_msg)

                    self.get_logger().error("Measurement accuracy degraded for 10 seconds.")
                elif self.last_valid_distance is not None:
                    distance_msg = Float32()
                    distance_msg.data = self.last_valid_distance
                    self.distance_publisher.publish(distance_msg)
        else:
            self.get_logger().error("Failed to get distance data from Ping device.")


def main(args=None):
    rclpy.init(args=args)
    node = Ping2Publisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
