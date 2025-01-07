import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
import socket
import json
import threading

class DataReceiver(Node):
   def __init__(self):
       super().__init__('data_receiver')
    
       self.udp_ip = "0.0.0.0"
       self.udp_port = 5005
       self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
       self.sock.bind((self.udp_ip, self.udp_port))
       
       self.qr_publisher = self.create_publisher(String, '/qr_data', 10)
       self.pose_publisher = self.create_publisher(String, '/pose_data', 10)
       self.waypoints_publisher = self.create_publisher(Float32MultiArray, '/waypoints_data', 10)
       self.get_logger().info('Data Receiver node running!')
       
       self.thread = threading.Thread(target=self.receive_udp_data)
       self.thread.daemon = True
       self.thread.start()       
   
   def receive_udp_data(self):       
       while True:
           try:
               self.get_logger().info('Data receiving!')
               data, addr = self.sock.recvfrom(1024)               
               message = json.loads(data.decode('utf-8'))
               data_type = message.get('type')
               payload = message.get('data')
               
               if data_type == 'qr_info':
                   self.publish_qr_data(payload)
               elif data_type == 'pose':
                   self.publish_pose_data(payload)
               elif data_type == 'waypoints':
                   self.publish_waypoints_data(payload)
               else:
                   self.get_logger().warn(f"Unknown data type received: {data_type}")
           except Exception as e:
               self.get_logger().error(f"Error receiving or processing data: {e}")
   def publish_qr_data(self, data):
       msg = String()
       msg.data = data.get('qr_data', '')
       self.qr_publisher.publish(msg)
       self.get_logger().info(f"Published QR data: {msg.data}")
   def publish_pose_data(self, data):
       msg = String()
       msg.data = data.get('direction', '')
       self.pose_publisher.publish(msg)
       self.get_logger().info(f"Published Pose data: {msg.data}")
   def publish_waypoints_data(self, data):
       msg = Float32MultiArray()
       x = data.get('x', 0.0)
       y = data.get('y', 0.0)
       msg.data = [x, y]
       self.waypoints_publisher.publish(msg)
       self.get_logger().info(f"Published Waypoints data: x={x}, y={y}")

def main(args=None):
   rclpy.init(args=args)
   node = DataReceiver()
   try:
       rclpy.spin(node)
   except KeyboardInterrupt:
       pass
   finally:
       node.destroy_node()
       rclpy.shutdown()

if __name__ == '__main__':
   main()
