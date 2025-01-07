#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray


class RopeDirectionPublisher(Node):
    def __init__(self):
        super().__init__('rope_pose_waypoints_pub')

        self.rope_pose_publisher = self.create_publisher(
            msg_type=String, topic='/pose', qos_profile=10)
        
        self.rope_waypoint_publisher = self.create_publisher(
            msg_type=Float32MultiArray, topic='/waypoints', qos_profile=10)
        
        self.timer = self.create_timer(timer_period_sec=0.1, callback=self.timer_callback)
        self.initial_time = self.get_clock().now()
        self.straight_waypoint = 320.0
        self.right_turn_waypoint = 350.0
        self.long_leg_duration = 12
        self.turn_duration = 4
        self.short_leg_duration = 7

    def timer_callback(self):
        pose_msg = String()
        waypoint_msg = Float32MultiArray()
        # msg.data = "lost"

        current_time = self.get_clock().now()
        elapsed_time = (current_time - self.initial_time).nanoseconds / 1e9
        
        if elapsed_time < 5:
            pose_msg.data = "lost"
            waypoint_msg.data = [self.straight_waypoint]
            
        elif elapsed_time >= 5 and elapsed_time < 37:
            pose_msg.data = "straight"
            waypoint_msg.data = [self.straight_waypoint]
            
        elif elapsed_time >= 37 and elapsed_time < 41:
            pose_msg.data = "turn"
            waypoint_msg.data = [self.right_turn_waypoint]
            
        elif elapsed_time >= 41 and elapsed_time < 48:
            pose_msg.data = "straight"
            waypoint_msg.data = [self.straight_waypoint]
            
        elif elapsed_time >= 48 and elapsed_time < 52:
            pose_msg.data = "turn"
            waypoint_msg.data = [self.right_turn_waypoint]
            
        elif elapsed_time >= 52 and elapsed_time < 64:
            pose_msg.data = "straight"
            waypoint_msg.data = [self.straight_waypoint]
            
        elif elapsed_time >= 64 and elapsed_time < 68:
            pose_msg.data = "turn"
            waypoint_msg.data = [self.right_turn_waypoint]
            
        elif elapsed_time >= 68 and elapsed_time < 75:
            pose_msg.data = "straight"
            waypoint_msg.data = [self.straight_waypoint]
            
        elif elapsed_time >= 75 and elapsed_time < 79:
            pose_msg.data = "turn"
            waypoint_msg.data = [self.right_turn_waypoint]
            
        elif elapsed_time >= 79:
            pose_msg.data = "lost"
            waypoint_msg.data = [self.straight_waypoint]

        self.rope_pose_publisher.publish(pose_msg)
        self.rope_waypoint_publisher.publish(waypoint_msg)



def main(args=None):
    rclpy.init(args=args)

    node = RopeDirectionPublisher()
    try:
        rclpy.spin(node)
    except Exception as e:
        node.get_logger().error('Error: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
