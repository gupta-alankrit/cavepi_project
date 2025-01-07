#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time, math
from std_msgs.msg import String, Int32, Bool, Float32
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
 
 
class AutopilotNode(Node):
    def __init__(self):
        super().__init__('autopilot')
 
        '''
        LIGHTS SIGNAL
        0: OFF
        100: ON
        1: Blink lights once
        2: Blink lights twice
        3: Blink lights 20 times
        '''
 
        # Pixhawk Arm/Disarm Information Subscriber
        self.pixhawk_arm_subscriber = self. create_subscription(
            msg_type=Bool, topic='/pixhawk_arm_state', callback=self.callback_pixhawk_arm, qos_profile=10)
 
        # Rope Status Data Subscriber
        self.motion_direction_subscriber = self.create_subscription(
            msg_type=String, topic='/pose', callback=self.callback_motion_direction, qos_profile=10)
        
        # # IMU Data Subscriber
        # self.imu_subscriber = self.create_subscription(
        #     msg_type=Imu, topic='/imu_data', callback=self.callback_imu, qos_profile=10)
        
        # # Sonar Data Subscriber ####### CHECK THE MESSAGE TYPE AND TOPIC NAME ###########
        # self.imu_subscriber = self.create_subscription(
        #     msg_type=Vector3, topic='/sonar_data', callback=self.callback_sonar, qos_profile=10)
        
        # # Depth Data Subscriber
        # self.depth_subscriber = self.create_subscription(
        #     msg_type=Float32, topic='/depth_data', callback=self.callback_depth, qos_profile=10)

        # # Keyboard Shutdown Information Subscriber
        # self.shutdown_subscriber = self.create_subscription(
        #     msg_type=Bool, topic='/shutdown_state', callback=self.callback_shutdown, qos_profile=10)
        
        # Depth Hold State Information Publisher
        self.depth_hold_publisher = self.create_publisher(
            msg_type=Bool, topic='/depth_hold_state', qos_profile=10)
 
        # Lights Information Publisher
        self.light_publisher = self.create_publisher(
            msg_type=Int32, topic='/light_info', qos_profile=10)
        
        # Tracking Signal Publisher
        self.tracking_publisher = self.create_publisher(
            msg_type=Int32, topic='/tracking_signal', qos_profile=10)


        self.is_pixhawk_armed = False
        self.current_motion_status = "lost"
        self.tracking = 0
        self.lights = 0
        self.is_depth_hold_started = False
        self.is_rope_in_view = False
        
        self.qr_info = 0              
        self.current_depth = 0.0
        self.yaw = 0.0
        
        self.status_change_instant = self.get_clock().now()
 

    def callback_pixhawk_arm(self, msg):
        self.is_pixhawk_armed = msg.data
 
 
    def callback_motion_direction(self, msg):
        previous_motion_status = self.current_motion_status
        previous_tracking = self.tracking
        self.current_motion_status = msg.data

        '''
        TRACKING SIGNAL
        0: lost
        1: straight
        2: turn
        3: wait
        4: search_mode
        '''
        
        # If Pixhawk is not in armed condition.
        if not self.is_pixhawk_armed:
            self.lights = 0
            self.status_change_instant = self.get_clock().now()

        # If Pixhawk is in armed condition.
        else:
            self.lights = 100

            # If pixhawk is armed and depth hold is not started.
            if not self.is_depth_hold_started:

                # If pixhawk is armed, depth hold not started, and current rope position not 'lost'.
                if self.current_motion_status != "lost":

                    # If pixhawk is armed, depth hold not started, and current rope position not 'lost' but previous rope position was 'lost'.
                    if previous_motion_status == "lost":  
                        self.status_change_instant = self.get_clock().now()

                    # If pixhawk is armed, depth hold not started, current and previous rope positions are not 'lost'.
                    else:
                        current_time = self.get_clock().now()
                        elapsed_time = (current_time - self.status_change_instant).nanoseconds/1e9
                        
                        # If 5 seconds have passed for the rope in the view.
                        if elapsed_time >= 5.0:
                            self.lights = 2
                            self.is_rope_in_view = True
                            # time.sleep(5)
                            if elapsed_time >= 10.0:
                                self.is_depth_hold_started = True
                                
                                if elapsed_time >= 10.0 and elapsed_time < 20:
                                    self.tracking = 3  # wait
                                elif elapsed_time >= 20.0:
                                    if self.current_motion_status == "straight":
                                        self.tracking = 1
                                    elif self.current_motion_status == "turn":
                                        self.tracking = 2

            # If pixhawk is armed and depth hold is started.
            else:
                # If pixhawk is armed, depth hold started, and current rope position not 'lost'.
                if self.current_motion_status != "lost":
                    self.is_rope_in_view = True
                    if self.current_motion_status == "straight":
                        self.tracking = 1
                    elif self.current_motion_status == "turn":
                        self.tracking = 2
                    
                    # If pixhawk is armed, depth hold started, and current rope position not 'lost', but previous rope position was 'lost'.
                    if previous_motion_status == "lost":  # If the rope was not in the view previously.
                        self.lights = 2
                        self.get_logger().info("Rope found again!")

                # If pixhawk is armed, depth hold started, and current rope position is 'lost'.
                else:
                    # If pixhawk is armed, depth hold started, and current rope position 'lost', but previous rope position was not 'lost'.
                    if previous_motion_status != "lost":
                        self.status_change_instant = self.get_clock().now()
                        self.tracking = previous_tracking
                        yaw_record = self.yaw
                        
                    # If pixhawk is armed, depth hold started, and current and previous rope positions are 'lost'.
                    else:
                        counter = 0
                        current_time = self.get_clock().now()
                        elapsed_time = (current_time - self.status_change_instant).nanoseconds/1e9

                        # If rope is lost for more than 3 seconds, inform the vehicle.
                        if elapsed_time >= 3.0:
                            self.is_rope_in_view = False
                            self.tracking = 4
                            if counter == 0:                                
                                self.lights = 1 # Blink lights once
                                counter += 1
                                
 
                            # # TURN THE ROBOT +/-120 deg. (2 CYCLES) TO SEARCH THE ROPE
                            # self.tracking = 3
                            # self.lights = 3 # Blink lights 20 times
                            # # if self.yaw - yaw_record >= math.radians(120):
                            # time.sleep(5)
                            # self.tracking = 0
                            # self.is_depth_hold_started = False
                            # self.is_pixhawk_armed = False    
 
                            # # TURN THE ROBOT +/-120 deg. (2 CYCLES) TO SEARCH THE ROPE
                            # self.tracking = 3
                            # self.lights = 3 # Blink lights 20 times
                            # self.status_change_instant = self.get_clock().now()
                            # if (current_time - self.status_change_instant).nanoseconds/1e9 >= 30.0:
                            #     self.lights = 3 # Blink lights 20 times
                            #     status_change_instant = self.get_clock().now()
                            #     if (current_time - self.status_change_instant).nanoseconds/1e9 >= 30.0:
                            #         self.lights = 3 # Blink lights 20 times
                            #         self.is_depth_hold_started = False
                            #         self.tracking = 0
 
 
        
        # Publish Depth-Hold Data
        depth_hold_state = Bool()
        depth_hold_state.data = self.is_depth_hold_started
        self.depth_hold_publisher.publish(depth_hold_state)
 
        # Publish light_info data
        light_info = Int32()
        light_info.data = self.lights
        self.light_publisher.publish(light_info)
 
        # Publish Tracking Signal Data
        tracking_signal = Int32()
        tracking_signal.data = self.tracking
        self.tracking_publisher.publish(tracking_signal)
 

    # Callback Function for IMU Data
    def callback_imu(self, msg):
        self.yaw = msg.orientation.z


    # Callback Function of Sonar Data
    def callback_sonar(self, msg):
        pass
 
    
    # Callback Function of Pressure-Depth Sensor Data
    def callback_depth(self, msg):
        self.current_depth = msg.data

    
    # # Callback Function for Handling Keyboard Shutdown
    # def callback_shutdown(self, msg):
    #     shutdown_state = msg.data
    #     self.get_logger().info('Keyboard interruption, exiting autopilot node!')
    #     if shutdown_state:
    #         rclpy.shutdown()
 
                
 
def main(args=None):
    rclpy.init(args=args)
    node = AutopilotNode()   
 
    try:
        rclpy.spin(node)
    except Exception as e:
        node.get_logger().error(f"Error occurred: {e}")
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
 
if __name__ == '__main__':
    main()
