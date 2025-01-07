#!/usr/bin/env python3
 
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32, Int32, Bool, Float32MultiArray
from sensor_msgs.msg import Imu, FluidPressure
from geometry_msgs.msg import Vector3
from pymavlink import mavutil
import time, threading, sys, socket, signal, math
 
 
class TestDepthHoldNode(Node):
    def __init__(self):
        super().__init__('test_depth_hold')
 
        # MAVLink setup
        self.master = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
        self.master.wait_heartbeat()
        self.get_logger().info("Connected to Pixhawk.")
        signal.signal(signal.SIGINT, self.signal_handler)
        
        # QR Data Subscriber
        self.qr_subscriber = self.create_subscription(
            msg_type=Int32, topic='/qr_data', callback=self.callback_qr, qos_profile=10)
               
        # Lights Information Subscriber
        self.light_subscriber = self.create_subscription(
            msg_type=Int32, topic='/light_info', callback=self.callback_lights, qos_profile=10)
 
        # Waypoints Subscriber
        self.waypoints_subscription = self.create_subscription(
            msg_type=Float32MultiArray, topic='/waypoints', callback=self.waypoints_callback, qos_profile=10)
       
        # Depth-Hold State Information Subscriber
        self.depth_hold_subscriber = self.create_subscription(
            msg_type=Bool, topic='/depth_hold_state', callback=self.callback_depth_hold, qos_profile=10)
       
        # Tracking Signal Subscriber
        self.tracking_subscriber = self.create_subscription(
            msg_type=Int32, topic='/tracking_signal', callback=self.callback_track_rope, qos_profile=10)
       
        # Pixhawk Arm/Disarm Information Publisher
        self.pixhawk_arm_publisher = self.create_publisher(
            msg_type=Bool, topic='/pixhawk_arm_state', qos_profile=10)
 
        # Depth Data Publisher
        self.depth_publisher = self.create_publisher(
            msg_type=Float32, topic='/depth_data', qos_profile=10)
        self.timer = self.create_timer(timer_period_sec=0.001, callback=self.publish_imu_depth_data)

       
        # Tracking parameters
        self.is_turning = False
        self.current_turn_direction = 0
        self.screen_center_x = 320.0  # Assume the screen center x-coordinate
        self.straight_confirm_count = 0
        self.STRAIGHT_THRESHOLD = 5  # Threshold for confirming straight direction
        self.center_x = 0.0
        self.yaw = 0.0
 
        # Other Parameters
        self.is_pixhawk_armed = False
        self.is_depth_hold_started = False
        self.previous_depth_hold_state = False
        self.target_depth = -0.4 # meters
        self.qr_info = 9
        self.pressure = 0.0
        self.shutdown_state = False
 
        # Lights Activation Parameters
        self.lights_off_value = 0
        self.lights_on_value = 100
        self.blink_once_value = 1
        self.blink_twice_value = 2
        self.blink_rapid_value = 3
        self.are_lights_on = False

        self.arm_vehicle()

        self.manual_control_timer = self.create_timer(0.1, self.send_last_manual_control)  # 10 Hz
        self.last_manual_control = {"x": 0, "r": 0}

    # Callback Function to Arm/Disarm The Vehicle Using QR Codes
    def callback_qr(self, msg):
       
        '''
        QR Data Interpretation
        9: Disarm the vehicle
        1: Arm the vehicle
        100: No/Invalid QR code shown
        '''
 
        previous_arm = self.is_pixhawk_armed
        previous_depth_hold = self.is_depth_hold_started
        previous_qr = self.qr_info
        self.qr_info = msg.data
        self.get_logger().info(f"Received QR Code: {self.qr_info}")
       
        if self.qr_info != previous_qr:
            if self.qr_info == self.arm_pixhawk_qr:                
                self.are_lights_on = True
                self.is_pixhawk_armed = True
                self.arm_vehicle()
                self.get_logger().info("Vehicle is armed.")
            elif self.qr_info == self.disarm_pixhawk_qr:                
                self.are_lights_on = False
                self.is_depth_hold_started = False
                self.is_pixhawk_armed = False
                self.disarm_vehicle()
                self.get_logger().info("Vehicle is disarmed.")
        elif self.qr_info == self.invalid_qr or self.qr_info == previous_qr:            
            self.is_depth_hold_started = previous_depth_hold
            self.is_pixhawk_armed = previous_arm
 
        # Publish the Current Pixhawk Arm State
        pixhawk_arm_state = Bool()
        pixhawk_arm_state.data = self.is_pixhawk_armed
        self.pixhawk_arm_publisher.publish(pixhawk_arm_state)
        self.get_logger().info(f"Pixhawk Arm State: {self.is_pixhawk_armed}")
        

    # Callback Function to Take Decision About Lights
    def callback_lights(self, msg):
        self.are_lights_on = True
        self.is_pixhawk_armed = True
        
        # Publish the Current Pixhawk Arm State
        pixhawk_arm_state = Bool()
        pixhawk_arm_state.data = self.is_pixhawk_armed
        self.pixhawk_arm_publisher.publish(pixhawk_arm_state)
        self.get_logger().info(f"Pixhawk Arm State: {self.is_pixhawk_armed}")
        '''
        LIGHT DATA INTERPRETATION
        0: Turn off the lights
        100: Turn on the lights
        1: Blink the lights once
        2: Blink the lights twice
        3: Blink the lights 20 times
        '''
 
        lights_state = msg.data
        if self.are_lights_on:
            self.turn_on_lights()
            self.get_logger().info("Lights are on.")
            if lights_state == self.blink_once_value:
                self.blink_lights(n=1, t=1.0)
                self.get_logger().info("Lights are blinking once.")
            elif lights_state == self.blink_twice_value:
                self.blink_lights(n=2, t=0.5)
                self.get_logger().info("Lights are blinking twice.")
            elif lights_state == self.blink_rapid_value:
                self.blink_lights(n=20, t=0.5)
                self.get_logger().info("Lights are blinking rapidly.")
        else:
            self.turn_off_lights()
            self.get_logger().info("Lights are off.")
   
 
    # Callback Function for Taking Decision Regarding Depth Hold
    def callback_depth_hold(self, msg):
        self.get_logger().info(f"Depth Hold State: {msg.data}")
        self.depth_hold_state = msg.data
        if self.depth_hold_state != self.previous_depth_hold_state:
            if self.depth_hold_state:
                self.activate_depth_hold()
                self.get_logger().info("Depth Hold Activated!")
            else:
                self.deactivate_depth_hold()
                self.get_logger().info("Depth Hold Deactivated!")

            
        self.previous_depth_hold_state = self.depth_hold_state
 
 
    # Function to Publish IMU and Depth Data
    def publish_imu_depth_data(self):
        try:
            msg = self.master.recv_match(blocking=False)
            if msg:
                msg_type = msg.get_type()
                # self.get_logger().info(f"Received message type: {msg_type}")
                if msg_type == 'SCALED_PRESSURE2':
                    depth = self.calculate_depth_from_pressure(msg.press_abs)
                    depth_data = Float32()
                    depth_data.data = depth
                    self.depth_publisher.publish(depth_data)
                    self.get_logger().info(f"Depth: {depth:.2f} meters")
        except Exception as e:
            self.get_logger().error(f"Error reading data: {e}")
    
 
    # Function to Calculate Depth from Pressure Data
    def calculate_depth_from_pressure(self, pressure):
        water_density = 1000 # kg/m^3  
        gravity = 9.81 # m/s^2
        reference_pressure = 1010.50 # hPa (Hecto Pascal)
        gauge_pressure = 100*(pressure - reference_pressure) # Pascal
        depth = gauge_pressure / (water_density * gravity)
        return depth

 
    # Callback Function for Waypoints
    def waypoints_callback(self, msg):
        waypoint = msg.data
        self.center_x = waypoint[0]
 
 
    # Rope Tracking Logic
    def callback_track_rope(self, msg):
        # pass
        self.get_logger().info("callback_track_rope funtion in pixhawk_logic node is called!")

        """Main tracking logic to follow the rope."""
        offset_x = self.center_x - self.screen_center_x
        tracking_signal = msg.data

        if not self.is_depth_hold_started:
            self.get_logger().info("Depth Hold is not active. Ignoring tracking signal.")
            return

        if tracking_signal == 1:
            if self.is_turning:
                # Confirm straight movement after completing a turn
                self.straight_confirm_count += 1
                if self.straight_confirm_count >= self.STRAIGHT_THRESHOLD:
                    self.is_turning = False
                    self.current_turn_direction = 0
                    self.straight_confirm_count = 0
                    self.get_logger().info("Finished turning, moving straight.")
                    self.send_manual_control(190, 0)
            else:
                # Adjust offset when moving straight
                if abs(offset_x) < 20:
                    self.send_manual_control(190, 0)
                    self.get_logger().info("Moving straight.")
                else:
                    # Dynamically adjust turn speed based on offset
                    turn_speed = min(200, abs(offset_x) * 2)
                    if offset_x > 0:
                        self.send_manual_control(190, turn_speed)
                        self.get_logger().info("Adjusting right.")
                    else:
                        self.send_manual_control(190, -turn_speed)
                        self.get_logger().info("Adjusting left.")
        elif tracking_signal == 2:
            # Handle turning logic
            self.straight_confirm_count = 0
            self.is_turning = True
            if offset_x > 0:
                self.current_turn_direction = 205
                self.send_manual_control(190, self.current_turn_direction)
                self.get_logger().info("Turning right.")
            else:
                self.current_turn_direction = -205
                self.send_manual_control(190, self.current_turn_direction)
                self.get_logger().info("Turning left.")
        elif tracking_signal == 4:
            self.get_logger().info("Searching for rope.")
            self.send_manual_control(0, 200)  # Rotate right
            # time.sleep(1)
            # self.send_manual_control(0, -200)  # Rotate left
            # time.sleep(1)
        elif tracking_signal == 3:
            self.send_manual_control(0, -1)  # wait
            self.get_logger().info("Holding position.")
 
 
    # Function to Turn On Lights
    def turn_on_lights(self):
        # Turn both lights on (~1900us)
        # The two lights are connected at 'AUX OUT 1' (SERVO9) and 'AUX OUT 10' (SERVO10) pins of Pixhawk.
        self.set_servo(9, 1900)   # Light 1 on SERVO9
        self.set_servo(10, 1900)  # Light 2 on SERVO10
 
   
    # Function to Turn Off Lights
    def turn_off_lights(self):
        # Turn both lights off (~1100us)
        # The two lights are connected at 'AUX OUT 1' (SERVO9) and 'AUX OUT 10' (SERVO10) pins of Pixhawk.
        self.set_servo(9, 1100)   # Light 1 on SERVO9
        self.set_servo(10, 1100)  # Light 2 on SERVO10
 
 
    # Function to Blink Lights 'n' times with a gap of 't' seconds
    def blink_lights(self, n, t):
        self.blink_count = 0
        self.blink_limit = n
        self.blink_interval = t
        self.blink_state = False  # Start with lights off
 
        # Start a timer to control blinking
        self.blink_timer = self.create_timer(self.blink_interval, self.toggle_blink)
 
    def toggle_blink(self):
        if self.blink_count >= self.blink_limit:
            # Stop blinking after reaching the limit
            self.blink_timer.cancel()
            self.turn_on_lights()  # Leave lights on after blinking
            return
 
        # Toggle light state
        if self.blink_state:
            self.turn_off_lights()
        else:
            self.turn_on_lights()
 
        # Update state and count
        self.blink_state = not self.blink_state
        self.blink_count += 0.5  # Half a blink per toggle (on + off = 1 blink)
   

    # Function to Arm the Vehicle
    def arm_vehicle(self):
        # self.is_pixhawk_armed = True
        self.master.arducopter_arm()
        ack = self.master.recv_match(type='COMMAND_ACK', blocking=True, timeout=1)
        if ack and ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            pass
        else:
            print("Arming failed or timed out.")
        manual_mode_id = self.master.mode_mapping()['MANUAL']
        self.master.set_mode(manual_mode_id)
        self.get_logger().info('Vehicle is armed in Manual mode!')
 
 
    # Function to Disarm the Vehicle
    def disarm_vehicle(self):
        manual_mode_id = self.master.mode_mapping()['MANUAL']
        self.master.set_mode(manual_mode_id)
        self.get_logger().info("Vehicle is disarmed in Manual mode!")
        self.master.arducopter_disarm()
        self.master.motors_disarmed_wait()
        
   
    # Function to Activate Depth Hold Mode at Current Depths of The Vehicle
    # def activate_depth_hold(self):
    #     depth_hold_mode = self.master.mode_mapping().get('ALT_HOLD', None)
    #     if depth_hold_mode is None:
    #         self.get_logger().error("Depth Hold mode not available.")
    #         sys.exit(1)
 
    #     self.master.set_mode(depth_hold_mode)
    #     self.get_logger().info("Depth Hold mode activated!")

    #     if self.master.wait_heartbeat().custom_mode == depth_hold_mode:
    #         self.set_target_depth(self.target_depth)
    #     self.get_logger().info(f"Target depth set to {self.target_depth} meters below the surface.")
    #     self.is_depth_hold_started = True
    #     # time.sleep(5)
    #     self.get_logger().info("Depth Hold stabilized at target depth. Starting tracking mode...")
    def activate_depth_hold(self):
        depth_hold_mode = self.master.mode_mapping().get('ALT_HOLD', None)
        if depth_hold_mode is None:
            self.get_logger().error("Depth Hold mode not available.")
            return

        self.get_logger().info("Attempting to activate Depth Hold mode...")
        attempts = 0
        max_attempts = 5

        while attempts < max_attempts:
            self.master.set_mode(depth_hold_mode)
            msg = self.master.recv_match(type='HEARTBEAT', blocking=False)
            if msg and msg.custom_mode == depth_hold_mode:
                self.get_logger().info("Depth Hold mode activated!")
                self.set_target_depth(self.target_depth)
                self.get_logger().info(f"Target depth set to {self.target_depth} meters below the surface.")
                self.is_depth_hold_started = True
                return
            time.sleep(0.5)
            attempts += 1

        self.get_logger().error("Failed to activate Depth Hold mode after multiple attempts.")
 
    # Function to Deactivate Depth Hold Mode
    def deactivate_depth_hold(self):
        self.depth_hold_state = False
        self.is_depth_hold_started = False
        self.disarm_vehicle()
        self.get_logger().info("Deactivated Depth Hold mode.")
 
 
    # Function to Set The Depth for Depth Hold Mode
    def set_target_depth(self, depth):
        self.master.mav.set_position_target_global_int_send(
            0,
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
            (mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE |
             mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE |
             mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE |
             mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE |
             mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
             mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
             mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
             mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
             mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE |
             mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE),
            0, 0, depth,
            0, 0, 0,
            0, 0, 0,
            0, 0)
       
 
   
    def set_servo(self, channel, pwm):
        self.master.mav.command_long_send(
            self.master.target_system,        # target_system
            self.master.target_component,     # target_component
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO, # command
            0,                           # confirmation
            channel,                     # param1: servo channel
            pwm,                         # param2: PWM value
            0,0,0,0,0                   # param3-7 (unused)
        )
 
    # def send_manual_control(self, x, r):
    #     self.master.mav.manual_control_send(
    #         self.master.target_system,
    #         int(x),  # Forward movement # Range [-1000, 1000]
    #         0,       # No lateral movement # Range [-1000, 1000]
    #         500,     # Constant throttle # Range [0, 1000]
    #         int(r),  # Rotation # Range [-1000, 1000]
    #         0        # No button press
    #     )

    def send_manual_control(self, x, r):
        self.last_manual_control["x"] = int(x)
        self.last_manual_control["r"] = int(r)
        self.master.mav.manual_control_send(
            self.master.target_system,
            int(x),  # Forward movement
            0,       # No lateral movement
            500,     # Constant throttle
            int(r),  # Rotation
            0        # No button press
        )

    def send_last_manual_control(self):
        self.master.mav.manual_control_send(
            self.master.target_system,
            self.last_manual_control["x"],
            0,
            500,
            self.last_manual_control["r"],
            0
        )

 
       
    def signal_handler(self, sig, frame):
        """Handle CTRL+C for cleanup."""
        self.get_logger().info("Interrupt received. Disarming ROV and exiting.")        
        try:
            self.shutdown_state = True
            self.turn_off_lights()
            self.depth_hold_state = False
            self.is_depth_hold_started = False
            self.is_pixhawk_armed = False
            self.disarm_vehicle()
            self.master.close()
        except Exception as e:
            self.get_logger().error(f"Error closing Pixhawk connection: {e}")
        sys.exit(0)
   
 
def main(args=None):
    rclpy.init(args=args)
    node = TestDepthHoldNode()
 
    try:
        rclpy.spin(node)
    except Exception as e:
        node.get_logger().error(f"Error occurred: {e}")
        node.turn_off_lights()
        node.depth_hold_state = False
        node.is_depth_hold_started = False
        node.is_pixhawk_armed = False
        node.disarm_vehicle()
    finally:
        node.destroy_node()
        rclpy.shutdown()
 
 
if __name__ == '__main__':
    main()
