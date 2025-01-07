import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32, Int32, Bool, Float32MultiArray
from sensor_msgs.msg import Imu
from pymavlink import mavutil
import time, threading, sys, socket, signal, math


class PixhawkInitiationNode(Node):
    def __init__(self):
        super().__init__('pixhawk_logic')

        '''
        LIGHTS SIGNAL
        0: OFF
        100: ON
        1: Blink lights once
        2: Blink lights twice
        3: Blink lights 20 times
        '''

        # MAVLink setup
        self.master = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
        self.lock = threading.Lock()
        self.master.wait_heartbeat()
        self.get_logger().info("Connected to Pixhawk.")

        # QR Data Subscriber
        self.qr_subscrieber = self.create_subscription(
            msg_type=Int32, topic='/qr_info', callback=self.callback_qr, qos_profile=10)
        
        # Lights Information Subscriber
        self.light_subscriber = self.create_subscription(
            msg_type=Int32, topic='/light_info', callback=self.callback_lights, qos_profile=10)

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
        
        # IMU Data Publisher
        self.imu_publisher = self.create_publisher(
            msg_type=Imu, topic='/imu_data', qos_profile=10)
        
        

        self.pose_subscription = self.create_subscription(String, '/pose', self.pose_callback, 10)
        self.waypoints_subscription = self.create_subscription(
            msg_type=Float32MultiArray, topic='/waypoints', callback=self.waypoints_callback, qos_profile=10)

        # self.current_depth = None
        # self.target_depth = None
        # self.current_pose = "lost"
        # self.depth_hold_active = False
        # self.last_pose_time = time.time()

        # Tracking parameters
        self.is_turning = False
        self.current_turn_direction = 0
        self.screen_center_x = 400
        self.straight_confirm_count = 0
        self.STRAIGHT_THRESHOLD = 10
        self.center_x = 0.0
        self.pose = 'no_pose'
        self.yaw = 0.0

        # Other Parameters
        self.is_pixhawk_armed = False
        self.is_depth_hold_started = False
        self.target_depth = 0.0 # meters
        self.arm_vehicle_signal = 1
        self.disarm_vehicle_signal = 9
        self.qr_info = 0
        

        #threading.Thread(target=self.pixhawk_main_loop, daemon=True).start()
        signal.signal(signal.SIGINT, self.signal_handler)
        # Timer to test reading data from Pixhawk
        self.create_timer(0.5, self.read_pixhawk_data)



    # def pose_callback(self, msg):
    #     self.current_pose = msg.data
    #     current_time = time.time()

    #     if self.current_pose != "lost":
    #         if not self.depth_hold_active and current_time - self.last_pose_time > 10 and self.is_armed():
    #             self.activate_depth_hold()
    #     else:
    #         if self.depth_hold_active and current_time - self.last_pose_time > 10:
    #             self.deactivate_depth_hold()

    #     self.last_pose_time = current_time

        
    # def is_armed(self):
    #     msg = self.master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
    #     if msg:
    #         return msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
    #     return False
    
    


    def waypoints_callback(self, msg):
        waypoint = msg.data
        self.center_x = waypoint[0]
        
    def read_pixhawk_data(self):
        """Test reading data from Pixhawk in a non-threaded way."""
        try:
            with self.lock:
                msg = self.master.recv_match(blocking=True, timeout=1)
            if msg:
                msg_type = msg.get_type()
                self.get_logger().info(f"Received message type: {msg_type}")
                if msg_type == 'SCALED_PRESSURE2':
                    depth = self.calculate_depth_from_pressure(msg.press_abs)
                    self.get_logger().info(f"Depth: {depth:.2f} meters")
                elif msg_type == 'ATTITUDE':
                    self.get_logger().info(f"IMU Data - Roll: {msg.roll}, Pitch: {msg.pitch}, Yaw: {msg.yaw}")
        except Exception as e:
            self.get_logger().error(f"Error reading data: {e}")


    def pose_callback(self, msg):
        self.pose = msg.data


    # Callback Function to Arm/Disarm The Vehicle Using QR Codes
    def callback_qr(self, msg):
        self.get_logger().info(f"QR Code Received: {msg.data}")
        previous_qr = self.qr_info
        self.qr_info = msg.data
        
        
        '''
        QR Data Interpretation
        9: Disarm the vehicle
        1: Arm the vehicle
        100: No QR code shown
        '''

        if self.qr_info != previous_qr:
            if self.qr_info == self.arm_vehicle_signal:
                # Arm the Vehicle
                self.arm_vehicle()
                self.is_pixhawk_armed = True
                self.turn_on_lights()
            elif self.qr_info == self.disarm_vehicle_signal:
                # Disarm the Vehicle
                self.disarm_vehicle()
                self.is_pixhawk_armed = False
                self.turn_off_lights()
        elif self.qr_info == 100:
            self.qr_info = previous_qr

        # Publish the Current Pixhawk Arm State
        pixhawk_arm_state = Bool()
        pixhawk_arm_state.data = self.is_pixhawk_armed
        self.pixhawk_arm_publisher.publish(pixhawk_arm_state)


    # Callback Function to Take Decision About Lights
    def callback_lights(self, msg):
        lights_state = msg.data
        if lights_state == 0:
            self.turn_off_lights()
        elif lights_state == 100:
            self.turn_on_lights()
        elif lights_state == 1:
            self.blink_lights(n=1, t=1.0)
        elif lights_state == 2:
            self.blink_lights(n=2, t=0.5)
        elif lights_state == 3:
            self.blink_lights(n=20, t=0.5)
    

    # Callback Function for Taking Decision Regarding Depth Hold
    def callback_depth_hold(self, msg):
        self.get_logger().info(f"Depth Hold State Received: {msg.data}")
        depth_hold_state = msg.data
        if depth_hold_state:
            self.activate_depth_hold()
        else:
            self.deactivate_depth_hold()


    # Function to Publish Depth Data in ROS Topic
    


    # Function to Calculate Depth from Pressure Data
    def calculate_depth_from_pressure(self, pressure):
        water_density = 1000 # kg/m^3  
        gravity = 9.81 # m/s^2
        reference_pressure = 1013.25  ############ NEED TO CHECK THIS ###################
        depth = (pressure - reference_pressure) / (water_density * gravity / 100)
        return depth
    

    # Callback Function for IMU Data
    


    # Rope Tracking Logic
    def callback_track_rope(self, msg):
        tracking_signal = msg.data
        while True:
            # data, _ = self.sock.recvfrom(1024)
            # direction, center_x, center_y = data.decode().split(',')

            self.center_x = int(self.center_x)
            offset_x = self.center_x - self.screen_center_x

            if tracking_signal == 1:
                self.get_logger().info("Starting rope tracking...")            
    
                if self.direction == "straight" and self.is_turning:
                    self.straight_confirm_count += 1
                    if self.straight_confirm_count >= self.STRAIGHT_THRESHOLD:
                        self.is_turning = False
                        self.current_turn_direction = 0
                        self.straight_confirm_count = 0
                        self.get_logger().info("Finished turning, moving straight.")
                        self.send_manual_control(190, 0)
                elif self.direction == "straight" and not self.is_turning:
                    if abs(offset_x) < 20:
                        self.send_manual_control(190, 0)
                        self.get_logger().info("Moving straight.")
                    else:
                        if offset_x > 0:
                            self.send_manual_control(190, 200)
                            self.get_logger().info("Adjusting right.")
                        else:
                            self.send_manual_control(190, -200)
                            self.get_logger().info("Adjusting left.")
                elif self.direction == "turn":
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
                elif self.direction == "lost" and self.is_turning:
                    self.send_manual_control(190, self.current_turn_direction)
                    self.get_logger().info(f"Lost sight while turning. Continuing to turn {'right' if self.current_turn_direction > 0 else 'left'}.")
                # elif direction == "lost" and not self.is_turning:
                #     self.get_logger().info("Rope lost. Stopping movement.")
                #     self.send_manual_control(0, 0)
                #     time.sleep(0.5)
    
                time.sleep(0.1)
            # elif tracking_signal == 2:

            


    # Function to Turn On Lights
    def turn_on_lights(self):
        # Turn both lights on (~1900us)
        # The two lights are connected at 'AUX OUT 1' (SERVO9) and 'AUX OUT 10' (SERVO10) pins of Pixhawk.
        self.set_servo(9, 1900)   # Light 1 on SERVO9
        self.set_servo(10, 1900)  # Light 2 on SERVO10

    
    # Function to Turn On Lights
    def turn_off_lights(self):
        # Turn both lights off (~1100us)
        # The two lights are connected at 'AUX OUT 1' (SERVO9) and 'AUX OUT 10' (SERVO10) pins of Pixhawk.
        self.set_servo(9, 1100)   # Light 1 on SERVO9
        self.set_servo(10, 1100)  # Light 2 on SERVO10


    # Function to Blink Lights 'n' times with a gap of 't' seconds
    def blink_lights(self, n, t):
        for i in range(n):
            self.turn_off_lights()
            time.sleep(t)
            self.turn_on_lights()
            time.sleep(t)
        self.turn_on_lights()

    
    # Function to Arm the Vehicle
    def arm_vehicle(self):
        self.master.arducopter_arm()
        self.master.motors_armed_wait()
        self.get_logger().info("Vehicle is armed!")


    # Function to Disarm the Vehicle
    def disarm_vehicle(self):
        self.master.arducopter_disarm()
        self.master.motors_disarmed_wait()
        self.get_logger().info("Vehicle is disarmed!")

    
    # Function to Activate Depth Hold Mode at Current Depths of The Vehicle
    def activate_depth_hold(self):
        DEPTH_HOLD = 'ALT_HOLD'
        depth_hold_mode = self.master.mode_mapping().get(DEPTH_HOLD, None)
        if depth_hold_mode is None:
            self.get_logger().error("Depth Hold mode not available.")
            sys.exit(1)

        self.master.set_mode(depth_hold_mode)
        self.get_logger().info("Depth Hold mode activated!")
        
        # if self.current_depth is not None:
        #     self.set_target_depth(self.current_depth)
        #     self.get_logger().info(f"Setting target depth to current depth: {self.current_depth:.2f} meters.")
        # else:
        #     self.get_logger().error("Unable to read current depth, cannot activate Depth Hold.")
        #     return
        self.target_depth = -0.5
        self.set_target_depth(self.target_depth)
        self.get_logger().info(f"Target depth set to {self.target_depth} meters below the surface.")
        time.sleep(5)
        self.get_logger().info("Depth Hold stabilized at target depth. Starting tracking mode...")


    # Function to Deactivate Depth Hold Mode
    def deactivate_depth_hold(self):
        self.depth_hold_active = False
        self.disarm_vehicle()
        self.get_logger().info("Deactivated Depth Hold mode.")


    # Function to Set The Depth for Depth Hold Mode
    def set_target_depth(self, depth):
        self.master.mav.set_position_target_global_int_send(
            0,
            self.master.target_system, self.master.target_component,
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
        """
        Sends a MAV_CMD_DO_SET_SERVO command to set a servo output channel to a given PWM value.
    
        channel: integer servo channel (e.g. 9 for SERVO9, 10 for SERVO10)
        pwm: integer PWM in microseconds (e.g. 1100 for off, 1900 for brightest)
        """
        self.master.mav.command_long_send(
            self.master.target_system,        # target_system
            self.master.target_component,     # target_component
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO, # command
            0,                           # confirmation
            channel,                     # param1: servo channel
            pwm,                         # param2: PWM value
            0,0,0,0,0                   # param3-7 (unused)
        )

    def send_manual_control(self, x, r):
        self.master.mav.manual_control_send(
            self.master.target_system,
            int(x),  # Forward movement # Range [-1000, 1000]
            0,       # No lateral movement # Range [-1000, 1000]
            500,     # Constant throttle # Range [0, 1000]
            int(r),  # Rotation # Range [-1000, 1000]
            0        # No button press
        )

        
    def signal_handler(self, sig, frame):
        """Handle CTRL+C for cleanup."""
        self.get_logger().info("Interrupt received. Disarming ROV and exiting.")
        try:
            self.master.arducopter_disarm()
            self.master.motors_disarmed_wait()
            self.get_logger().info("ROV is disarmed.")
            self.master.close()
        except Exception as e:
            self.get_logger().error(f"Error closing Pixhawk connection: {e}")
        sys.exit(0)


    

def main(args=None):
    rclpy.init(args=args)
    node = PixhawkInitiationNode()

    try:
        rclpy.spin(node)
    except Exception as e:
        node.get_logger().error(f"Error occurred: {e}")
        node.master.arducopter_disarm()
        node.master.motors_disarmed_wait()
        node.get_logger().info("Vehicle is disarmed due to an error.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
