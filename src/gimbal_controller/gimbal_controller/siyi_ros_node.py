#!/usr/bin/env python3

import sys
import os
from time import sleep
import math

from gimbal_controller.siyi_sdk.siyi_sdk import SIYISDK

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3, Vector3Stamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float32, Float64
from transforms3d.euler import quat2euler

class SiyiGimbalNode(Node):
    def __init__(self):
        super().__init__('siyi_gimbal_node') # Initialize the Node with a name

        # --- Parameters (Optional but good practice) ---
        self.declare_parameter('server_ip', '192.168.144.26') # Default IP
        self.declare_parameter('server_port', 37260)        # Default Port
        self.declare_parameter('publish_rate_hz', 100.0)     # Publish rate
        # Yaw/Pitch direction multipliers can also be parameters if needed
        self.declare_parameter('yaw_direction', 1.0)
        self.declare_parameter('pitch_direction', -1.0)
        # Encoder & aircraft attitude parameters
        self.declare_parameter('enable_encoder_stream', True)
        self.declare_parameter('enable_aircraft_attitude', True)
        self.declare_parameter('encoder_stream_freq', 100)

        server_ip = self.get_parameter('server_ip').get_parameter_value().string_value
        server_port = self.get_parameter('server_port').get_parameter_value().integer_value
        publish_rate = self.get_parameter('publish_rate_hz').get_parameter_value().double_value
        self.yaw_direction = self.get_parameter('yaw_direction').get_parameter_value().double_value
        self.pitch_direction = self.get_parameter('pitch_direction').get_parameter_value().double_value
        self.enable_encoder_stream = self.get_parameter('enable_encoder_stream').get_parameter_value().bool_value
        self.enable_aircraft_attitude = self.get_parameter('enable_aircraft_attitude').get_parameter_value().bool_value
        encoder_stream_freq = self.get_parameter('encoder_stream_freq').get_parameter_value().integer_value

        # --- SIYI SDK Initialization ---
        self.get_logger().info(f"Attempting to connect to SIYI SDK at {server_ip}:{server_port}")
        self.cam = SIYISDK(server_ip=server_ip, port=server_port)
        if not self.cam.connect():
            self.get_logger().error("Failed to connect to SIYI camera. Exiting.")
            # Raise an exception or handle differently if needed, sys.exit() might be too abrupt
            raise ConnectionError("Could not connect to SIYI camera")
        else:
            self.get_logger().info(f"Successfully connected to SIYI camera.")

        try:
            # Request Hardware ID - Important for angle limits
            self.cam.requestHardwareID()
            self.get_logger().info("Requested Hardware ID from camera.")
            # Add a small delay if needed for the camera to process the request
            sleep(0.5)
        except Exception as e:
            self.get_logger().error(f"Error during camera initialization (Hardware ID): {e}")
            self.cam.disconnect()
            raise

        # --- Initialize encoder data stream ---
        # NOTE: 0x26 magnetic encoder stream not supported on A8 mini (ZT30 only).
        # Joint angles are derived from 0x0D + aircraft attitude instead.
        # if self.enable_encoder_stream:
        #     self.cam.requestDataStreamEncoderAngle(encoder_stream_freq)
        #     self.get_logger().info(f"Requested encoder angle stream at {encoder_stream_freq} Hz.")

        # --- ROS2 Publisher and Subscriber ---
        # QoS Profile: KeepLast(10) is similar to queue_size=10
        qos_profile = rclpy.qos.QoSProfile(depth=10)

        self.angle_pub = self.create_publisher(
            Vector3,
            'siyi_gimbal_angles/state_rpy_deg',
            qos_profile)

        # Angular rate state from SDK 0x0D attitude message (getAttitudeSpeed).
        # Published in MAS sign convention (direction multipliers applied),
        # so signs match angle state and rate-command signs.
        self.state_rate_pub = self.create_publisher(
            Vector3,
            'siyi_gimbal_angles/state_rate_rpy_deg',
            qos_profile)

        # Echo of the most recent rate command (raw, as received on
        # gimbal_cmd_los_rate). Exists so bag-based identification can align
        # command and response without needing a separate subscriber snapshot.
        self.cmd_rate_echo_pub = self.create_publisher(
            Vector3,
            'siyi_gimbal_angles/cmd_rate_rpy_norm',
            qos_profile)

        self.angle_sub = self.create_subscription(
            Vector3,
            'siyi_gimbal_angles/command_rpy_deg',
            self.angle_callback,
            qos_profile)

        # Rate command subscriber (0x07 gimbal rotation)
        # Heading-frame LOS rate: gimbal moves camera pointing direction in world frame.
        # In Lock mode: world heading/elevation rate. In Follow mode: yaw=body offset rate.
        self.rate_sub = self.create_subscription(
            Vector3,
            'gimbal_cmd_los_rate',
            self.rate_callback,
            qos_profile)

        # Zoom command subscriber
        self.zoom_sub = self.create_subscription(
            Float32,
            'zoom_cmd',
            self.zoom_callback,
            qos_profile)

        # Encoder angles publisher
        if self.enable_encoder_stream:
            self.encoder_pub = self.create_publisher(
                Vector3,
                'siyi_gimbal_angles/encoder_rpy_deg',
                qos_profile)

        best_effort_qos = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            depth=10)

        # IMU subscriber — aircraft attitude for joint angle derivation + body angular velocity
        self.imu_sub = self.create_subscription(
            Imu,
            'mavros/imu/data',
            self.imu_callback,
            best_effort_qos)

        # Aircraft attitude injection (0x22) + GPS data (0x3E) — subscribe to odom and GPS
        if self.enable_aircraft_attitude:
            self.odom_sub = self.create_subscription(
                Odometry,
                'common_frame/odom',
                self.odom_callback,
                best_effort_qos)
            self.gps_sub = self.create_subscription(
                NavSatFix,
                'mavros/global_position/global',
                self.gps_callback,
                best_effort_qos)

        # --- Combined angular velocity publisher ---
        self.combined_ang_vel_w_pub = self.create_publisher(
            Vector3Stamped, 'combined_ang_vel_w', qos_profile)
        # --- Zoom level publisher ---
        self.zoom_level_pub = self.create_publisher(
            Float64, 'camera/zoom_level', qos_profile)

        # --- State for finite-difference gimbal rate estimation ---
        self._prev_enc_yaw = 0.0
        self._prev_enc_pitch = 0.0
        self._prev_enc_time = self.get_clock().now().nanoseconds / 1e9
        self._body_ang_vel_b = np.zeros(3)
        self._body_quat_wxyz = np.array([1.0, 0.0, 0.0, 0.0])

        # Aircraft attitude cache for joint angle derivation (ENU, degrees)
        # 0x0D reports: yaw=joint(encoder), pitch/roll=heading(world)
        # Raw 0x0D pitch: positive=up, same as ENU pitch convention
        # Joint angle = 0x0D_heading - aircraft_ENU
        self._aircraft_pitch_enu_deg = 0.0
        self._aircraft_roll_enu_deg = 0.0

        # Velocity cache (ENU, m/s) for 0x3E GPS data
        self._velocity_enu = np.zeros(3)

        # IMU cache for 0x22 attitude injection (NED, sent by timer at 100 Hz)
        self._imu_valid = False
        self._imu_roll_ned = 0.0
        self._imu_pitch_ned = 0.0
        self._imu_yaw_ned = 0.0
        self._imu_wx_ned = 0.0
        self._imu_wy_ned = 0.0
        self._imu_wz_ned = 0.0
        self._imu_time_ms = 0

        # --- ROS2 Timer for Periodic Publishing ---
        timer_period = 1.0 / publish_rate  # seconds
        self.timer = self.create_timer(timer_period, self.publish_angles_callback)

        # --- 0x22 attitude injection timer (100 Hz, only when enabled) ---
        if self.enable_aircraft_attitude:
            self.att_inject_timer = self.create_timer(0.01, self.attitude_inject_callback)

        self.get_logger().info(f"Siyi Gimbal Node Started. Publishing state at {publish_rate} Hz.")

    def angle_callback(self, msg: Vector3):
        """Callback function for receiving angle commands."""

        # Assuming msg.data = [roll_cmd, pitch_cmd, yaw_cmd] (even if roll is unused)
        # Apply direction multipliers
        target_yaw_deg = msg.z * self.yaw_direction
        target_pitch_deg = msg.y * self.pitch_direction

        self.get_logger().debug(f"Received command: Roll={msg.x:.2f}, Pitch={msg.y:.2f}, Yaw={msg.z:.2f}")
        self.get_logger().debug(f"Sending to camera: Pitch={target_pitch_deg:.2f}, Yaw={target_yaw_deg:.2f}")

        try:
            # Send command to the camera
            # Note: SDK function might expect yaw, pitch order
            self.cam.requestSetAngles(target_yaw_deg, target_pitch_deg)
        except Exception as e:
            self.get_logger().error(f"Failed to send angle command to camera: {e}")

    def rate_callback(self, msg: Vector3):
        """Heading-frame LOS rate command via 0x07.
        Input: x=yaw_rate, y=pitch_rate (normalized -1..1).
        0x07 is heading-frame: gimbal internally handles joint actuation + stabilization.
        Positive yaw=pan left, positive pitch=tilt down. Send 0 to stop.
        """
        try:
            # Scale from [-1,1] to [-100,100] int8, clamp
            yaw_speed = max(-100, min(100, int(msg.x * 100)))
            pitch_speed = max(-100, min(100, int(msg.y * 100)))
            self.cam.requestGimbalSpeed(yaw_speed, pitch_speed)
            # Echo the received command on a side topic for rosbag-based
            # identification (keeps cmd/response timing aligned in the bag).
            echo = Vector3()
            echo.x = float(msg.x)
            echo.y = float(msg.y)
            echo.z = float(msg.z)
            self.cmd_rate_echo_pub.publish(echo)
        except Exception as e:
            self.get_logger().error(f"Failed to send rate command: {e}")

    def zoom_callback(self, msg: Float32):
        """Zoom rate command. Positive=zoom in, negative=zoom out, 0=stop."""
        try:
            zoom_val = msg.data
            if zoom_val > 0:
                self.cam.requestZoomIn()
            elif zoom_val < 0:
                self.cam.requestZoomOut()
            else:
                self.cam.requestZoomHold()
        except Exception as e:
            self.get_logger().error(f"Failed to send zoom command: {e}")

    def publish_angles_callback(self):
        """Callback function called by the timer to publish current angles."""
        try:
            # Get current attitude from the camera
            # The SDK returns (yaw, pitch, roll) - confirm this order
            current_yaw, current_pitch, current_roll = self.cam.getAttitude()

            # Create the message
            angles_msg = Vector3()

            # Apply direction multipliers for consistent output frame
            # Output format [roll, pitch, yaw] degrees
            angles_msg.x = float(current_roll) # Assuming roll doesn't need a multiplier
            angles_msg.y = float(current_pitch * self.pitch_direction)
            angles_msg.z = float(current_yaw * self.yaw_direction)


            # Publish the message
            self.angle_pub.publish(angles_msg)
            self.get_logger().debug(f"Published angles: {angles_msg.x}, {angles_msg.y}, {angles_msg.z}")

            # Angular rate from SDK (0x0D fields yaw_speed/pitch_speed/roll_speed,
            # deg/s, SDK convention). Apply direction multipliers so signs match
            # the MAS-frame angle state.
            try:
                yaw_rate_sdk, pitch_rate_sdk, roll_rate_sdk = self.cam.getAttitudeSpeed()
                rate_msg = Vector3()
                rate_msg.x = float(roll_rate_sdk)
                rate_msg.y = float(pitch_rate_sdk * self.pitch_direction)
                rate_msg.z = float(yaw_rate_sdk * self.yaw_direction)
                self.state_rate_pub.publish(rate_msg)
            except Exception as e:
                self.get_logger().debug(f"state_rate publish skipped: {e}")

            # Publish joint-frame angles (derived from 0x0D + aircraft attitude)
            # 0x0D: yaw=joint(encoder), pitch/roll=heading(world)
            # Rotate aircraft attitude into gimbal frame (account for yaw joint offset)
            # then: joint = aircraft_in_gimbal_frame - heading
            if self.enable_encoder_stream:
                joint_yaw = current_yaw   # already joint frame (encoder-based)

                # Rotate aircraft pitch/roll by yaw joint angle into gimbal frame
                yaw_rad = math.radians(current_yaw)
                cos_y = math.cos(yaw_rad)
                sin_y = math.sin(yaw_rad)
                ac_pitch_gimbal = (self._aircraft_pitch_enu_deg * cos_y
                                   + self._aircraft_roll_enu_deg * sin_y)
                ac_roll_gimbal = (-self._aircraft_pitch_enu_deg * sin_y
                                  + self._aircraft_roll_enu_deg * cos_y)

                # 0x0D is NED: pitch_NED = -pitch_ENU, roll_NED = roll_ENU
                # Aircraft is ENU. Different subtraction signs for pitch vs roll.
                joint_pitch = ac_pitch_gimbal - current_pitch
                joint_roll = current_roll - ac_roll_gimbal

                enc_msg = Vector3()
                enc_msg.x = float(joint_roll)
                enc_msg.y = float(joint_pitch * self.pitch_direction)
                enc_msg.z = float(joint_yaw * self.yaw_direction)
                self.encoder_pub.publish(enc_msg)

                # Compute gimbal rates from joint angle finite differences (body-frame, radians)
                enc_yaw_rad = math.radians(enc_msg.z)
                enc_pitch_rad = math.radians(enc_msg.y)
                now = self.get_clock().now().nanoseconds / 1e9
                dt = now - self._prev_enc_time
                if dt > 0.001:
                    gimbal_yaw_rate = (enc_yaw_rad - self._prev_enc_yaw) / dt
                    gimbal_pitch_rate = (enc_pitch_rad - self._prev_enc_pitch) / dt
                else:
                    gimbal_yaw_rate = 0.0
                    gimbal_pitch_rate = 0.0
                self._prev_enc_yaw = enc_yaw_rad
                self._prev_enc_pitch = enc_pitch_rad
                self._prev_enc_time = now

                # Combined angular velocity (body + gimbal) in world frame
                gimbal_ang_vel_b = np.array([0.0, gimbal_pitch_rate, gimbal_yaw_rate])
                combined_b = self._body_ang_vel_b + gimbal_ang_vel_b
                # Rotate body-frame to world-frame via quaternion
                q = self._body_quat_wxyz
                w, u = q[0], q[1:4]
                t = 2.0 * np.cross(u, combined_b)
                combined_w = combined_b + w * t + np.cross(u, t)
                cav_msg = Vector3Stamped()
                cav_msg.header.stamp = self.get_clock().now().to_msg()
                cav_msg.vector.x = float(combined_w[0])
                cav_msg.vector.y = float(combined_w[1])
                cav_msg.vector.z = float(combined_w[2])
                self.combined_ang_vel_w_pub.publish(cav_msg)

            # Publish zoom level from SDK
            try:
                zoom = self.cam.getZoomLevel()
                if zoom is not None:
                    zoom_msg = Float64()
                    zoom_msg.data = float(zoom)
                    self.zoom_level_pub.publish(zoom_msg)
            except Exception:
                pass  # getZoomLevel may not be available on all models

        except TypeError as e:
             self.get_logger().warn(f"Could not get attitude from camera (likely not ready or error): {e}. Check connection and camera status.")
        except Exception as e:
            self.get_logger().error(f"Error getting or publishing angles: {e}")

    def imu_callback(self, msg: Imu):
        """Cache aircraft attitude and angular velocity from MAVROS IMU (ENU)."""
        try:
            # Cache body angular velocity (ENU body-frame)
            av = msg.angular_velocity
            self._body_ang_vel_b = np.array([av.x, av.y, av.z])

            # Cache body quaternion for combined_ang_vel_w rotation
            q = msg.orientation
            self._body_quat_wxyz = np.array([q.w, q.x, q.y, q.z])

            # Extract Euler angles (ENU)
            roll_enu, pitch_enu, yaw_enu = quat2euler([q.w, q.x, q.y, q.z], axes='sxyz')

            # Cache aircraft attitude (ENU, degrees) for joint angle derivation
            self._aircraft_pitch_enu_deg = math.degrees(pitch_enu)
            self._aircraft_roll_enu_deg = math.degrees(roll_enu)

            # Cache NED attitude + rates for 0x22 injection (sent by timer, not here)
            self._imu_roll_ned = roll_enu          # NED roll = ENU roll
            self._imu_pitch_ned = -pitch_enu       # NED pitch = -ENU pitch
            yaw_ned = math.pi / 2.0 - yaw_enu
            self._imu_yaw_ned = math.atan2(math.sin(yaw_ned), math.cos(yaw_ned))
            self._imu_wx_ned = av.x
            self._imu_wy_ned = -av.y
            self._imu_wz_ned = -av.z
            self._imu_time_ms = (msg.header.stamp.sec * 1000 + msg.header.stamp.nanosec // 1_000_000) % (2**32)
            self._imu_valid = True

        except Exception as e:
            self.get_logger().error(f"Error in imu_callback: {e}")

    def odom_callback(self, msg: Odometry):
        """Cache velocity from odom (ENU) for 0x3E GPS data."""
        try:
            v = msg.twist.twist.linear
            self._velocity_enu = np.array([v.x, v.y, v.z])
        except Exception as e:
            self.get_logger().error(f"Error in odom_callback: {e}")

    def gps_callback(self, msg: NavSatFix):
        """Send GPS raw data to gimbal via 0x3E."""
        try:
            time_ms = msg.header.stamp.sec * 1000 + msg.header.stamp.nanosec // 1_000_000

            lat = int(msg.latitude * 1e7)    # degE7
            lon = int(msg.longitude * 1e7)   # degE7
            alt_msl = int(msg.altitude * 100)  # cm
            alt_ellipsoid = alt_msl  # approximate (no undulation available)

            # Velocity ENU → NED (mm/s)
            vn = int(self._velocity_enu[1] * 1000)  # ENU Y (North) → NED X
            ve = int(self._velocity_enu[0] * 1000)  # ENU X (East)  → NED Y
            vd = int(-self._velocity_enu[2] * 1000)  # ENU Z (Up)   → NED -Z

            self.cam.requestSendGPSRawData(
                time_ms, lat, lon, alt_msl, alt_ellipsoid,
                vn, ve, vd)

        except Exception as e:
            self.get_logger().error(f"Error in gps_callback: {e}")

    def attitude_inject_callback(self):
        """Send cached 0x22 attitude to gimbal at 100 Hz. Only sends if IMU data received."""
        if not self._imu_valid:
            return
        try:
            self.cam.requestSendAircraftAttitude(
                self._imu_time_ms,
                self._imu_roll_ned, self._imu_pitch_ned, self._imu_yaw_ned,
                self._imu_wx_ned, self._imu_wy_ned, self._imu_wz_ned)
        except Exception as e:
            self.get_logger().error(f"Error in attitude_inject_callback: {e}")

    def disconnect_camera(self):
        """Safely disconnect the camera."""
        if hasattr(self, 'cam'):
            try:
                self.cam.disconnect()
            except Exception:
                pass

# The original test() function remains unchanged as it doesn't use ROS directly
def test():
    cam = SIYISDK(server_ip="192.168.144.25", port=37260)

    if not cam.connect():
        print("No connection ")
        exit(1)
    cam.requestHardwareID() # Important to get the angles limits defined in cameras.py
    sleep(1)
    target_yaw_deg = 0.5
    target_pitch_deg = -25.0
    cam.requestSetAngles(target_yaw_deg, target_pitch_deg)

    i =0
    while i<10: # Reduced iterations for quicker test
        target_yaw_deg = 90.0 if i%2 == 0 else -90.0
        target_pitch_deg = -25.0 if i%2 == 0 else 25.0
        # cam.requestSetAngles(target_yaw_deg, target_pitch_deg) # Uncomment to test setting angles
        a = cam.getAttitude()
        print(f"Attitude (yaw, pitch, roll): {a}")
        sleep(0.5)
        i += 1

    print('DONE')
    cam.disconnect()

def main(args=None):
    # test() # Uncomment this line ONLY if you want to run the non-ROS test function
    # return   # Exit after test if uncommented

    rclpy.init(args=args)
    siyi_gimbal_node = None
    try:
        siyi_gimbal_node = SiyiGimbalNode()
        rclpy.spin(siyi_gimbal_node)
    except ConnectionError as e:
         if siyi_gimbal_node:
            siyi_gimbal_node.get_logger().error(f"Connection Error: {e}")
         else:
             print(f"Connection Error during initialization: {e}") # Logger not available yet
    except KeyboardInterrupt:
        print("Ctrl+C detected, shutting down.")
    except Exception as e:
        if siyi_gimbal_node:
            siyi_gimbal_node.get_logger().fatal(f"Unhandled exception: {e}")
        else:
            print(f"Unhandled exception during initialization: {e}")
    finally:
        # Cleanup
        if siyi_gimbal_node:
            siyi_gimbal_node.disconnect_camera()
            siyi_gimbal_node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass

if __name__ == "__main__":
    main()