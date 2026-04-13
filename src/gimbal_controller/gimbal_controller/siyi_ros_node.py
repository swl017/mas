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
from std_msgs.msg import Float64
from tf_transformations import euler_from_quaternion

class SiyiGimbalNode(Node):
    def __init__(self):
        super().__init__('siyi_gimbal_node') # Initialize the Node with a name

        # --- Parameters (Optional but good practice) ---
        self.declare_parameter('server_ip', '192.168.144.25') # Default IP
        self.declare_parameter('server_port', 37260)        # Default Port
        self.declare_parameter('publish_rate_hz', 25.0)      # Publish rate
        # Yaw/Pitch direction multipliers can also be parameters if needed
        self.declare_parameter('yaw_direction', 1.0)
        self.declare_parameter('pitch_direction', -1.0)
        # Encoder & aircraft attitude parameters
        self.declare_parameter('enable_encoder_stream', True)
        self.declare_parameter('enable_aircraft_attitude', True)
        self.declare_parameter('encoder_stream_freq', 50)

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
        if self.enable_encoder_stream:
            self.cam.requestDataStreamEncoderAngle(encoder_stream_freq)
            self.get_logger().info(f"Requested encoder angle stream at {encoder_stream_freq} Hz.")

        # --- ROS2 Publisher and Subscriber ---
        # QoS Profile: KeepLast(10) is similar to queue_size=10
        qos_profile = rclpy.qos.QoSProfile(depth=10)

        self.angle_pub = self.create_publisher(
            Vector3,
            'siyi_gimbal_angles/state_rpy_deg',
            qos_profile)

        self.angle_sub = self.create_subscription(
            Vector3,
            'siyi_gimbal_angles/command_rpy_deg',
            self.angle_callback,
            qos_profile)

        # Encoder angles publisher
        if self.enable_encoder_stream:
            self.encoder_pub = self.create_publisher(
                Vector3,
                'siyi_gimbal_angles/encoder_rpy_deg',
                qos_profile)

        # Aircraft attitude injection — subscribe to odom
        if self.enable_aircraft_attitude:
            best_effort_qos = rclpy.qos.QoSProfile(
                reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                depth=10)
            self.odom_sub = self.create_subscription(
                Odometry,
                'common_frame/odom',
                self.odom_callback,
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

        # --- ROS2 Timer for Periodic Publishing ---
        timer_period = 1.0 / publish_rate  # seconds
        self.timer = self.create_timer(timer_period, self.publish_angles_callback)

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

            # Publish encoder angles if enabled
            if self.enable_encoder_stream:
                enc_yaw, enc_pitch, enc_roll = self.cam.getGimbalEncoderAngles()
                enc_msg = Vector3()
                enc_msg.x = float(enc_roll)
                enc_msg.y = float(enc_pitch * self.pitch_direction)
                enc_msg.z = float(enc_yaw * self.yaw_direction)
                self.encoder_pub.publish(enc_msg)

                # Compute gimbal rates from encoder finite differences (body-frame, radians)
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

    def odom_callback(self, msg: Odometry):
        """Convert ENU odom to NED and send aircraft attitude to gimbal (0x22)."""
        try:
            # Cache body angular velocity and quaternion for combined_ang_vel_w
            av = msg.twist.twist.angular
            self._body_ang_vel_b = np.array([av.x, av.y, av.z])
            q = msg.pose.pose.orientation
            self._body_quat_wxyz = np.array([q.w, q.x, q.y, q.z])

            # Extract quaternion from odom
            roll_enu, pitch_enu, yaw_enu = euler_from_quaternion([q.x, q.y, q.z, q.w])

            # ENU → NED conversion
            roll_ned = roll_enu
            pitch_ned = -pitch_enu
            yaw_ned = math.pi / 2.0 - yaw_enu
            # Wrap yaw to [-pi, pi]
            yaw_ned = math.atan2(math.sin(yaw_ned), math.cos(yaw_ned))

            # Angular velocities: ENU → NED
            wx_ned = msg.twist.twist.angular.x
            wy_ned = -msg.twist.twist.angular.y
            wz_ned = -msg.twist.twist.angular.z

            # Timestamp in ms from header
            time_ms = msg.header.stamp.sec * 1000 + msg.header.stamp.nanosec // 1_000_000

            self.cam.requestSendAircraftAttitude(
                time_ms, roll_ned, pitch_ned, yaw_ned,
                wx_ned, wy_ned, wz_ned)

        except Exception as e:
            self.get_logger().error(f"Error in odom_callback: {e}")

    def disconnect_camera(self):
        """Safely disconnect the camera."""
        if hasattr(self, 'cam') and self.cam.is_connected:
            self.get_logger().info("Disconnecting SIYI camera.")
            self.cam.disconnect()

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
        rclpy.shutdown()

if __name__ == "__main__":
    main()