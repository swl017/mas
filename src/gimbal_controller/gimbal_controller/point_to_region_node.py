"""
@file: point_to_region_node.py
@brief: Node to control the gimbal to point the camera to a specific region.
@date: 2025-05-05
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import PointStamped, Vector3, PoseStamped
from sensor_msgs.msg import CameraInfo
from nav_msgs.msg import Odometry

from gimbal_controller import point_to_region
import numpy as np

class PointToRegionNode(Node):
    def __init__(self):
        super().__init__('point_to_region_node')
        self.get_logger().info('PointToRegionNode initialized.')

        # Create a PointToRegion object
        self.point_to_region = point_to_region.PointToRegion()

        # Set up QoS profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )

        # Create a subscription to the gimbal state topic
        self.create_subscription(
            Vector3,
            'gimbal_state_rpy_deg',
            self.gimbal_state_callback,
            qos_profile
        )
        self.create_subscription(
            CameraInfo,
            'camera/color/camera_info',
            self.camera_info_callback,
            10
        )
        # Only need one of pose or odom
        self.create_subscription(
            PoseStamped,
            'common_frame/pose_',
            self.robot_pose_callback,
            qos_profile
        )
        self.create_subscription(
            Odometry,
            'common_frame/odom',
            self.robot_odom_callback,
            qos_profile
        )
        self.create_subscription(
            PointStamped,
            '/target_region',
            self.target_region_callback,
            qos_profile
        )

        # Create a publisher for the gimbal state
        self.gimbal_command_publisher = self.create_publisher(
            Vector3,
            'gimbal_command_los_world_deg',
            10
        )
        self.create_timer(
            0.1,
            self.timer_callback
        )

        self.point_to_region.camera.K = None
        self.point_to_region.target_region = None

    # --- Subscriber Callbacks ---
    def gimbal_state_callback(self, msg: Vector3):
        # Update the gimbal state in the PointToRegion object
        self.point_to_region.gimbal.rpy_rad = [msg.x*np.pi/180.0, msg.y*np.pi/180.0, msg.z*np.pi/180.0]
        self.point_to_region.set_gimbal_mount_orientation_in_robot_frame(
            gimbal_mount_R_robot_gimbal=[0.0, 0.0, 0.0, 1.0]
        )
        self.point_to_region.set_gimbal_mount_position_in_robot_frame(
            gimbal_mount_t_robot_gimbal=[0.0, 0.0, 0.0] # Temp
        )
        self.get_logger().info(f'Gimbal state updated: {msg.x}, {msg.y}, {msg.z}', once=True)

    def camera_info_callback(self, msg: CameraInfo):
        self.point_to_region.set_camera_intrinsic_matrix(
            K=[
                [msg.k[0], msg.k[1], msg.k[2]],
                [msg.k[3], msg.k[4], msg.k[5]],
                [msg.k[6], msg.k[7], msg.k[8]]
            ]
        )
        self.point_to_region.set_camera_width_height(
            width=msg.width,
            height=msg.height
        )
        self.get_logger().info(f'Camera info updated: {msg.width}x{msg.height}, K={msg.k}', once=True)

    def robot_pose_callback(self, msg: PoseStamped):
        self.point_to_region.set_robot_position_in_world_frame(
            robot_t_world_robot=[msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        )
        self.point_to_region.set_robot_orientation_in_world_frame(
            robot_R_world_robot=[msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        )
        self.get_logger().info(f'Robot pose updated: {msg.pose.position.x}, {msg.pose.position.y}, {msg.pose.position.z}', once=True)

    def robot_odom_callback(self, msg: Odometry):
        self.point_to_region.set_robot_position_in_world_frame(
            robot_t_world_robot=[msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]
        )
        self.point_to_region.set_robot_orientation_in_world_frame(
            robot_R_world_robot=[msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        )
        self.get_logger().info(f'Robot odom updated: {msg.pose.pose.position.x}, {msg.pose.pose.position.y}, {msg.pose.pose.position.z}', once=True)

    def target_region_callback(self, msg: PointStamped):
        self.point_to_region.set_target_region(
            target_region=[msg.point.x, msg.point.y, msg.point.z]
        )
        self.get_logger().info(f'Target region updated: {msg.point.x}, {msg.point.y}, {msg.point.z}')

    def timer_callback(self):
        # This function can be used to periodically check the gimbal state or perform other tasks
        if self.point_to_region.target_region is None:
            self.get_logger().debug('Target region not set.', throttle_duration_sec=5.0)
            return
        if self.point_to_region.gimbal.position is None:
            self.get_logger().debug('Gimbal position not set.', throttle_duration_sec=5.0)
            return
        if self.point_to_region.gimbal.orientation is None:
            self.get_logger().debug('Gimbal orientation not set.', throttle_duration_sec=5.0)
            return
        if self.point_to_region.robot.position is None:
            self.get_logger().debug('Robot position not set.', throttle_duration_sec=5.0)
            return
        if self.point_to_region.robot.orientation is None:
            self.get_logger().debug('Robot orientation not set.', throttle_duration_sec=5.0)
            return
        if self.point_to_region.camera.width is None or self.point_to_region.camera.height is None:
            self.get_logger().debug('Camera width/height not set.', throttle_duration_sec=5.0)
            return
        if self.point_to_region.camera.K is None:
            self.get_logger().debug('Camera intrinsic matrix not set.', throttle_duration_sec=5.0)
            return
        if self.point_to_region.gimbal.rpy_rad is None:
            self.get_logger().debug('Gimbal RPY radians not set.', throttle_duration_sec=5.0)
            return
        # if self.point_to_region.gimbal.rpy_deg is None:
        #     self.get_logger().warn('Gimbal RPY degrees not set.')
        #     return
        self.get_logger().info('Timer callback executed.', throttle_duration_sec=5.0)

        (azimuth, elevation) = self.point_to_region.get_gimbal_command_deg_world_frame(self.point_to_region.target_region)
        self.gimbal_command_publisher.publish(Vector3(x=0.0, y=elevation, z=azimuth))
        self.get_logger().info(f'Current target region: {self.point_to_region.target_region}', throttle_duration_sec=5.0)

def main():
    rclpy.init()
    node = PointToRegionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
