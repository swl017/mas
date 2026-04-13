"""
@file: point_to_region.py
@brief: Control the gimbal to point the camera to a specific region.
@date: 2025-05-05
"""

from typing import List, Tuple
import numpy as np
from scipy.spatial.transform import Rotation

class Camera:
    def __init__(self):
        self.height = None
        self.width = None
        self.position: List[float] = [0.0, 0.0, 0.0] # Camera optical center relative to gimbal mount point
        self.orientation: List[float] = [0.0, 0.0, 0.0, 1.0] # Camera optical center relative to gimbal mount point
        self.K = None  # Intrinsic camera matrix
        self.R = None  # Rotation matrix
        self.t = None  # Translation vector

class Gimbal:
    def __init__(self) -> None:
        self.position: List[float] = None # Relative to robot frame
        self.orientation: List[float] = None # Relative to robot frame
        self.rpy_rad: List[float] = None
        self.rpy_deg: List[float] = None
        self.azimuth_rad: float = None
        self.elevation_rad: float = None
        self.roll_rad: float = None
        self.azimuth_deg: float = None
        self.elevation_deg: float = None
        self.roll_deg: float = None

class Robot:
    def __init__(self) -> None:
        self.frame_convention: str = "flu"
        self.position: List[float] = None
        self.orientation: List[float] = None

class PointToRegion:
    def __init__(self) -> None:
        self.camera = Camera()
        self.gimbal = Gimbal()
        self.robot = Robot()
        self.target_region: List[float]  # Target region in world coordinates

    # --- Parameter Setters ---
    # Physical position/orientation parameters
    def set_robot_position_in_world_frame(self, robot_t_world_robot: List[float] = [0.0, 0.0, 0.0]) -> None:
        self.robot.position = robot_t_world_robot if len(robot_t_world_robot) == 3 else [0.0, 0.0, 0.0]
    def set_robot_orientation_in_world_frame(self, robot_R_world_robot: List[float] = [0.0, 0.0, 0.0, 1.0]) -> None:
        self.robot.orientation = robot_R_world_robot if len(robot_R_world_robot) == 4 else [0.0, 0.0, 0.0, 1.0]
    def set_gimbal_mount_position_in_robot_frame(self, gimbal_mount_t_robot_gimbal: List[float] = [0.0, 0.0, 0.0]) -> None:
        self.gimbal.position = gimbal_mount_t_robot_gimbal if len(gimbal_mount_t_robot_gimbal) == 3 else [0.0, 0.0, 0.0]
    def set_gimbal_mount_orientation_in_robot_frame(self, gimbal_mount_R_robot_gimbal: List[float] = [0.0, 0.0, 0.0, 1.0]) -> None:
        self.gimbal.orientation = gimbal_mount_R_robot_gimbal if len(gimbal_mount_R_robot_gimbal) == 4 else [0.0, 0.0, 0.0, 1.0]
    def set_camera_mount_position_in_gimbal_frame(self, camera_t_gimbal_camera_flu: List[float] = [0.0, 0.0, 0.0]) -> None:
        self.camera.position = camera_t_gimbal_camera_flu if len(camera_t_gimbal_camera_flu) == 3 else [0.0, 0.0, 0.0]
    def set_camera_mount_orientation_in_gimbal_frame(self, camera_R_gimbal_camera: List[float] = [0.0, 0.0, 0.0, 1.0]) -> None:
        self.camera.orientation = camera_R_gimbal_camera if len(camera_R_gimbal_camera) == 4 else [0.0, 0.0, 0.0, 1.0]

    # Camera parameters
    def set_camera_width_height(self, width: int = 640, height: int = 480) -> None:
        self.camera.width = width
        self.camera.height = height
    def set_camera_intrinsic_matrix(self, K: List[List[float]] = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 1.0]]) -> None:
        self.camera.K = np.array(K) if len(K) == 3 and len(K[0]) == 3 else print("Invalid intrinsic matrix")

    # Target region
    def set_target_region(self, target_region: List[float] = [0.0, 0.0, 0.0]) -> None:
        self.target_region = target_region if len(target_region) == 3 else [0.0, 0.0, 0.0]

    def get_combined_camera_pose(self):

        # 1. Robot Pose in World (World ENU from Robot FLU)
        odom_t_world_robot = self.robot.position
        odom_R_world_robot = Rotation.from_quat(self.robot.orientation).as_matrix()

        # 2. Gimbal Rotation relative to Robot Body (Robot FLU from Gimbal Frame)
        gimbal_R_robot_gimbal = Rotation.from_quat(self.gimbal.orientation).as_matrix() \
            * Rotation.from_euler('zyx', [self.gimbal.rpy_rad[2], self.gimbal.rpy_rad[1], self.gimbal.rpy_rad[0]], degrees=False).as_matrix()

        camera_R_gimbal_camera = Rotation.from_quat(self.camera.orientation).as_matrix()

        # 3. Combined Gimbal Pose in World (World ENU from Gimbal Frame)
        combined_R_world_camera = odom_R_world_robot @ gimbal_R_robot_gimbal @ camera_R_gimbal_camera

        # 4. Static Transformations
        # Gimbal mount point relative to robot origin (in FLU)
        gimbal_mount_t_robot_gimbal = self.gimbal.position # Using existing temp param
        # Camera optical center relative to gimbal mount point (in FLU/Gimbal frame)
        camera_t_gimbal_camera_flu = self.camera.position # This offset is in FLU coords

        # 5. Calculate Camera Pose in World (World ENU from Camera RDF)
        # Position of Gimbal Mount in World ENU
        gimbal_mount_t_world_gimbal = odom_t_world_robot + odom_R_world_robot @ gimbal_mount_t_robot_gimbal
        # Position of Camera Optical Center in World ENU
        # Need to rotate camera offset (in FLU) by combined rotation into world frame
        # Offset is defined relative to gimbal mount, rotate by R_world_gimbal
        camera_t_world_camera = gimbal_mount_t_world_gimbal + combined_R_world_camera @ camera_t_gimbal_camera_flu

        self.camera.R = combined_R_world_camera
        self.camera.t = camera_t_world_camera

    def get_gimbal_command_deg(self, target_region: List[float]) -> Tuple[float, float]:
        """
        Calculate the azimuth and elevation in gimbal frame to point the camera to a specific region.
        :param target_region: The target region in world coordinates (x, y, z).
        :return: The gimbal yaw and pitch command in radians.
        """
        # Update the combined pose first to ensure we have latest transforms
        # self.get_combined_camera_pose()

        # Get current camera position in world frame
        camera_position = self.camera.t

        # Calculate direction vector from camera to target
        direction = np.array(target_region) - self.robot.position
        direction = direction / np.linalg.norm(direction)  # Normalize

        # Convert to gimbal coordinates
        # Need to transform from world coordinates to robot coordinates
        robot_R_world_robot = Rotation.from_quat(self.robot.orientation).as_matrix().T  # Transpose for inverse rotation

        # Direction in robot frame
        direction_in_robot = robot_R_world_robot @ direction

        # Transform from robot frame to gimbal frame
        gimbal_R_robot_gimbal = Rotation.from_quat(self.gimbal.orientation).as_matrix().T  # Transpose for inverse rotation
        direction_in_gimbal = gimbal_R_robot_gimbal @ direction_in_robot

        # Calculate the azimuth (yaw) and elevation (pitch) angles needed to point to the target
        azimuth = np.arctan2(direction_in_gimbal[1], direction_in_gimbal[0])
        elevation = -np.arcsin(direction_in_gimbal[2])  # Negative because camera typically points down for positive pitch

        # Return the gimbal command angles in radians
        return (azimuth * 180.0 / np.pi, elevation * 180.0 / np.pi)

    def get_gimbal_command_deg_horizontal_body_frame(self, target_region: List[float]) -> Tuple[float, float]:
        direction = np.array(target_region) - self.robot.position
        direction = direction / np.linalg.norm(direction)  # Normalize
        robot_R_world_robot = Rotation.from_euler(
            'z',
            Rotation.from_quat(self.robot.orientation).as_euler('zxy', degrees=False)[0],
            degrees=False).as_matrix().T
        direction_in_robot = robot_R_world_robot @ direction

        # Calculate the azimuth (yaw) and elevation (pitch) angles needed to point to the target
        azimuth = np.arctan2(direction_in_robot[1], direction_in_robot[0])
        elevation = -np.arcsin(direction_in_robot[2])  # Negative because camera typically points down for positive pitch

        return (azimuth * 180.0 / np.pi, elevation * 180.0 / np.pi)

    def get_gimbal_command_deg_world_frame(self, target_region: List[float]) -> Tuple[float, float]:
        """Compute world-frame azimuth/elevation to point at target.

        Returns (azimuth_deg, elevation_deg) in ENU world frame.
        azimuth: angle from +X (East) toward +Y (North), [-180, 180].
        elevation: angle from horizontal, positive up, [-90, 90].
        """
        direction = np.array(target_region) - np.array(self.robot.position)
        xy_dist = np.sqrt(direction[0] ** 2 + direction[1] ** 2)
        azimuth = np.arctan2(direction[1], direction[0])
        elevation = -np.arctan2(direction[2], xy_dist)
        return (azimuth * 180.0 / np.pi, elevation * 180.0 / np.pi)


if __name__ == "__main__":
    # Example usage
    point_to_region = PointToRegion()
    point_to_region.set_camera_width_height(640, 480)
    point_to_region.set_camera_intrinsic_matrix([[500.0, 0.0, 320.0], [0.0, 500.0, 240.0], [0.0, 0.0, 1.0]])
    point_to_region.set_camera_mount_position_in_gimbal_frame([0.0, 0.0, 0.0])
    # point_to_region.set_camera_mount_position_in_gimbal_frame([0.5, 0.5, -1.5])
    point_to_region.set_camera_mount_orientation_in_gimbal_frame([0.0, 0.0, 0.0, 1.0])
    point_to_region.set_gimbal_mount_position_in_robot_frame([0.0, 0.0, 0.0])
    point_to_region.set_gimbal_mount_orientation_in_robot_frame([0.0, 0.0, 0.0, 1.0])
    point_to_region.set_robot_position_in_world_frame([0.0, 0.0, 0.0])
    point_to_region.set_robot_orientation_in_world_frame([0.0, 0.0, 0.0, 1.0])
    point_to_region.set_camera_mount_orientation_in_gimbal_frame([0.0, 0.0, 0.0])
    # point_to_region.set_camera_mount_orientation_in_gimbal_frame([1.57, 0.0, 3.14])

    target_region = [10.0, 0.0, 10.0]#, 0.0]
    azimuth, elevation = point_to_region.get_gimbal_command_deg(target_region)
    print(f"Azimuth: {azimuth}, Elevation: {elevation}")