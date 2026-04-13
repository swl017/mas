"""
File: common_frame.py
Author: Seungwook Lee
Date: 2025-04-13
Description: This file contains the CommonFrame class, which is used to create a common frame for the robots.
"""

import numpy as np
from scipy.spatial.transform import Rotation

class Robot:
    """
    Class to store and manage data for a robot (drone).

    Attributes:
        robot_id: Unique identifier for the robot
        local_origin_gps: GPS coordinates (lat, lon, alt) of the local frame origin
        current_gps: Current GPS coordinates (lat, lon, alt) of the robot
        current_orientation: Current orientation (roll, pitch, yaw) in radians
    """
    def __init__(self, robot_id, local_origin_gps=None):
        """
        Initialize a robot with its local frame origin

        Args:
            robot_id: Unique identifier for the robot
            local_origin_gps: (lat, lon, alt) of the local frame origin in degrees
        """
        self.robot_id = robot_id
        self.local_origin_gps = local_origin_gps or (0, 0, 0)
        self.current_gps = (0, 0, 0)  # (lat, lon, alt) in degrees
        self.current_orientation = (0, 0, 0)  # (roll, pitch, yaw) in radians
        self.current_velocity_linear = (0, 0, 0)  # (vx, vy, vz) in m/s
        self.current_velocity_angular = (0, 0, 0)  # (wx, wy, wz) in rad/s
        self.current_local_position = (0, 0, 0)  # (x, y, z) in meters, EKF local frame
        self.current_pose_covariance = None  # 36-element list from pose_cov
        self.mission_frame_offset = None  # (east, north, up) offset to mission frame
        self.initialized = False  # True after mission_frame_offset is computed

    def update_gps(self, lat, lon, alt):
        """Update the current GPS coordinates (in degrees)"""
        self.current_gps = (lat, lon, alt)

    def update_orientation(self, roll, pitch, yaw):
        """Update the current orientation (in radians)"""
        self.current_orientation = (roll, pitch, yaw)

    def update_velocity(self, vx, vy, vz, wx, wy, wz):
        """Update the current velocity (in m/s)"""
        self.current_velocity_linear = (vx, vy, vz)
        self.current_velocity_angular = (wx, wy, wz)

    def update_local_position(self, x, y, z):
        """Update the current EKF local position (in meters)"""
        self.current_local_position = (x, y, z)

    def update_pose_covariance(self, covariance):
        """Update the pose covariance (36-element row-major array)"""
        self.current_pose_covariance = covariance

    def get_quaternion(self):
        """Convert roll, pitch, yaw to quaternion"""
        return Rotation.from_euler('xyz', self.current_orientation).as_quat()

    def __str__(self):
        return f"Robot {self.robot_id} at GPS: {self.current_gps}, " \
               f"Orientation: {self.current_orientation}"


class CommonFrame:
    """
    Class to manage coordinate transformations between robot local frames
    and a common reference frame.

    Attributes:
        origin_gps: GPS coordinates (lat, lon, alt) of the common frame origin
        robots: Dictionary of robots being tracked
    """
    def __init__(self, origin_gps):
        """
        Initialize a common reference frame with its origin in GPS coordinates

        Args:
            origin_gps: (lat, lon, alt) of the common frame origin in degrees
        """
        self.origin_gps = origin_gps
        self.robots = {}  # Dictionary to store robots by ID

        # WGS84 ellipsoid parameters
        self.a = 6378137.0  # semi-major axis in meters
        self.f = 1.0/298.257223563  # flattening
        self.e2 = 2*self.f - self.f*self.f  # eccentricity squared

    def add_robot(self, robot):
        """Add a robot to be tracked in this common frame"""
        self.robots[robot.robot_id] = robot

    def gps_to_ecef(self, lat, lon, alt):
        """
        Convert GPS coordinates to ECEF coordinates

        Args:
            lat: Latitude in degrees
            lon: Longitude in degrees
            alt: Altitude in meters

        Returns:
            (X, Y, Z) coordinates in ECEF frame in meters
        """
        lat_rad = np.radians(lat)
        lon_rad = np.radians(lon)

        N = self.a / np.sqrt(1 - self.e2 * np.sin(lat_rad)**2)

        X = (N + alt) * np.cos(lat_rad) * np.cos(lon_rad)
        Y = (N + alt) * np.cos(lat_rad) * np.sin(lon_rad)
        Z = (N * (1 - self.e2) + alt) * np.sin(lat_rad)

        return (X, Y, Z)

    def ecef_to_enu(self, X, Y, Z, ref_lat, ref_lon, ref_alt):
        """
        Convert ECEF coordinates to ENU coordinates relative to a reference point

        Args:
            X, Y, Z: ECEF coordinates in meters
            ref_lat, ref_lon, ref_alt: Reference point GPS coordinates

        Returns:
            (east, north, up) coordinates in meters
        """
        # Convert reference to ECEF
        X0, Y0, Z0 = self.gps_to_ecef(ref_lat, ref_lon, ref_alt)

        # Calculate displacement in ECEF
        dX = X - X0
        dY = Y - Y0
        dZ = Z - Z0

        # Convert reference to radians
        ref_lat_rad = np.radians(ref_lat)
        ref_lon_rad = np.radians(ref_lon)

        # Rotation matrix from ECEF to ENU
        east = -np.sin(ref_lon_rad) * dX + np.cos(ref_lon_rad) * dY
        north = -np.sin(ref_lat_rad) * np.cos(ref_lon_rad) * dX - np.sin(ref_lat_rad) * np.sin(ref_lon_rad) * dY + np.cos(ref_lat_rad) * dZ
        up = np.cos(ref_lat_rad) * np.cos(ref_lon_rad) * dX + np.cos(ref_lat_rad) * np.sin(ref_lon_rad) * dY + np.sin(ref_lat_rad) * dZ

        return (east, north, up)

    def gps_to_enu(self, gps, origin_gps):
        """
        Convert GPS coordinates to ENU coordinates relative to an origin

        Args:
            gps: (lat, lon, alt) coordinates to convert
            origin_gps: (lat, lon, alt) of the reference origin

        Returns:
            (east, north, up) coordinates in meters
        """
        # Extract coordinates
        lat, lon, alt = gps
        ref_lat, ref_lon, ref_alt = origin_gps

        # Convert to ECEF
        X, Y, Z = self.gps_to_ecef(lat, lon, alt)

        # Convert ECEF to ENU
        return self.ecef_to_enu(X, Y, Z, ref_lat, ref_lon, ref_alt)

    def enu_to_gps(self, enu, origin_gps):
        """
        Convert ENU coordinates to GPS coordinates

        Args:
            enu: (east, north, up) coordinates in meters
            origin_gps: (lat, lon, alt) of the reference origin

        Returns:
            (lat, lon, alt) coordinates
        """
        east, north, up = enu
        ref_lat, ref_lon, ref_alt = origin_gps

        # Convert reference to radians
        ref_lat_rad = np.radians(ref_lat)
        ref_lon_rad = np.radians(ref_lon)

        # Convert reference to ECEF
        X0, Y0, Z0 = self.gps_to_ecef(ref_lat, ref_lon, ref_alt)

        # Rotation matrix from ENU to ECEF
        dX = -np.sin(ref_lon_rad) * east - np.sin(ref_lat_rad) * np.cos(ref_lon_rad) * north + np.cos(ref_lat_rad) * np.cos(ref_lon_rad) * up
        dY = np.cos(ref_lon_rad) * east - np.sin(ref_lat_rad) * np.sin(ref_lon_rad) * north + np.cos(ref_lat_rad) * np.sin(ref_lon_rad) * up
        dZ = np.cos(ref_lat_rad) * north + np.sin(ref_lat_rad) * up

        # Calculate ECEF of target point
        X = X0 + dX
        Y = Y0 + dY
        Z = Z0 + dZ

        # Convert ECEF to geodetic coordinates
        # This is an approximation - iterative methods are more accurate for high precision
        p = np.sqrt(X**2 + Y**2)
        theta = np.arctan2(Z * self.a, p * self.a * (1 - self.e2))

        lat_rad = np.arctan2(
            Z + self.e2 * self.a * np.sin(theta)**3,
            p - self.e2 * self.a * np.cos(theta)**3
        )
        lon_rad = np.arctan2(Y, X)

        N = self.a / np.sqrt(1 - self.e2 * np.sin(lat_rad)**2)
        alt = p / np.cos(lat_rad) - N

        # Convert to degrees
        lat = np.degrees(lat_rad)
        lon = np.degrees(lon_rad)

        return (lat, lon, alt)

    def euler_to_quaternion(self, roll, pitch, yaw):
        """Convert euler angles to quaternion"""
        return Rotation.from_euler('xyz', [roll, pitch, yaw]).as_quat()

    def quaternion_to_euler(self, quat):
        """Convert quaternion to euler angles"""
        return Rotation.from_quat(quat).as_euler('xyz')

    def transform_orientation(self, orientation, local_origin_gps):
        """
        Transform orientation from local ENU frame to common ENU frame

        Args:
            orientation: (roll, pitch, yaw) in radians in local ENU frame
            local_origin_gps: GPS coordinates of the local frame origin

        Returns:
            (roll, pitch, yaw) in the common ENU frame
        """
        # Convert Euler angles to quaternion
        quat_local = self.euler_to_quaternion(*orientation)

        # Get reference point coordinates
        ref_lat, ref_lon, _ = local_origin_gps
        common_lat, common_lon, _ = self.origin_gps

        # Compute rotation matrix from local ENU to ECEF
        ref_lat_rad = np.radians(ref_lat)
        ref_lon_rad = np.radians(ref_lon)

        # Rotation matrix from ENU to ECEF for local frame
        R_enu_to_ecef_local = np.array([
            [-np.sin(ref_lon_rad), -np.sin(ref_lat_rad)*np.cos(ref_lon_rad), np.cos(ref_lat_rad)*np.cos(ref_lon_rad)],
            [np.cos(ref_lon_rad), -np.sin(ref_lat_rad)*np.sin(ref_lon_rad), np.cos(ref_lat_rad)*np.sin(ref_lon_rad)],
            [0, np.cos(ref_lat_rad), np.sin(ref_lat_rad)]
        ])

        # Rotation matrix from ECEF to ENU for common frame
        common_lat_rad = np.radians(common_lat)
        common_lon_rad = np.radians(common_lon)

        R_ecef_to_enu_common = np.array([
            [-np.sin(common_lon_rad), np.cos(common_lon_rad), 0],
            [-np.sin(common_lat_rad)*np.cos(common_lon_rad), -np.sin(common_lat_rad)*np.sin(common_lon_rad), np.cos(common_lat_rad)],
            [np.cos(common_lat_rad)*np.cos(common_lon_rad), np.cos(common_lat_rad)*np.sin(common_lon_rad), np.sin(common_lat_rad)]
        ])

        # Combined rotation matrix from local ENU to common ENU
        R_local_to_common = R_ecef_to_enu_common @ R_enu_to_ecef_local

        # Convert to quaternion
        r = Rotation.from_matrix(R_local_to_common)
        q_local_to_common = r.as_quat()

        # Apply rotation to the orientation quaternion
        q_rot = Rotation.from_quat(q_local_to_common)
        q_local = Rotation.from_quat(quat_local)

        q_common = q_rot * q_local

        # Convert back to euler angles
        return q_common.as_euler('xyz')

    def compute_mission_offset(self, home_gps):
        """
        Compute the one-time offset from a vehicle's local ENU origin to the mission frame.

        Args:
            home_gps: (lat, lon, alt) of the vehicle's EKF local frame origin

        Returns:
            (east, north, up) offset in meters in the mission frame
        """
        return self.gps_to_enu(home_gps, self.origin_gps)

    def get_robot_pose_ekf_direct(self, robot_id):
        """
        Get a robot's pose in the mission frame using the EKF-direct path.
        Position = local_position + mission_frame_offset (no GPS round-trip).
        Orientation = transformed from local ENU to mission ENU.

        Returns:
            (x, y, z, roll, pitch, yaw) in the mission frame
        """
        if robot_id not in self.robots:
            raise ValueError(f"Robot {robot_id} not found in the common frame")

        robot = self.robots[robot_id]

        if robot.mission_frame_offset is None:
            raise ValueError(f"Robot {robot_id} mission frame offset not initialized")

        # Position: local EKF position + one-time offset
        px = robot.current_local_position[0] + robot.mission_frame_offset[0]
        py = robot.current_local_position[1] + robot.mission_frame_offset[1]
        pz = robot.current_local_position[2] + robot.mission_frame_offset[2]

        # Orientation: rotate from local ENU to mission ENU
        orientation_mission = self.transform_orientation(
            robot.current_orientation,
            robot.local_origin_gps
        )

        return (px, py, pz, *orientation_mission)

    def get_robot_pose_in_common_frame(self, robot_id):
        """
        Get a robot's pose (position and orientation) in the common frame

        Returns:
            (x, y, z, roll, pitch, yaw) in the common frame
        """
        if robot_id not in self.robots:
            raise ValueError(f"Robot {robot_id} not found in the common frame")

        robot = self.robots[robot_id]

        # Convert robot's GPS to ENU coordinates in the common frame
        position_enu = self.gps_to_enu(robot.current_gps, self.origin_gps)

        # Transform orientation from robot's local frame to common frame
        orientation_common = self.transform_orientation(
            robot.current_orientation,
            robot.local_origin_gps
        )

        return (*position_enu, *orientation_common)

    def get_all_robot_poses(self):
        """
        Get poses of all robots in the common frame

        Returns:
            Dictionary of robot_id -> (x, y, z, roll, pitch, yaw)
        """
        poses = {}
        for robot_id in self.robots:
            poses[robot_id] = self.get_robot_pose_in_common_frame(robot_id)
        return poses

    def get_relative_pose(self, from_robot_id, to_robot_id):
        """
        Calculate the relative pose between two robots in the common frame

        Args:
            from_robot_id: ID of the reference robot
            to_robot_id: ID of the target robot

        Returns:
            Relative pose (x, y, z, roll, pitch, yaw)
        """
        if from_robot_id not in self.robots or to_robot_id not in self.robots:
            raise ValueError("Robot IDs not found in the common frame")

        # Get poses in common frame
        from_pose = self.get_robot_pose_in_common_frame(from_robot_id)
        to_pose = self.get_robot_pose_in_common_frame(to_robot_id)

        # Extract position and orientation
        from_pos = from_pose[:3]
        from_ori = from_pose[3:]
        to_pos = to_pose[:3]
        to_ori = to_pose[3:]

        # Calculate relative position (simple vector subtraction in Cartesian space)
        rel_pos = (
            to_pos[0] - from_pos[0],
            to_pos[1] - from_pos[1],
            to_pos[2] - from_pos[2]
        )

        # Calculate relative orientation
        from_quat = self.euler_to_quaternion(*from_ori)
        to_quat = self.euler_to_quaternion(*to_ori)

        # Use inverse of 'from' quaternion to get relative rotation
        from_quat_inv = Rotation.from_quat(from_quat).inv().as_quat()

        # Multiply quaternions (order matters!)
        rel_quat = Rotation.from_quat(from_quat_inv) * Rotation.from_quat(to_quat)
        rel_euler = rel_quat.as_euler('xyz')

        return (*rel_pos, *rel_euler)


# Example usage
if __name__ == "__main__":
    # Define a common frame with its origin
    common_frame_origin = (36.3740841,
                    127.3660736,
                    89.8,)
    common_frame = CommonFrame(common_frame_origin)

    # Create two drones with different local frame origins
    drone1_origin = (37.7749, -122.4194, 0)
    drone1 = Robot("drone1", drone1_origin)

    drone2_origin = (37.7739, -122.4312, 0)
    drone2 = Robot("drone2", drone2_origin)

    # Add drones to the common frame
    common_frame.add_robot(drone1)
    common_frame.add_robot(drone2)

    # Update drone positions and orientations
    drone1.update_gps(36.391771, 127.397762,
                      88.66174697532114)  # Slightly offset, 50m altitude
    drone1.update_orientation(0.1, 0.2, 0.3)  # Example roll, pitch, yaw in radians

    drone2.update_gps(36.3915200, 127.3978678, 59)  # Slightly offset, 60m altitude
    # drone2.update_gps(36.3918029, 127.3979558, 59)  # Slightly offset, 60m altitude
    drone2.update_orientation(0.2, 0.3, 0.4)  # Example roll, pitch, yaw in radians

    # Get positions in the common frame
    print("Drone 1 pose in common frame:", common_frame.get_robot_pose_in_common_frame("drone1"))
    print("Drone 2 pose in common frame:", common_frame.get_robot_pose_in_common_frame("drone2"))

    # Get all poses
    all_poses = common_frame.get_all_robot_poses()
    print("\nAll poses in common frame:")
    for robot_id, pose in all_poses.items():
        x, y, z, roll, pitch, yaw = pose
        print(f"{robot_id}: Position (x={x:.2f}, y={y:.2f}, z={z:.2f}), "
              f"Orientation (roll={roll:.2f}, pitch={pitch:.2f}, yaw={yaw:.2f})")

    # Calculate relative pose
    rel_pose = common_frame.get_relative_pose("drone1", "drone2")
    print("\nRelative pose from drone1 to drone2:")
    x, y, z, roll, pitch, yaw = rel_pose
    print(f"Position (x={x:.2f}, y={y:.2f}, z={z:.2f}), "
          f"Orientation (roll={roll:.2f}, pitch={pitch:.2f}, yaw={yaw:.2f})")