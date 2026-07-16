import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PointStamped, PoseStamped, PoseWithCovarianceStamped, TwistStamped
from nav_msgs.msg import Odometry
from mavros_msgs.msg import HomePosition
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import numpy as np
from scipy.spatial.transform import Rotation

# Import our coordinate transformation library
from mas_common_frame import Robot, CommonFrame

class CommonFrameNode(Node):
    def __init__(self) -> None:
        super().__init__('common_frame_node')

        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('vehicle_name_prefix', 'px4_'),
                ('num_vehicles', 2),
                ('common_frame_origin', [37.7749, -122.4194, 0.0])
            ]
        )

        # Get parameter values
        self.vehicle_name_prefix = self.get_parameter('vehicle_name_prefix').value
        self.num_vehicles = self.get_parameter('num_vehicles').value
        common_frame_origin = self.get_parameter('common_frame_origin').value

        # Initialize common frame
        self.common_frame = CommonFrame(tuple(common_frame_origin))

        # Initialize robot dictionary
        self.robots = {}

        # Set up QoS profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # MAVROS publishes home_position as latched (TRANSIENT_LOCAL + RELIABLE);
        # subscribers must match TRANSIENT_LOCAL to pick up the retained message
        # when starting after the home has already been set.
        home_position_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Initialize transform broadcaster for tf2
        self.tf_broadcaster = TransformBroadcaster(self)

        # Latched QoS for the static common→local offset publisher (matches
        # the home_position latching pattern; one-shot on EKF init).
        latched_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Per-vehicle publisher for common_frame/local_origin
        self.local_origin_pubs: dict[str, rclpy.publisher.Publisher] = {}

        # Create publishers and subscribers for each vehicle
        self.gp_origin_subs = []
        self.local_position_subs = []
        self.pose_cov_subs = []
        self.velocity_local_subs = []
        self.common_frame_pubs = {}
        self.common_frame_odom_pubs = {}

        for i in range(1, self.num_vehicles + 1):
            vehicle_name = f"{self.vehicle_name_prefix}{i}"

            # Create robot instance
            self.robots[vehicle_name] = Robot(vehicle_name)
            self.common_frame.add_robot(self.robots[vehicle_name])

            # home_position — one-time init to compute mission frame offset
            self.gp_origin_subs.append(
                self.create_subscription(
                    HomePosition,
                    f'/{vehicle_name}/mavros/home_position/home',
                    lambda msg, vn=vehicle_name: self.home_position_callback(msg, vn),
                    home_position_qos
                )
            )

            # local_position/pose — EKF local position + orientation
            self.local_position_subs.append(
                self.create_subscription(
                    PoseStamped,
                    f'/{vehicle_name}/mavros/local_position/pose',
                    lambda msg, vn=vehicle_name: self.local_position_callback(msg, vn),
                    qos_profile
                )
            )

            # local_position/pose_cov — EKF pose covariance
            self.pose_cov_subs.append(
                self.create_subscription(
                    PoseWithCovarianceStamped,
                    f'/{vehicle_name}/mavros/local_position/pose_cov',
                    lambda msg, vn=vehicle_name: self.pose_cov_callback(msg, vn),
                    qos_profile
                )
            )

            # local_position/velocity_local — EKF velocities
            self.velocity_local_subs.append(
                self.create_subscription(
                    TwistStamped,
                    f'/{vehicle_name}/mavros/local_position/velocity_local',
                    lambda msg, vn=vehicle_name: self.velocity_callback(msg, vn),
                    qos_profile
                )
            )

            # Publishers (keyed by vehicle name for callback-driven publishing)
            self.common_frame_pubs[vehicle_name] = self.create_publisher(
                PoseStamped,
                f'/{vehicle_name}/common_frame/pose',
                qos_profile
            )

            self.common_frame_odom_pubs[vehicle_name] = self.create_publisher(
                Odometry,
                f'/{vehicle_name}/common_frame/odom',
                qos_profile
            )

            # Static common→local offset, latched (see single-vehicle node).
            self.local_origin_pubs[vehicle_name] = self.create_publisher(
                PointStamped,
                f'/{vehicle_name}/common_frame/local_origin',
                latched_qos,
            )

        self.get_logger().info(f'CommonFrameNode initialized with {self.num_vehicles} vehicles (EKF-direct, callback-driven)')

    def home_position_callback(self, msg: HomePosition, vehicle_name):
        """One-time init: compute mission frame offset from home position GPS."""
        if vehicle_name not in self.robots:
            return

        robot = self.robots[vehicle_name]
        home_gps = (msg.geo.latitude, msg.geo.longitude, msg.geo.altitude)

        # Store local origin GPS (needed for orientation transform)
        robot.local_origin_gps = home_gps

        # Compute one-time offset from local ENU origin to mission frame
        robot.mission_frame_offset = self.common_frame.compute_mission_offset(home_gps)
        robot.initialized = True

        self.get_logger().info(
            f"[{vehicle_name}] Mission frame offset computed: "
            f"home=({home_gps[0]:.7f}, {home_gps[1]:.7f}, {home_gps[2]:.2f}) → "
            f"offset=({robot.mission_frame_offset[0]:.2f}, {robot.mission_frame_offset[1]:.2f}, {robot.mission_frame_offset[2]:.2f})m",
            once=True
        )

        # Latched broadcast of the constant common→local offset.
        origin_msg = PointStamped()
        origin_msg.header.stamp = self.get_clock().now().to_msg()
        origin_msg.header.frame_id = 'common_frame'
        origin_msg.point.x = float(robot.mission_frame_offset[0])
        origin_msg.point.y = float(robot.mission_frame_offset[1])
        origin_msg.point.z = float(robot.mission_frame_offset[2])
        self.local_origin_pubs[vehicle_name].publish(origin_msg)

    def local_position_callback(self, msg: PoseStamped, vehicle_name):
        """Cache EKF local position and orientation, then publish immediately."""
        if vehicle_name not in self.robots:
            return

        robot = self.robots[vehicle_name]

        # Cache local position
        robot.update_local_position(
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        )

        # Extract orientation as Euler
        q = msg.pose.orientation
        euler = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_euler('xyz')
        robot.update_orientation(euler[0], euler[1], euler[2])

        self._publish_single(vehicle_name)

    def pose_cov_callback(self, msg: PoseWithCovarianceStamped, vehicle_name):
        """Cache EKF pose covariance."""
        if vehicle_name not in self.robots:
            return

        self.robots[vehicle_name].update_pose_covariance(list(msg.pose.covariance))

    def velocity_callback(self, msg: TwistStamped, vehicle_name):
        """Cache EKF local velocities (passed through without rotation)."""
        if vehicle_name not in self.robots:
            return

        self.robots[vehicle_name].update_velocity(
            msg.twist.linear.x,
            msg.twist.linear.y,
            msg.twist.linear.z,
            msg.twist.angular.x,
            msg.twist.angular.y,
            msg.twist.angular.z
        )

    def _publish_single(self, vehicle_name):
        """Publish transform and pose for a single robot in the mission frame."""
        robot = self.robots[vehicle_name]
        if not robot.initialized:
            return

        try:
            # EKF-direct: p_mission = p_local + offset, orientation rotated
            common_pose = self.common_frame.get_robot_pose_ekf_direct(robot.robot_id)
            now = self.get_clock().now().to_msg()

            # PoseStamped message
            pose_msg = PoseStamped()
            pose_msg.header.stamp = now
            pose_msg.header.frame_id = "common_frame"
            pose_msg.pose.position.x = float(common_pose[0])
            pose_msg.pose.position.y = float(common_pose[1])
            pose_msg.pose.position.z = float(common_pose[2])

            quat = Rotation.from_euler('xyz', common_pose[3:6]).as_quat()
            pose_msg.pose.orientation.x = float(quat[0])
            pose_msg.pose.orientation.y = float(quat[1])
            pose_msg.pose.orientation.z = float(quat[2])
            pose_msg.pose.orientation.w = float(quat[3])

            # Odometry message
            odom_msg = Odometry()
            odom_msg.header.stamp = now
            odom_msg.header.frame_id = "common_frame"
            odom_msg.child_frame_id = f"{vehicle_name}/base_link"
            odom_msg.pose.pose = pose_msg.pose

            # Pose covariance (pass through from EKF)
            if robot.current_pose_covariance is not None:
                odom_msg.pose.covariance = robot.current_pose_covariance

            # Velocities (pass through, no rotation)
            odom_msg.twist.twist.linear.x = float(robot.current_velocity_linear[0])
            odom_msg.twist.twist.linear.y = float(robot.current_velocity_linear[1])
            odom_msg.twist.twist.linear.z = float(robot.current_velocity_linear[2])
            odom_msg.twist.twist.angular.x = float(robot.current_velocity_angular[0])
            odom_msg.twist.twist.angular.y = float(robot.current_velocity_angular[1])
            odom_msg.twist.twist.angular.z = float(robot.current_velocity_angular[2])

            # Publish
            self.common_frame_pubs[vehicle_name].publish(pose_msg)
            self.common_frame_odom_pubs[vehicle_name].publish(odom_msg)

            # TF2 broadcast
            transform = TransformStamped()
            transform.header.stamp = now
            transform.header.frame_id = "common_frame"
            transform.child_frame_id = f"{vehicle_name}/base_link"
            transform.transform.translation.x = float(common_pose[0])
            transform.transform.translation.y = float(common_pose[1])
            transform.transform.translation.z = float(common_pose[2])
            transform.transform.rotation.x = float(quat[0])
            transform.transform.rotation.y = float(quat[1])
            transform.transform.rotation.z = float(quat[2])
            transform.transform.rotation.w = float(quat[3])
            self.tf_broadcaster.sendTransform(transform)

        except Exception as e:
            self.get_logger().error(f"Error publishing transform for {vehicle_name}: {str(e)}")


def main(args=None):
    rclpy.init(args=args)

    common_frame_node = CommonFrameNode()

    try:
        rclpy.spin(common_frame_node)
    except KeyboardInterrupt:
        pass
    finally:
        common_frame_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
