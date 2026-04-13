import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TwistStamped
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
                ('common_frame_origin', [37.7749, -122.4194, 0.0]),
            ]
        )

        # Get node namespace as vehicle name
        self.vehicle_name = self.get_namespace().strip('/')
        common_frame_origin = self.get_parameter('common_frame_origin').value

        # Initialize common frame
        self.common_frame = CommonFrame(tuple(common_frame_origin))

        # Initialize robot
        self.robot = Robot(self.vehicle_name)
        self.common_frame.add_robot(self.robot)

        # Set up QoS profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Initialize transform broadcaster for tf2
        self.tf_broadcaster = TransformBroadcaster(self)

        vn = self.vehicle_name

        # home_position — one-time init to compute mission frame offset
        self.home_position_sub = self.create_subscription(
            HomePosition,
            f'/{vn}/mavros/home_position/home',
            self.home_position_callback,
            qos_profile
        )

        # local_position/pose — EKF local position + orientation
        self.local_position_sub = self.create_subscription(
            PoseStamped,
            f'/{vn}/mavros/local_position/pose',
            self.local_position_callback,
            qos_profile
        )

        # local_position/pose_cov — EKF pose covariance
        self.pose_cov_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            f'/{vn}/mavros/local_position/pose_cov',
            self.pose_cov_callback,
            qos_profile
        )

        # local_position/velocity_local — EKF velocities
        self.velocity_sub = self.create_subscription(
            TwistStamped,
            f'/{vn}/mavros/local_position/velocity_local',
            self.velocity_callback,
            qos_profile
        )

        # Publishers
        self.common_frame_pub = self.create_publisher(
            PoseStamped,
            f'/{vn}/common_frame/pose',
            qos_profile
        )

        self.common_frame_odom_pub = self.create_publisher(
            Odometry,
            f'/{vn}/common_frame/odom',
            qos_profile
        )

        self.get_logger().info(f'CommonFrameNode (single) initialized for {vn} (EKF-direct, callback-driven)')

    def home_position_callback(self, msg: HomePosition):
        """One-time init: compute mission frame offset from home position GPS."""
        home_gps = (msg.geo.latitude, msg.geo.longitude, msg.geo.altitude)

        # Store local origin GPS (needed for orientation transform)
        self.robot.local_origin_gps = home_gps

        # Compute one-time offset from local ENU origin to mission frame
        self.robot.mission_frame_offset = self.common_frame.compute_mission_offset(home_gps)
        self.robot.initialized = True

        self.get_logger().info(
            f"[{self.vehicle_name}] Mission frame offset computed: "
            f"home=({home_gps[0]:.7f}, {home_gps[1]:.7f}, {home_gps[2]:.2f}) → "
            f"offset=({self.robot.mission_frame_offset[0]:.2f}, {self.robot.mission_frame_offset[1]:.2f}, {self.robot.mission_frame_offset[2]:.2f})m",
            once=True
        )

    def local_position_callback(self, msg: PoseStamped):
        """Cache EKF local position and orientation, then publish immediately."""
        self.robot.update_local_position(
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        )

        q = msg.pose.orientation
        euler = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_euler('xyz')
        self.robot.update_orientation(euler[0], euler[1], euler[2])

        self.publish_transforms()

    def pose_cov_callback(self, msg: PoseWithCovarianceStamped):
        """Cache EKF pose covariance."""
        self.robot.update_pose_covariance(list(msg.pose.covariance))

    def velocity_callback(self, msg: TwistStamped):
        """Cache EKF local velocities (passed through without rotation)."""
        self.robot.update_velocity(
            msg.twist.linear.x,
            msg.twist.linear.y,
            msg.twist.linear.z,
            msg.twist.angular.x,
            msg.twist.angular.y,
            msg.twist.angular.z
        )

    def publish_transforms(self):
        """Publish transforms and poses in the mission frame."""
        if not self.robot.initialized:
            return

        try:
            # EKF-direct: p_mission = p_local + offset, orientation rotated
            common_pose = self.common_frame.get_robot_pose_ekf_direct(self.robot.robot_id)
            now = self.get_clock().now().to_msg()

            # PoseStamped message
            pose_msg = PoseStamped()
            pose_msg.header.stamp = now
            pose_msg.header.frame_id = "common_frame"
            pose_msg.pose.position.x = common_pose[0]
            pose_msg.pose.position.y = common_pose[1]
            pose_msg.pose.position.z = common_pose[2]

            quat = Rotation.from_euler('xyz', common_pose[3:6]).as_quat()
            pose_msg.pose.orientation.x = quat[0]
            pose_msg.pose.orientation.y = quat[1]
            pose_msg.pose.orientation.z = quat[2]
            pose_msg.pose.orientation.w = quat[3]

            # Odometry message
            odom_msg = Odometry()
            odom_msg.header.stamp = now
            odom_msg.header.frame_id = "common_frame"
            odom_msg.child_frame_id = f"{self.vehicle_name}/base_link"
            odom_msg.pose.pose = pose_msg.pose

            # Pose covariance (pass through from EKF)
            if self.robot.current_pose_covariance is not None:
                odom_msg.pose.covariance = self.robot.current_pose_covariance

            # Velocities (pass through, no rotation)
            odom_msg.twist.twist.linear.x = self.robot.current_velocity_linear[0]
            odom_msg.twist.twist.linear.y = self.robot.current_velocity_linear[1]
            odom_msg.twist.twist.linear.z = self.robot.current_velocity_linear[2]
            odom_msg.twist.twist.angular.x = self.robot.current_velocity_angular[0]
            odom_msg.twist.twist.angular.y = self.robot.current_velocity_angular[1]
            odom_msg.twist.twist.angular.z = self.robot.current_velocity_angular[2]

            # Publish
            self.common_frame_pub.publish(pose_msg)
            self.common_frame_odom_pub.publish(odom_msg)

            # TF2 broadcast
            transform = TransformStamped()
            transform.header.stamp = now
            transform.header.frame_id = "common_frame"
            transform.child_frame_id = f"{self.vehicle_name}/base_link"
            transform.transform.translation.x = common_pose[0]
            transform.transform.translation.y = common_pose[1]
            transform.transform.translation.z = common_pose[2]
            transform.transform.rotation.x = quat[0]
            transform.transform.rotation.y = quat[1]
            transform.transform.rotation.z = quat[2]
            transform.transform.rotation.w = quat[3]
            self.tf_broadcaster.sendTransform(transform)

        except Exception as e:
            self.get_logger().error(f"Error publishing transform for {self.vehicle_name}: {str(e)}")


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
