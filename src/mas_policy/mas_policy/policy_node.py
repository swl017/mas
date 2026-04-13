"""Per-vehicle policy deployment ROS2 node.

One instance per vehicle, launched inside the vehicle's namespace.
Orchestrates the inference loop at 25 Hz:
1. Assemble observations from ego + peer cached topic data
2. Normalize with RunningStandardScaler from training
3. Forward pass through policy network (with GRU hidden state)
4. Apply CBF safety filter to velocity commands
5. Publish actions via relative topics (resolved by namespace)
"""

from __future__ import annotations

import logging

import numpy as np
import torch

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import Float32, Float32MultiArray
from .policy_loader import load_checkpoint, PolicyNetRNN, ValueNetRNN
from .observation_assembler import ObservationAssembler
from .action_publisher import ActionPublisher
from .cbf_filter import DeploymentCBFFilter, DeploymentFilterConfig

logger = logging.getLogger(__name__)


class PolicyDeployNode(Node):
    """Per-vehicle ROS2 node that runs trained MARL policy inference at 25 Hz."""

    def __init__(self, parameter_overrides=None):
        super().__init__('policy_node', parameter_overrides=parameter_overrides or [])

        # --- Declare parameters ---
        self.declare_parameter('vehicle_name', '')
        self.declare_parameter('peer_names', [''])
        self.declare_parameter('checkpoint_path', '')
        self.declare_parameter('agent_id', '')
        self.declare_parameter('num_agents', 2)
        self.declare_parameter('action_dim', 7)
        self.declare_parameter('architecture', 'mappo_rnn')
        self.declare_parameter('hidden_size', 64)
        self.declare_parameter('gru_hidden_size', 64)
        self.declare_parameter('gru_num_layers', 1)
        self.declare_parameter('control_frequency', 25.0)
        self.declare_parameter('max_lin_vel', 10.0)
        self.declare_parameter('max_yaw_rate', 0.7854)
        self.declare_parameter('max_gimbal_rate', 3.141592653589793)
        self.declare_parameter('max_zoom_rate', 1.0)
        self.declare_parameter('enable_cbf', True)
        self.declare_parameter('enable_triangulation', False)
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('max_bbox_aoi', 20.0)
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('dry_run', False)
        self.declare_parameter('use_common_frame', True)
        self.declare_parameter('stale_timeout', 2.0)

        # CBF parameters
        self.declare_parameter('cbf_D_s', 2.0)
        self.declare_parameter('cbf_v_max', 15.0)
        self.declare_parameter('cbf_tau_delay_max', 0.2)
        self.declare_parameter('cbf_tau_px4', 0.3)
        self.declare_parameter('cbf_gamma_deploy', 1.0)
        self.declare_parameter('cbf_num_iters', 2)

        # --- Read parameters ---
        self._vehicle_name = self.get_parameter('vehicle_name').get_parameter_value().string_value
        self._peer_names = [
            p for p in self.get_parameter('peer_names').get_parameter_value().string_array_value
            if p  # filter empty strings
        ]
        checkpoint_path = self.get_parameter('checkpoint_path').get_parameter_value().string_value
        agent_id = self.get_parameter('agent_id').get_parameter_value().string_value
        num_agents = self.get_parameter('num_agents').get_parameter_value().integer_value
        self._action_dim = self.get_parameter('action_dim').get_parameter_value().integer_value
        architecture = self.get_parameter('architecture').get_parameter_value().string_value
        hidden_size = self.get_parameter('hidden_size').get_parameter_value().integer_value
        gru_hidden_size = self.get_parameter('gru_hidden_size').get_parameter_value().integer_value
        gru_num_layers = self.get_parameter('gru_num_layers').get_parameter_value().integer_value
        control_freq = self.get_parameter('control_frequency').get_parameter_value().double_value
        max_lin_vel = self.get_parameter('max_lin_vel').get_parameter_value().double_value
        max_yaw_rate = self.get_parameter('max_yaw_rate').get_parameter_value().double_value
        max_gimbal_rate = self.get_parameter('max_gimbal_rate').get_parameter_value().double_value
        max_zoom_rate = self.get_parameter('max_zoom_rate').get_parameter_value().double_value
        enable_cbf = self.get_parameter('enable_cbf').get_parameter_value().bool_value
        enable_tri = self.get_parameter('enable_triangulation').get_parameter_value().bool_value
        image_w = self.get_parameter('image_width').get_parameter_value().integer_value
        image_h = self.get_parameter('image_height').get_parameter_value().integer_value
        max_bbox_aoi = self.get_parameter('max_bbox_aoi').get_parameter_value().double_value
        device_str = self.get_parameter('device').get_parameter_value().string_value
        self._dry_run = self.get_parameter('dry_run').get_parameter_value().bool_value
        use_common_frame = self.get_parameter('use_common_frame').get_parameter_value().bool_value
        self._stale_timeout = self.get_parameter('stale_timeout').get_parameter_value().double_value

        self._device = torch.device(device_str)
        self._architecture = architecture
        self._max_lin_vel = max_lin_vel
        self._num_agents = 1 + len(self._peer_names)

        # Compute obs_dim from num_agents: 30 ego + 16*(N-1) inter-agent [+ 6 tri]
        self._obs_dim = 30 + 16 * (num_agents - 1) + (6 if enable_tri else 0)

        # Validate peer count matches num_agents
        if self._num_agents != num_agents:
            self.get_logger().error(
                f"num_agents={num_agents} but got {len(self._peer_names)} peers "
                f"({self._num_agents} total). obs_dim={self._obs_dim} may be wrong."
            )

        # Use namespace as vehicle_name fallback
        if not self._vehicle_name:
            ns = self.get_namespace().strip('/')
            self._vehicle_name = ns if ns else 'px4_1'

        self.get_logger().info(
            f"Vehicle: {self._vehicle_name}, peers: {self._peer_names}"
        )

        # --- Load policy + value network ---
        if not checkpoint_path:
            self.get_logger().error("No checkpoint_path provided. Running in dry_run mode.")
            self._dry_run = True
            self._policy = None
            self._scaler = None
            self._value_net = None
            self._shared_scaler = None
        else:
            self._policy, self._scaler, self._value_net, self._shared_scaler = load_checkpoint(
                checkpoint_path=checkpoint_path,
                obs_dim=self._obs_dim,
                action_dim=self._action_dim,
                architecture=architecture,
                hidden_size=hidden_size,
                gru_hidden_size=gru_hidden_size,
                gru_num_layers=gru_num_layers,
                num_agents=num_agents,
                device=self._device,
                agent_id=agent_id,
            )
            self.get_logger().info(
                f"Policy loaded: {architecture}, obs={self._obs_dim} "
                f"(agents={num_agents}, tri={enable_tri}), act={self._action_dim}"
            )
            if self._value_net is not None:
                self.get_logger().info("Value network loaded — V(s) monitoring enabled")

        # --- Initialize GRU hidden states (single agent) ---
        self._hidden_state = None
        if architecture == 'mappo_rnn' and isinstance(self._policy, PolicyNetRNN):
            self._hidden_state = self._policy.init_hidden(self._device)

        self._value_hidden_state = None
        if isinstance(self._value_net, ValueNetRNN):
            self._value_hidden_state = self._value_net.init_hidden(self._device)

        # --- Observation assembler ---
        self._assembler = ObservationAssembler(
            node=self,
            ego_name=self._vehicle_name,
            peer_names=self._peer_names,
            image_width=image_w,
            image_height=image_h,
            enable_triangulation=enable_tri,
            use_common_frame=use_common_frame,
            max_bbox_aoi=max_bbox_aoi,
        )

        # Validate obs_dim consistency between node formula and assembler
        assert self._assembler.obs_dim == self._obs_dim, (
            f"obs_dim mismatch: node={self._obs_dim}, assembler={self._assembler.obs_dim}"
        )

        # --- Action publisher (relative topics) ---
        self._action_pub = ActionPublisher(
            node=self,
            max_lin_vel=max_lin_vel,
            max_yaw_rate=max_yaw_rate,
            max_gimbal_rate=max_gimbal_rate,
            max_zoom_rate=max_zoom_rate,
        )

        # --- Value publisher ---
        self._value_pub = self.create_publisher(Float32, 'policy/value', 1)

        # --- Cross-agent observation exchange for value network ---
        obs_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._obs_pub = self.create_publisher(
            Float32MultiArray, 'policy/observation', obs_qos,
        )
        self._peer_obs: dict[str, np.ndarray | None] = {
            p: None for p in self._peer_names
        }
        for peer in self._peer_names:
            self.create_subscription(
                Float32MultiArray,
                f'/{peer}/policy/observation',
                lambda msg, v=peer: self._peer_obs_callback(msg, v),
                obs_qos,
            )

        # --- CBF safety filter ---
        self._cbf_filter = None
        if enable_cbf:
            cbf_cfg = DeploymentFilterConfig(
                D_s=self.get_parameter('cbf_D_s').get_parameter_value().double_value,
                v_max=self.get_parameter('cbf_v_max').get_parameter_value().double_value,
                tau_delay_max=self.get_parameter('cbf_tau_delay_max').get_parameter_value().double_value,
                tau_px4=self.get_parameter('cbf_tau_px4').get_parameter_value().double_value,
                gamma_deploy=self.get_parameter('cbf_gamma_deploy').get_parameter_value().double_value,
                num_iters=self.get_parameter('cbf_num_iters').get_parameter_value().integer_value,
            )
            self._cbf_filter = DeploymentCBFFilter(cbf_cfg, self._num_agents)

        # --- Hidden state reset service ---
        self.create_service(Trigger, '~/reset_hidden_state', self._reset_hidden_callback)

        # --- Control timer ---
        timer_period = 1.0 / control_freq
        self._timer = self.create_timer(timer_period, self._control_loop)

        self._tick_count = 0
        self.get_logger().info(
            f"Policy node started: {self._num_agents} agents, "
            f"{control_freq} Hz, dry_run={self._dry_run}, cbf={enable_cbf}"
        )

    def _reset_hidden_callback(self, request, response):
        """Service callback to reset GRU hidden state."""
        if isinstance(self._policy, PolicyNetRNN):
            self._hidden_state = self._policy.init_hidden(self._device)
        if isinstance(self._value_net, ValueNetRNN):
            self._value_hidden_state = self._value_net.init_hidden(self._device)
        response.success = True
        response.message = "Hidden state reset"
        self.get_logger().info("GRU hidden state reset via service call")
        return response

    def _peer_obs_callback(self, msg: Float32MultiArray, veh: str):
        """Cache peer's assembled observation vector for value network."""
        self._peer_obs[veh] = np.array(msg.data, dtype=np.float32)

    def _control_loop(self):
        """Main control loop running at policy frequency (25 Hz)."""
        now = self.get_clock().now().nanoseconds / 1e9

        # Check if ego has data
        ego_state = self._assembler.ego_state
        if not ego_state.odom_received:
            if self._tick_count % 100 == 0:
                self.get_logger().warn("No ego odometry received yet — waiting...")
            self._tick_count += 1
            return

        # 1. Assemble observation
        obs_np = self._assembler.assemble()

        # 1b. Publish ego observation for peer value networks
        obs_msg = Float32MultiArray()
        obs_msg.data = obs_np.tolist()
        self._obs_pub.publish(obs_msg)

        # 2. Check for stale ego data → reset hidden state, publish zero
        if ego_state.motion_timestamp > 0 and now - ego_state.motion_timestamp > self._stale_timeout:
            if isinstance(self._policy, PolicyNetRNN):
                self._hidden_state = self._policy.init_hidden(self._device)
            if isinstance(self._value_net, ValueNetRNN):
                self._value_hidden_state = self._value_net.init_hidden(self._device)
            if not self._dry_run:
                self._action_pub.publish_zero()
            self._tick_count += 1
            return

        # 3. Run inference
        if self._policy is None:
            action_np = np.zeros(self._action_dim)
        else:
            with torch.no_grad():
                obs_tensor = torch.tensor(
                    obs_np, dtype=torch.float32, device=self._device
                ).unsqueeze(0)
                obs_norm = self._scaler.normalize(obs_tensor)

                if self._architecture == 'mappo_rnn' and isinstance(self._policy, PolicyNetRNN):
                    action_tensor, self._hidden_state = self._policy(obs_norm, self._hidden_state)
                else:
                    action_tensor = self._policy(obs_norm)

                action_np = action_tensor.squeeze(0).cpu().numpy()
                # Training uses raw mean_actions (no tanh/clip) for deterministic
                # inference. Clip to [-1, 1] as safety bound for deployment only.
                action_np = np.clip(action_np, -1.0, 1.0)

                # 3b. Value inference (monitoring)
                if self._value_net is not None and self._shared_scaler is not None:
                    # Construct true shared state: concatenate all agents'
                    # ego-perspective observations, matching training env.state()
                    # which does torch.cat([obs_dict[agent] for agent in agents])
                    peer_obs_list = []
                    for peer in self._peer_names:
                        p_obs = self._peer_obs.get(peer)
                        if p_obs is not None and len(p_obs) == self._obs_dim:
                            peer_obs_list.append(
                                torch.tensor(p_obs, dtype=torch.float32, device=self._device).unsqueeze(0)
                            )
                        else:
                            # Peer obs not yet received — use zeros
                            peer_obs_list.append(
                                torch.zeros(1, self._obs_dim, device=self._device)
                            )
                    shared_obs = torch.cat([obs_tensor] + peer_obs_list, dim=-1)
                    shared_obs_norm = self._shared_scaler.normalize(shared_obs)
                    value_tensor, self._value_hidden_state = self._value_net(
                        shared_obs_norm, self._value_hidden_state,
                    )
                    value_msg = Float32()
                    value_msg.data = float(value_tensor.item())
                    self._value_pub.publish(value_msg)

        # 4. Apply CBF safety filter to velocity portion
        if self._cbf_filter is not None:
            all_names = self._assembler.all_names
            positions = np.array([
                self._assembler.get_vehicle_state(n).position_w for n in all_names
            ])
            velocities = np.array([
                self._assembler.get_vehicle_state(n).velocity_w for n in all_names
            ])

            # Zero out velocity for stale peers (most conservative CBF assumption)
            for i, name in enumerate(all_names):
                if i == 0:
                    continue  # ego is always fresh (checked above)
                peer_state = self._assembler.get_vehicle_state(name)
                if peer_state.motion_timestamp > 0 and now - peer_state.motion_timestamp > self._stale_timeout:
                    velocities[i] = np.zeros(3)
                    if self._tick_count % 100 == 0:
                        self.get_logger().warn(
                            f"CBF: peer {name} odom stale ({now - peer_state.motion_timestamp:.1f}s), using zero velocity"
                        )

            # Build nominal velocities: ego uses policy output, peers use current velocity
            v_nom = velocities.copy()
            v_nom[0] = action_np[:3] * self._max_lin_vel  # ego is index 0

            v_safe, cbf_info = self._cbf_filter.filter(v_nom, positions, velocities)

            # Write back filtered ego velocity (re-normalize to [-1, 1])
            action_np[:3] = np.clip(v_safe[0] / self._max_lin_vel, -1.0, 1.0)

            if cbf_info["deploy_cbf/agents_filtered"] > 0:
                self.get_logger().debug(
                    f"CBF active: {cbf_info['deploy_cbf/agents_filtered']} agents filtered"
                )

        # 5. Publish
        if self._dry_run:
            if self._tick_count % 25 == 0:
                self.get_logger().info(
                    f"[DRY RUN] obs_dim={len(obs_np)}, "
                    f"pos=[{obs_np[0]:.1f},{obs_np[1]:.1f},{obs_np[2]:.1f}], "
                    f"act=[{action_np[0]:.2f},{action_np[1]:.2f},{action_np[2]:.2f},"
                    f"{action_np[3]:.2f},{action_np[4]:.2f},{action_np[5]:.2f},{action_np[6]:.2f}]"
                )
        else:
            self._action_pub.publish(action_np)

        self._tick_count += 1


def main(args=None):
    rclpy.init(args=args)
    node = PolicyDeployNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down — publishing zero actions")
        node._action_pub.publish_zero()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
