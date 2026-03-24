"""Main policy deployment ROS2 node.

Orchestrates the full inference loop at 25 Hz:
1. Assemble observations from cached ROS2 topic data
2. Normalize observations with RunningStandardScaler from training
3. Forward pass through policy network (with GRU hidden state)
4. Apply CBF safety filter to velocity commands
5. Publish actions to offboard_py and los_rate_controller
"""

from __future__ import annotations

import logging

import numpy as np
import torch

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

from .policy_loader import load_checkpoint, PolicyNetRNN
from .observation_assembler import ObservationAssembler
from .action_publisher import ActionPublisher
from .cbf_filter import DeploymentCBFFilter, DeploymentFilterConfig

logger = logging.getLogger(__name__)


class PolicyDeployNode(Node):
    """ROS2 node that runs trained MARL policy inference at 25 Hz."""

    def __init__(self):
        super().__init__('mas_policy_node')

        # --- Declare parameters ---
        self.declare_parameter('checkpoint_path', '')
        self.declare_parameter('num_agents', 2)
        self.declare_parameter('vehicle_names', ['px4_1', 'px4_2'])
        self.declare_parameter('obs_dim', 62)
        self.declare_parameter('action_dim', 7)
        self.declare_parameter('architecture', 'mappo_rnn')
        self.declare_parameter('hidden_size', 64)
        self.declare_parameter('gru_hidden_size', 64)
        self.declare_parameter('gru_num_layers', 1)
        self.declare_parameter('control_frequency', 25.0)
        self.declare_parameter('max_lin_vel', 10.0)
        self.declare_parameter('max_yaw_rate', 0.7854)
        self.declare_parameter('enable_cbf', True)
        self.declare_parameter('enable_triangulation', False)
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('yaw_joint_offset', -1.5708)
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
        checkpoint_path = self.get_parameter('checkpoint_path').get_parameter_value().string_value
        self._num_agents = self.get_parameter('num_agents').get_parameter_value().integer_value
        self._vehicle_names = (
            self.get_parameter('vehicle_names').get_parameter_value().string_array_value
        )
        self._obs_dim = self.get_parameter('obs_dim').get_parameter_value().integer_value
        self._action_dim = self.get_parameter('action_dim').get_parameter_value().integer_value
        architecture = self.get_parameter('architecture').get_parameter_value().string_value
        hidden_size = self.get_parameter('hidden_size').get_parameter_value().integer_value
        gru_hidden_size = self.get_parameter('gru_hidden_size').get_parameter_value().integer_value
        gru_num_layers = self.get_parameter('gru_num_layers').get_parameter_value().integer_value
        control_freq = self.get_parameter('control_frequency').get_parameter_value().double_value
        max_lin_vel = self.get_parameter('max_lin_vel').get_parameter_value().double_value
        max_yaw_rate = self.get_parameter('max_yaw_rate').get_parameter_value().double_value
        enable_cbf = self.get_parameter('enable_cbf').get_parameter_value().bool_value
        enable_tri = self.get_parameter('enable_triangulation').get_parameter_value().bool_value
        image_w = self.get_parameter('image_width').get_parameter_value().integer_value
        image_h = self.get_parameter('image_height').get_parameter_value().integer_value
        yaw_offset = self.get_parameter('yaw_joint_offset').get_parameter_value().double_value
        device_str = self.get_parameter('device').get_parameter_value().string_value
        self._dry_run = self.get_parameter('dry_run').get_parameter_value().bool_value
        use_common_frame = self.get_parameter('use_common_frame').get_parameter_value().bool_value
        self._stale_timeout = self.get_parameter('stale_timeout').get_parameter_value().double_value

        self._device = torch.device(device_str)
        self._architecture = architecture

        # Validate vehicle_names length
        if len(self._vehicle_names) != self._num_agents:
            self.get_logger().warn(
                f"vehicle_names length ({len(self._vehicle_names)}) != num_agents ({self._num_agents}). "
                f"Using first {self._num_agents} names."
            )
            self._vehicle_names = self._vehicle_names[:self._num_agents]

        # --- Load policy ---
        if not checkpoint_path:
            self.get_logger().error("No checkpoint_path provided. Node will run in dry_run mode.")
            self._dry_run = True
            self._policy = None
            self._scaler = None
        else:
            self._policy, self._scaler = load_checkpoint(
                checkpoint_path=checkpoint_path,
                obs_dim=self._obs_dim,
                action_dim=self._action_dim,
                architecture=architecture,
                hidden_size=hidden_size,
                gru_hidden_size=gru_hidden_size,
                gru_num_layers=gru_num_layers,
                device=self._device,
            )
            self.get_logger().info(f"Policy loaded: {architecture}, obs={self._obs_dim}, act={self._action_dim}")

        # --- Initialize GRU hidden states ---
        self._hidden_states: dict[str, torch.Tensor] = {}
        if architecture == 'mappo_rnn' and isinstance(self._policy, PolicyNetRNN):
            for veh in self._vehicle_names:
                self._hidden_states[veh] = self._policy.init_hidden(self._device)

        # --- Observation assembler ---
        self._assembler = ObservationAssembler(
            node=self,
            vehicle_names=self._vehicle_names,
            image_width=image_w,
            image_height=image_h,
            yaw_joint_offset=yaw_offset,
            enable_triangulation=enable_tri,
            use_common_frame=use_common_frame,
        )

        # --- Action publisher ---
        self._action_pub = ActionPublisher(
            node=self,
            vehicle_names=self._vehicle_names,
            max_lin_vel=max_lin_vel,
            max_yaw_rate=max_yaw_rate,
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
            f"Policy deploy node started: {self._num_agents} agents, "
            f"{control_freq} Hz, dry_run={self._dry_run}, cbf={enable_cbf}"
        )

    def _reset_hidden_callback(self, request, response):
        """Service callback to reset all GRU hidden states."""
        for veh in self._vehicle_names:
            if veh in self._hidden_states and isinstance(self._policy, PolicyNetRNN):
                self._hidden_states[veh] = self._policy.init_hidden(self._device)
        response.success = True
        response.message = "Hidden states reset"
        self.get_logger().info("GRU hidden states reset via service call")
        return response

    def _control_loop(self):
        """Main control loop running at policy frequency (25 Hz)."""
        now = self.get_clock().now().nanoseconds / 1e9

        # 1. Assemble observations
        obs_dict = self._assembler.assemble()

        # Check if any agent has data
        any_data = any(
            self._assembler.get_vehicle_state(veh).odom_received
            for veh in self._vehicle_names
        )
        if not any_data:
            if self._tick_count % 100 == 0:
                self.get_logger().warn("No odometry data received yet — waiting...")
            self._tick_count += 1
            return

        # 2. Run inference for each agent
        actions = {}
        with torch.no_grad():
            for veh in self._vehicle_names:
                obs_np = obs_dict[veh]

                # Check for stale data → reset hidden state
                state = self._assembler.get_vehicle_state(veh)
                if now - state.motion_timestamp > self._stale_timeout and state.motion_timestamp > 0:
                    if veh in self._hidden_states and isinstance(self._policy, PolicyNetRNN):
                        self._hidden_states[veh] = self._policy.init_hidden(self._device)
                    actions[veh] = np.zeros(self._action_dim)
                    continue

                if self._policy is None:
                    actions[veh] = np.zeros(self._action_dim)
                    continue

                # Convert to tensor and normalize
                obs_tensor = torch.tensor(obs_np, dtype=torch.float32, device=self._device).unsqueeze(0)
                obs_norm = self._scaler.normalize(obs_tensor)

                # Forward pass
                if self._architecture == 'mappo_rnn' and isinstance(self._policy, PolicyNetRNN):
                    hidden = self._hidden_states[veh]
                    action_tensor, new_hidden = self._policy(obs_norm, hidden)
                    self._hidden_states[veh] = new_hidden
                else:
                    action_tensor = self._policy(obs_norm)

                # Clip to [-1, 1]
                action_np = action_tensor.squeeze(0).cpu().numpy()
                action_np = np.clip(action_np, -1.0, 1.0)
                actions[veh] = action_np

        # 3. Apply CBF safety filter to velocity portion
        if self._cbf_filter is not None:
            positions = np.array([
                self._assembler.get_vehicle_state(veh).position_w
                for veh in self._vehicle_names
            ])
            velocities = np.array([
                self._assembler.get_vehicle_state(veh).velocity_w
                for veh in self._vehicle_names
            ])

            # Extract nominal velocity commands (scaled to physical units)
            v_nom = np.array([
                actions[veh][:3] * self._action_pub._max_lin_vel
                for veh in self._vehicle_names
            ])

            v_safe, cbf_info = self._cbf_filter.filter(v_nom, positions, velocities)

            # Write back filtered velocities (re-normalize to [-1, 1])
            for i, veh in enumerate(self._vehicle_names):
                actions[veh][:3] = v_safe[i] / self._action_pub._max_lin_vel
                # Re-clip after CBF modification
                actions[veh][:3] = np.clip(actions[veh][:3], -1.0, 1.0)

            if cbf_info["deploy_cbf/agents_filtered"] > 0:
                self.get_logger().debug(
                    f"CBF active: {cbf_info['deploy_cbf/agents_filtered']} agents filtered"
                )

        # 4. Publish actions
        if self._dry_run:
            if self._tick_count % 25 == 0:  # Log once per second
                for veh in self._vehicle_names:
                    obs = obs_dict[veh]
                    act = actions[veh]
                    self.get_logger().info(
                        f"[DRY RUN] {veh}: obs_dim={len(obs)}, "
                        f"pos=[{obs[0]:.1f},{obs[1]:.1f},{obs[2]:.1f}], "
                        f"act=[{act[0]:.2f},{act[1]:.2f},{act[2]:.2f},{act[3]:.2f},"
                        f"{act[4]:.2f},{act[5]:.2f},{act[6]:.2f}]"
                    )
        else:
            self._action_pub.publish(actions)

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
