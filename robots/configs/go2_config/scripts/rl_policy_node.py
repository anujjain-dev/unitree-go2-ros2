#!/usr/bin/env python3
# Copyright (c) 2026 Rahgir Rafi
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http:#www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Runs an IsaacLab/rsl_rl-trained locomotion policy in place of the CHAMP gait controller.

The actor MLP is rebuilt directly from the rsl_rl checkpoint's ``model_state_dict``, so
neither rsl_rl nor IsaacLab has to be importable at runtime -- plain torch is enough.

Two loops run at different rates, mirroring how the policy was trained:

* the policy loop (``policy_rate``, 50 Hz = IsaacLab's sim dt 0.005 x decimation 4)
  assembles the observation and produces joint position targets, and
* the PD loop (``pd_rate``) converts those targets into joint efforts. IsaacLab drove
  the robot with an implicit PD actuator at the physics rate, so holding a torque for a
  whole 20 ms policy step would under-damp the legs noticeably.
"""

import math
import threading

import numpy as np
import rclpy
import torch
import torch.nn as nn
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64MultiArray


def build_actor(state_dict, hidden_dims, activation="elu"):
    """Rebuild the rsl_rl ActorCritic actor from a checkpoint state dict.

    rsl_rl stores the actor as a flat nn.Sequential, so the layer shapes in the
    checkpoint fully determine the architecture; hidden_dims is only used to check
    that the checkpoint is the one the config claims it is.
    """
    actor_keys = sorted(
        (k for k in state_dict if k.startswith("actor.") and k.endswith(".weight")),
        key=lambda k: int(k.split(".")[1]),
    )
    if not actor_keys:
        raise RuntimeError("checkpoint contains no 'actor.*' weights")

    dims = [state_dict[actor_keys[0]].shape[1]] + [state_dict[k].shape[0] for k in actor_keys]
    found_hidden = dims[1:-1]
    if list(hidden_dims) and found_hidden != list(hidden_dims):
        raise RuntimeError(
            f"checkpoint hidden dims {found_hidden} do not match configured {list(hidden_dims)}"
        )

    act = {"elu": nn.ELU, "relu": nn.ReLU, "tanh": nn.Tanh}[activation.lower()]
    layers = []
    for i in range(len(dims) - 1):
        layers.append(nn.Linear(dims[i], dims[i + 1]))
        if i < len(dims) - 2:
            layers.append(act())
    actor = nn.Sequential(*layers)

    actor.load_state_dict(
        {k[len("actor."):]: v for k, v in state_dict.items() if k.startswith("actor.")}
    )
    actor.eval()
    for p in actor.parameters():
        p.requires_grad_(False)
    return actor, dims[0], dims[-1]


def quat_rotate_inverse(q_xyzw, v):
    """Rotate v from the world frame into the frame described by quaternion q.

    Matches isaaclab.utils.math.quat_rotate_inverse, which IsaacLab uses to build the
    projected_gravity observation.
    """
    x, y, z, w = q_xyzw
    q_vec = np.array([x, y, z])
    a = v * (2.0 * w * w - 1.0)
    b = np.cross(q_vec, v) * (2.0 * w)
    c = q_vec * (2.0 * np.dot(q_vec, v))
    return a - b + c


class RLPolicyNode(Node):

    def __init__(self):
        super().__init__("rl_policy_node")

        self.declare_parameter("checkpoint", "")
        self.declare_parameter("device", "cpu")
        self.declare_parameter("policy_rate", 50.0)
        self.declare_parameter("pd_rate", 200.0)
        self.declare_parameter("hidden_dims", [512, 256, 128])
        self.declare_parameter("activation", "elu")

        # Joint order the policy was trained with. IsaacLab orders an articulation's
        # joints breadth-first (all hips, all thighs, all calves) -- NOT the per-leg
        # order the URDF and ros2_control use. ros_joint_names[i] is the URDF name of
        # policy joint i, so these two lists must stay index-aligned.
        self.declare_parameter("ros_joint_names", [
            "lf_hip_joint", "rf_hip_joint", "lh_hip_joint", "rh_hip_joint",
            "lf_upper_leg_joint", "rf_upper_leg_joint", "lh_upper_leg_joint", "rh_upper_leg_joint",
            "lf_lower_leg_joint", "rf_lower_leg_joint", "lh_lower_leg_joint", "rh_lower_leg_joint",
        ])
        self.declare_parameter("default_joint_pos", [
            0.1, -0.1, 0.1, -0.1,
            0.8, 0.8, 1.0, 1.0,
            -1.5, -1.5, -1.5, -1.5,
        ])

        self.declare_parameter("action_scale", 0.25)
        self.declare_parameter("kp", 25.0)
        self.declare_parameter("kd", 0.5)
        self.declare_parameter("torque_limit", 23.5)

        # The trained observation includes a 17x11 downward height-scan grid that no
        # sensor in this Gazebo stack provides. Every shipped world is a flat ground
        # plane, where the scan reads a single constant: base_height - ray_offset.
        self.declare_parameter("height_scan_dim", 187)
        self.declare_parameter("height_scan_constant", -0.15)

        self.declare_parameter("max_lin_vel_x", 1.0)
        self.declare_parameter("max_lin_vel_y", 1.0)
        self.declare_parameter("max_ang_vel_z", 1.0)
        self.declare_parameter("cmd_vel_timeout", 0.5)

        self.declare_parameter("startup_stand_duration", 2.0)
        # Gazebo spawns the robot with every joint at its URDF zero, which is a sprawl,
        # and it lands on its belly. The trained gains (kp 25) are far too soft to push
        # it back up from there -- IsaacLab never had to, since it initialised the robot
        # already standing. So the stand-up phase uses its own stiffer gains and only
        # hands over to the trained ones once the robot is actually on its feet.
        self.declare_parameter("stand_kp", 100.0)
        self.declare_parameter("stand_kd", 2.0)
        self.declare_parameter("command_topic", "/rl_joint_effort_controller/commands")
        self.declare_parameter("odom_topic", "/odom_ground_truth")
        self.declare_parameter("imu_topic", "/imu/data")

        p = self.get_parameter
        checkpoint = p("checkpoint").value
        if not checkpoint:
            raise RuntimeError("the 'checkpoint' parameter is required")

        self.device = torch.device(p("device").value)
        self.ros_joint_names = list(p("ros_joint_names").value)
        self.default_joint_pos = np.array(p("default_joint_pos").value, dtype=np.float64)
        self.action_scale = float(p("action_scale").value)
        self.kp = float(p("kp").value)
        self.kd = float(p("kd").value)
        self.torque_limit = float(p("torque_limit").value)
        self.height_scan_dim = int(p("height_scan_dim").value)
        self.height_scan_constant = float(p("height_scan_constant").value)
        self.max_cmd = np.array([
            float(p("max_lin_vel_x").value),
            float(p("max_lin_vel_y").value),
            float(p("max_ang_vel_z").value),
        ])
        self.cmd_vel_timeout = float(p("cmd_vel_timeout").value)
        self.stand_duration = float(p("startup_stand_duration").value)
        self.stand_kp = float(p("stand_kp").value)
        self.stand_kd = float(p("stand_kd").value)

        n_joints = len(self.ros_joint_names)
        if len(self.default_joint_pos) != n_joints:
            raise RuntimeError(
                f"default_joint_pos has {len(self.default_joint_pos)} entries "
                f"but ros_joint_names has {n_joints}"
            )

        torch.set_num_threads(1)
        ckpt = torch.load(checkpoint, map_location="cpu", weights_only=False)
        state_dict = ckpt.get("model_state_dict", ckpt)
        self.actor, self.obs_dim, self.action_dim = build_actor(
            state_dict, p("hidden_dims").value, p("activation").value
        )
        self.actor.to(self.device)

        if self.action_dim != n_joints:
            raise RuntimeError(
                f"policy outputs {self.action_dim} actions but {n_joints} joints are configured"
            )
        expected_obs = 3 + 3 + 3 + 3 + 2 * n_joints + self.action_dim + self.height_scan_dim
        if self.obs_dim != expected_obs:
            raise RuntimeError(
                f"policy expects a {self.obs_dim}-dim observation but the configured terms "
                f"build {expected_obs} "
                f"(check height_scan_dim, currently {self.height_scan_dim})"
            )
        self.get_logger().info(
            f"loaded policy from {checkpoint} "
            f"(obs {self.obs_dim}, actions {self.action_dim}, iter {ckpt.get('iter', '?')})"
        )

        # --- state shared between the subscriptions and the two timers ---
        self.lock = threading.Lock()
        self.joint_pos = None
        self.joint_vel = None
        self.joint_index = None          # JointState index for each policy joint
        self.base_lin_vel = np.zeros(3)
        self.base_ang_vel = np.zeros(3)
        self.projected_gravity = np.array([0.0, 0.0, -1.0])
        self.commands = np.zeros(3)
        self.last_cmd_time = None
        self.last_action = np.zeros(self.action_dim)
        self.joint_pos_target = self.default_joint_pos.copy()
        self.stand_start_pos = None
        self.start_time = None
        self.standing_up = True
        self.height_scan = np.full(self.height_scan_dim, self.height_scan_constant)

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(JointState, "/joint_states", self.on_joint_state, 10)
        self.create_subscription(Imu, p("imu_topic").value, self.on_imu, sensor_qos)
        self.create_subscription(Odometry, p("odom_topic").value, self.on_odom, sensor_qos)
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd_vel, 10)
        self.cmd_pub = self.create_publisher(Float64MultiArray, p("command_topic").value, 10)

        self.create_timer(1.0 / float(p("policy_rate").value), self.policy_step)
        self.create_timer(1.0 / float(p("pd_rate").value), self.pd_step)

    # ------------------------------------------------------------------ inputs

    def on_joint_state(self, msg):
        with self.lock:
            if self.joint_index is None:
                try:
                    self.joint_index = [msg.name.index(n) for n in self.ros_joint_names]
                except ValueError as exc:
                    self.get_logger().error(
                        f"joint missing from /joint_states: {exc}. "
                        f"Available: {list(msg.name)}"
                    )
                    return
            idx = self.joint_index
            self.joint_pos = np.array([msg.position[i] for i in idx])
            self.joint_vel = (
                np.array([msg.velocity[i] for i in idx])
                if len(msg.velocity) == len(msg.position)
                else np.zeros(len(idx))
            )

    def on_imu(self, msg):
        q = msg.orientation
        gravity = quat_rotate_inverse(
            (q.x, q.y, q.z, q.w), np.array([0.0, 0.0, -1.0])
        )
        with self.lock:
            self.projected_gravity = gravity
            self.base_ang_vel = np.array([
                msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z
            ])

    def on_odom(self, msg):
        # Ignition's OdometryPublisher reports the twist in the robot base frame,
        # which is what base_lin_vel was during training.
        with self.lock:
            self.base_lin_vel = np.array([
                msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z
            ])

    def on_cmd_vel(self, msg):
        cmd = np.array([msg.linear.x, msg.linear.y, msg.angular.z])
        with self.lock:
            self.commands = np.clip(cmd, -self.max_cmd, self.max_cmd)
            self.last_cmd_time = self.get_clock().now()

    # ------------------------------------------------------------------- loops

    def policy_step(self):
        with self.lock:
            if self.joint_pos is None:
                return
            now = self.get_clock().now()
            if self.start_time is None:
                self.start_time = now
                self.stand_start_pos = self.joint_pos.copy()

            elapsed = (now - self.start_time).nanoseconds * 1e-9
            if elapsed < self.stand_duration:
                # Ease into the trained default stance before handing over, so the
                # policy starts from a pose it has actually seen.
                alpha = elapsed / self.stand_duration
                self.joint_pos_target = (
                    (1.0 - alpha) * self.stand_start_pos + alpha * self.default_joint_pos
                )
                return
            if self.standing_up:
                self.standing_up = False
                self.get_logger().info("stand-up complete, policy taking over")

            commands = self.commands
            if (
                self.last_cmd_time is None
                or (now - self.last_cmd_time).nanoseconds * 1e-9 > self.cmd_vel_timeout
            ):
                commands = np.zeros(3)

            obs = np.concatenate([
                self.base_lin_vel,
                self.base_ang_vel,
                self.projected_gravity,
                commands,
                self.joint_pos - self.default_joint_pos,
                self.joint_vel,
                self.last_action,
                self.height_scan,
            ])

        with torch.inference_mode():
            action = self.actor(
                torch.as_tensor(obs, dtype=torch.float32, device=self.device).unsqueeze(0)
            ).squeeze(0).cpu().numpy().astype(np.float64)

        with self.lock:
            self.last_action = action
            self.joint_pos_target = self.default_joint_pos + self.action_scale * action

    def pd_step(self):
        with self.lock:
            if self.joint_pos is None:
                return
            error = self.joint_pos_target - self.joint_pos
            kp, kd = (self.stand_kp, self.stand_kd) if self.standing_up else (self.kp, self.kd)
            torque = kp * error - kd * self.joint_vel
        torque = np.clip(torque, -self.torque_limit, self.torque_limit)

        msg = Float64MultiArray()
        msg.data = torque.tolist()
        self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RLPolicyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
