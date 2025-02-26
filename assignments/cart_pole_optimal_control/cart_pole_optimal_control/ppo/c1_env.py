# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from __future__ import annotations

import math
import torch
from collections.abc import Sequence

from isaaclab_assets.robots.cartpole import CARTPOLE_CFG

import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation, ArticulationCfg
from isaaclab.envs import DirectRLEnv, DirectRLEnvCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sim import SimulationCfg
from isaaclab.sim.spawners.from_files import GroundPlaneCfg, spawn_ground_plane
from isaaclab.utils import configclass
from isaaclab.utils.math import sample_uniform

from isaaclab.sim import SimulationContext

import csv
import os

@configclass
class C1EnvCfg(DirectRLEnvCfg):
    # env
    decimation = 2
    episode_length_s = 5.0
    action_scale = 100.0  # [N]
    action_space = 1
    observation_space = 4
    state_space = 0

    # simulation
    sim: SimulationCfg = SimulationCfg(dt=1 / 120, render_interval=decimation)

    # robot
    robot_cfg: ArticulationCfg = CARTPOLE_CFG.replace(prim_path="/World/envs/env_.*/Robot")
    cart_dof_name = "slider_to_cart"
    pole_dof_name = "cart_to_pole"

    # scene
    scene: InteractiveSceneCfg = InteractiveSceneCfg(num_envs=4096, env_spacing=4.0, replicate_physics=True)

    # reset
    max_cart_pos = 2.5  # the cart is reset if it exceeds that position [m]
    initial_pole_angle_range = [-0.05, 0.05]  # the range in which the pole angle is sampled from on reset [rad]

    # reward scales
    rew_scale_alive = 1.0
    rew_scale_terminated = -2.0
    rew_scale_pole_pos = -1.0
    rew_scale_cart_vel = -0.01
    rew_scale_pole_vel = -0.005

    # Earthquake force parameters
    earthquake_base_amplitude = 15.0  # Base force amplitude in N
    earthquake_frequency_range = [0.5, 4.0]  # Frequency range in Hz
    earthquake_num_waves = 5  # Number of sine waves to superimpose
    earthquake_noise_std = 1.5  # Standard deviation of added noise

class C1Env(DirectRLEnv):
    cfg: C1EnvCfg

    def __init__(self, cfg: C1EnvCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)

        self.sim_context = SimulationContext.instance()

        self._cart_dof_idx, _ = self.cartpole.find_joints(self.cfg.cart_dof_name)
        self._pole_dof_idx, _ = self.cartpole.find_joints(self.cfg.pole_dof_name)
        self.action_scale = self.cfg.action_scale

        self.joint_pos = self.cartpole.data.joint_pos
        self.joint_vel = self.cartpole.data.joint_vel

        # Initialize earthquake force parameters
        self.earthquake_force = torch.zeros(self.num_envs, device=self.device)
        self.earthquake_frequencies = torch.rand(self.cfg.earthquake_num_waves, device=self.device) * (
            self.cfg.earthquake_frequency_range[1] - self.cfg.earthquake_frequency_range[0]
        ) + self.cfg.earthquake_frequency_range[0]
        self.earthquake_phases = torch.rand(self.cfg.earthquake_num_waves, device=self.device) * 2 * math.pi

        # Storage for visualization
        self.cart_positions = []
        self.pole_angles = []
        self.actions_log = []
        self.earthquake_log = []
        self.time_steps = []

        # Create a CSV file for logging
        self.log_file = "simulation_data.csv"
        if not os.path.exists(self.log_file):
            with open(self.log_file, mode="w", newline="") as file:
                writer = csv.writer(file)
                writer.writerow(["Iteration", "Elapsed Time (s)", "Cart Displacement (m)", 
                                "Pole Deviation (rad)", "Earthquake Force (N)", 
                                "Total Applied Force (N)", "Total Reward"])


    def _setup_scene(self):
        self.cartpole = Articulation(self.cfg.robot_cfg)
        # add ground plane
        spawn_ground_plane(prim_path="/World/ground", cfg=GroundPlaneCfg())
        # clone and replicate
        self.scene.clone_environments(copy_from_source=False)
        # add articulation to scene
        self.scene.articulations["cartpole"] = self.cartpole
        # add lights
        light_cfg = sim_utils.DomeLightCfg(intensity=2000.0, color=(0.75, 0.75, 0.75))
        light_cfg.func("/World/Light", light_cfg)

    def _generate_earthquake_force(self):
        """Generate earthquake force using multiple sine waves."""
        dt = self.sim_context.get_physics_dt()  # Get physics time step
        time = dt * self.episode_length_buf  # Manually compute elapsed time

        force = torch.zeros(self.num_envs, device=self.device)

        for freq, phase in zip(self.earthquake_frequencies, self.earthquake_phases):
            amplitude = self.cfg.earthquake_base_amplitude * (0.8 + 0.4 * torch.rand(1, device=self.device))
            force += amplitude * torch.sin(2 * math.pi * freq * time + phase)

        force += torch.randn(self.num_envs, device=self.device) * self.cfg.earthquake_noise_std
        return force


    def _pre_physics_step(self, actions: torch.Tensor) -> None:
        self.actions = self.action_scale * actions.clone()
        self.earthquake_force = self._generate_earthquake_force()

        # Get simulation time step
        dt = self.sim_context.get_physics_dt()
        elapsed_time = len(self.time_steps) * dt  # Compute elapsed time in simulation

        # Store data for plotting and evaluation
        cart_pos = self.joint_pos[:, self._cart_dof_idx[0]].mean().item()  # Mean cart position
        pole_angle = self.joint_pos[:, self._pole_dof_idx[0]].mean().item()  # Mean pole angle
        applied_force = self.actions.mean().item()  # Mean applied action force
        earthquake_force = self.earthquake_force.mean().item()  # Mean earthquake force
        total_force = applied_force + earthquake_force  # Combined force
        current_step = len(self.time_steps)  # Iteration number

        # Calculate total reward
        total_reward = compute_rewards(
            self.cfg.rew_scale_alive,
            self.cfg.rew_scale_terminated,
            self.cfg.rew_scale_pole_pos,
            self.cfg.rew_scale_cart_vel,
            self.cfg.rew_scale_pole_vel,
            self.joint_pos[:, self._pole_dof_idx[0]],
            self.joint_vel[:, self._pole_dof_idx[0]],
            self.joint_pos[:, self._cart_dof_idx[0]],
            self.joint_vel[:, self._cart_dof_idx[0]],
            self.reset_terminated,
        ).mean().item()  # Compute mean reward

        # Append data to storage
        self.cart_positions.append(cart_pos)
        self.pole_angles.append(pole_angle)
        self.actions_log.append(applied_force)
        self.earthquake_log.append(earthquake_force)
        self.time_steps.append(current_step)

        # Write data to CSV
        with open(self.log_file, mode="a", newline="") as file:
            writer = csv.writer(file)
            writer.writerow([current_step, elapsed_time, cart_pos, pole_angle, earthquake_force, total_force, total_reward])



    def _apply_action(self) -> None:
        """Apply both PPO control force and earthquake force to the cart."""
        
        # Ensure actions and earthquake forces are correctly shaped
        actions = self.actions.view(self.num_envs, 1)  # Ensure shape [64, 1]
        earthquake_force = self.earthquake_force.view(self.num_envs, 1)  # Ensure shape [64, 1]

        # Combine forces
        total_force = actions + earthquake_force  # Resulting shape: [64, 1]

        # Apply force to the cart
        self.cartpole.set_joint_effort_target(total_force, joint_ids=self._cart_dof_idx)

    def _get_observations(self) -> dict:
        obs = torch.cat(
            (
                self.joint_pos[:, self._pole_dof_idx[0]].unsqueeze(dim=1),
                self.joint_vel[:, self._pole_dof_idx[0]].unsqueeze(dim=1),
                self.joint_pos[:, self._cart_dof_idx[0]].unsqueeze(dim=1),
                self.joint_vel[:, self._cart_dof_idx[0]].unsqueeze(dim=1),
            ),
            dim=-1,
        )
        observations = {"policy": obs}
        return observations

    def _get_rewards(self) -> torch.Tensor:
        total_reward = compute_rewards(
            self.cfg.rew_scale_alive,
            self.cfg.rew_scale_terminated,
            self.cfg.rew_scale_pole_pos,
            self.cfg.rew_scale_cart_vel,
            self.cfg.rew_scale_pole_vel,
            self.joint_pos[:, self._pole_dof_idx[0]],
            self.joint_vel[:, self._pole_dof_idx[0]],
            self.joint_pos[:, self._cart_dof_idx[0]],
            self.joint_vel[:, self._cart_dof_idx[0]],
            self.reset_terminated,
        )
        return total_reward

    def _get_dones(self) -> tuple[torch.Tensor, torch.Tensor]:
        self.joint_pos = self.cartpole.data.joint_pos
        self.joint_vel = self.cartpole.data.joint_vel

        time_out = self.episode_length_buf >= self.max_episode_length - 1
        out_of_bounds = torch.any(torch.abs(self.joint_pos[:, self._cart_dof_idx]) > self.cfg.max_cart_pos, dim=1)
        out_of_bounds = out_of_bounds | torch.any(torch.abs(self.joint_pos[:, self._pole_dof_idx]) > math.pi / 2, dim=1)
        return out_of_bounds, time_out

    def _reset_idx(self, env_ids: Sequence[int] | None):
        if env_ids is None:
            env_ids = self.cartpole._ALL_INDICES
        super()._reset_idx(env_ids)

        joint_pos = self.cartpole.data.default_joint_pos[env_ids]
        joint_pos[:, self._pole_dof_idx] += sample_uniform(
            self.cfg.initial_pole_angle_range[0] * math.pi,
            self.cfg.initial_pole_angle_range[1] * math.pi,
            joint_pos[:, self._pole_dof_idx].shape,
            joint_pos.device,
        )
        joint_vel = self.cartpole.data.default_joint_vel[env_ids]

        default_root_state = self.cartpole.data.default_root_state[env_ids]
        default_root_state[:, :3] += self.scene.env_origins[env_ids]

        self.joint_pos[env_ids] = joint_pos
        self.joint_vel[env_ids] = joint_vel

        self.cartpole.write_root_pose_to_sim(default_root_state[:, :7], env_ids)
        self.cartpole.write_root_velocity_to_sim(default_root_state[:, 7:], env_ids)
        self.cartpole.write_joint_state_to_sim(joint_pos, joint_vel, None, env_ids)


@torch.jit.script
def compute_rewards(
    rew_scale_alive: float,
    rew_scale_terminated: float,
    rew_scale_pole_pos: float,
    rew_scale_cart_vel: float,
    rew_scale_pole_vel: float,
    pole_pos: torch.Tensor,
    pole_vel: torch.Tensor,
    cart_pos: torch.Tensor,
    cart_vel: torch.Tensor,
    reset_terminated: torch.Tensor,
):
    rew_alive = rew_scale_alive * (1.0 - reset_terminated.float())
    rew_termination = rew_scale_terminated * reset_terminated.float()
    rew_pole_pos = rew_scale_pole_pos * torch.sum(torch.square(pole_pos).unsqueeze(dim=1), dim=-1)
    rew_cart_vel = rew_scale_cart_vel * torch.sum(torch.abs(cart_vel).unsqueeze(dim=1), dim=-1)
    rew_pole_vel = rew_scale_pole_vel * torch.sum(torch.abs(pole_vel).unsqueeze(dim=1), dim=-1)
    total_reward = rew_alive + rew_termination + rew_pole_pos + rew_cart_vel + rew_pole_vel
    return total_reward
