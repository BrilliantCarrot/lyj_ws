import csv
import json
import os
from typing import Dict, List

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32, String

from .swarm_marl_env import SwarmMARLEnv
from .swarm_utils import yaw_from_velocity, yaw_to_quaternion


class SwarmMARLRolloutNode(Node):
    def __init__(self):
        super().__init__("swarm_marl_rollout_node")
        self.agent_count = int(self.declare_parameter("agent_count", 5).value)
        self.world_frame = str(self.declare_parameter("world_frame", "world").value)
        self.env_rate_hz = float(self.declare_parameter("env_rate_hz", 30.0).value)
        self.policy = str(self.declare_parameter("policy", "goal_seeking").value)
        self.metrics_csv = str(
            self.declare_parameter("marl_metrics_csv", "eval/swarm/marl_rollout_metrics.csv").value
        )
        self.auto_reset = bool(self.declare_parameter("auto_reset", True).value)

        config = self.read_env_config()
        self.env = SwarmMARLEnv(config)
        self.obs, self.info = self.env.reset()
        self.episode_index = 0
        self.yaws = [0.0 for _ in range(self.agent_count)]
        self.last_rewards = {idx: 0.0 for idx in range(self.agent_count)}

        self.odom_publishers = [
            self.create_publisher(Odometry, f"/swarm/agent_{idx}/odom", 10)
            for idx in range(self.agent_count)
        ]
        self.reward_publishers = [
            self.create_publisher(Float32, f"/swarm/agent_{idx}/reward", 10)
            for idx in range(self.agent_count)
        ]
        self.obs_publishers = [
            self.create_publisher(String, f"/swarm/agent_{idx}/observation", 10)
            for idx in range(self.agent_count)
        ]
        self.stats_pub = self.create_publisher(String, "/swarm/marl/episode_stats", 10)
        self.task_state_pub = self.create_publisher(String, "/swarm/task_state", 10)

        self.csv_file = None
        self.csv_writer = None
        self.init_csv()

        self.timer = self.create_timer(1.0 / self.env_rate_hz, self.step)
        self.get_logger().info(
            f"[Swarm MARL Rollout] agents={self.agent_count} rate={self.env_rate_hz:.1f}Hz "
            f"policy={self.policy} metrics_csv={self.metrics_csv}"
        )

    def read_env_config(self) -> Dict:
        defaults = {
            "agent_count": 5,
            "env_rate_hz": 30.0,
            "max_episode_steps": 1200,
            "max_accel_mps2": 2.0,
            "max_speed_mps": 2.0,
            "min_altitude_m": 0.2,
            "boundary_m": 12.0,
            "goal_radius_m": 0.45,
            "waypoint_radius_m": 0.45,
            "collision_radius_m": 0.35,
            "obstacle_collision_margin_m": 0.25,
            "terminate_on_agent_collision": True,
            "terminate_on_obstacle_collision": True,
            "obstacle_avoidance_activation_margin_m": 2.0,
            "obstacle_avoidance_gain": 1.4,
            "obstacle_avoidance_tangent_gain": 0.8,
            "obstacle_detour_clearance_m": 1.4,
            "obstacle_detour_altitude_margin_m": 0.8,
            "formation_footprint_margin_m": 0.35,
            "formation_footprint_scale": 0.60,
            "agent_separation_activation_m": 1.0,
            "agent_separation_gain": 1.2,
            "randomize_scenario": False,
            "initial_position_jitter_m": 0.25,
            "goal_position_jitter_m": 0.45,
            "obstacle_position_jitter_m": 0.35,
            "obstacle_radius_jitter_m": 0.10,
            "reward_goal_weight": 1.0,
            "reward_formation_weight": 0.35,
            "reward_control_weight": 0.02,
            "reward_collision_penalty": 25.0,
            "reward_obstacle_penalty": 20.0,
            "reward_success_bonus": 10.0,
            "reward_boundary_penalty": 15.0,
            "marl_seed": 11,
            "initial_positions": [
                0.0, 0.0, 1.5,
                -1.0, 1.0, 1.5,
                -1.0, -1.0, 1.5,
                -2.0, 2.0, 1.5,
                -2.0, -2.0, 1.5,
            ],
            "formation_offsets": [
                0.0, 0.0, 0.0,
                -1.2, 1.0, 0.0,
                -1.2, -1.0, 0.0,
                -2.4, 1.8, 0.0,
                -2.4, -1.8, 0.0,
            ],
            "marl_goals": [
                6.0, 6.0, 2.0,
                4.8, 7.0, 2.0,
                4.8, 5.0, 2.0,
                3.6, 7.8, 2.0,
                3.6, 4.2, 2.0,
            ],
            "marl_leader_waypoints": [
                0.0, 0.0, 1.5,
                6.0, 0.0, 2.0,
                6.0, 5.0, 2.0,
                0.5, 5.5, 2.0,
                0.0, 0.0, 1.5,
            ],
            "static_obstacles": [
                3.8, 0.8, 1.8, 0.60,
                7.1, 5.0, 3.1, 0.65,
                -0.8, 3.1, 1.8, 0.55,
            ],
        }
        return {key: self.read_parameter_or(key, default) for key, default in defaults.items()}

    def read_parameter_or(self, name: str, default):
        if not self.has_parameter(name):
            self.declare_parameter(name, default)
        return self.get_parameter(name).value

    def init_csv(self):
        if not self.metrics_csv:
            return
        directory = os.path.dirname(self.metrics_csv)
        if directory:
            os.makedirs(directory, exist_ok=True)
        self.csv_file = open(self.metrics_csv, "w", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            "episode",
            "step",
            "time_s",
            "scenario_seed",
            "randomized_scenario",
            "mean_reward",
            "mean_goal_distance_m",
            "max_goal_distance_m",
            "formation_rmse_m",
            "min_agent_distance_m",
            "collision_count",
            "obstacle_collision_count",
            "all_goals_reached",
            "waypoint_cycle_complete",
            "success",
            "out_of_bounds",
        ])

    def step(self):
        actions = self.env.scripted_action(self.policy)
        self.obs, rewards, terminated, truncated, self.info = self.env.step(actions)
        self.last_rewards = rewards
        stamp = self.get_clock().now()
        self.publish_agent_topics(stamp)
        self.publish_stats(terminated, truncated)
        self.publish_task_state()
        self.write_metrics()

        if terminated or truncated:
            self.get_logger().info(
                f"[Swarm MARL Episode] episode={self.episode_index} "
                f"steps={self.info['step']} terminated={terminated} truncated={truncated} "
                f"mean_goal_dist={self.info['mean_goal_distance_m']:.3f}m "
                f"formation_rmse={self.info['formation_rmse_m']:.3f}m "
                f"agent_collisions={self.info['collision_count']} "
                f"obstacle_collisions={self.info['obstacle_collision_count']} "
                f"success={self.info['success']}"
            )
            if self.auto_reset:
                self.episode_index += 1
                self.obs, self.info = self.env.reset()

    def publish_agent_topics(self, stamp):
        for idx in range(self.agent_count):
            position = self.env.positions[idx]
            velocity = self.env.velocities[idx]
            self.yaws[idx] = yaw_from_velocity(velocity, self.yaws[idx])
            msg = Odometry()
            msg.header.stamp = stamp.to_msg()
            msg.header.frame_id = self.world_frame
            msg.child_frame_id = f"agent_{idx}/base_link"
            msg.pose.pose.position.x = position[0]
            msg.pose.pose.position.y = position[1]
            msg.pose.pose.position.z = position[2]
            qx, qy, qz, qw = yaw_to_quaternion(self.yaws[idx])
            msg.pose.pose.orientation.x = qx
            msg.pose.pose.orientation.y = qy
            msg.pose.pose.orientation.z = qz
            msg.pose.pose.orientation.w = qw
            msg.twist.twist.linear.x = velocity[0]
            msg.twist.twist.linear.y = velocity[1]
            msg.twist.twist.linear.z = velocity[2]
            self.odom_publishers[idx].publish(msg)

            reward_msg = Float32()
            reward_msg.data = float(self.last_rewards.get(idx, 0.0))
            self.reward_publishers[idx].publish(reward_msg)
            self.obs_publishers[idx].publish(String(data=json.dumps(self.obs[idx])))

    def publish_stats(self, terminated: bool, truncated: bool):
        payload = dict(self.info)
        payload["episode"] = self.episode_index
        payload["policy"] = self.policy
        payload["terminated"] = terminated
        payload["truncated"] = truncated
        payload["mean_reward"] = sum(self.last_rewards.values()) / max(1, len(self.last_rewards))
        self.stats_pub.publish(String(data=json.dumps(payload)))

    def publish_task_state(self):
        tasks = []
        completed_count = 0
        for idx, goal in enumerate(self.env.goals):
            if idx < len(self.env.positions):
                dx = goal[0] - self.env.positions[idx][0]
                dy = goal[1] - self.env.positions[idx][1]
                dz = goal[2] - self.env.positions[idx][2]
                distance = (dx * dx + dy * dy + dz * dz) ** 0.5
            else:
                distance = float("inf")
            completed = distance <= self.env.goal_radius
            if completed:
                completed_count += 1
            tasks.append({
                "task_id": idx,
                "position": goal,
                "status": "completed" if completed else "assigned",
                "assigned_agent": idx,
                "completed_time_s": self.info["time_s"] if completed else None,
            })
        payload = {
            "mode": "marl_rollout",
            "paused": False,
            "allocation_mode": self.policy,
            "auction_seq": 0,
            "completed_task_count": completed_count,
            "total_task_count": len(tasks),
            "task_completion_ratio": completed_count / max(1, len(tasks)),
            "mean_task_completion_time_s": 0.0,
            "tasks": tasks,
        }
        self.task_state_pub.publish(String(data=json.dumps(payload)))

    def write_metrics(self):
        if self.csv_writer is None:
            return
        mean_reward = sum(self.last_rewards.values()) / max(1, len(self.last_rewards))
        self.csv_writer.writerow([
            self.episode_index,
            self.info["step"],
            f"{self.info['time_s']:.3f}",
            self.info["scenario_seed"],
            int(bool(self.info["randomized_scenario"])),
            f"{mean_reward:.6f}",
            f"{self.info['mean_goal_distance_m']:.6f}",
            f"{self.info['max_goal_distance_m']:.6f}",
            f"{self.info['formation_rmse_m']:.6f}",
            f"{self.info['min_agent_distance_m']:.6f}",
            self.info["collision_count"],
            self.info["obstacle_collision_count"],
            int(self.info["all_goals_reached"]),
            int(self.info["waypoint_cycle_complete"]),
            int(self.info["success"]),
            int(self.info["out_of_bounds"]),
        ])
        if self.info["step"] % int(max(1, self.env_rate_hz)) == 0:
            self.csv_file.flush()

    def destroy_node(self):
        if self.csv_file is not None:
            self.csv_file.flush()
            self.csv_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SwarmMARLRolloutNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
