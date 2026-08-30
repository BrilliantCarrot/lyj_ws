import csv
import json
import os
from pathlib import Path
from typing import Dict, List, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String

from .swarm_utils import add, dynamic_obstacle_list, norm, obstacle_list, rmse, sub, vec3_list


State = Tuple[List[float], List[float]]


class SwarmEvalNode(Node):
    def __init__(self):
        super().__init__("swarm_eval_node")
        self.agent_count = int(self.declare_parameter("agent_count", 5).value)
        self.eval_rate_hz = float(self.declare_parameter("eval_rate_hz", 10.0).value)
        self.collision_radius = float(self.declare_parameter("collision_radius_m", 0.35).value)
        self.obstacle_collision_margin = float(
            self.declare_parameter("obstacle_collision_margin_m", 0.25).value
        )
        self.dynamic_obstacle_collision_margin = float(
            self.declare_parameter("dynamic_obstacle_collision_margin_m", 0.25).value
        )
        self.formation_reference_mode = str(
            self.declare_parameter("formation_reference_mode", "leader_lookahead").value
        )
        self.metrics_csv = str(
            self.declare_parameter("metrics_csv", "eval/swarm/swarm_demo_metrics.csv").value
        )
        self.formation_offsets = vec3_list(
            self.declare_parameter(
                "formation_offsets",
                [
                    0.0, 0.0, 0.0,
                    -1.2, 1.0, 0.0,
                    -1.2, -1.0, 0.0,
                    -2.4, 1.8, 0.0,
                    -2.4, -1.8, 0.0,
                ],
            ).value
        )
        while len(self.formation_offsets) < self.agent_count:
            self.formation_offsets.append([0.0, 0.0, 0.0])
        self.formation_edges = [
            int(v)
            for v in self.declare_parameter(
                "formation_edges",
                [
                    0, 1,
                    0, 2,
                    1, 3,
                    2, 4,
                    1, 2,
                    3, 4,
                ],
            ).value
        ]
        self.static_obstacles = obstacle_list(
            self.declare_parameter(
                "static_obstacles",
                [
                    2.0, 1.8, 1.8, 0.55,
                    -1.2, 2.6, 1.7, 0.45,
                    2.8, -1.2, 1.8, 0.50,
                ],
            ).value
        )
        self.dynamic_obstacle_specs = dynamic_obstacle_list(
            self.declare_parameter(
                "dynamic_obstacles",
                [
                    4.0, -1.2, 1.8, 0.45, 0.0, 0.8, 0.0, 3.0,
                    9.0, 4.0, 2.2, 0.50, -0.7, 0.0, 0.0, 3.5,
                ],
            ).value
        )

        self.states: Dict[int, State] = {}
        self.dynamic_obstacles: Dict[int, State] = {}
        self.references: Dict[int, List[float]] = {}
        self.formation_error_history: List[float] = []
        self.min_distance_history: List[float] = []
        self.collision_samples = 0
        self.obstacle_collision_samples = 0
        self.dynamic_obstacle_collision_samples = 0
        self.sample_count = 0
        self.task_completion_ratio = 0.0
        self.completed_task_count = 0
        self.total_task_count = 0
        self.mean_task_completion_time_s = 0.0

        self.odom_subs = [
            self.create_subscription(
                Odometry,
                f"/swarm/agent_{i}/odom",
                lambda msg, idx=i: self.odom_callback(msg, idx),
                10,
            )
            for i in range(self.agent_count)
        ]
        self.reference_subs = [
            self.create_subscription(
                PoseStamped,
                f"/swarm/agent_{i}/reference",
                lambda msg, idx=i: self.reference_callback(msg, idx),
                10,
            )
            for i in range(self.agent_count)
        ]
        self.dynamic_obstacle_subs = [
            self.create_subscription(
                Odometry,
                f"/swarm/obstacle_{i}/odom",
                lambda msg, idx=i: self.dynamic_obstacle_callback(msg, idx),
                10,
            )
            for i in range(len(self.dynamic_obstacle_specs))
        ]
        self.task_state_sub = self.create_subscription(
            String,
            "/swarm/task_state",
            self.task_state_callback,
            10,
        )
        self.metrics_pub = self.create_publisher(String, "/swarm/metrics", 10)
        self.csv_path = self.resolve_csv_path(self.metrics_csv)
        self.prepare_csv()
        self.timer = self.create_timer(1.0 / self.eval_rate_hz, self.eval_step)
        self.get_logger().info(
            f"[Swarm Eval] metrics_csv={self.csv_path} "
            f"formation_reference_mode={self.formation_reference_mode}"
        )

    def resolve_csv_path(self, csv_path: str) -> Path:
        path = Path(csv_path)
        if path.is_absolute():
            return path
        return Path.cwd() / path

    def prepare_csv(self):
        self.csv_path.parent.mkdir(parents=True, exist_ok=True)
        with self.csv_path.open("w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                "time_s",
                "formation_rmse_m",
                "instant_formation_error_m",
                "edge_error_rmse_m",
                "min_inter_agent_distance_m",
                "min_obstacle_clearance_m",
                "min_dynamic_obstacle_clearance_m",
                "collision_samples",
                "obstacle_collision_samples",
                "dynamic_obstacle_collision_samples",
                "task_completion_ratio",
                "completed_task_count",
                "total_task_count",
                "mean_task_completion_time_s",
            ])

    def odom_callback(self, msg: Odometry, idx: int):
        position = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        ]
        velocity = [
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z,
        ]
        self.states[idx] = (position, velocity)

    def dynamic_obstacle_callback(self, msg: Odometry, idx: int):
        position = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        ]
        velocity = [
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z,
        ]
        self.dynamic_obstacles[idx] = (position, velocity)

    def reference_callback(self, msg: PoseStamped, idx: int):
        self.references[idx] = [
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        ]

    def task_state_callback(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        self.task_completion_ratio = float(payload.get("task_completion_ratio", 0.0))
        self.completed_task_count = int(payload.get("completed_task_count", 0))
        self.total_task_count = int(payload.get("total_task_count", 0))
        self.mean_task_completion_time_s = float(
            payload.get("mean_task_completion_time_s", 0.0)
        )

    def eval_step(self):
        if len(self.states) < self.agent_count or 0 not in self.states:
            return
        self.sample_count += 1
        leader_position, _ = self.states[0]

        follower_errors = []
        for idx in range(1, self.agent_count):
            position, _ = self.states[idx]
            desired = add(leader_position, self.formation_offsets[idx])
            follower_errors.append(norm(sub(position, desired)))
        instant_formation_error = rmse(follower_errors)
        self.formation_error_history.append(instant_formation_error)
        edge_error_rmse = self.compute_edge_error_rmse()

        min_distance = self.compute_min_distance()
        self.min_distance_history.append(min_distance)
        if min_distance < self.collision_radius:
            self.collision_samples += 1
        min_obstacle_clearance = self.compute_min_obstacle_clearance()
        if min_obstacle_clearance < self.obstacle_collision_margin:
            self.obstacle_collision_samples += 1
        min_dynamic_obstacle_clearance = self.compute_min_dynamic_obstacle_clearance()
        if min_dynamic_obstacle_clearance < self.dynamic_obstacle_collision_margin:
            self.dynamic_obstacle_collision_samples += 1

        formation_rmse = rmse(self.formation_error_history)
        summary = (
            f"formation_rmse={formation_rmse:.3f}m "
            f"instant={instant_formation_error:.3f}m "
            f"edge_rmse={edge_error_rmse:.3f}m "
            f"min_dist={min_distance:.3f}m "
            f"obs_clearance={min_obstacle_clearance:.3f}m "
            f"dyn_obs_clearance={min_dynamic_obstacle_clearance:.3f}m "
            f"collision_samples={self.collision_samples} "
            f"obstacle_collision_samples={self.obstacle_collision_samples} "
            f"dynamic_obstacle_collision_samples={self.dynamic_obstacle_collision_samples} "
            f"task_completion={self.completed_task_count}/{self.total_task_count} "
            f"task_ratio={self.task_completion_ratio:.2f}"
        )
        self.metrics_pub.publish(String(data=summary))

        if self.sample_count % max(1, int(self.eval_rate_hz * 2.0)) == 0:
            self.get_logger().info(f"[Swarm Metrics] {summary}")
        self.append_csv(
            formation_rmse,
            instant_formation_error,
            edge_error_rmse,
            min_distance,
            min_obstacle_clearance,
            min_dynamic_obstacle_clearance,
        )

    def compute_edge_error_rmse(self) -> float:
        if len(self.formation_edges) < 2:
            return 0.0
        edges = self.formation_edges[:-1] if len(self.formation_edges) % 2 else self.formation_edges
        edge_errors = []
        for k in range(0, len(edges), 2):
            a_idx = edges[k]
            b_idx = edges[k + 1]
            if (
                a_idx < 0
                or b_idx < 0
                or a_idx >= self.agent_count
                or b_idx >= self.agent_count
                or a_idx not in self.states
                or b_idx not in self.states
            ):
                continue
            pa, _ = self.states[a_idx]
            pb, _ = self.states[b_idx]
            actual_relative = sub(pa, pb)
            desired_relative = sub(
                self.formation_offsets[a_idx],
                self.formation_offsets[b_idx],
            )
            edge_errors.append(norm(sub(actual_relative, desired_relative)))
        return rmse(edge_errors)

    def compute_min_distance(self) -> float:
        minimum = float("inf")
        for i in range(self.agent_count):
            for j in range(i + 1, self.agent_count):
                pi, _ = self.states[i]
                pj, _ = self.states[j]
                minimum = min(minimum, norm(sub(pi, pj)))
        return minimum if minimum < float("inf") else 0.0

    def compute_min_obstacle_clearance(self) -> float:
        if not self.static_obstacles:
            return float("inf")
        minimum = float("inf")
        for position, _ in self.states.values():
            for obstacle in self.static_obstacles:
                clearance = norm(sub(position, obstacle[:3])) - obstacle[3]
                minimum = min(minimum, clearance)
        return minimum if minimum < float("inf") else 0.0

    def compute_min_dynamic_obstacle_clearance(self) -> float:
        if not self.dynamic_obstacles:
            return float("inf")
        minimum = float("inf")
        for agent_position, _ in self.states.values():
            for idx, (obstacle_position, _) in self.dynamic_obstacles.items():
                radius = self.dynamic_obstacle_specs[idx][3]
                clearance = norm(sub(agent_position, obstacle_position)) - radius
                minimum = min(minimum, clearance)
        return minimum if minimum < float("inf") else 0.0

    def append_csv(
        self,
        formation_rmse: float,
        instant_error: float,
        edge_error_rmse: float,
        min_distance: float,
        min_obstacle_clearance: float,
        min_dynamic_obstacle_clearance: float,
    ):
        now = self.get_clock().now().nanoseconds * 1.0e-9
        with self.csv_path.open("a", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                f"{now:.6f}",
                f"{formation_rmse:.6f}",
                f"{instant_error:.6f}",
                f"{edge_error_rmse:.6f}",
                f"{min_distance:.6f}",
                f"{min_obstacle_clearance:.6f}",
                f"{min_dynamic_obstacle_clearance:.6f}",
                self.collision_samples,
                self.obstacle_collision_samples,
                self.dynamic_obstacle_collision_samples,
                f"{self.task_completion_ratio:.6f}",
                self.completed_task_count,
                self.total_task_count,
                f"{self.mean_task_completion_time_s:.6f}",
            ])


def main(args=None):
    rclpy.init(args=args)
    node = SwarmEvalNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
