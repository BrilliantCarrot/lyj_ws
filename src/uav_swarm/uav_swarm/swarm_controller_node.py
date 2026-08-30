from typing import Dict, List, Tuple

import rclpy
from geometry_msgs.msg import AccelStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String

from .swarm_utils import add, clamp_norm, dynamic_obstacle_list, norm, obstacle_list, scale, sub, vec3_list


State = Tuple[List[float], List[float]]


class SwarmControllerNode(Node):
    def __init__(self):
        super().__init__("swarm_controller_node")
        self.agent_count = int(self.declare_parameter("agent_count", 5).value)
        self.control_rate_hz = float(self.declare_parameter("control_rate_hz", 30.0).value)
        self.kp = float(self.declare_parameter("kp_position", 1.6).value)
        self.kd = float(self.declare_parameter("kd_velocity", 1.4).value)
        self.max_accel = float(self.declare_parameter("max_accel_mps2", 2.0).value)
        self.leader_speed = float(self.declare_parameter("leader_speed_mps", 1.0).value)
        self.use_communication_layer = bool(
            self.declare_parameter("use_communication_layer", True).value
        )
        self.formation_reference_mode = str(
            self.declare_parameter("formation_reference_mode", "leader_lookahead").value
        )
        self.formation_lookahead_s = float(
            self.declare_parameter("formation_lookahead_s", 0.8).value
        )
        self.repulsion_gain = float(self.declare_parameter("repulsion_gain", 0.55).value)
        self.repulsion_radius = float(self.declare_parameter("repulsion_radius_m", 1.5).value)
        self.obstacle_repulsion_gain = float(
            self.declare_parameter("obstacle_repulsion_gain", 1.0).value
        )
        self.obstacle_repulsion_radius = float(
            self.declare_parameter("obstacle_repulsion_radius_m", 2.0).value
        )
        self.dynamic_obstacle_repulsion_gain = float(
            self.declare_parameter("dynamic_obstacle_repulsion_gain", 1.2).value
        )
        self.dynamic_obstacle_repulsion_radius = float(
            self.declare_parameter("dynamic_obstacle_repulsion_radius_m", 2.8).value
        )
        self.dynamic_obstacle_closing_gain = float(
            self.declare_parameter("dynamic_obstacle_closing_gain", 0.8).value
        )
        self.formation_control_topology = str(
            self.declare_parameter("formation_control_topology", "parent_graph").value
        )
        self.parent_graph_use_reference = bool(
            self.declare_parameter("parent_graph_use_reference", False).value
        )
        self.neighbor_leader_anchor_gain = float(
            self.declare_parameter("neighbor_leader_anchor_gain", 0.25).value
        )
        self.consensus_gain = float(self.declare_parameter("consensus_gain", 1.0).value)
        self.consensus_velocity_gain = float(
            self.declare_parameter("consensus_velocity_gain", 1.0).value
        )
        self.consensus_leader_gain = float(
            self.declare_parameter("consensus_leader_gain", 1.0).value
        )
        self.mission_mode = str(self.declare_parameter("mission_mode", "formation").value)
        self.paused = False

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
        self.parent_indices = [
            int(v)
            for v in self.declare_parameter(
                "formation_parent_indices",
                [0, 0, 0, 1, 2],
            ).value
        ]
        while len(self.parent_indices) < self.agent_count:
            self.parent_indices.append(0)
        self.parent_indices = self.parent_indices[: self.agent_count]
        self.parent_indices[0] = 0
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
        self.neighbor_indices = self.build_neighbor_graph(self.formation_edges)
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

        self.leader_waypoints = vec3_list(
            self.declare_parameter(
                "leader_waypoints",
                [
                    0.0, 0.0, 1.5,
                    4.0, 0.0, 2.0,
                    4.0, 4.0, 2.5,
                    0.0, 4.0, 2.0,
                    0.0, 0.0, 1.5,
                ],
            ).value
        )
        if not self.leader_waypoints:
            self.leader_waypoints = [[0.0, 0.0, 1.5]]
        self.target_index = 1 if len(self.leader_waypoints) > 1 else 0

        self.states: Dict[int, State] = {}
        self.reference_positions: Dict[int, List[float]] = {}
        self.reference_velocities: Dict[int, List[float]] = {}
        self.mission_goals: Dict[int, List[float]] = {}
        self.center_estimates: Dict[int, List[float]] = {}
        self.center_velocity_estimates: Dict[int, List[float]] = {}
        self.dynamic_obstacles: Dict[int, State] = {}
        odom_suffix = "odom_comm" if self.use_communication_layer else "odom"
        self.odom_subs = [
            self.create_subscription(
                Odometry,
                f"/swarm/agent_{i}/{odom_suffix}",
                lambda msg, idx=i: self.odom_callback(msg, idx),
                10,
            )
            for i in range(self.agent_count)
        ]
        self.cmd_publishers = [
            self.create_publisher(AccelStamped, f"/swarm/agent_{i}/accel_cmd", 10)
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
        self.mission_goal_subs = [
            self.create_subscription(
                PoseStamped,
                f"/swarm/agent_{i}/mission_goal",
                lambda msg, idx=i: self.mission_goal_callback(msg, idx),
                10,
            )
            for i in range(self.agent_count)
        ]
        self.command_sub = self.create_subscription(
            String,
            "/swarm/mission_command",
            self.command_callback,
            10,
        )
        self.reference_publishers = [
            self.create_publisher(PoseStamped, f"/swarm/agent_{i}/reference", 10)
            for i in range(self.agent_count)
        ]
        self.timer = self.create_timer(1.0 / self.control_rate_hz, self.control_step)
        self.get_logger().info(
            f"[Swarm Controller] agents={self.agent_count} rate={self.control_rate_hz:.1f}Hz "
            f"leader_speed={self.leader_speed:.2f} formation_reference_mode={self.formation_reference_mode} "
            f"use_communication_layer={self.use_communication_layer} topology={self.formation_control_topology} "
            f"parent_graph_use_reference={self.parent_graph_use_reference} "
            f"neighbor_leader_anchor_gain={self.neighbor_leader_anchor_gain:.2f} "
            f"consensus_gain={self.consensus_gain:.2f} mission_mode={self.mission_mode}"
        )

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
        if idx not in self.center_estimates and idx < len(self.formation_offsets):
            self.center_estimates[idx] = sub(position, self.formation_offsets[idx])
            self.center_velocity_estimates[idx] = velocity

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

    def mission_goal_callback(self, msg: PoseStamped, idx: int):
        self.mission_goals[idx] = [
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        ]

    def command_callback(self, msg: String):
        command = msg.data.strip().lower()
        if command in ("formation", "f"):
            self.mission_mode = "formation"
            self.paused = False
        elif command in ("task", "task_allocation", "t"):
            self.mission_mode = "task_allocation"
            self.paused = False
        elif command in ("pause", "p"):
            self.paused = True
        elif command in ("resume", "r"):
            self.paused = False
        elif command in ("reset", "reset_tasks", "x"):
            self.mission_goals.clear()
            self.mission_mode = "formation"
            self.paused = False
        else:
            return
        self.get_logger().info(
            f"[Swarm Controller] command={command} mission_mode={self.mission_mode} paused={self.paused}"
        )

    def control_step(self):
        if 0 not in self.states:
            return
        if self.paused:
            for idx in range(self.agent_count):
                if idx in self.states:
                    self.publish_accel(idx, [0.0, 0.0, 0.0])
            return

        leader_position, leader_velocity = self.states[0]
        leader_target, leader_v_des = self.update_leader_target(leader_position)
        leader_formation_anchor, leader_formation_velocity = self.leader_formation_anchor(
            leader_position,
            leader_velocity,
            leader_target,
            leader_v_des,
        )
        self.reference_positions[0] = leader_formation_anchor
        self.reference_velocities[0] = leader_formation_velocity
        if self.formation_control_topology == "consensus_center":
            self.update_consensus_estimates(leader_formation_anchor, leader_formation_velocity)

        for idx in range(self.agent_count):
            if idx not in self.states:
                continue
            position, velocity = self.states[idx]
            if self.mission_mode == "task_allocation" and idx in self.mission_goals:
                desired_position = self.mission_goals[idx]
                desired_velocity = [0.0, 0.0, 0.0]
            elif idx == 0:
                desired_position = leader_target
                desired_velocity = leader_v_des
            elif self.formation_control_topology == "parent_graph":
                desired_position, desired_velocity = self.parent_graph_reference(idx)
            elif self.formation_control_topology == "neighbor_graph":
                desired_position, desired_velocity = self.neighbor_graph_reference(
                    idx,
                    leader_position,
                    leader_velocity,
                    leader_formation_anchor,
                    leader_formation_velocity,
                )
            elif self.formation_control_topology == "consensus_center":
                desired_position, desired_velocity = self.consensus_center_reference(idx)
            else:
                if self.formation_reference_mode == "leader_actual":
                    desired_position = add(leader_position, self.formation_offsets[idx])
                    desired_velocity = leader_velocity
                elif self.formation_reference_mode == "leader_waypoint":
                    desired_position = add(leader_target, self.formation_offsets[idx])
                    desired_velocity = leader_v_des
                else:
                    leader_lookahead = add(
                        leader_position,
                        scale(leader_v_des, self.formation_lookahead_s),
                    )
                    desired_position = add(leader_lookahead, self.formation_offsets[idx])
                    desired_velocity = leader_v_des

            accel = add(
                scale(sub(desired_position, position), self.kp),
                scale(sub(desired_velocity, velocity), self.kd),
            )
            accel = add(accel, self.repulsion_accel(idx))
            accel = add(accel, self.obstacle_repulsion_accel(position))
            accel = add(accel, self.dynamic_obstacle_repulsion_accel(position, velocity))
            accel = clamp_norm(accel, self.max_accel)
            if idx == 0:
                self.reference_positions[idx] = leader_formation_anchor
                self.reference_velocities[idx] = leader_formation_velocity
            else:
                self.reference_positions[idx] = desired_position
                self.reference_velocities[idx] = desired_velocity
            self.publish_accel(idx, accel)
            self.publish_reference(idx, desired_position)

    def build_neighbor_graph(self, flat_edges: List[int]):
        neighbors = {idx: [] for idx in range(self.agent_count)}
        if len(flat_edges) % 2 != 0:
            self.get_logger().warn("formation_edges length must be even; last value is ignored")
            flat_edges = flat_edges[:-1]

        for k in range(0, len(flat_edges), 2):
            a = flat_edges[k]
            b = flat_edges[k + 1]
            if a < 0 or b < 0 or a >= self.agent_count or b >= self.agent_count or a == b:
                self.get_logger().warn(f"ignoring invalid formation edge ({a}, {b})")
                continue
            if b not in neighbors[a]:
                neighbors[a].append(b)
            if a not in neighbors[b]:
                neighbors[b].append(a)
        return neighbors

    def update_consensus_estimates(
        self,
        leader_anchor: List[float],
        leader_anchor_velocity: List[float],
    ):
        dt = 1.0 / max(self.control_rate_hz, 1.0e-6)
        next_centers = {}
        next_center_velocities = {}

        for idx in range(self.agent_count):
            if idx not in self.states:
                continue
            if idx not in self.center_estimates:
                position, velocity = self.states[idx]
                self.center_estimates[idx] = sub(position, self.formation_offsets[idx])
                self.center_velocity_estimates[idx] = velocity

            center = self.center_estimates[idx]
            center_velocity = self.center_velocity_estimates.get(idx, [0.0, 0.0, 0.0])
            position_correction = [0.0, 0.0, 0.0]
            velocity_correction = [0.0, 0.0, 0.0]
            valid_neighbors = 0

            for neighbor_idx in self.neighbor_indices.get(idx, []):
                if neighbor_idx not in self.center_estimates:
                    continue
                position_correction = add(
                    position_correction,
                    sub(self.center_estimates[neighbor_idx], center),
                )
                velocity_correction = add(
                    velocity_correction,
                    sub(
                        self.center_velocity_estimates.get(neighbor_idx, [0.0, 0.0, 0.0]),
                        center_velocity,
                    ),
                )
                valid_neighbors += 1

            if valid_neighbors > 0:
                position_correction = scale(position_correction, self.consensus_gain / valid_neighbors)
                velocity_correction = scale(
                    velocity_correction,
                    self.consensus_velocity_gain / valid_neighbors,
                )

            if idx == 0:
                position_correction = add(
                    position_correction,
                    scale(sub(leader_anchor, center), self.consensus_leader_gain),
                )
                velocity_correction = add(
                    velocity_correction,
                    scale(sub(leader_anchor_velocity, center_velocity), self.consensus_leader_gain),
                )

            next_center_velocities[idx] = add(center_velocity, scale(velocity_correction, dt))
            next_centers[idx] = add(
                add(center, scale(next_center_velocities[idx], dt)),
                scale(position_correction, dt),
            )

        self.center_estimates.update(next_centers)
        self.center_velocity_estimates.update(next_center_velocities)

    def leader_formation_anchor(
        self,
        leader_position: List[float],
        leader_velocity: List[float],
        leader_target: List[float],
        leader_v_des: List[float],
    ):
        if self.formation_reference_mode == "leader_actual":
            return leader_position, leader_velocity
        if self.formation_reference_mode == "leader_waypoint":
            return leader_target, leader_v_des
        return add(leader_position, scale(leader_v_des, self.formation_lookahead_s)), leader_v_des

    def parent_graph_reference(self, idx: int):
        parent_idx = self.parent_indices[idx]
        if parent_idx == idx or parent_idx not in self.states:
            parent_idx = 0

        relative_offset = sub(
            self.formation_offsets[idx],
            self.formation_offsets[parent_idx],
        )

        if self.formation_reference_mode == "leader_actual":
            parent_position, parent_velocity = self.states[parent_idx]
            return add(parent_position, relative_offset), parent_velocity

        if self.parent_graph_use_reference and parent_idx in self.reference_positions:
            parent_reference = self.reference_positions[parent_idx]
            parent_reference_velocity = self.reference_velocities.get(parent_idx, [0.0, 0.0, 0.0])
            return add(parent_reference, relative_offset), parent_reference_velocity

        parent_position, parent_velocity = self.states[parent_idx]
        if self.formation_reference_mode == "leader_lookahead":
            parent_reference_velocity = self.reference_velocities.get(parent_idx, parent_velocity)
            parent_anchor = add(
                parent_position,
                scale(parent_reference_velocity, self.formation_lookahead_s),
            )
            return add(parent_anchor, relative_offset), parent_reference_velocity

        return add(parent_position, relative_offset), parent_velocity

    def neighbor_graph_reference(
        self,
        idx: int,
        leader_position: List[float],
        leader_velocity: List[float],
        leader_anchor: List[float],
        leader_anchor_velocity: List[float],
    ):
        neighbor_positions = []
        neighbor_velocities = []
        for neighbor_idx in self.neighbor_indices.get(idx, []):
            if neighbor_idx not in self.states:
                continue
            neighbor_position, neighbor_velocity = self.states[neighbor_idx]
            relative_offset = sub(
                self.formation_offsets[idx],
                self.formation_offsets[neighbor_idx],
            )
            neighbor_positions.append(add(neighbor_position, relative_offset))
            neighbor_velocities.append(neighbor_velocity)

        if not neighbor_positions:
            return add(leader_position, self.formation_offsets[idx]), leader_velocity

        neighbor_desired_position = [
            sum(point[axis] for point in neighbor_positions) / len(neighbor_positions)
            for axis in range(3)
        ]
        neighbor_desired_velocity = [
            sum(vel[axis] for vel in neighbor_velocities) / len(neighbor_velocities)
            for axis in range(3)
        ]
        anchor = max(0.0, min(1.0, self.neighbor_leader_anchor_gain))
        leader_desired_position = add(leader_anchor, self.formation_offsets[idx])
        desired_position = add(
            scale(neighbor_desired_position, 1.0 - anchor),
            scale(leader_desired_position, anchor),
        )
        desired_velocity = add(
            scale(neighbor_desired_velocity, 1.0 - anchor),
            scale(leader_anchor_velocity, anchor),
        )
        return desired_position, desired_velocity

    def consensus_center_reference(self, idx: int):
        if idx not in self.center_estimates:
            position, velocity = self.states[idx]
            self.center_estimates[idx] = sub(position, self.formation_offsets[idx])
            self.center_velocity_estimates[idx] = velocity
        center = self.center_estimates[idx]
        center_velocity = self.center_velocity_estimates.get(idx, [0.0, 0.0, 0.0])
        return add(center, self.formation_offsets[idx]), center_velocity

    def update_leader_target(self, leader_position: List[float]):
        target = self.leader_waypoints[self.target_index]
        if norm(sub(target, leader_position)) < 0.35 and len(self.leader_waypoints) > 1:
            self.target_index = (self.target_index + 1) % len(self.leader_waypoints)
            target = self.leader_waypoints[self.target_index]

        direction = sub(target, leader_position)
        distance = norm(direction)
        if distance < 1.0e-6:
            return target, [0.0, 0.0, 0.0]
        speed = min(self.leader_speed, distance)
        return target, scale(direction, speed / distance)

    def repulsion_accel(self, idx: int):
        if idx not in self.states:
            return [0.0, 0.0, 0.0]
        position, _ = self.states[idx]
        total = [0.0, 0.0, 0.0]
        for other_idx, (other_position, _) in self.states.items():
            if other_idx == idx:
                continue
            delta = sub(position, other_position)
            distance = norm(delta)
            if distance < 1.0e-6 or distance >= self.repulsion_radius:
                continue
            strength = self.repulsion_gain * (1.0 / distance - 1.0 / self.repulsion_radius)
            total = add(total, scale(delta, strength / (distance * distance)))
        return total

    def obstacle_repulsion_accel(self, position: List[float]):
        total = [0.0, 0.0, 0.0]
        for obstacle in self.static_obstacles:
            center = obstacle[:3]
            radius = obstacle[3]
            delta = sub(position, center)
            center_distance = norm(delta)
            clearance = center_distance - radius
            if center_distance < 1.0e-6 or clearance >= self.obstacle_repulsion_radius:
                continue
            effective_clearance = max(0.05, clearance)
            strength = self.obstacle_repulsion_gain * (
                1.0 / effective_clearance - 1.0 / self.obstacle_repulsion_radius
            )
            total = add(total, scale(delta, strength / (center_distance * effective_clearance)))
        return total

    def dynamic_obstacle_repulsion_accel(self, position: List[float], velocity: List[float]):
        total = [0.0, 0.0, 0.0]
        for idx, (obstacle_position, obstacle_velocity) in self.dynamic_obstacles.items():
            radius = self.dynamic_obstacle_specs[idx][3]
            delta = sub(position, obstacle_position)
            relative_velocity = sub(velocity, obstacle_velocity)
            center_distance = norm(delta)
            clearance = center_distance - radius
            if center_distance < 1.0e-6 or clearance >= self.dynamic_obstacle_repulsion_radius:
                continue

            effective_clearance = max(0.05, clearance)
            closing_speed = -sum(delta[axis] * relative_velocity[axis] for axis in range(3)) / center_distance
            closing_boost = max(0.0, closing_speed) * self.dynamic_obstacle_closing_gain
            strength = (self.dynamic_obstacle_repulsion_gain + closing_boost) * (
                1.0 / effective_clearance - 1.0 / self.dynamic_obstacle_repulsion_radius
            )
            total = add(total, scale(delta, strength / (center_distance * effective_clearance)))
        return total

    def publish_accel(self, idx: int, accel: List[float]):
        msg = AccelStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        msg.accel.linear.x = accel[0]
        msg.accel.linear.y = accel[1]
        msg.accel.linear.z = accel[2]
        self.cmd_publishers[idx].publish(msg)

    def publish_reference(self, idx: int, position: List[float]):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        msg.pose.position.x = position[0]
        msg.pose.position.y = position[1]
        msg.pose.position.z = position[2]
        msg.pose.orientation.w = 1.0
        self.reference_publishers[idx].publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SwarmControllerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
