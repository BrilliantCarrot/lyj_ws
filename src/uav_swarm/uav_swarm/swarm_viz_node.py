import json
from typing import Dict, List, Tuple

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from .swarm_utils import dynamic_obstacle_list, obstacle_list


State = Tuple[List[float], List[float]]


class SwarmVizNode(Node):
    def __init__(self):
        super().__init__("swarm_viz_node")
        self.agent_count = int(self.declare_parameter("agent_count", 5).value)
        self.world_frame = str(self.declare_parameter("world_frame", "world").value)
        self.viz_rate_hz = float(self.declare_parameter("viz_rate_hz", 20.0).value)
        self.safety_radius = float(self.declare_parameter("safety_radius_m", 0.7).value)
        self.formation_control_topology = str(
            self.declare_parameter("formation_control_topology", "parent_graph").value
        )
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
        self.states: Dict[int, State] = {}
        self.dynamic_obstacles: Dict[int, State] = {}
        self.tasks = []
        self.mission_mode = "formation"
        self.allocation_mode = "greedy"
        self.paths: Dict[int, List[List[float]]] = {i: [] for i in range(self.agent_count)}

        self.odom_subs = [
            self.create_subscription(
                Odometry,
                f"/swarm/agent_{i}/odom",
                lambda msg, idx=i: self.odom_callback(msg, idx),
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
        self.marker_pub = self.create_publisher(MarkerArray, "/swarm/markers", 10)
        self.timer = self.create_timer(1.0 / self.viz_rate_hz, self.publish_markers)
        self.get_logger().info(f"[Swarm Viz] markers=/swarm/markers frame={self.world_frame}")

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
        path = self.paths[idx]
        if not path or self.distance_sq(path[-1], position) > 0.0025:
            path.append(position)
            if len(path) > 500:
                del path[0]

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

    def task_state_callback(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        self.mission_mode = str(payload.get("mode", self.mission_mode))
        self.allocation_mode = str(payload.get("allocation_mode", self.allocation_mode))
        self.tasks = payload.get("tasks", [])

    def publish_markers(self):
        if not self.states:
            return
        stamp = self.get_clock().now().to_msg()
        markers = []
        for idx, (position, velocity) in self.states.items():
            markers.append(self.agent_marker(idx, position, stamp))
            markers.append(self.velocity_marker(idx, position, velocity, stamp))
            markers.append(self.path_marker(idx, stamp))
            if self.formation_control_topology not in ("neighbor_graph", "consensus_center"):
                parent_idx = self.parent_indices[idx]
                if idx > 0 and parent_idx in self.states:
                    markers.append(self.link_marker(idx, self.states[parent_idx][0], position, stamp))
        if self.formation_control_topology in ("neighbor_graph", "consensus_center"):
            markers.extend(self.neighbor_link_markers(stamp))
        for idx, obstacle in enumerate(self.static_obstacles):
            markers.append(self.obstacle_marker(idx, obstacle, stamp))
        for idx, (position, velocity) in self.dynamic_obstacles.items():
            radius = self.dynamic_obstacle_specs[idx][3]
            markers.append(self.dynamic_obstacle_marker(idx, position, radius, stamp))
            markers.append(self.dynamic_obstacle_velocity_marker(idx, position, velocity, stamp))
        for task in self.tasks:
            markers.append(self.task_marker(task, stamp))
            assignment_marker = self.task_assignment_marker(task, stamp)
            if assignment_marker is not None:
                markers.append(assignment_marker)
        markers.append(self.mode_marker(stamp))
        self.marker_pub.publish(MarkerArray(markers=markers))

    def base_marker(self, idx: int, marker_id: int, stamp) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.world_frame
        marker.header.stamp = stamp
        marker.ns = "swarm"
        marker.id = marker_id
        marker.action = Marker.ADD
        marker.lifetime.sec = 1
        return marker

    def agent_marker(self, idx: int, position: List[float], stamp) -> Marker:
        marker = self.base_marker(idx, idx, stamp)
        marker.type = Marker.SPHERE
        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = position[2]
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.safety_radius
        marker.scale.y = self.safety_radius
        marker.scale.z = 0.25
        if idx == 0:
            marker.color.r = 1.0
            marker.color.g = 0.25
            marker.color.b = 0.05
        else:
            marker.color.r = 0.1
            marker.color.g = 0.55
            marker.color.b = 1.0
        marker.color.a = 0.85
        return marker

    def velocity_marker(self, idx: int, position: List[float], velocity: List[float], stamp) -> Marker:
        marker = self.base_marker(idx, 100 + idx, stamp)
        marker.type = Marker.ARROW
        marker.scale.x = 0.04
        marker.scale.y = 0.08
        marker.scale.z = 0.12
        marker.color.r = 0.05
        marker.color.g = 0.05
        marker.color.b = 0.05
        marker.color.a = 0.9
        start = Point()
        start.x, start.y, start.z = position
        end = Point()
        end.x = position[0] + velocity[0] * 0.6
        end.y = position[1] + velocity[1] * 0.6
        end.z = position[2] + velocity[2] * 0.6
        marker.points.append(start)
        marker.points.append(end)
        return marker

    def path_marker(self, idx: int, stamp) -> Marker:
        marker = self.base_marker(idx, 200 + idx, stamp)
        marker.type = Marker.LINE_STRIP
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.035
        marker.color.r = 1.0 if idx == 0 else 0.0
        marker.color.g = 0.35 if idx == 0 else 0.7
        marker.color.b = 0.0 if idx == 0 else 1.0
        marker.color.a = 0.9
        for point in self.paths[idx]:
            p = Point()
            p.x, p.y, p.z = point
            marker.points.append(p)
        return marker

    def link_marker(self, idx: int, leader: List[float], follower: List[float], stamp) -> Marker:
        marker = self.base_marker(idx, 300 + idx, stamp)
        marker.type = Marker.LINE_LIST
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.02
        marker.color.r = 0.2
        marker.color.g = 0.8
        marker.color.b = 0.2
        marker.color.a = 0.55
        a = Point()
        a.x, a.y, a.z = leader
        b = Point()
        b.x, b.y, b.z = follower
        marker.points.append(a)
        marker.points.append(b)
        return marker

    def neighbor_link_markers(self, stamp):
        markers = []
        if len(self.formation_edges) % 2 != 0:
            edges = self.formation_edges[:-1]
        else:
            edges = self.formation_edges
        for edge_idx in range(0, len(edges), 2):
            a_idx = edges[edge_idx]
            b_idx = edges[edge_idx + 1]
            if a_idx not in self.states or b_idx not in self.states:
                continue
            marker_id = 350 + edge_idx // 2
            markers.append(
                self.link_marker(
                    marker_id,
                    self.states[a_idx][0],
                    self.states[b_idx][0],
                    stamp,
                )
            )
        return markers

    def obstacle_marker(self, idx: int, obstacle: List[float], stamp) -> Marker:
        marker = self.base_marker(idx, 400 + idx, stamp)
        marker.ns = "swarm_obstacles"
        marker.type = Marker.SPHERE
        marker.pose.position.x = obstacle[0]
        marker.pose.position.y = obstacle[1]
        marker.pose.position.z = obstacle[2]
        marker.pose.orientation.w = 1.0
        diameter = obstacle[3] * 2.0
        marker.scale.x = diameter
        marker.scale.y = diameter
        marker.scale.z = diameter
        marker.color.r = 0.9
        marker.color.g = 0.1
        marker.color.b = 0.05
        marker.color.a = 0.35
        return marker

    def dynamic_obstacle_marker(self, idx: int, position: List[float], radius: float, stamp) -> Marker:
        marker = self.base_marker(idx, 500 + idx, stamp)
        marker.ns = "swarm_dynamic_obstacles"
        marker.type = Marker.SPHERE
        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = position[2]
        marker.pose.orientation.w = 1.0
        marker.scale.x = radius * 2.0
        marker.scale.y = radius * 2.0
        marker.scale.z = radius * 2.0
        marker.color.r = 0.65
        marker.color.g = 0.1
        marker.color.b = 0.95
        marker.color.a = 0.65
        return marker

    def dynamic_obstacle_velocity_marker(
        self,
        idx: int,
        position: List[float],
        velocity: List[float],
        stamp,
    ) -> Marker:
        marker = self.base_marker(idx, 550 + idx, stamp)
        marker.ns = "swarm_dynamic_obstacle_velocity"
        marker.type = Marker.ARROW
        marker.scale.x = 0.05
        marker.scale.y = 0.10
        marker.scale.z = 0.14
        marker.color.r = 0.45
        marker.color.g = 0.0
        marker.color.b = 0.75
        marker.color.a = 0.9
        start = Point()
        start.x, start.y, start.z = position
        end = Point()
        end.x = position[0] + velocity[0] * 0.8
        end.y = position[1] + velocity[1] * 0.8
        end.z = position[2] + velocity[2] * 0.8
        marker.points.append(start)
        marker.points.append(end)
        return marker

    def task_marker(self, task, stamp) -> Marker:
        task_id = int(task.get("task_id", 0))
        position = task.get("position", [0.0, 0.0, 0.0])
        status = task.get("status", "unassigned")
        marker = self.base_marker(task_id, 700 + task_id, stamp)
        marker.ns = "swarm_tasks"
        marker.type = Marker.CUBE
        marker.pose.position.x = float(position[0])
        marker.pose.position.y = float(position[1])
        marker.pose.position.z = float(position[2])
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.35
        marker.scale.y = 0.35
        marker.scale.z = 0.35
        if status == "completed":
            marker.color.r = 0.1
            marker.color.g = 0.9
            marker.color.b = 0.2
        elif status == "assigned":
            marker.color.r = 1.0
            marker.color.g = 0.7
            marker.color.b = 0.05
        else:
            marker.color.r = 0.95
            marker.color.g = 0.95
            marker.color.b = 0.1
        marker.color.a = 0.9
        return marker

    def task_assignment_marker(self, task, stamp):
        agent_idx = task.get("assigned_agent")
        if agent_idx is None or int(agent_idx) not in self.states:
            return None
        task_id = int(task.get("task_id", 0))
        task_position = task.get("position", [0.0, 0.0, 0.0])
        agent_position = self.states[int(agent_idx)][0]
        marker = self.base_marker(task_id, 800 + task_id, stamp)
        marker.ns = "swarm_task_assignments"
        marker.type = Marker.LINE_LIST
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.035
        marker.color.r = 1.0
        marker.color.g = 0.75
        marker.color.b = 0.05
        marker.color.a = 0.75
        a = Point()
        a.x, a.y, a.z = agent_position
        b = Point()
        b.x = float(task_position[0])
        b.y = float(task_position[1])
        b.z = float(task_position[2])
        marker.points.append(a)
        marker.points.append(b)
        return marker

    def mode_marker(self, stamp) -> Marker:
        marker = self.base_marker(0, 900, stamp)
        marker.ns = "swarm_mode"
        marker.type = Marker.TEXT_VIEW_FACING
        marker.pose.position.x = -5.0
        marker.pose.position.y = -3.5
        marker.pose.position.z = 3.0
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.35
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 0.95
        marker.text = f"mode: {self.mission_mode} | allocation: {self.allocation_mode}"
        return marker

    def distance_sq(self, a: List[float], b: List[float]) -> float:
        dx = a[0] - b[0]
        dy = a[1] - b[1]
        dz = a[2] - b[2]
        return dx * dx + dy * dy + dz * dz


def main(args=None):
    rclpy.init(args=args)
    node = SwarmVizNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
