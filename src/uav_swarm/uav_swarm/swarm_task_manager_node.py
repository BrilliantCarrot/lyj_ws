import json
from typing import Dict, List, Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String

from .swarm_utils import norm, sub, vec3_list


class Task:
    def __init__(self, task_id: int, position: List[float]):
        self.task_id = task_id
        self.position = position
        self.status = "unassigned"
        self.assigned_agent: Optional[int] = None
        self.completed_time_s: Optional[float] = None


class SwarmTaskManagerNode(Node):
    def __init__(self):
        super().__init__("swarm_task_manager_node")
        self.agent_count = int(self.declare_parameter("agent_count", 5).value)
        self.world_frame = str(self.declare_parameter("world_frame", "world").value)
        self.task_rate_hz = float(self.declare_parameter("task_rate_hz", 10.0).value)
        self.allocation_mode = str(
            self.declare_parameter("task_allocation_mode", "greedy").value
        )
        self.completion_radius = float(
            self.declare_parameter("task_completion_radius_m", 0.45).value
        )
        self.auto_start_task_mode = bool(
            self.declare_parameter("auto_start_task_mode", False).value
        )
        initial_mode = str(self.declare_parameter("mission_mode", "formation").value)
        task_positions = vec3_list(
            self.declare_parameter(
                "task_positions",
                [
                    2.5, 1.5, 1.7,
                    5.5, 6.0, 2.0,
                    -2.5, 5.5, 1.8,
                    7.0, -1.0, 2.0,
                ],
            ).value
        )
        self.initial_task_positions = [task[:] for task in task_positions]
        self.tasks = [Task(idx, position) for idx, position in enumerate(task_positions)]
        self.mode = "task_allocation" if self.auto_start_task_mode else initial_mode
        self.paused = False
        self.agent_positions: Dict[int, List[float]] = {}
        self.agent_task: Dict[int, int] = {}
        self.agent_last_goal: Dict[int, List[float]] = {}
        self.latest_bids: Dict[tuple, Dict] = {}
        self.auction_seq = 0
        self.start_time = self.get_clock().now()

        self.odom_subs = [
            self.create_subscription(
                Odometry,
                f"/swarm/agent_{idx}/odom",
                lambda msg, agent_idx=idx: self.odom_callback(msg, agent_idx),
                10,
            )
            for idx in range(self.agent_count)
        ]
        self.command_sub = self.create_subscription(
            String,
            "/swarm/mission_command",
            self.command_callback,
            10,
        )
        self.bid_sub = self.create_subscription(
            String,
            "/swarm/task_bids",
            self.bid_callback,
            10,
        )
        self.goal_publishers = [
            self.create_publisher(PoseStamped, f"/swarm/agent_{idx}/mission_goal", 10)
            for idx in range(self.agent_count)
        ]
        self.announcement_pub = self.create_publisher(String, "/swarm/task_announcement", 10)
        self.task_state_pub = self.create_publisher(String, "/swarm/task_state", 10)
        self.mode_pub = self.create_publisher(String, "/swarm/mission_mode", 10)
        self.timer = self.create_timer(1.0 / self.task_rate_hz, self.step)
        self.get_logger().info(
            f"[Swarm Task Manager] tasks={len(self.tasks)} mode={self.mode} "
            f"allocation={self.allocation_mode} completion_radius={self.completion_radius:.2f}m"
        )

    def odom_callback(self, msg: Odometry, agent_idx: int):
        self.agent_positions[agent_idx] = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        ]

    def command_callback(self, msg: String):
        command = msg.data.strip().lower()
        if command in ("formation", "f"):
            self.mode = "formation"
            self.paused = False
        elif command in ("task", "task_allocation", "t"):
            self.mode = "task_allocation"
            self.paused = False
        elif command in ("pause", "p"):
            self.paused = True
        elif command in ("resume", "r"):
            self.paused = False
        elif command in ("reset", "reset_tasks", "x"):
            self.reset_tasks()
        else:
            self.get_logger().warn(f"[Swarm Task Manager] ignoring unknown command: {msg.data}")
            return
        self.get_logger().info(f"[Swarm Task Manager] command={command} mode={self.mode} paused={self.paused}")

    def bid_callback(self, msg: String):
        try:
            bid = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if int(bid.get("auction_seq", -1)) != self.auction_seq:
            return
        agent_id = int(bid.get("agent_id", -1))
        task_id = int(bid.get("task_id", -1))
        cost = float(bid.get("cost", float("inf")))
        if agent_id < 0 or agent_id >= self.agent_count:
            return
        if task_id < 0 or task_id >= len(self.tasks):
            return
        if cost == float("inf"):
            return
        self.latest_bids[(agent_id, task_id)] = bid

    def reset_tasks(self):
        self.tasks = [Task(idx, position[:]) for idx, position in enumerate(self.initial_task_positions)]
        self.agent_task.clear()
        self.agent_last_goal.clear()
        self.latest_bids.clear()
        self.auction_seq += 1
        self.start_time = self.get_clock().now()

    def step(self):
        if self.mode == "task_allocation" and not self.paused:
            self.update_completed_tasks()
            if self.allocation_mode == "auction":
                self.assign_tasks_from_bids()
                self.publish_task_announcement()
            else:
                self.assign_tasks_greedy()
            self.publish_goals()
        self.publish_status()

    def update_completed_tasks(self):
        now_s = (self.get_clock().now() - self.start_time).nanoseconds * 1.0e-9
        for task in self.tasks:
            if task.status != "assigned" or task.assigned_agent is None:
                continue
            agent_position = self.agent_positions.get(task.assigned_agent)
            if agent_position is None:
                continue
            if norm(sub(agent_position, task.position)) <= self.completion_radius:
                task.status = "completed"
                task.completed_time_s = now_s
                self.agent_task.pop(task.assigned_agent, None)

    def assign_tasks_greedy(self):
        idle_agents = [
            idx
            for idx in range(self.agent_count)
            if idx in self.agent_positions and idx not in self.agent_task
        ]
        unassigned_tasks = [task for task in self.tasks if task.status == "unassigned"]
        while idle_agents and unassigned_tasks:
            best_agent = None
            best_task = None
            best_cost = float("inf")
            for agent_idx in idle_agents:
                agent_position = self.agent_positions[agent_idx]
                for task in unassigned_tasks:
                    cost = norm(sub(agent_position, task.position))
                    if cost < best_cost:
                        best_cost = cost
                        best_agent = agent_idx
                        best_task = task
            if best_agent is None or best_task is None:
                break
            best_task.status = "assigned"
            best_task.assigned_agent = best_agent
            self.agent_task[best_agent] = best_task.task_id
            self.agent_last_goal[best_agent] = best_task.position[:]
            idle_agents.remove(best_agent)
            unassigned_tasks.remove(best_task)

    def assign_tasks_from_bids(self):
        idle_agents = {
            idx
            for idx in range(self.agent_count)
            if idx in self.agent_positions and idx not in self.agent_task
        }
        unassigned_task_ids = {
            task.task_id for task in self.tasks if task.status == "unassigned"
        }
        if not idle_agents or not unassigned_task_ids:
            return

        candidates = []
        for (agent_id, task_id), bid in self.latest_bids.items():
            if agent_id not in idle_agents or task_id not in unassigned_task_ids:
                continue
            candidates.append((float(bid["cost"]), agent_id, task_id, bid))
        candidates.sort(key=lambda item: item[0])
        if not candidates:
            return

        used_agents = set()
        used_tasks = set()
        for cost, agent_id, task_id, bid in candidates:
            if agent_id in used_agents or task_id in used_tasks:
                continue
            task = self.tasks[task_id]
            if task.status != "unassigned":
                continue
            task.status = "assigned"
            task.assigned_agent = agent_id
            self.agent_task[agent_id] = task_id
            self.agent_last_goal[agent_id] = task.position[:]
            used_agents.add(agent_id)
            used_tasks.add(task_id)
            self.get_logger().info(
                f"[Swarm Task Manager] auction assign task={task_id} agent={agent_id} cost={cost:.3f}"
            )
        self.latest_bids.clear()

    def publish_task_announcement(self):
        self.auction_seq += 1
        payload = {
            "auction_seq": self.auction_seq,
            "mode": self.mode,
            "paused": self.paused,
            "allocation_mode": self.allocation_mode,
            "assigned_agents": list(self.agent_task.keys()),
            "unassigned_tasks": [
                {
                    "task_id": task.task_id,
                    "position": task.position,
                }
                for task in self.tasks
                if task.status == "unassigned"
            ],
        }
        self.announcement_pub.publish(String(data=json.dumps(payload)))

    def publish_goals(self):
        for agent_idx, task_id in list(self.agent_task.items()):
            self.agent_last_goal[agent_idx] = self.tasks[task_id].position[:]
        for agent_idx, goal in self.agent_last_goal.items():
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.world_frame
            msg.pose.position.x = goal[0]
            msg.pose.position.y = goal[1]
            msg.pose.position.z = goal[2]
            msg.pose.orientation.w = 1.0
            self.goal_publishers[agent_idx].publish(msg)

    def publish_status(self):
        completed = sum(1 for task in self.tasks if task.status == "completed")
        completion_times = [
            task.completed_time_s for task in self.tasks if task.completed_time_s is not None
        ]
        mean_completion_time = (
            sum(completion_times) / len(completion_times) if completion_times else 0.0
        )
        payload = {
            "mode": self.mode,
            "paused": self.paused,
            "allocation_mode": self.allocation_mode,
            "auction_seq": self.auction_seq,
            "completed_task_count": completed,
            "total_task_count": len(self.tasks),
            "task_completion_ratio": completed / max(1, len(self.tasks)),
            "mean_task_completion_time_s": mean_completion_time,
            "tasks": [
                {
                    "task_id": task.task_id,
                    "position": task.position,
                    "status": task.status,
                    "assigned_agent": task.assigned_agent,
                    "completed_time_s": task.completed_time_s,
                }
                for task in self.tasks
            ],
        }
        self.task_state_pub.publish(String(data=json.dumps(payload)))
        self.mode_pub.publish(String(data=self.mode))


def main(args=None):
    rclpy.init(args=args)
    node = SwarmTaskManagerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
