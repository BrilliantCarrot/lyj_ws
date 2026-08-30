import json
from typing import List, Optional

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String

from .swarm_utils import norm, sub


class SwarmAgentBidderNode(Node):
    def __init__(self):
        super().__init__("swarm_agent_bidder_node")
        self.agent_id = int(self.declare_parameter("agent_id", 0).value)
        self.load_penalty = float(self.declare_parameter("bid_load_penalty", 2.0).value)
        self.position: Optional[List[float]] = None
        self.assigned_task_count = 0

        self.odom_sub = self.create_subscription(
            Odometry,
            f"/swarm/agent_{self.agent_id}/odom",
            self.odom_callback,
            10,
        )
        self.announcement_sub = self.create_subscription(
            String,
            "/swarm/task_announcement",
            self.announcement_callback,
            10,
        )
        self.bid_pub = self.create_publisher(String, "/swarm/task_bids", 10)
        self.get_logger().info(
            f"[Swarm Agent Bidder] agent={self.agent_id} load_penalty={self.load_penalty:.2f}"
        )

    def odom_callback(self, msg: Odometry):
        self.position = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        ]

    def announcement_callback(self, msg: String):
        if self.position is None:
            return
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if payload.get("mode") != "task_allocation" or payload.get("paused", False):
            return

        assigned_agents = payload.get("assigned_agents", [])
        self.assigned_task_count = 1 if self.agent_id in assigned_agents else 0
        if self.assigned_task_count > 0:
            return

        for task in payload.get("unassigned_tasks", []):
            task_id = int(task["task_id"])
            task_position = [float(v) for v in task["position"]]
            distance_cost = norm(sub(self.position, task_position))
            cost = distance_cost + self.load_penalty * self.assigned_task_count
            bid = {
                "auction_seq": int(payload.get("auction_seq", 0)),
                "agent_id": self.agent_id,
                "task_id": task_id,
                "cost": cost,
                "distance_cost": distance_cost,
                "load_cost": self.load_penalty * self.assigned_task_count,
            }
            self.bid_pub.publish(String(data=json.dumps(bid)))


def main(args=None):
    rclpy.init(args=args)
    node = SwarmAgentBidderNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
