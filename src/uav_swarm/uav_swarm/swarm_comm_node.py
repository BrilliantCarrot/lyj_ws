import random
from collections import deque
from typing import Deque, Dict, Tuple

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


BufferedOdom = Tuple[int, Odometry]


class SwarmCommNode(Node):
    def __init__(self):
        super().__init__("swarm_comm_node")
        self.agent_count = int(self.declare_parameter("agent_count", 5).value)
        self.comm_rate_hz = float(self.declare_parameter("comm_rate_hz", 20.0).value)
        self.delay_s = float(self.declare_parameter("comm_delay_s", 0.15).value)
        self.dropout_prob = float(self.declare_parameter("comm_dropout_prob", 0.03).value)
        seed = int(self.declare_parameter("comm_seed", 7).value)
        self.rng = random.Random(seed)

        self.buffers: Dict[int, Deque[BufferedOdom]] = {
            i: deque() for i in range(self.agent_count)
        }
        self.dropped = 0
        self.forwarded = 0

        self.subs = [
            self.create_subscription(
                Odometry,
                f"/swarm/agent_{i}/odom",
                lambda msg, idx=i: self.odom_callback(msg, idx),
                30,
            )
            for i in range(self.agent_count)
        ]
        self.pubs = [
            self.create_publisher(Odometry, f"/swarm/agent_{i}/odom_comm", 10)
            for i in range(self.agent_count)
        ]
        self.timer = self.create_timer(1.0 / self.comm_rate_hz, self.publish_due_messages)
        self.get_logger().info(
            f"[Swarm Comm] agents={self.agent_count} rate={self.comm_rate_hz:.1f}Hz "
            f"delay={self.delay_s:.3f}s dropout={self.dropout_prob:.3f}"
        )

    def odom_callback(self, msg: Odometry, idx: int):
        now_ns = self.get_clock().now().nanoseconds
        release_ns = now_ns + int(max(0.0, self.delay_s) * 1.0e9)
        self.buffers[idx].append((release_ns, msg))

        max_buffer_len = max(10, int(self.comm_rate_hz * max(1.0, self.delay_s + 1.0)))
        while len(self.buffers[idx]) > max_buffer_len:
            self.buffers[idx].popleft()

    def publish_due_messages(self):
        now_ns = self.get_clock().now().nanoseconds
        for idx, buffer in self.buffers.items():
            latest_due = None
            while buffer and buffer[0][0] <= now_ns:
                latest_due = buffer.popleft()[1]
            if latest_due is None:
                continue

            if self.rng.random() < self.dropout_prob:
                self.dropped += 1
                continue

            self.pubs[idx].publish(latest_due)
            self.forwarded += 1

        total = self.forwarded + self.dropped
        if total > 0 and total % max(1, int(self.comm_rate_hz * 5.0)) == 0:
            drop_rate = self.dropped / total
            self.get_logger().info(
                f"[Swarm Comm] forwarded={self.forwarded} dropped={self.dropped} "
                f"observed_drop_rate={drop_rate:.3f}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = SwarmCommNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
