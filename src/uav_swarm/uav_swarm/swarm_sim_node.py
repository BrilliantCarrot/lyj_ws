from typing import Dict, List

import rclpy
from geometry_msgs.msg import AccelStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node

from .swarm_utils import clamp_norm, vec3_list, yaw_from_velocity, yaw_to_quaternion


class SwarmSimNode(Node):
    def __init__(self):
        super().__init__("swarm_sim_node")
        self.agent_count = int(self.declare_parameter("agent_count", 5).value)
        self.world_frame = str(self.declare_parameter("world_frame", "world").value)
        self.sim_rate_hz = float(self.declare_parameter("sim_rate_hz", 100.0).value)
        self.max_accel = float(self.declare_parameter("max_accel_mps2", 2.0).value)
        self.max_speed = float(self.declare_parameter("max_speed_mps", 2.0).value)
        self.min_altitude = float(self.declare_parameter("min_altitude_m", 0.1).value)

        initial_positions = self.declare_parameter(
            "initial_positions",
            [
                0.0, 0.0, 1.5,
                -1.0, 1.0, 1.5,
                -1.0, -1.0, 1.5,
                -2.0, 2.0, 1.5,
                -2.0, -2.0, 1.5,
            ],
        ).value
        self.positions: List[List[float]] = vec3_list(initial_positions)
        while len(self.positions) < self.agent_count:
            self.positions.append([0.0, 0.0, 1.5])
        self.positions = self.positions[: self.agent_count]

        self.velocities = [[0.0, 0.0, 0.0] for _ in range(self.agent_count)]
        self.accel_cmds = [[0.0, 0.0, 0.0] for _ in range(self.agent_count)]
        self.yaws = [0.0 for _ in range(self.agent_count)]

        self.odom_publishers = [
            self.create_publisher(Odometry, f"/swarm/agent_{i}/odom", 10)
            for i in range(self.agent_count)
        ]
        self.cmd_subs = [
            self.create_subscription(
                AccelStamped,
                f"/swarm/agent_{i}/accel_cmd",
                lambda msg, idx=i: self.accel_callback(msg, idx),
                10,
            )
            for i in range(self.agent_count)
        ]

        self.last_time = self.get_clock().now()
        self.timer = self.create_timer(1.0 / self.sim_rate_hz, self.step)
        self.get_logger().info(
            f"[Swarm Sim] agents={self.agent_count} rate={self.sim_rate_hz:.1f}Hz "
            f"max_accel={self.max_accel:.2f} max_speed={self.max_speed:.2f}"
        )

    def accel_callback(self, msg: AccelStamped, idx: int):
        self.accel_cmds[idx] = clamp_norm(
            [msg.accel.linear.x, msg.accel.linear.y, msg.accel.linear.z],
            self.max_accel,
        )

    def step(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1.0e-9
        self.last_time = now
        if dt <= 0.0 or dt > 0.2:
            dt = 1.0 / self.sim_rate_hz

        for i in range(self.agent_count):
            accel = self.accel_cmds[i]
            velocity = self.velocities[i]
            position = self.positions[i]

            velocity[0] += accel[0] * dt
            velocity[1] += accel[1] * dt
            velocity[2] += accel[2] * dt
            velocity[:] = clamp_norm(velocity, self.max_speed)

            position[0] += velocity[0] * dt
            position[1] += velocity[1] * dt
            position[2] += velocity[2] * dt
            if position[2] < self.min_altitude:
                position[2] = self.min_altitude
                velocity[2] = max(0.0, velocity[2])

            self.yaws[i] = yaw_from_velocity(velocity, self.yaws[i])
            self.publish_odom(i, now)

    def publish_odom(self, idx: int, stamp):
        msg = Odometry()
        msg.header.stamp = stamp.to_msg()
        msg.header.frame_id = self.world_frame
        msg.child_frame_id = f"agent_{idx}/base_link"

        p = self.positions[idx]
        v = self.velocities[idx]
        msg.pose.pose.position.x = p[0]
        msg.pose.pose.position.y = p[1]
        msg.pose.pose.position.z = p[2]
        qx, qy, qz, qw = yaw_to_quaternion(self.yaws[idx])
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.twist.twist.linear.x = v[0]
        msg.twist.twist.linear.y = v[1]
        msg.twist.twist.linear.z = v[2]
        self.odom_publishers[idx].publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SwarmSimNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
