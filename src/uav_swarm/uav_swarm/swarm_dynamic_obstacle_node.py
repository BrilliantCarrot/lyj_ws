from typing import List

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node

from .swarm_utils import dynamic_obstacle_list, norm, scale, yaw_from_velocity, yaw_to_quaternion


class SwarmDynamicObstacleNode(Node):
    def __init__(self):
        super().__init__("swarm_dynamic_obstacle_node")
        self.world_frame = str(self.declare_parameter("world_frame", "world").value)
        self.rate_hz = float(self.declare_parameter("dynamic_obstacle_rate_hz", 30.0).value)
        self.specs = dynamic_obstacle_list(
            self.declare_parameter(
                "dynamic_obstacles",
                [
                    4.0, -1.2, 1.8, 0.45, 0.0, 0.8, 0.0, 3.0,
                    9.0, 4.0, 2.2, 0.50, -0.7, 0.0, 0.0, 3.5,
                ],
            ).value
        )
        self.positions = [spec[:3] for spec in self.specs]
        self.start_positions = [spec[:3] for spec in self.specs]
        self.velocities = [spec[4:7] for spec in self.specs]
        self.travel_ranges = [max(0.0, spec[7]) for spec in self.specs]
        self.yaws = [0.0 for _ in self.specs]
        self.odom_publishers = [
            self.create_publisher(Odometry, f"/swarm/obstacle_{idx}/odom", 10)
            for idx in range(len(self.specs))
        ]
        self.last_time = self.get_clock().now()
        self.timer = self.create_timer(1.0 / self.rate_hz, self.step)
        self.get_logger().info(
            f"[Swarm Dynamic Obstacles] count={len(self.specs)} rate={self.rate_hz:.1f}Hz"
        )

    def step(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1.0e-9
        self.last_time = now
        if dt <= 0.0 or dt > 0.2:
            dt = 1.0 / self.rate_hz

        for idx in range(len(self.specs)):
            velocity = self.velocities[idx]
            position = self.positions[idx]
            for axis in range(3):
                position[axis] += velocity[axis] * dt

            travel_range = self.travel_ranges[idx]
            if travel_range > 0.0:
                displacement = [
                    position[axis] - self.start_positions[idx][axis]
                    for axis in range(3)
                ]
                direction = self.specs[idx][4:7]
                direction_norm = norm(direction)
                if direction_norm > 1.0e-6:
                    unit_direction = scale(direction, 1.0 / direction_norm)
                    signed_distance = sum(displacement[axis] * unit_direction[axis] for axis in range(3))
                    if abs(signed_distance) > travel_range:
                        for axis in range(3):
                            velocity[axis] *= -1.0
                            position[axis] = (
                                self.start_positions[idx][axis]
                                + unit_direction[axis] * travel_range * (1.0 if signed_distance > 0.0 else -1.0)
                            )

            self.yaws[idx] = yaw_from_velocity(velocity, self.yaws[idx])
            self.publish_odom(idx, now)

    def publish_odom(self, idx: int, stamp):
        msg = Odometry()
        msg.header.stamp = stamp.to_msg()
        msg.header.frame_id = self.world_frame
        msg.child_frame_id = f"obstacle_{idx}/base_link"
        position = self.positions[idx]
        velocity = self.velocities[idx]
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


def main(args=None):
    rclpy.init(args=args)
    node = SwarmDynamicObstacleNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
