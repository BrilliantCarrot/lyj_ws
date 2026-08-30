import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SwarmKeyboardCommandNode(Node):
    def __init__(self):
        super().__init__("swarm_keyboard_command_node")
        self.publisher = self.create_publisher(String, "/swarm/mission_command", 10)
        self.timer = self.create_timer(0.05, self.poll_key)
        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        self.get_logger().info(
            "[Swarm Keyboard] f=formation, t=task_allocation, p=pause, r=resume, x=reset_tasks, q=quit"
        )

    def poll_key(self):
        if not select.select([sys.stdin], [], [], 0.0)[0]:
            return
        key = sys.stdin.read(1).lower()
        mapping = {
            "f": "formation",
            "t": "task_allocation",
            "p": "pause",
            "r": "resume",
            "x": "reset_tasks",
        }
        if key == "q":
            raise KeyboardInterrupt
        if key not in mapping:
            return
        command = mapping[key]
        self.publisher.publish(String(data=command))
        self.get_logger().info(f"[Swarm Keyboard] command={command}")

    def destroy_node(self):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SwarmKeyboardCommandNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
