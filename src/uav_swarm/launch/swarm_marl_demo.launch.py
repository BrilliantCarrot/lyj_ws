from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("uav_swarm"),
        "config",
        "swarm_marl.yaml",
    )

    policy = LaunchConfiguration("policy")
    metrics_csv = LaunchConfiguration("metrics_csv")
    max_episode_steps = LaunchConfiguration("max_episode_steps")
    auto_reset = LaunchConfiguration("auto_reset")

    overrides = {
        "policy": ParameterValue(policy, value_type=str),
        "marl_metrics_csv": ParameterValue(metrics_csv, value_type=str),
        "max_episode_steps": ParameterValue(max_episode_steps, value_type=int),
        "auto_reset": ParameterValue(auto_reset, value_type=bool),
    }
    common = [config, overrides]

    return LaunchDescription([
        DeclareLaunchArgument(
            "policy",
            default_value="goal_seeking",
            description="Scripted baseline policy: random, goal_seeking, formation, or formation_waypoint.",
        ),
        DeclareLaunchArgument(
            "metrics_csv",
            default_value="eval/swarm/marl_rollout_metrics.csv",
            description="CSV output path for MARL rollout metrics.",
        ),
        DeclareLaunchArgument(
            "max_episode_steps",
            default_value="1200",
            description="Maximum steps per MARL episode.",
        ),
        DeclareLaunchArgument(
            "auto_reset",
            default_value="true",
            description="Automatically reset after termination or truncation.",
        ),
        Node(
            package="uav_swarm",
            executable="swarm_marl_rollout_node",
            name="swarm_marl_rollout_node",
            output="screen",
            parameters=common,
        ),
        Node(
            package="uav_swarm",
            executable="swarm_viz_node",
            name="swarm_viz_node",
            output="screen",
            parameters=common,
        ),
    ])
