from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("uav_swarm"),
        "config",
        "swarm_demo.yaml",
    )

    use_communication_layer = LaunchConfiguration("use_communication_layer")
    formation_control_topology = LaunchConfiguration("formation_control_topology")
    parent_graph_use_reference = LaunchConfiguration("parent_graph_use_reference")
    neighbor_leader_anchor_gain = LaunchConfiguration("neighbor_leader_anchor_gain")
    consensus_gain = LaunchConfiguration("consensus_gain")
    consensus_velocity_gain = LaunchConfiguration("consensus_velocity_gain")
    consensus_leader_gain = LaunchConfiguration("consensus_leader_gain")
    comm_delay_s = LaunchConfiguration("comm_delay_s")
    comm_dropout_prob = LaunchConfiguration("comm_dropout_prob")
    obstacle_repulsion_gain = LaunchConfiguration("obstacle_repulsion_gain")
    obstacle_repulsion_radius_m = LaunchConfiguration("obstacle_repulsion_radius_m")
    enable_task_allocation = LaunchConfiguration("enable_task_allocation")
    mission_mode = LaunchConfiguration("mission_mode")
    auto_start_task_mode = LaunchConfiguration("auto_start_task_mode")
    task_allocation_mode = LaunchConfiguration("task_allocation_mode")
    metrics_csv = LaunchConfiguration("metrics_csv")

    overrides = {
        "use_communication_layer": ParameterValue(use_communication_layer, value_type=bool),
        "formation_control_topology": ParameterValue(formation_control_topology, value_type=str),
        "parent_graph_use_reference": ParameterValue(parent_graph_use_reference, value_type=bool),
        "neighbor_leader_anchor_gain": ParameterValue(
            neighbor_leader_anchor_gain,
            value_type=float,
        ),
        "consensus_gain": ParameterValue(consensus_gain, value_type=float),
        "consensus_velocity_gain": ParameterValue(consensus_velocity_gain, value_type=float),
        "consensus_leader_gain": ParameterValue(consensus_leader_gain, value_type=float),
        "comm_delay_s": ParameterValue(comm_delay_s, value_type=float),
        "comm_dropout_prob": ParameterValue(comm_dropout_prob, value_type=float),
        "obstacle_repulsion_gain": ParameterValue(obstacle_repulsion_gain, value_type=float),
        "obstacle_repulsion_radius_m": ParameterValue(
            obstacle_repulsion_radius_m,
            value_type=float,
        ),
        "mission_mode": ParameterValue(mission_mode, value_type=str),
        "auto_start_task_mode": ParameterValue(auto_start_task_mode, value_type=bool),
        "task_allocation_mode": ParameterValue(task_allocation_mode, value_type=str),
        "metrics_csv": ParameterValue(metrics_csv, value_type=str),
    }

    common = [config, overrides]

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_communication_layer",
            default_value="true",
            description="Use delayed/dropout odom_comm topics for swarm control.",
        ),
        DeclareLaunchArgument(
            "formation_control_topology",
            default_value="parent_graph",
            description="Formation topology: leader_relative, parent_graph, neighbor_graph, or consensus_center.",
        ),
        DeclareLaunchArgument(
            "parent_graph_use_reference",
            default_value="true",
            description="Use parent reference propagation instead of parent state.",
        ),
        DeclareLaunchArgument(
            "neighbor_leader_anchor_gain",
            default_value="0.25",
            description="Blend factor from neighbor graph reference toward leader-relative reference.",
        ),
        DeclareLaunchArgument(
            "consensus_gain",
            default_value="1.0",
            description="Position consensus gain for consensus_center topology.",
        ),
        DeclareLaunchArgument(
            "consensus_velocity_gain",
            default_value="1.0",
            description="Velocity consensus gain for consensus_center topology.",
        ),
        DeclareLaunchArgument(
            "consensus_leader_gain",
            default_value="1.0",
            description="Leader anchor gain for consensus_center topology.",
        ),
        DeclareLaunchArgument(
            "comm_delay_s",
            default_value="0.15",
            description="Communication delay in seconds.",
        ),
        DeclareLaunchArgument(
            "comm_dropout_prob",
            default_value="0.03",
            description="Message dropout probability in the communication relay.",
        ),
        DeclareLaunchArgument(
            "obstacle_repulsion_gain",
            default_value="1.1",
            description="Static obstacle repulsion gain.",
        ),
        DeclareLaunchArgument(
            "obstacle_repulsion_radius_m",
            default_value="2.2",
            description="Static obstacle repulsion activation radius.",
        ),
        DeclareLaunchArgument(
            "metrics_csv",
            default_value="eval/swarm/swarm_demo_metrics.csv",
            description="CSV output path for swarm metrics.",
        ),
        DeclareLaunchArgument(
            "enable_task_allocation",
            default_value="false",
            description="Start task manager and agent bidder nodes.",
        ),
        DeclareLaunchArgument(
            "mission_mode",
            default_value="formation",
            description="Initial mission mode: formation or task_allocation.",
        ),
        DeclareLaunchArgument(
            "auto_start_task_mode",
            default_value="false",
            description="Start task manager in task_allocation mode.",
        ),
        DeclareLaunchArgument(
            "task_allocation_mode",
            default_value="greedy",
            description="Task allocation mode: greedy or auction.",
        ),
        Node(
            package="uav_swarm",
            executable="swarm_sim_node",
            name="swarm_sim_node",
            output="screen",
            parameters=common,
        ),
        Node(
            package="uav_swarm",
            executable="swarm_controller_node",
            name="swarm_controller_node",
            output="screen",
            parameters=common,
        ),
        Node(
            package="uav_swarm",
            executable="swarm_comm_node",
            name="swarm_comm_node",
            output="screen",
            parameters=common,
        ),
        Node(
            package="uav_swarm",
            executable="swarm_dynamic_obstacle_node",
            name="swarm_dynamic_obstacle_node",
            output="screen",
            parameters=common,
        ),
        Node(
            package="uav_swarm",
            executable="swarm_task_manager_node",
            name="swarm_task_manager_node",
            output="screen",
            parameters=common,
            condition=IfCondition(enable_task_allocation),
        ),
        Node(
            package="uav_swarm",
            executable="swarm_agent_bidder_node",
            name="swarm_agent_0_bidder_node",
            output="screen",
            parameters=[config, overrides, {"agent_id": 0}],
            condition=IfCondition(enable_task_allocation),
        ),
        Node(
            package="uav_swarm",
            executable="swarm_agent_bidder_node",
            name="swarm_agent_1_bidder_node",
            output="screen",
            parameters=[config, overrides, {"agent_id": 1}],
            condition=IfCondition(enable_task_allocation),
        ),
        Node(
            package="uav_swarm",
            executable="swarm_agent_bidder_node",
            name="swarm_agent_2_bidder_node",
            output="screen",
            parameters=[config, overrides, {"agent_id": 2}],
            condition=IfCondition(enable_task_allocation),
        ),
        Node(
            package="uav_swarm",
            executable="swarm_agent_bidder_node",
            name="swarm_agent_3_bidder_node",
            output="screen",
            parameters=[config, overrides, {"agent_id": 3}],
            condition=IfCondition(enable_task_allocation),
        ),
        Node(
            package="uav_swarm",
            executable="swarm_agent_bidder_node",
            name="swarm_agent_4_bidder_node",
            output="screen",
            parameters=[config, overrides, {"agent_id": 4}],
            condition=IfCondition(enable_task_allocation),
        ),
        Node(
            package="uav_swarm",
            executable="swarm_eval_node",
            name="swarm_eval_node",
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
