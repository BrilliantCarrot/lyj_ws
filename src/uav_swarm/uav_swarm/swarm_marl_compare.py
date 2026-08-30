import argparse
import csv
import os
from typing import Dict, List

from ament_index_python.packages import get_package_share_directory

from .swarm_marl_env import SwarmMARLEnv


POLICIES = [
    "random",
    "goal_seeking",
    "obstacle_aware_goal_seeking",
    "formation",
    "formation_waypoint",
    "obstacle_aware_formation_waypoint",
    "formation_aware_formation_waypoint",
]


def load_ros_yaml_config(path: str) -> Dict:
    try:
        import yaml
    except ImportError:
        return {}
    if not path or not os.path.exists(path):
        return {}
    with open(path, "r") as f:
        data = yaml.safe_load(f) or {}
    return data.get("/**", {}).get("ros__parameters", {})


def run_episode(config: Dict, policy: str, seed_offset: int = 0) -> Dict:
    episode_config = dict(config)
    episode_config["marl_seed"] = int(episode_config.get("marl_seed", 11)) + seed_offset
    env = SwarmMARLEnv(episode_config)
    env.reset()
    reward_sum = 0.0
    min_agent_distance = float("inf")
    max_collision_count = 0
    max_obstacle_collision_count = 0
    terminated = False
    truncated = False
    info = env.info()

    while not terminated and not truncated:
        actions = env.scripted_action(policy)
        _, rewards, terminated, truncated, info = env.step(actions)
        reward_sum += sum(rewards.values()) / max(1, len(rewards))
        min_agent_distance = min(min_agent_distance, info["min_agent_distance_m"])
        max_collision_count = max(max_collision_count, int(info["collision_count"]))
        max_obstacle_collision_count = max(
            max_obstacle_collision_count,
            int(info["obstacle_collision_count"]),
        )

    steps = max(1, int(info["step"]))
    return {
        "policy": policy,
        "scenario_seed": int(info["scenario_seed"]),
        "randomized_scenario": int(bool(info["randomized_scenario"])),
        "success": int(bool(info["success"])),
        "terminated": int(terminated),
        "truncated": int(truncated),
        "steps": steps,
        "time_s": info["time_s"],
        "mean_reward": reward_sum / steps,
        "final_mean_goal_distance_m": info["mean_goal_distance_m"],
        "final_max_goal_distance_m": info["max_goal_distance_m"],
        "final_formation_rmse_m": info["formation_rmse_m"],
        "min_agent_distance_m": 0.0 if min_agent_distance == float("inf") else min_agent_distance,
        "collision_count": max_collision_count,
        "obstacle_collision_count": max_obstacle_collision_count,
        "all_goals_reached": int(bool(info["all_goals_reached"])),
        "waypoint_cycle_complete": int(bool(info["waypoint_cycle_complete"])),
        "out_of_bounds": int(bool(info["out_of_bounds"])),
    }


def summarize(rows: List[Dict]) -> List[Dict]:
    summaries = []
    policies = sorted(set(row["policy"] for row in rows))
    numeric_fields = [
        "success",
        "randomized_scenario",
        "steps",
        "time_s",
        "mean_reward",
        "final_mean_goal_distance_m",
        "final_max_goal_distance_m",
        "final_formation_rmse_m",
        "min_agent_distance_m",
        "collision_count",
        "obstacle_collision_count",
        "all_goals_reached",
        "waypoint_cycle_complete",
        "out_of_bounds",
    ]
    for policy in policies:
        policy_rows = [row for row in rows if row["policy"] == policy]
        summary = {"policy": policy, "episodes": len(policy_rows)}
        for field in numeric_fields:
            values = [float(row[field]) for row in policy_rows]
            summary[f"mean_{field}"] = sum(values) / max(1, len(values))
        summaries.append(summary)
    return summaries


def write_csv(path: str, rows: List[Dict]):
    if not rows:
        return
    directory = os.path.dirname(path)
    if directory:
        os.makedirs(directory, exist_ok=True)
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def parse_args():
    default_config = os.path.join(
        get_package_share_directory("uav_swarm"),
        "config",
        "swarm_marl.yaml",
    )
    parser = argparse.ArgumentParser(description="Compare scripted baseline policies in SwarmMARLEnv.")
    parser.add_argument("--config", default=default_config, help="Path to swarm_marl.yaml.")
    parser.add_argument(
        "--policies",
        default=",".join(POLICIES),
        help="Comma-separated policy list.",
    )
    parser.add_argument("--episodes", type=int, default=5, help="Episodes per policy.")
    parser.add_argument(
        "--output",
        default="eval/swarm/marl_policy_compare.csv",
        help="Per-episode CSV output path.",
    )
    parser.add_argument(
        "--summary-output",
        default="eval/swarm/marl_policy_compare_summary.csv",
        help="Per-policy summary CSV output path.",
    )
    parser.add_argument(
        "--max-episode-steps",
        type=int,
        default=None,
        help="Override max_episode_steps.",
    )
    parser.add_argument(
        "--randomize-scenarios",
        action="store_true",
        help="Randomize starts, goals, waypoints, and obstacles for each episode.",
    )
    parser.add_argument(
        "--initial-jitter",
        type=float,
        default=None,
        help="Override initial_position_jitter_m.",
    )
    parser.add_argument(
        "--goal-jitter",
        type=float,
        default=None,
        help="Override goal_position_jitter_m.",
    )
    parser.add_argument(
        "--obstacle-jitter",
        type=float,
        default=None,
        help="Override obstacle_position_jitter_m.",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    config = load_ros_yaml_config(args.config)
    if args.max_episode_steps is not None:
        config["max_episode_steps"] = args.max_episode_steps
    if args.randomize_scenarios:
        config["randomize_scenario"] = True
    if args.initial_jitter is not None:
        config["initial_position_jitter_m"] = args.initial_jitter
    if args.goal_jitter is not None:
        config["goal_position_jitter_m"] = args.goal_jitter
    if args.obstacle_jitter is not None:
        config["obstacle_position_jitter_m"] = args.obstacle_jitter

    policies = [policy.strip() for policy in args.policies.split(",") if policy.strip()]
    rows = []
    for policy_index, policy in enumerate(policies):
        for episode in range(args.episodes):
            row = run_episode(config, policy, seed_offset=policy_index * 1000 + episode)
            row["episode"] = episode
            rows.append(row)
            print(
                f"[MARL Compare] policy={policy} episode={episode} "
                f"seed={row['scenario_seed']} "
                f"success={row['success']} steps={row['steps']} "
                f"mean_reward={row['mean_reward']:.3f} "
                f"goal_dist={row['final_mean_goal_distance_m']:.3f}m "
                f"formation_rmse={row['final_formation_rmse_m']:.3f}m "
                f"agent_collisions={row['collision_count']} "
                f"obstacle_collisions={row['obstacle_collision_count']} "
                f"waypoint_done={row['waypoint_cycle_complete']}"
            )

    write_csv(args.output, rows)
    summary_rows = summarize(rows)
    write_csv(args.summary_output, summary_rows)
    print(f"[MARL Compare] wrote {args.output}")
    print(f"[MARL Compare] wrote {args.summary_output}")


if __name__ == "__main__":
    main()
