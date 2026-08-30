#!/usr/bin/env python3
import argparse
import math
import os
import sys
from typing import Dict, List

import matplotlib.pyplot as plt


WORKSPACE_SRC = os.path.join(os.path.dirname(os.path.abspath(__file__)), "src", "uav_swarm")
if WORKSPACE_SRC not in sys.path:
    sys.path.insert(0, WORKSPACE_SRC)

from uav_swarm.swarm_marl_env import SwarmMARLEnv  # noqa: E402


POLICIES = [
    "formation_waypoint",
    "obstacle_aware_formation_waypoint",
    "formation_aware_formation_waypoint",
]

POLICY_LABELS = {
    "goal_seeking": "Goal",
    "obstacle_aware_goal_seeking": "OA Goal",
    "formation": "Form",
    "formation_waypoint": "Form WP",
    "obstacle_aware_formation_waypoint": "OA Form WP",
    "formation_aware_formation_waypoint": "FA Form WP",
}


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


def run_policy(config: Dict, policy: str, seed_offset: int = 0) -> Dict:
    episode_config = dict(config)
    episode_config["marl_seed"] = int(episode_config.get("marl_seed", 11)) + seed_offset
    env = SwarmMARLEnv(episode_config)
    env.reset()
    histories: Dict[int, List[List[float]]] = {
        idx: [env.positions[idx][:]] for idx in range(env.agent_count)
    }
    terminated = False
    truncated = False
    info = env.info()

    while not terminated and not truncated:
        actions = env.scripted_action(policy)
        _, _, terminated, truncated, info = env.step(actions)
        for idx in range(env.agent_count):
            histories[idx].append(env.positions[idx][:])

    return {
        "policy": policy,
        "env": env,
        "histories": histories,
        "info": info,
        "terminated": terminated,
        "truncated": truncated,
    }


def axis_limits(results: List[Dict]):
    xs = []
    ys = []
    for result in results:
        env = result["env"]
        for history in result["histories"].values():
            xs.extend(p[0] for p in history)
            ys.extend(p[1] for p in history)
        xs.extend(goal[0] for goal in env.final_goals)
        ys.extend(goal[1] for goal in env.final_goals)
        xs.extend(obstacle[0] for obstacle in env.static_obstacles)
        ys.extend(obstacle[1] for obstacle in env.static_obstacles)
    if not xs or not ys:
        return (-1.0, 1.0), (-1.0, 1.0)
    margin = 1.8
    return (min(xs) - margin, max(xs) + margin), (min(ys) - margin, max(ys) + margin)


def plot_result(ax, result: Dict, xlim, ylim):
    env = result["env"]
    policy = result["policy"]
    info = result["info"]
    histories = result["histories"]

    colors = ["#ff7f0e", "#1f77b4", "#2ca02c", "#9467bd", "#17becf", "#8c564b"]
    for idx, history in histories.items():
        xs = [p[0] for p in history]
        ys = [p[1] for p in history]
        color = colors[idx % len(colors)]
        label = "leader" if idx == 0 else f"agent {idx}"
        linewidth = 2.2 if idx == 0 else 1.4
        ax.plot(xs, ys, color=color, linewidth=linewidth, label=label)
        ax.scatter(xs[0], ys[0], color=color, marker="o", s=30)
        ax.scatter(xs[-1], ys[-1], color=color, marker=">", s=45)

    for obstacle in env.static_obstacles:
        circle = plt.Circle(
            (obstacle[0], obstacle[1]),
            obstacle[3] + env.obstacle_margin,
            color="#7f1d1d",
            alpha=0.45,
        )
        ax.add_patch(circle)

    for idx, goal in enumerate(env.final_goals):
        ax.scatter(goal[0], goal[1], marker="*", s=90, color="#facc15", edgecolor="#444444")
        ax.text(goal[0] + 0.08, goal[1] + 0.08, f"g{idx}", fontsize=8)

    if policy in ("formation_waypoint", "obstacle_aware_formation_waypoint", "formation_aware_formation_waypoint"):
        wx = [p[0] for p in env.leader_waypoints]
        wy = [p[1] for p in env.leader_waypoints]
        ax.plot(wx, wy, "--", color="#111827", linewidth=1.1, alpha=0.7, label="leader waypoints")

    title = POLICY_LABELS.get(policy, policy)
    status = "success" if info["success"] else "failed"
    ax.set_title(
        f"{title} | {status}, obs_col={info['obstacle_collision_count']}, "
        f"form={info['formation_rmse_m']:.2f}m, t={info['time_s']:.1f}s",
        fontsize=10,
        fontweight="bold",
    )
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_xlim(*xlim)
    ax.set_ylim(*ylim)
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.35)
    ax.legend(loc="upper right", fontsize=7)


def main():
    parser = argparse.ArgumentParser(description="Plot representative MARL swarm policy trajectories.")
    parser.add_argument("--config", default="src/uav_swarm/config/swarm_marl.yaml")
    parser.add_argument(
        "--policies",
        default=",".join(POLICIES),
        help="Comma-separated policy list.",
    )
    parser.add_argument("--seed-offset", type=int, default=0)
    parser.add_argument("--randomize-scenario", action="store_true")
    parser.add_argument("--max-episode-steps", type=int, default=None)
    parser.add_argument("--output", default="images/swarm_marl_trajectories.png")
    args = parser.parse_args()

    config = load_ros_yaml_config(args.config)
    if args.randomize_scenario:
        config["randomize_scenario"] = True
    if args.max_episode_steps is not None:
        config["max_episode_steps"] = args.max_episode_steps

    policies = [policy.strip() for policy in args.policies.split(",") if policy.strip()]
    results = [
        run_policy(config, policy, seed_offset=args.seed_offset + idx * 1000)
        for idx, policy in enumerate(policies)
    ]

    cols = min(2, len(results))
    rows = int(math.ceil(len(results) / max(1, cols)))
    fig, axes = plt.subplots(rows, cols, figsize=(8.5 * cols, 7.0 * rows), squeeze=False)
    fig.suptitle("Swarm MARL Representative XY Trajectories", fontsize=16, fontweight="bold")
    xlim, ylim = axis_limits(results)

    for ax, result in zip(axes.flat, results):
        plot_result(ax, result, xlim, ylim)
    for ax in list(axes.flat)[len(results):]:
        ax.axis("off")

    fig.text(
        0.5,
        0.02,
        "Circles show inflated static obstacles. Dots are starts, triangles are final positions, stars are per-agent goals.",
        ha="center",
        fontsize=10,
    )
    os.makedirs(os.path.dirname(args.output), exist_ok=True)
    fig.tight_layout(rect=[0.02, 0.04, 0.98, 0.94])
    fig.savefig(args.output, dpi=180)
    print(f"[Swarm MARL Trajectory Plot] wrote {args.output}")
    for result in results:
        info = result["info"]
        print(
            f"{result['policy']}: success={int(info['success'])} "
            f"time={info['time_s']:.2f}s formation_rmse={info['formation_rmse_m']:.3f}m "
            f"goal_dist={info['mean_goal_distance_m']:.3f}m "
            f"obstacle_collisions={info['obstacle_collision_count']}"
        )


if __name__ == "__main__":
    main()
