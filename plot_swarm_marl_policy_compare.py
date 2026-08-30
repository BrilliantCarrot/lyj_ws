#!/usr/bin/env python3
import argparse
import csv
import os
from typing import Dict, List

import matplotlib.pyplot as plt
import numpy as np


POLICY_LABELS = {
    "random": "Random",
    "goal_seeking": "Goal",
    "obstacle_aware_goal_seeking": "OA Goal",
    "formation": "Form",
    "formation_waypoint": "Form WP",
    "obstacle_aware_formation_waypoint": "OA Form WP",
    "formation_aware_formation_waypoint": "FA Form WP",
}


POLICY_COLORS = {
    "random": "#7f7f7f",
    "goal_seeking": "#d62728",
    "obstacle_aware_goal_seeking": "#2ca02c",
    "formation": "#ff7f0e",
    "formation_waypoint": "#9467bd",
    "obstacle_aware_formation_waypoint": "#1f77b4",
    "formation_aware_formation_waypoint": "#17becf",
}


def read_summary(path: str) -> List[Dict[str, float]]:
    rows = []
    with open(path, "r", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            converted = {"policy": row["policy"]}
            for key, value in row.items():
                if key == "policy":
                    continue
                converted[key] = float(value)
            rows.append(converted)
    order = [
        "random",
        "goal_seeking",
        "obstacle_aware_goal_seeking",
        "formation",
        "formation_waypoint",
        "obstacle_aware_formation_waypoint",
        "formation_aware_formation_waypoint",
    ]
    rank = {policy: idx for idx, policy in enumerate(order)}
    return sorted(rows, key=lambda row: rank.get(row["policy"], len(order)))


def bar_panel(ax, rows, field, title, ylabel, ylim=None, value_fmt="{:.2f}"):
    labels = [POLICY_LABELS.get(row["policy"], row["policy"]) for row in rows]
    values = [row[field] for row in rows]
    colors = [POLICY_COLORS.get(row["policy"], "#333333") for row in rows]
    x = np.arange(len(rows))
    bars = ax.bar(x, values, color=colors, edgecolor="#222222", linewidth=0.6)
    ax.set_title(title, fontsize=12, fontweight="bold")
    ax.set_ylabel(ylabel)
    ax.set_xticks(x)
    ax.set_xticklabels(labels, rotation=20, ha="right", fontsize=9)
    ax.grid(True, axis="y", alpha=0.28)
    if ylim is not None:
        ax.set_ylim(*ylim)
    for bar, value in zip(bars, values):
        y = bar.get_height()
        ax.text(
            bar.get_x() + bar.get_width() / 2.0,
            y + max(0.03 * max(values + [1.0]), 0.02),
            value_fmt.format(value),
            ha="center",
            va="bottom",
            fontsize=8,
        )


def main():
    parser = argparse.ArgumentParser(
        description="Plot MARL scripted policy comparison summary for the UAV swarm project."
    )
    parser.add_argument(
        "--summary",
        default="eval/swarm/marl_policy_compare_summary.csv",
        help="Input summary CSV from ros2 run uav_swarm swarm_marl_compare.",
    )
    parser.add_argument(
        "--output",
        default="images/swarm_marl_policy_compare.png",
        help="Output figure path.",
    )
    args = parser.parse_args()

    rows = read_summary(args.summary)
    if not rows:
        raise RuntimeError(f"No rows found in {args.summary}")

    os.makedirs(os.path.dirname(args.output), exist_ok=True)

    fig, axes = plt.subplots(2, 3, figsize=(19, 9))
    fig.suptitle(
        "Swarm MARL Baseline Policy Comparison",
        fontsize=18,
        fontweight="bold",
        y=0.98,
    )

    bar_panel(
        axes[0, 0],
        rows,
        "mean_success",
        "Mission Success Rate",
        "Success rate",
        ylim=(0.0, 1.15),
        value_fmt="{:.2f}",
    )
    bar_panel(
        axes[0, 1],
        rows,
        "mean_obstacle_collision_count",
        "Obstacle Collisions",
        "Mean count",
        value_fmt="{:.1f}",
    )
    bar_panel(
        axes[0, 2],
        rows,
        "mean_collision_count",
        "Agent-Agent Collisions",
        "Mean count",
        value_fmt="{:.1f}",
    )
    bar_panel(
        axes[1, 0],
        rows,
        "mean_final_formation_rmse_m",
        "Formation Error",
        "RMSE [m]",
        value_fmt="{:.2f}",
    )
    bar_panel(
        axes[1, 1],
        rows,
        "mean_final_mean_goal_distance_m",
        "Final Goal Error",
        "Mean distance [m]",
        value_fmt="{:.2f}",
    )
    bar_panel(
        axes[1, 2],
        rows,
        "mean_time_s",
        "Mission Time",
        "Time [s]",
        value_fmt="{:.1f}",
    )

    fig.text(
        0.5,
        0.02,
        "OA = obstacle-aware, FA = formation-aware. Obstacle-aware baselines use local detour targets, vertical clearance, and agent separation. "
        "Success requires mission completion without obstacle or agent collision.",
        ha="center",
        fontsize=10,
    )
    fig.tight_layout(rect=[0.02, 0.05, 0.98, 0.94])
    fig.savefig(args.output, dpi=180)
    print(f"[Swarm MARL Plot] wrote {args.output}")


if __name__ == "__main__":
    main()
