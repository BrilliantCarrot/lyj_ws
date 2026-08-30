import argparse
import csv
from pathlib import Path
from statistics import mean
from typing import Dict, List


def read_metrics(path: Path) -> Dict[str, List[float]]:
    data = {
        "time_s": [],
        "formation_rmse_m": [],
        "instant_formation_error_m": [],
        "edge_error_rmse_m": [],
        "min_inter_agent_distance_m": [],
        "min_obstacle_clearance_m": [],
        "min_dynamic_obstacle_clearance_m": [],
        "collision_samples": [],
        "obstacle_collision_samples": [],
        "dynamic_obstacle_collision_samples": [],
        "task_completion_ratio": [],
        "completed_task_count": [],
        "total_task_count": [],
        "mean_task_completion_time_s": [],
    }
    with path.open(newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            for key in data:
                if key in row and row[key] != "":
                    data[key].append(float(row[key]))
    if not data["time_s"]:
        raise ValueError(f"no rows in {path}")
    t0 = data["time_s"][0]
    data["time_s"] = [t - t0 for t in data["time_s"]]
    return data


def summarize_case(label: str, data: Dict[str, List[float]]) -> Dict[str, float]:
    row = {
        "case": label,
        "duration_s": data["time_s"][-1],
        "final_formation_rmse_m": data["formation_rmse_m"][-1],
        "mean_instant_error_m": mean(data["instant_formation_error_m"]),
        "max_instant_error_m": max(data["instant_formation_error_m"]),
        "min_inter_agent_distance_m": min(data["min_inter_agent_distance_m"]),
        "final_collision_samples": data["collision_samples"][-1],
    }
    if data["edge_error_rmse_m"]:
        row["mean_edge_error_rmse_m"] = mean(data["edge_error_rmse_m"])
        row["final_edge_error_rmse_m"] = data["edge_error_rmse_m"][-1]
    if data["min_obstacle_clearance_m"]:
        row["min_obstacle_clearance_m"] = min(data["min_obstacle_clearance_m"])
    if data["obstacle_collision_samples"]:
        row["final_obstacle_collision_samples"] = data["obstacle_collision_samples"][-1]
    if data["min_dynamic_obstacle_clearance_m"]:
        row["min_dynamic_obstacle_clearance_m"] = min(data["min_dynamic_obstacle_clearance_m"])
    if data["dynamic_obstacle_collision_samples"]:
        row["final_dynamic_obstacle_collision_samples"] = data[
            "dynamic_obstacle_collision_samples"
        ][-1]
    if data["task_completion_ratio"]:
        row["final_task_completion_ratio"] = data["task_completion_ratio"][-1]
    if data["completed_task_count"]:
        row["final_completed_task_count"] = data["completed_task_count"][-1]
    if data["total_task_count"]:
        row["total_task_count"] = data["total_task_count"][-1]
    if data["mean_task_completion_time_s"]:
        row["mean_task_completion_time_s"] = data["mean_task_completion_time_s"][-1]
    return row


def write_summary(path: Path, rows: List[Dict[str, float]]):
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = [
        "case",
        "duration_s",
        "final_formation_rmse_m",
        "mean_instant_error_m",
        "max_instant_error_m",
        "mean_edge_error_rmse_m",
        "final_edge_error_rmse_m",
        "min_inter_agent_distance_m",
        "min_obstacle_clearance_m",
        "min_dynamic_obstacle_clearance_m",
        "final_collision_samples",
        "final_obstacle_collision_samples",
        "final_dynamic_obstacle_collision_samples",
        "final_task_completion_ratio",
        "final_completed_task_count",
        "total_task_count",
        "mean_task_completion_time_s",
    ]
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def plot_cases(output_path: Path, cases):
    import matplotlib.pyplot as plt

    output_path.parent.mkdir(parents=True, exist_ok=True)
    has_obstacles = any(data["min_obstacle_clearance_m"] for _, data in cases)
    has_dynamic_obstacles = any(data["min_dynamic_obstacle_clearance_m"] for _, data in cases)
    has_edge_error = any(data["edge_error_rmse_m"] for _, data in cases)
    has_tasks = any(data["task_completion_ratio"] for _, data in cases)
    row_count = (
        3
        + int(has_edge_error)
        + int(has_obstacles)
        + int(has_dynamic_obstacles)
        + int(has_tasks)
    )
    fig, axes = plt.subplots(row_count, 1, figsize=(11, 3.0 * row_count), sharex=True)
    fig.suptitle("Swarm Formation Comparison", fontsize=16, fontweight="bold")

    for label, data in cases:
        t = data["time_s"]
        axes[0].plot(t, data["formation_rmse_m"], label=label)
        axes[1].plot(t, data["instant_formation_error_m"], label=label)
        axis_idx = 2
        if has_edge_error and data["edge_error_rmse_m"]:
            axes[axis_idx].plot(t, data["edge_error_rmse_m"], label=label)
        if has_edge_error:
            axis_idx += 1
        axes[axis_idx].plot(t, data["min_inter_agent_distance_m"], label=label)
        axis_idx += 1
        if has_obstacles and data["min_obstacle_clearance_m"]:
            axes[axis_idx].plot(t, data["min_obstacle_clearance_m"], label=label)
        if has_obstacles:
            axis_idx += 1
        if has_dynamic_obstacles and data["min_dynamic_obstacle_clearance_m"]:
            axes[axis_idx].plot(t, data["min_dynamic_obstacle_clearance_m"], label=label)
        if has_dynamic_obstacles:
            axis_idx += 1
        if has_tasks and data["task_completion_ratio"]:
            axes[axis_idx].plot(t, data["task_completion_ratio"], label=label)

    axes[0].set_ylabel("Cumulative RMSE [m]")
    axes[1].set_ylabel("Instant Error [m]")
    axis_idx = 2
    if has_edge_error:
        axes[axis_idx].set_ylabel("Edge RMSE [m]")
        axis_idx += 1
    axes[axis_idx].set_ylabel("Min Distance [m]")
    axis_idx += 1
    if has_obstacles:
        axes[axis_idx].set_ylabel("Obstacle Clearance [m]")
        axis_idx += 1
    if has_dynamic_obstacles:
        axes[axis_idx].set_ylabel("Dynamic Obs Clearance [m]")
        axis_idx += 1
    if has_tasks:
        axes[axis_idx].set_ylabel("Task Completion")
        axes[axis_idx].set_ylim(-0.05, 1.05)
    axes[-1].set_xlabel("Time [s]")

    for ax in axes:
        ax.grid(True, alpha=0.35)
        ax.legend()

    fig.tight_layout()
    fig.savefig(output_path, dpi=180)


def parse_case(arg: str):
    if "=" in arg:
        label, path = arg.split("=", 1)
        return label.strip(), Path(path).expanduser()
    path = Path(arg).expanduser()
    return path.stem, path


def main():
    parser = argparse.ArgumentParser(
        description="Compare uav_swarm metrics CSV files."
    )
    parser.add_argument(
        "cases",
        nargs="+",
        help="Metrics CSV paths. Use label=/path/to/file.csv to set labels.",
    )
    parser.add_argument(
        "--summary",
        default="eval/swarm/swarm_compare_summary.csv",
        help="Output summary CSV path.",
    )
    parser.add_argument(
        "--plot",
        default="images/swarm_compare_metrics.png",
        help="Output comparison plot path.",
    )
    args = parser.parse_args()

    cases = []
    summary_rows = []
    for case_arg in args.cases:
        label, path = parse_case(case_arg)
        data = read_metrics(path)
        cases.append((label, data))
        summary_rows.append(summarize_case(label, data))

    summary_path = Path(args.summary).expanduser()
    plot_path = Path(args.plot).expanduser()
    write_summary(summary_path, summary_rows)
    plot_cases(plot_path, cases)

    print(f"summary: {summary_path}")
    print(f"plot: {plot_path}")
    for row in summary_rows:
        print(
            f"{row['case']}: final_rmse={row['final_formation_rmse_m']:.3f}m, "
            f"mean_instant={row['mean_instant_error_m']:.3f}m, "
            f"mean_edge_rmse={row.get('mean_edge_error_rmse_m', float('nan')):.3f}m, "
            f"min_dist={row['min_inter_agent_distance_m']:.3f}m, "
            f"collision_samples={int(row['final_collision_samples'])}, "
            f"min_obstacle_clearance={row.get('min_obstacle_clearance_m', float('nan')):.3f}m, "
            f"obstacle_collision_samples={int(row.get('final_obstacle_collision_samples', 0))}, "
            f"min_dynamic_obstacle_clearance={row.get('min_dynamic_obstacle_clearance_m', float('nan')):.3f}m, "
            f"dynamic_obstacle_collision_samples={int(row.get('final_dynamic_obstacle_collision_samples', 0))}, "
            f"task_completion={row.get('final_task_completion_ratio', float('nan')):.2f}"
        )


if __name__ == "__main__":
    main()
