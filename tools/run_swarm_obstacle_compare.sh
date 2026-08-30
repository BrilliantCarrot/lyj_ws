#!/usr/bin/env bash
set -euo pipefail

RUN_SECONDS="${RUN_SECONDS:-45}"
OFF_GAIN="${OFF_GAIN:-0.0}"
ON_GAIN="${ON_GAIN:-1.1}"
OFF_CSV="${OFF_CSV:-eval/swarm/obstacle_avoidance_off.csv}"
ON_CSV="${ON_CSV:-eval/swarm/obstacle_avoidance_on.csv}"
SUMMARY_CSV="${SUMMARY_CSV:-eval/swarm/obstacle_avoidance_compare_summary.csv}"
PLOT_PATH="${PLOT_PATH:-images/swarm_obstacle_avoidance_compare.png}"

if ! command -v ros2 >/dev/null 2>&1; then
  echo "ros2 command not found. Source ROS2 and this workspace first."
  echo "  source /opt/ros/humble/setup.bash"
  echo "  source install/setup.bash"
  exit 1
fi

echo "[1/3] Running obstacle avoidance OFF for ${RUN_SECONDS}s"
timeout --preserve-status "${RUN_SECONDS}s" \
  ros2 launch uav_swarm swarm_demo.launch.py \
    obstacle_repulsion_gain:="${OFF_GAIN}" \
    metrics_csv:="${OFF_CSV}" || true

sleep 2

echo "[2/3] Running obstacle avoidance ON for ${RUN_SECONDS}s"
timeout --preserve-status "${RUN_SECONDS}s" \
  ros2 launch uav_swarm swarm_demo.launch.py \
    obstacle_repulsion_gain:="${ON_GAIN}" \
    metrics_csv:="${ON_CSV}" || true

echo "[3/3] Comparing metrics"
ros2 run uav_swarm swarm_compare_metrics \
  avoidance_off="${OFF_CSV}" \
  avoidance_on="${ON_CSV}" \
  --summary "${SUMMARY_CSV}" \
  --plot "${PLOT_PATH}"

echo "summary: ${SUMMARY_CSV}"
echo "plot: ${PLOT_PATH}"
