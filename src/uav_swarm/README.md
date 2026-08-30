# UAV Swarm MVP

This package is the first swarm-development layer for the UAV GNC workspace.
It is intentionally independent from PX4/Gazebo so that formation logic,
collision-avoidance behavior, and evaluation metrics can be debugged before
multi-vehicle SITL integration.

## Implemented

- 3D point-mass multi-agent simulation using `nav_msgs/Odometry`
- One leader and multiple followers
- Leader waypoint tracking
- Follower formation-offset tracking
- Leader-lookahead formation tracking mode for lower follower lag during turns
- Parent-graph formation topology for distributed leader-follower control
- Neighbor-graph formation topology using multiple relative-position edges
- Consensus-center formation topology using neighbor agreement on formation center
- Communication delay, dropout, and rate-limit relay on `/swarm/agent_i/odom_comm`
- Pairwise repulsive collision avoidance
- Static spherical obstacle avoidance using artificial potential-field repulsion
- Scripted dynamic obstacle publishing with relative-velocity-aware avoidance
- Formation RMSE, edge-error RMSE, minimum inter-agent distance, obstacle clearance, dynamic obstacle clearance, and collision sample logging
- Per-agent reference topics on `/swarm/agent_i/reference`
- RViz `MarkerArray` visualization on `/swarm/markers`

## Run

```bash
colcon build --symlink-install --packages-select uav_swarm
source install/setup.bash
ros2 launch uav_swarm swarm_demo.launch.py
```

Experiment presets can be launched by overriding arguments:

```bash
ros2 launch uav_swarm swarm_demo.launch.py \
  use_communication_layer:=false \
  formation_control_topology:=leader_relative \
  metrics_csv:=eval/swarm/baseline_leader_relative.csv
```

```bash
ros2 launch uav_swarm swarm_demo.launch.py \
  use_communication_layer:=true \
  formation_control_topology:=leader_relative \
  metrics_csv:=eval/swarm/comm_leader_relative.csv
```

```bash
ros2 launch uav_swarm swarm_demo.launch.py \
  use_communication_layer:=true \
  formation_control_topology:=parent_graph \
  metrics_csv:=eval/swarm/comm_parent_graph.csv
```

```bash
ros2 launch uav_swarm swarm_demo.launch.py \
  use_communication_layer:=true \
  formation_control_topology:=neighbor_graph \
  metrics_csv:=eval/swarm/comm_neighbor_graph.csv
```

```bash
ros2 launch uav_swarm swarm_demo.launch.py \
  use_communication_layer:=true \
  formation_control_topology:=consensus_center \
  metrics_csv:=eval/swarm/comm_consensus_center.csv
```

Useful checks:

```bash
ros2 topic echo /swarm/metrics
ros2 topic hz /swarm/agent_0/odom
```

In RViz, set the fixed frame to `world` and add a `MarkerArray` display for:

```text
/swarm/markers
```

The default RViz markers include:

- leader and follower positions
- velocity arrows
- agent path traces
- parent-child communication/control links
- static obstacle spheres
- dynamic obstacle spheres and velocity arrows

After running multiple cases, compare their CSV files:

```bash
ros2 run uav_swarm swarm_compare_metrics \
  baseline=eval/swarm/baseline_leader_relative.csv \
  comm_leader=eval/swarm/comm_leader_relative.csv \
  comm_parent=eval/swarm/comm_parent_graph.csv \
  comm_neighbor=eval/swarm/comm_neighbor_graph.csv \
  consensus=eval/swarm/comm_consensus_center.csv
```

This writes:

```text
eval/swarm/swarm_compare_summary.csv
images/swarm_compare_metrics.png
```

Obstacle avoidance can be compared with one command:

```bash
./tools/run_swarm_obstacle_compare.sh
```

By default this runs `avoidance_off` and `avoidance_on` for 45 seconds each,
then writes:

```text
eval/swarm/obstacle_avoidance_compare_summary.csv
images/swarm_obstacle_avoidance_compare.png
```

The run duration and gains can be overridden:

```bash
RUN_SECONDS=60 OFF_GAIN=0.0 ON_GAIN=1.4 ./tools/run_swarm_obstacle_compare.sh
```

Metrics are written to:

```text
eval/swarm/swarm_demo_metrics.csv
```

Communication constraints can be adjusted in `config/swarm_demo.yaml`:

```yaml
use_communication_layer: true
comm_rate_hz: 20.0
comm_delay_s: 0.15
comm_dropout_prob: 0.03
```

The default formation topology is:

```yaml
formation_parent_indices: [0, 0, 0, 1, 2]
formation_edges:
  [0, 1,
   0, 2,
   1, 3,
   2, 4,
   1, 2,
   3, 4]
formation_control_topology: "parent_graph"
parent_graph_use_reference: false
neighbor_leader_anchor_gain: 0.25
consensus_gain: 1.0
consensus_velocity_gain: 1.0
consensus_leader_gain: 1.0
```

This means agent 1 and 2 follow the leader, while agent 3 follows agent 1 and
agent 4 follows agent 2.

When `formation_control_topology` is set to `neighbor_graph`, each follower uses
the configured `formation_edges` instead. For every connected neighbor, it
computes the desired relative offset from `formation_offsets`, averages the
neighbor-implied references, and tracks that local graph reference. This is a
more general distributed-formation model than the single-parent tree.

`neighbor_leader_anchor_gain` weakly blends the neighbor-graph reference toward
the leader-relative formation reference. A small value reduces whole-swarm drift
from the leader formation while preserving the local edge-based behavior.

When `formation_control_topology` is set to `consensus_center`, each agent keeps
its own estimate of the formation center. Neighboring agents pull their center
estimates toward one another, while the leader estimate is anchored to the
mission trajectory. Each agent then tracks:

```text
agent_reference = local_center_estimate + formation_offset
```

Static obstacles are configured as flat `x, y, z, radius` tuples:

```yaml
static_obstacles:
  [3.8, 0.8, 1.8, 0.60,
   7.1, 3.0, 2.1, 0.65,
   4.2, 5.2, 2.1, 0.60,
   -0.8, 3.1, 1.8, 0.55]
obstacle_repulsion_gain: 1.1
obstacle_repulsion_radius_m: 2.4
obstacle_collision_margin_m: 0.25
```

The controller adds an obstacle-repulsion acceleration before applying the
global acceleration limit. The evaluation node records the minimum obstacle
clearance and counts obstacle-collision samples separately from inter-agent
collision samples.

`edge_error_rmse_m` measures whether the configured graph shape is preserved.
For each `formation_edges` pair, it compares the actual relative position
against the desired relative offset from `formation_offsets`. This is especially
useful when evaluating `neighbor_graph`, where local edge consistency can be
good even if leader-relative formation RMSE is large.

Dynamic obstacles are configured as flat `x, y, z, radius, vx, vy, vz, range`
tuples:

```yaml
dynamic_obstacles:
  [4.0, -1.2, 1.8, 0.45, 0.0, 0.8, 0.0, 3.0,
   9.0, 4.0, 2.2, 0.50, -0.7, 0.0, 0.0, 3.5]
dynamic_obstacle_repulsion_gain: 1.2
dynamic_obstacle_repulsion_radius_m: 2.8
dynamic_obstacle_closing_gain: 0.8
dynamic_obstacle_collision_margin_m: 0.25
```

`swarm_dynamic_obstacle_node` publishes each scripted moving obstacle as:

```text
/swarm/obstacle_i/odom
```

The controller only depends on obstacle odometry, not on how the obstacle was
created. Later, a perception or object-tracking node can publish the same
obstacle odometry interface and replace the scripted obstacle source.

## Runtime Task Allocation

The swarm now supports a simple mission layer on top of formation control.
`swarm_task_manager_node` owns a list of 3D task positions, assigns unassigned
tasks to idle agents with a greedy distance-based allocator, and publishes each
agent goal as:

```text
/swarm/agent_i/mission_goal
```

The controller listens to `/swarm/mission_command` and switches between:

```text
formation        # track the configured leader/follower formation
task_allocation  # track per-agent mission goals from the task manager
pause            # hold zero acceleration commands
resume
reset_tasks
```

The keyboard helper can be started in a separate terminal:

```bash
ros2 run uav_swarm swarm_keyboard_command_node
```

Keys:

```text
f: formation
t: task_allocation
p: pause
r: resume
x: reset_tasks
q: quit keyboard node
```

Task status is published on `/swarm/task_state`. RViz shows task cubes and
agent-task assignment lines in `/swarm/markers`. The metrics CSV includes task
completion ratio, completed task count, total task count, and mean completion
time.

The allocator can also run in auction mode:

```bash
ros2 launch uav_swarm swarm_demo.launch.py task_allocation_mode:=auction
```

In auction mode, `swarm_task_manager_node` publishes unassigned tasks on
`/swarm/task_announcement`. Each `swarm_agent_bidder_node` computes its own bid
and publishes it on `/swarm/task_bids`. The manager assigns each task to the
lowest-cost idle agent. The current MVP bid cost is:

```text
bid_cost = distance(agent, task) + load_penalty
```

This keeps the first version simple while leaving a clear path to add battery,
communication quality, sensor health, obstacle risk, and task priority into the
agent-local bid.

## Planned

- PX4 multi-vehicle SITL bridge
- MARL environment wrapper
