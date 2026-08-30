import math
import random
from typing import Dict, List, Tuple

from .swarm_utils import add, clamp_norm, norm, obstacle_list, scale, sub, vec3_list


Vector = List[float]
Observation = Dict[int, List[float]]
Action = Dict[int, Vector]


class SwarmMARLEnv:
    """Small Gymnasium-style multi-agent swarm environment.

    The class is intentionally ROS-free so it can later be wrapped by
    Gymnasium, PettingZoo, RLlib, or a ROS/Gazebo-backed simulator.
    """

    def __init__(self, config: Dict):
        self.agent_count = int(config.get("agent_count", 5))
        self.dt = 1.0 / float(config.get("env_rate_hz", 30.0))
        self.max_steps = int(config.get("max_episode_steps", 1200))
        self.max_accel = float(config.get("max_accel_mps2", 2.0))
        self.max_speed = float(config.get("max_speed_mps", 2.0))
        self.min_altitude = float(config.get("min_altitude_m", 0.2))
        self.boundary = float(config.get("boundary_m", 12.0))
        self.goal_radius = float(config.get("goal_radius_m", 0.45))
        self.collision_radius = float(config.get("collision_radius_m", 0.35))
        self.obstacle_margin = float(config.get("obstacle_collision_margin_m", 0.25))
        self.terminate_on_agent_collision = bool(config.get("terminate_on_agent_collision", True))
        self.terminate_on_obstacle_collision = bool(config.get("terminate_on_obstacle_collision", True))
        self.avoidance_activation_margin = float(
            config.get("obstacle_avoidance_activation_margin_m", 2.6)
        )
        self.avoidance_gain = float(config.get("obstacle_avoidance_gain", 3.0))
        self.avoidance_tangent_gain = float(config.get("obstacle_avoidance_tangent_gain", 1.1))
        self.agent_separation_activation = float(config.get("agent_separation_activation_m", 1.0))
        self.agent_separation_gain = float(config.get("agent_separation_gain", 1.2))
        self.detour_clearance = float(config.get("obstacle_detour_clearance_m", 1.4))
        self.detour_altitude_margin = float(config.get("obstacle_detour_altitude_margin_m", 0.8))
        self.formation_footprint_margin = float(config.get("formation_footprint_margin_m", 0.35))
        self.formation_footprint_scale = float(config.get("formation_footprint_scale", 0.60))
        self.randomize_scenario = bool(config.get("randomize_scenario", False))
        self.initial_position_jitter = float(config.get("initial_position_jitter_m", 0.25))
        self.goal_position_jitter = float(config.get("goal_position_jitter_m", 0.45))
        self.obstacle_position_jitter = float(config.get("obstacle_position_jitter_m", 0.35))
        self.obstacle_radius_jitter = float(config.get("obstacle_radius_jitter_m", 0.10))

        self.goal_weight = float(config.get("reward_goal_weight", 1.0))
        self.formation_weight = float(config.get("reward_formation_weight", 0.35))
        self.control_weight = float(config.get("reward_control_weight", 0.02))
        self.collision_penalty = float(config.get("reward_collision_penalty", 25.0))
        self.obstacle_penalty = float(config.get("reward_obstacle_penalty", 20.0))
        self.success_bonus = float(config.get("reward_success_bonus", 10.0))
        self.boundary_penalty = float(config.get("reward_boundary_penalty", 15.0))

        initial_positions = config.get(
            "initial_positions",
            [
                0.0, 0.0, 1.5,
                -1.0, 1.0, 1.5,
                -1.0, -1.0, 1.5,
                -2.0, 2.0, 1.5,
                -2.0, -2.0, 1.5,
            ],
        )
        self.initial_positions = vec3_list(initial_positions)
        while len(self.initial_positions) < self.agent_count:
            self.initial_positions.append([0.0, 0.0, 1.5])
        self.initial_positions = self.initial_positions[: self.agent_count]

        self.formation_offsets = vec3_list(
            config.get(
                "formation_offsets",
                [
                    0.0, 0.0, 0.0,
                    -1.2, 1.0, 0.0,
                    -1.2, -1.0, 0.0,
                    -2.4, 1.8, 0.0,
                    -2.4, -1.8, 0.0,
                ],
            )
        )
        while len(self.formation_offsets) < self.agent_count:
            self.formation_offsets.append([0.0, 0.0, 0.0])
        self.formation_offsets = self.formation_offsets[: self.agent_count]

        self.goals = vec3_list(
            config.get(
                "marl_goals",
                [
                    6.0, 6.0, 2.0,
                    4.8, 7.0, 2.0,
                    4.8, 5.0, 2.0,
                    3.6, 7.8, 2.0,
                    3.6, 4.2, 2.0,
                ],
            )
        )
        while len(self.goals) < self.agent_count:
            self.goals.append(self.goals[-1][:] if self.goals else [6.0, 6.0, 2.0])
        self.goals = self.goals[: self.agent_count]
        self.final_goals = [goal[:] for goal in self.goals]

        self.leader_waypoints = vec3_list(
            config.get(
                "marl_leader_waypoints",
                [
                    0.0, 0.0, 1.5,
                    6.0, 0.0, 2.0,
                    6.0, 5.0, 2.0,
                    0.5, 5.5, 2.0,
                    0.0, 0.0, 1.5,
                ],
            )
        )
        if not self.leader_waypoints:
            self.leader_waypoints = [[0.0, 0.0, 1.5]]
        self.waypoint_index = 1 if len(self.leader_waypoints) > 1 else 0
        self.waypoint_radius = float(config.get("waypoint_radius_m", 0.45))

        self.static_obstacles = obstacle_list(
            config.get(
                "static_obstacles",
                [
                    3.8, 0.8, 1.8, 0.60,
                    7.1, 5.0, 3.1, 0.65,
                    -0.8, 3.1, 1.8, 0.55,
                ],
            )
        )
        self.base_initial_positions = [position[:] for position in self.initial_positions]
        self.base_goals = [goal[:] for goal in self.final_goals]
        self.base_leader_waypoints = [waypoint[:] for waypoint in self.leader_waypoints]
        self.base_static_obstacles = [obstacle[:] for obstacle in self.static_obstacles]

        seed = config.get("marl_seed", 11)
        self.base_seed = int(seed)
        self.rng = random.Random(self.base_seed)
        self.scenario_seed = self.base_seed
        self.positions: List[Vector] = []
        self.velocities: List[Vector] = []
        self.last_actions: List[Vector] = []
        self.step_count = 0
        self.active_policy = "goal_seeking"
        self.waypoint_cycle_complete = False
        self.formation_aware_waypoints_built = False

    def reset(self) -> Tuple[Observation, Dict]:
        self.apply_scenario_randomization()
        self.positions = [p[:] for p in self.initial_positions]
        self.velocities = [[0.0, 0.0, 0.0] for _ in range(self.agent_count)]
        self.last_actions = [[0.0, 0.0, 0.0] for _ in range(self.agent_count)]
        self.step_count = 0
        self.waypoint_index = 1 if len(self.leader_waypoints) > 1 else 0
        self.waypoint_cycle_complete = False
        self.formation_aware_waypoints_built = False
        self.goals = [goal[:] for goal in self.final_goals]
        return self.observation(), self.info()

    def apply_scenario_randomization(self):
        self.scenario_seed = self.base_seed
        self.initial_positions = [position[:] for position in self.base_initial_positions]
        self.final_goals = [goal[:] for goal in self.base_goals]
        self.leader_waypoints = [waypoint[:] for waypoint in self.base_leader_waypoints]
        self.static_obstacles = [obstacle[:] for obstacle in self.base_static_obstacles]

        if not self.randomize_scenario:
            return

        self.scenario_seed = self.rng.randint(0, 2**31 - 1)
        scenario_rng = random.Random(self.scenario_seed)
        self.initial_positions = [
            self.jitter_vector(position, self.initial_position_jitter, scenario_rng)
            for position in self.initial_positions
        ]
        self.final_goals = [
            self.jitter_vector(goal, self.goal_position_jitter, scenario_rng)
            for goal in self.final_goals
        ]
        self.leader_waypoints = [
            self.jitter_vector(waypoint, self.goal_position_jitter, scenario_rng)
            for waypoint in self.leader_waypoints
        ]
        if self.leader_waypoints:
            self.leader_waypoints[0] = self.base_leader_waypoints[0][:]
            self.leader_waypoints[-1] = self.base_leader_waypoints[-1][:]
        randomized_obstacles = []
        for obstacle in self.static_obstacles:
            moved = self.jitter_vector(obstacle[:3], self.obstacle_position_jitter, scenario_rng)
            radius = max(
                0.20,
                obstacle[3] + scenario_rng.uniform(-self.obstacle_radius_jitter, self.obstacle_radius_jitter),
            )
            randomized_obstacles.append(moved + [radius])
        self.static_obstacles = randomized_obstacles

    def jitter_vector(self, value: Vector, limit: float, rng: random.Random) -> Vector:
        if limit <= 0.0:
            return value[:]
        return [
            value[0] + rng.uniform(-limit, limit),
            value[1] + rng.uniform(-limit, limit),
            max(self.min_altitude, value[2] + rng.uniform(-0.35 * limit, 0.35 * limit)),
        ]

    def step(self, actions: Action):
        self.step_count += 1
        clamped_actions = {}
        for idx in range(self.agent_count):
            action = actions.get(idx, [0.0, 0.0, 0.0])
            clamped_actions[idx] = clamp_norm([float(v) for v in action], self.max_accel)
            self.last_actions[idx] = clamped_actions[idx][:]

        for idx in range(self.agent_count):
            velocity = self.velocities[idx]
            position = self.positions[idx]
            accel = clamped_actions[idx]
            velocity[0] += accel[0] * self.dt
            velocity[1] += accel[1] * self.dt
            velocity[2] += accel[2] * self.dt
            velocity[:] = clamp_norm(velocity, self.max_speed)
            position[0] += velocity[0] * self.dt
            position[1] += velocity[1] * self.dt
            position[2] += velocity[2] * self.dt
            if position[2] < self.min_altitude:
                position[2] = self.min_altitude
                velocity[2] = max(0.0, velocity[2])

        rewards = self.rewards(clamped_actions)
        info = self.info()
        agent_collision_failure = (
            self.terminate_on_agent_collision and info["collision_count"] > 0
        )
        obstacle_collision_failure = (
            self.terminate_on_obstacle_collision and info["obstacle_collision_count"] > 0
        )
        terminated = info["success"] or agent_collision_failure or obstacle_collision_failure
        truncated = self.step_count >= self.max_steps or info["out_of_bounds"]
        return self.observation(), rewards, terminated, truncated, info

    def observation(self) -> Observation:
        obs = {}
        for idx in range(self.agent_count):
            nearest_obstacle = self.nearest_obstacle_relative(idx)
            nearest_agent = self.nearest_agent_relative(idx)
            obs[idx] = (
                self.positions[idx][:]
                + self.velocities[idx][:]
                + sub(self.goals[idx], self.positions[idx])
                + nearest_obstacle
                + nearest_agent
            )
        return obs

    def rewards(self, actions: Action) -> Dict[int, float]:
        rewards = {}
        leader_position = self.positions[0]
        for idx in range(self.agent_count):
            goal_distance = norm(sub(self.goals[idx], self.positions[idx]))
            formation_error = norm(
                sub(
                    self.positions[idx],
                    add(leader_position, self.formation_offsets[idx]),
                )
            )
            control_effort = norm(actions[idx])
            reward = -self.goal_weight * goal_distance
            reward -= self.formation_weight * formation_error
            reward -= self.control_weight * control_effort
            if goal_distance <= self.goal_radius:
                reward += self.success_bonus
            if self.agent_collision(idx):
                reward -= self.collision_penalty
            if self.obstacle_collision(idx):
                reward -= self.obstacle_penalty
            if self.agent_out_of_bounds(idx):
                reward -= self.boundary_penalty
            rewards[idx] = reward
        return rewards

    def info(self) -> Dict:
        goal_distances = [
            norm(sub(self.goals[idx], self.positions[idx]))
            for idx in range(self.agent_count)
        ]
        formation_errors = self.formation_errors()
        collision_count = sum(1 for idx in range(self.agent_count) if self.agent_collision(idx))
        obstacle_collision_count = sum(
            1 for idx in range(self.agent_count) if self.obstacle_collision(idx)
        )
        all_goals_reached = all(d <= self.goal_radius for d in goal_distances)
        out_of_bounds = any(self.agent_out_of_bounds(i) for i in range(self.agent_count))
        waypoint_policy = self.active_policy in (
            "formation_waypoint",
            "waypoint_formation",
            "obstacle_aware_formation_waypoint",
            "safe_formation_waypoint",
            "formation_aware_formation_waypoint",
            "footprint_formation_waypoint",
        )
        collision_failure = (
            (self.terminate_on_agent_collision and collision_count > 0)
            or (self.terminate_on_obstacle_collision and obstacle_collision_count > 0)
        )
        success = self.waypoint_cycle_complete if waypoint_policy else all_goals_reached
        success = success and not collision_failure and not out_of_bounds
        return {
            "step": self.step_count,
            "time_s": self.step_count * self.dt,
            "scenario_seed": self.scenario_seed,
            "randomized_scenario": self.randomize_scenario,
            "mean_goal_distance_m": sum(goal_distances) / max(1, len(goal_distances)),
            "max_goal_distance_m": max(goal_distances) if goal_distances else 0.0,
            "formation_rmse_m": math.sqrt(
                sum(v * v for v in formation_errors) / max(1, len(formation_errors))
            ),
            "min_agent_distance_m": self.min_agent_distance(),
            "collision_count": collision_count,
            "obstacle_collision_count": obstacle_collision_count,
            "all_goals_reached": all_goals_reached,
            "waypoint_cycle_complete": self.waypoint_cycle_complete,
            "success": success,
            "out_of_bounds": out_of_bounds,
        }

    def scripted_action(self, policy: str) -> Action:
        policy = policy.lower()
        self.active_policy = policy
        if policy == "random":
            return {
                idx: [
                    self.rng.uniform(-self.max_accel, self.max_accel),
                    self.rng.uniform(-self.max_accel, self.max_accel),
                    self.rng.uniform(-0.5 * self.max_accel, 0.5 * self.max_accel),
                ]
                for idx in range(self.agent_count)
            }
        if policy in ("obstacle_aware_goal_seeking", "safe_goal_seeking"):
            return self.goal_seeking_action(obstacle_aware=True)
        if policy in ("formation_aware_formation_waypoint", "footprint_formation_waypoint"):
            return self.formation_waypoint_action(obstacle_aware=True, formation_aware=True)
        if policy in ("obstacle_aware_formation_waypoint", "safe_formation_waypoint"):
            return self.formation_waypoint_action(obstacle_aware=True)
        if policy in ("formation_waypoint", "waypoint_formation"):
            return self.formation_waypoint_action()
        if policy in ("obstacle_aware_formation", "safe_formation"):
            return self.formation_policy_action(obstacle_aware=True)
        if policy == "formation":
            return self.formation_policy_action()
        return self.goal_seeking_action()

    def goal_seeking_action(self, obstacle_aware: bool = False) -> Action:
        actions = {}
        for idx in range(self.agent_count):
            target = self.local_detour_target(idx, self.goals[idx]) if obstacle_aware else self.goals[idx]
            pos_error = sub(target, self.positions[idx])
            vel_error = scale(self.velocities[idx], -1.0)
            accel = add(scale(pos_error, 0.8), scale(vel_error, 1.2))
            if obstacle_aware:
                accel = add(accel, self.obstacle_avoidance(idx))
                accel = add(accel, self.agent_separation(idx))
            else:
                accel = add(accel, self.obstacle_repulsion(idx))
            actions[idx] = clamp_norm(accel, self.max_accel)
        return actions

    def formation_waypoint_action(
        self,
        obstacle_aware: bool = False,
        formation_aware: bool = False,
    ) -> Action:
        if formation_aware and not self.formation_aware_waypoints_built:
            self.leader_waypoints = self.build_formation_aware_waypoints(self.leader_waypoints)
            self.waypoint_index = 1 if len(self.leader_waypoints) > 1 else 0
            self.waypoint_cycle_complete = False
            self.formation_aware_waypoints_built = True
        leader_target = self.current_leader_waypoint()
        for idx in range(self.agent_count):
            self.goals[idx] = add(leader_target, self.formation_offsets[idx])
        return self.formation_policy_action(
            obstacle_aware=obstacle_aware,
            use_follower_detour=True,
        )

    def current_leader_waypoint(self) -> Vector:
        if self.waypoint_cycle_complete:
            return self.leader_waypoints[-1]
        target = self.leader_waypoints[self.waypoint_index]
        if norm(sub(target, self.positions[0])) <= self.waypoint_radius:
            if self.waypoint_index < len(self.leader_waypoints) - 1:
                self.waypoint_index += 1
            else:
                self.waypoint_cycle_complete = True
            target = self.leader_waypoints[self.waypoint_index]
        return target

    def formation_policy_action(
        self,
        obstacle_aware: bool = False,
        use_follower_detour: bool = True,
    ) -> Action:
        actions = {}
        leader_target = self.local_detour_target(0, self.goals[0]) if obstacle_aware else self.goals[0]
        leader_error = sub(leader_target, self.positions[0])
        leader_accel = add(scale(leader_error, 0.8), scale(self.velocities[0], -1.2))
        leader_avoidance = self.obstacle_avoidance(0) if obstacle_aware else self.obstacle_repulsion(0)
        actions[0] = clamp_norm(add(leader_accel, leader_avoidance), self.max_accel)
        for idx in range(1, self.agent_count):
            desired = add(self.positions[0], self.formation_offsets[idx])
            if obstacle_aware and use_follower_detour:
                desired = self.local_detour_target(idx, desired)
            pos_error = sub(desired, self.positions[idx])
            vel_error = sub(self.velocities[0], self.velocities[idx])
            accel = add(scale(pos_error, 1.0), scale(vel_error, 1.4))
            if obstacle_aware:
                accel = add(accel, self.obstacle_avoidance(idx))
                accel = add(accel, self.agent_separation(idx))
            else:
                accel = add(accel, self.obstacle_repulsion(idx))
            actions[idx] = clamp_norm(accel, self.max_accel)
        return actions

    def formation_footprint_radius(self) -> float:
        radius = 0.0
        for offset in self.formation_offsets:
            radius = max(radius, norm([offset[0], offset[1], 0.0]))
        return radius * self.formation_footprint_scale + self.collision_radius + self.formation_footprint_margin

    def formation_errors(self) -> List[float]:
        if not self.positions:
            return []
        leader_position = self.positions[0]
        return [
            norm(sub(self.positions[idx], add(leader_position, self.formation_offsets[idx])))
            for idx in range(1, self.agent_count)
        ]

    def nearest_agent_relative(self, idx: int) -> Vector:
        nearest = [0.0, 0.0, 0.0]
        nearest_dist = float("inf")
        for other in range(self.agent_count):
            if other == idx:
                continue
            rel = sub(self.positions[other], self.positions[idx])
            dist = norm(rel)
            if dist < nearest_dist:
                nearest = rel
                nearest_dist = dist
        return nearest

    def nearest_obstacle_relative(self, idx: int) -> Vector:
        nearest = [0.0, 0.0, 0.0]
        nearest_dist = float("inf")
        for obstacle in self.static_obstacles:
            rel = sub(obstacle[:3], self.positions[idx])
            dist = norm(rel) - obstacle[3]
            if dist < nearest_dist:
                nearest = rel
                nearest_dist = dist
        return nearest

    def obstacle_repulsion(self, idx: int) -> Vector:
        accel = [0.0, 0.0, 0.0]
        position = self.positions[idx]
        for obstacle in self.static_obstacles:
            away = sub(position, obstacle[:3])
            dist = norm(away)
            activation = obstacle[3] + 1.6
            if dist < 1.0e-6 or dist >= activation:
                continue
            strength = 1.0 * (1.0 / max(dist - obstacle[3], 0.1) - 1.0 / activation)
            accel = add(accel, scale(away, strength / dist))
        return accel

    def obstacle_avoidance(self, idx: int) -> Vector:
        accel = [0.0, 0.0, 0.0]
        position = self.positions[idx]
        velocity = self.velocities[idx]
        for obstacle in self.static_obstacles:
            away = sub(position, obstacle[:3])
            dist = norm(away)
            activation = obstacle[3] + self.avoidance_activation_margin
            if dist < 1.0e-6 or dist >= activation:
                continue

            clearance = max(dist - obstacle[3], 0.08)
            gain = self.avoidance_gain
            goal_distance = norm(sub(self.goals[idx], position))
            if goal_distance < 1.2 and clearance > self.obstacle_margin + 0.35:
                gain *= 0.25
            radial = scale(away, gain / (clearance * clearance * dist))

            # Add a horizontal side-step term so agents do not push straight back
            # into their incoming path when they approach an obstacle head-on.
            tangent = [-away[1], away[0], 0.0]
            tangent_norm = norm(tangent)
            if tangent_norm > 1.0e-6:
                tangent = scale(tangent, 1.0 / tangent_norm)
                tangent_sign = 1.0
                if velocity[0] * tangent[0] + velocity[1] * tangent[1] < 0.0:
                    tangent_sign = -1.0
                tangent_strength = self.avoidance_tangent_gain / max(clearance, 0.2)
                if goal_distance < 1.2 and clearance > self.obstacle_margin + 0.35:
                    tangent_strength *= 0.25
                accel = add(accel, scale(tangent, tangent_sign * tangent_strength))
            accel = add(accel, radial)
        return accel

    def local_detour_target(self, idx: int, target: Vector) -> Vector:
        position = self.positions[idx]
        path = sub(target, position)
        path_xy = [path[0], path[1], 0.0]
        path_len = norm(path_xy)
        if path_len < 1.0e-6:
            return target

        best_obstacle = None
        best_projection = float("inf")
        direction = scale(path_xy, 1.0 / path_len)
        for obstacle in self.static_obstacles:
            rel = sub(obstacle[:3], position)
            projection = rel[0] * direction[0] + rel[1] * direction[1]
            if projection <= 0.0 or projection >= path_len:
                continue
            closest = [
                position[0] + direction[0] * projection,
                position[1] + direction[1] * projection,
                obstacle[2],
            ]
            lateral_error = norm([
                obstacle[0] - closest[0],
                obstacle[1] - closest[1],
                0.0,
            ])
            safe_radius = obstacle[3] + self.obstacle_margin + self.detour_clearance
            if lateral_error < safe_radius and projection < best_projection:
                best_obstacle = obstacle
                best_projection = projection

        if best_obstacle is None:
            return target

        left = [-direction[1], direction[0], 0.0]
        rel_agent = sub(position, best_obstacle[:3])
        side = 1.0 if (rel_agent[0] * left[0] + rel_agent[1] * left[1]) >= 0.0 else -1.0
        if abs(rel_agent[0]) + abs(rel_agent[1]) < 1.0e-6:
            side = 1.0 if idx % 2 == 0 else -1.0
        detour_radius = best_obstacle[3] + self.obstacle_margin + self.detour_clearance
        return [
            best_obstacle[0] + left[0] * side * detour_radius,
            best_obstacle[1] + left[1] * side * detour_radius,
            max(target[2], best_obstacle[2] + best_obstacle[3] + self.obstacle_margin + self.detour_altitude_margin),
        ]

    def build_formation_aware_waypoints(self, waypoints: List[Vector]) -> List[Vector]:
        if len(waypoints) < 2:
            return [waypoint[:] for waypoint in waypoints]

        planned = [waypoints[0][:]]
        footprint_radius = self.formation_footprint_radius()
        for start, target in zip(waypoints[:-1], waypoints[1:]):
            returning_to_start = norm(sub(target, waypoints[0])) <= self.waypoint_radius
            path = sub(target, start)
            path_xy = [path[0], path[1], 0.0]
            path_len = norm(path_xy)
            if path_len < 1.0e-6:
                planned.append(target[:])
                continue

            direction = scale(path_xy, 1.0 / path_len)
            left = [-direction[1], direction[0], 0.0]
            detours = []
            for obstacle in ([] if returning_to_start else self.static_obstacles):
                rel = sub(obstacle[:3], start)
                projection = rel[0] * direction[0] + rel[1] * direction[1]
                if projection <= 0.0 or projection >= path_len:
                    continue
                closest = [
                    start[0] + direction[0] * projection,
                    start[1] + direction[1] * projection,
                    obstacle[2],
                ]
                lateral_error = norm([
                    obstacle[0] - closest[0],
                    obstacle[1] - closest[1],
                    0.0,
                ])
                safe_radius = obstacle[3] + footprint_radius + self.detour_clearance
                if lateral_error >= safe_radius:
                    continue

                rel_start = sub(start, obstacle[:3])
                side = 1.0 if (rel_start[0] * left[0] + rel_start[1] * left[1]) >= 0.0 else -1.0
                detour_radius = obstacle[3] + footprint_radius + self.detour_clearance
                detour = [
                    obstacle[0] + left[0] * side * detour_radius,
                    obstacle[1] + left[1] * side * detour_radius,
                    max(
                        target[2],
                        obstacle[2]
                        + obstacle[3]
                        + self.obstacle_margin
                        + self.detour_altitude_margin,
                    ),
                ]
                detours.append((projection, detour))

            for _, detour in sorted(detours, key=lambda item: item[0]):
                if norm(sub(planned[-1], detour)) > self.waypoint_radius:
                    planned.append(detour)
            planned.append(target[:])
        return planned

    def formation_aware_leader_target(self, target: Vector) -> Vector:
        position = self.positions[0]
        path = sub(target, position)
        path_xy = [path[0], path[1], 0.0]
        path_len = norm(path_xy)
        if path_len < 1.0e-6:
            return target

        direction = scale(path_xy, 1.0 / path_len)
        footprint_radius = self.formation_footprint_radius()
        best_obstacle = None
        best_projection = float("inf")

        for obstacle in self.static_obstacles:
            rel = sub(obstacle[:3], position)
            projection = rel[0] * direction[0] + rel[1] * direction[1]
            if projection <= 0.0 or projection >= path_len:
                continue
            closest = [
                position[0] + direction[0] * projection,
                position[1] + direction[1] * projection,
                obstacle[2],
            ]
            lateral_error = norm([
                obstacle[0] - closest[0],
                obstacle[1] - closest[1],
                0.0,
            ])
            safe_radius = obstacle[3] + footprint_radius + self.detour_clearance
            if lateral_error < safe_radius and projection < best_projection:
                best_obstacle = obstacle
                best_projection = projection

        if best_obstacle is None:
            return target

        left = [-direction[1], direction[0], 0.0]
        rel_leader = sub(position, best_obstacle[:3])
        side = 1.0 if (rel_leader[0] * left[0] + rel_leader[1] * left[1]) >= 0.0 else -1.0
        if abs(rel_leader[0]) + abs(rel_leader[1]) < 1.0e-6:
            side = 1.0
        detour_radius = best_obstacle[3] + footprint_radius + self.detour_clearance
        return [
            best_obstacle[0] + left[0] * side * detour_radius,
            best_obstacle[1] + left[1] * side * detour_radius,
            max(
                target[2],
                best_obstacle[2]
                + best_obstacle[3]
                + self.obstacle_margin
                + self.detour_altitude_margin,
            ),
        ]

    def agent_separation(self, idx: int) -> Vector:
        accel = [0.0, 0.0, 0.0]
        position = self.positions[idx]
        for other in range(self.agent_count):
            if other == idx:
                continue
            away = sub(position, self.positions[other])
            dist = norm(away)
            if dist < 1.0e-6 or dist >= self.agent_separation_activation:
                continue
            strength = self.agent_separation_gain * (
                1.0 / max(dist - self.collision_radius, 0.08)
                - 1.0 / max(self.agent_separation_activation - self.collision_radius, 0.08)
            )
            accel = add(accel, scale(away, strength / dist))
        return accel

    def min_agent_distance(self) -> float:
        min_dist = float("inf")
        for i in range(self.agent_count):
            for j in range(i + 1, self.agent_count):
                min_dist = min(min_dist, norm(sub(self.positions[i], self.positions[j])))
        return 0.0 if min_dist == float("inf") else min_dist

    def agent_collision(self, idx: int) -> bool:
        for other in range(self.agent_count):
            if other != idx and norm(sub(self.positions[idx], self.positions[other])) <= self.collision_radius:
                return True
        return False

    def obstacle_collision(self, idx: int) -> bool:
        position = self.positions[idx]
        for obstacle in self.static_obstacles:
            if norm(sub(position, obstacle[:3])) <= obstacle[3] + self.obstacle_margin:
                return True
        return False

    def agent_out_of_bounds(self, idx: int) -> bool:
        p = self.positions[idx]
        return (
            abs(p[0]) > self.boundary
            or abs(p[1]) > self.boundary
            or p[2] < self.min_altitude
            or p[2] > self.boundary
        )
