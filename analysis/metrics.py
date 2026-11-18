"""Mathematically precise metric definitions and per-run tracking for Phase 5 benchmarks.

Strictly independent of Pygame. All physical quantities are in SI units.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
import math
from typing import Sequence

from algorithms.astar import segment_to_square_distance_m
from config import WorldConfig
from simulation.dynamic_obstacles import DynamicObstacle, swept_circle_min_distance_m
from simulation.entities import StaticObstacle
from simulation.geometry import CircleGeometry, compute_geometry_distance
from simulation.objects import SimObject
from simulation.safety import (
    compute_dynamic_obstacle_clearance,
    compute_static_obstacle_clearance,
)


@dataclass(frozen=True)
class RunMetrics:
    """Complete raw per-run benchmark metrics (all physical units in SI)."""

    scenario_name: str
    controller_id: str
    dt_s: float
    goal_reached: bool
    termination_reason: str
    completion_time_s: float | None
    sim_time_s: float
    first_collision_time_s: float | None
    physical_collision_episode_count: int
    physical_collision_step_count: int
    safety_margin_violation_episode_count: int
    safety_margin_violation_step_count: int
    min_instantaneous_clearance_m: float
    min_swept_clearance_m: float
    stop_count: int
    goal_final_stop: bool
    replan_attempts: int
    replan_successes: int
    no_path_replan_count: int
    blocked_step_count: int
    blocked_episode_count: int
    path_length_m: float
    avg_dwa_time_ms: float | None
    max_dwa_time_ms: float | None
    p95_dwa_time_ms: float | None

    def to_dict(self) -> dict[str, object]:
        """Convert to JSON/CSV-friendly dictionary."""
        return asdict(self)


def compute_instantaneous_clearance_m(
    robot_x_m: float,
    robot_y_m: float,
    robot_radius_m: float,
    world_cfg: WorldConfig,
    static_obstacles: Sequence[StaticObstacle],
    dynamic_obstacles: Sequence[DynamicObstacle],
    sim_objects: Sequence[SimObject] = (),
) -> float:
    """Compute exact instantaneous surface-to-surface clearance (m) at (robot_x_m, robot_y_m)."""
    min_c = min(
        robot_x_m - robot_radius_m,
        world_cfg.width_m - robot_x_m - robot_radius_m,
        robot_y_m - robot_radius_m,
        world_cfg.height_m - robot_y_m - robot_radius_m,
    )
    for s in static_obstacles:
        surf_c, _, _ = compute_static_obstacle_clearance(
            robot_x_m, robot_y_m, robot_radius_m, s
        )
        if surf_c < min_c:
            min_c = surf_c
    for obs in dynamic_obstacles:
        if not obs.active:
            continue
        surf_c, _, _ = compute_dynamic_obstacle_clearance(
            robot_x_m, robot_y_m, robot_radius_m, obs
        )
        if surf_c < min_c:
            min_c = surf_c
    if sim_objects:
        robot_geom = CircleGeometry(robot_x_m, robot_y_m, robot_radius_m)
        for obj in sim_objects:
            if not obj.motion.active:
                continue
            surf_c = compute_geometry_distance(robot_geom, obj.geometry()).surface_clearance_m
            if surf_c < min_c:
                min_c = surf_c
    return min_c


def compute_swept_clearance_m(
    robot_prev_m: tuple[float, float],
    robot_curr_m: tuple[float, float],
    robot_radius_m: float,
    world_cfg: WorldConfig,
    static_obstacles: Sequence[StaticObstacle],
    dyn_transitions: Sequence[tuple[DynamicObstacle, tuple[float, float], tuple[float, float]]],
    sim_objects: Sequence[SimObject] = (),
) -> float:
    """Compute continuous swept surface-to-surface clearance (m) over one step."""
    min_c = math.inf
    for rx, ry in (robot_prev_m, robot_curr_m):
        c_wall = min(
            rx - robot_radius_m,
            world_cfg.width_m - rx - robot_radius_m,
            ry - robot_radius_m,
            world_cfg.height_m - ry - robot_radius_m,
        )
        if c_wall < min_c:
            min_c = c_wall

    for s in static_obstacles:
        d_sq = segment_to_square_distance_m(
            robot_prev_m,
            robot_curr_m,
            s.x_m,
            s.y_m,
            s.x_m + s.width_m,
            s.y_m + s.height_m,
        )
        surf_c = d_sq - robot_radius_m
        if surf_c < min_c:
            min_c = surf_c

    for obs, o_prev, o_curr in dyn_transitions:
        if not obs.active:
            continue
        min_center_d, _ = swept_circle_min_distance_m(
            robot_prev_m, robot_curr_m, o_prev, o_curr
        )
        surf_c = min_center_d - robot_radius_m - obs.radius_m
        if surf_c < min_c:
            min_c = surf_c

    if sim_objects:
        # Linearly interpolate robot along [0, 1] against active SimObjects
        for frac in (0.0, 0.25, 0.5, 0.75, 1.0):
            rx = robot_prev_m[0] + frac * (robot_curr_m[0] - robot_prev_m[0])
            ry = robot_prev_m[1] + frac * (robot_curr_m[1] - robot_prev_m[1])
            r_geom = CircleGeometry(rx, ry, robot_radius_m)
            for obj in sim_objects:
                if not obj.motion.active:
                    continue
                surf_c = compute_geometry_distance(r_geom, obj.geometry()).surface_clearance_m
                if surf_c < min_c:
                    min_c = surf_c

    return min_c


def _percentile(values: list[float], q: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    idx = (len(ordered) - 1) * q
    lo = int(math.floor(idx))
    hi = int(math.ceil(idx))
    if lo == hi:
        return ordered[lo]
    return ordered[lo] + (idx - lo) * (ordered[hi] - ordered[lo])


class RunMetricsTracker:
    """Accumulates step-by-step benchmark metrics under strict SI rules."""

    def __init__(
        self,
        scenario_name: str,
        controller_id: str,
        dt_s: float,
        safety_margin_m: float = 0.15,
        stop_speed_threshold_mps: float = 0.05,
        is_dwa: bool = False,
    ) -> None:
        self._scenario_name = scenario_name
        self._controller_id = controller_id
        self._dt_s = dt_s
        self._safety_margin_m = safety_margin_m
        self._stop_thresh_mps = stop_speed_threshold_mps
        self._is_dwa = is_dwa

        self._min_inst_clearance_m = math.inf
        self._min_swept_clearance_m = math.inf

        self._phys_col_steps = 0
        self._phys_col_episodes = 0
        self._in_collision = False
        self._first_collision_time_s: float | None = None

        self._margin_viol_steps = 0
        self._margin_viol_episodes = 0
        self._in_margin_violation = False

        self._has_started_moving = False
        self._was_moving = False
        self._stop_count = 0

        self._replan_attempts = 0
        self._replan_successes = 0
        self._no_path_replans = 0

        self._blocked_steps = 0
        self._blocked_episodes = 0
        self._in_blocked = False

        self._path_length_m = 0.0
        self._dwa_times_ms: list[float] = []

    @property
    def has_collided(self) -> bool:
        return self._phys_col_steps > 0

    def record_replan(self, success: bool) -> None:
        self._replan_attempts += 1
        if success:
            self._replan_successes += 1
        else:
            self._no_path_replans += 1

    def record_dwa_call(self, duration_ms: float, blocked: bool) -> None:
        self._dwa_times_ms.append(duration_ms)
        if blocked:
            self._blocked_steps += 1
            if not self._in_blocked:
                self._blocked_episodes += 1
                self._in_blocked = True
        else:
            self._in_blocked = False

    def record_step(
        self,
        sim_time_s: float,
        prev_pos_m: tuple[float, float],
        curr_pos_m: tuple[float, float],
        speed_mps: float,
        inst_clearance_m: float,
        swept_clearance_m: float,
        is_goal_step: bool = False,
    ) -> None:
        self._path_length_m += math.hypot(
            curr_pos_m[0] - prev_pos_m[0],
            curr_pos_m[1] - prev_pos_m[1],
        )

        if inst_clearance_m < self._min_inst_clearance_m:
            self._min_inst_clearance_m = inst_clearance_m
        if swept_clearance_m < self._min_swept_clearance_m:
            self._min_swept_clearance_m = swept_clearance_m

        step_min_c = min(inst_clearance_m, swept_clearance_m)
        is_collision = step_min_c <= 0.0
        is_margin_viol = (not is_collision) and (step_min_c < self._safety_margin_m)

        if is_collision:
            self._phys_col_steps += 1
            if not self._in_collision:
                self._phys_col_episodes += 1
                self._in_collision = True
            if self._first_collision_time_s is None:
                self._first_collision_time_s = sim_time_s
            self._in_margin_violation = False
        else:
            self._in_collision = False
            if is_margin_viol:
                self._margin_viol_steps += 1
                if not self._in_margin_violation:
                    self._margin_viol_episodes += 1
                    self._in_margin_violation = True
            else:
                self._in_margin_violation = False

        # Stop count: exclude initial stationary state and final goal-arrival stop
        moving_now = abs(speed_mps) >= self._stop_thresh_mps
        if moving_now:
            self._has_started_moving = True
            self._was_moving = True
        elif self._has_started_moving and self._was_moving and not is_goal_step:
            self._stop_count += 1
            self._was_moving = False
        elif not moving_now:
            self._was_moving = False

    def finalize(
        self,
        goal_reached: bool,
        termination_reason: str,
        sim_time_s: float,
        final_speed_mps: float,
    ) -> RunMetrics:
        avg_ms: float | None = None
        max_ms: float | None = None
        p95_ms: float | None = None
        if self._is_dwa and self._dwa_times_ms:
            avg_ms = sum(self._dwa_times_ms) / len(self._dwa_times_ms)
            max_ms = max(self._dwa_times_ms)
            p95_ms = _percentile(self._dwa_times_ms, 0.95)

        return RunMetrics(
            scenario_name=self._scenario_name,
            controller_id=self._controller_id,
            dt_s=self._dt_s,
            goal_reached=goal_reached,
            termination_reason=termination_reason,
            completion_time_s=sim_time_s if goal_reached else None,
            sim_time_s=sim_time_s,
            first_collision_time_s=self._first_collision_time_s,
            physical_collision_episode_count=self._phys_col_episodes,
            physical_collision_step_count=self._phys_col_steps,
            safety_margin_violation_episode_count=self._margin_viol_episodes,
            safety_margin_violation_step_count=self._margin_viol_steps,
            min_instantaneous_clearance_m=self._min_inst_clearance_m,
            min_swept_clearance_m=self._min_swept_clearance_m,
            stop_count=self._stop_count,
            goal_final_stop=goal_reached and (abs(final_speed_mps) < self._stop_thresh_mps),
            replan_attempts=self._replan_attempts,
            replan_successes=self._replan_successes,
            no_path_replan_count=self._no_path_replans,
            blocked_step_count=self._blocked_steps,
            blocked_episode_count=self._blocked_episodes,
            path_length_m=self._path_length_m,
            avg_dwa_time_ms=avg_ms,
            max_dwa_time_ms=max_ms,
            p95_dwa_time_ms=p95_ms,
        )
