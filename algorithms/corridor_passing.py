"""Causal predictive corridor assessment and proactive lateral replanning (SI units).

Evaluates path obstruction, available lateral passing space, and candidate A*
passing paths using exclusively timestamped causal observations. Never accesses
hidden scenario schedules, waypoints, or future reversal timers.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Callable

import numpy as np

from algorithms.astar import (
    AStarPlanner,
    grid_to_world,
    inflate_occupancy_grid,
    is_segment_collision_free,
    rasterize_geometry_onto_grid,
    world_to_grid,
)
from algorithms.observation import ObservationFrame, ObservedObstacle
from config import DWAConfig, ObservationConfig, RobotConfig, WorldConfig
from simulation.entities import RobotState, StaticObstacle
from simulation.geometry import CircleGeometry


@dataclass(frozen=True)
class CorridorAssessment:
    """Causal assessment of corridor obstruction and lateral passing feasibility."""

    path_obstructed: bool
    obstructing_obstacle_id: str | None
    min_path_clearance_m: float
    predicted_path_clearance_m: float
    estimated_ttc_s: float | None
    braking_distance_m: float
    lateral_clearance_left_m: float
    lateral_clearance_right_m: float
    trigger_replan: bool
    reason: str


@dataclass(frozen=True)
class CandidatePathEvaluation:
    """Validation result for a single candidate passing path."""

    side: str
    path_world_m: list[tuple[float, float]]
    geometric_min_clearance_m: float
    dwa_safe: bool
    dwa_v_mps: float
    dwa_omega_radps: float
    dwa_predicted_clearance_m: float
    dwa_braking_clearance_m: float | None
    goal_preserved: bool
    accepted: bool
    rejection_reason: str | None = None


def build_static_occupancy_grid(
    world_cfg: WorldConfig,
    static_obstacles: list[StaticObstacle],
) -> np.ndarray:
    """Rasterize static obstacles into a binary occupancy grid (0=free, 1=occupied)."""
    res = world_cfg.grid_resolution_m
    rows = max(1, int(round(world_cfg.height_m / res)))
    cols = max(1, int(round(world_cfg.width_m / res)))
    grid = np.zeros((rows, cols), dtype=np.uint8)
    for obs in static_obstacles:
        r0 = max(0, int(math.floor(obs.y_m / res)))
        r1 = min(rows, int(math.ceil((obs.y_m + obs.height_m) / res)))
        c0 = max(0, int(math.floor(obs.x_m / res)))
        c1 = min(cols, int(math.ceil((obs.x_m + obs.width_m) / res)))
        if r1 > r0 and c1 > c0:
            grid[r0:r1, c0:c1] = 1
    return grid


def _point_to_segment_distance_m(
    px: float,
    py: float,
    ax: float,
    ay: float,
    bx: float,
    by: float,
) -> float:
    dx = bx - ax
    dy = by - ay
    len_sq = dx * dx + dy * dy
    if len_sq <= 1e-12:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / len_sq))
    cx = ax + t * dx
    cy = ay + t * dy
    return math.hypot(px - cx, py - cy)


def _remaining_path_ahead(
    rx: float,
    ry: float,
    path_world_m: list[tuple[float, float]],
    max_dist_m: float,
) -> tuple[list[tuple[float, float]], tuple[float, float]]:
    """Return local path points ahead of (rx, ry) up to max_dist_m and unit tangent."""
    if not path_world_m:
        return ([], (1.0, 0.0))
    closest_idx = min(
        range(len(path_world_m)),
        key=lambda i: (path_world_m[i][0] - rx) ** 2 + (path_world_m[i][1] - ry) ** 2,
    )
    pts: list[tuple[float, float]] = [(rx, ry)]
    acc = 0.0
    prev = (rx, ry)
    for i in range(closest_idx, len(path_world_m)):
        p = path_world_m[i]
        seg = math.hypot(p[0] - prev[0], p[1] - prev[1])
        pts.append(p)
        acc += seg
        prev = p
        if acc >= max_dist_m:
            break
    if len(pts) >= 2:
        tx = pts[-1][0] - pts[0][0]
        ty = pts[-1][1] - pts[0][1]
        norm = math.hypot(tx, ty)
        tangent = (tx / norm, ty / norm) if norm > 1e-9 else (1.0, 0.0)
    else:
        tangent = (1.0, 0.0)
    return (pts, tangent)


def _ray_static_clearance_m(
    ox: float,
    oy: float,
    dx: float,
    dy: float,
    world_cfg: WorldConfig,
    static_obstacles: list[StaticObstacle],
    max_probe_m: float = 4.0,
    step_m: float = 0.1,
) -> float:
    """Probe distance along (dx, dy) from (ox, oy) to nearest wall/boundary."""
    d = step_m
    while d <= max_probe_m + 1e-9:
        px = ox + dx * d
        py = oy + dy * d
        if px < 0.0 or px > world_cfg.width_m or py < 0.0 or py > world_cfg.height_m:
            return max(0.0, d - step_m)
        for s in static_obstacles:
            if s.x_m <= px <= s.x_m + s.width_m and s.y_m <= py <= s.y_m + s.height_m:
                return max(0.0, d - step_m)
        d += step_m
    return max_probe_m


def _causal_ttc_s(
    rx: float,
    ry: float,
    rvx: float,
    rvy: float,
    rr: float,
    ox: float,
    oy: float,
    ovx: float,
    ovy: float,
    orad: float,
) -> float | None:
    dx = ox - rx
    dy = oy - ry
    dist_sq = dx * dx + dy * dy
    rad_sum = rr + orad
    if dist_sq <= rad_sum * rad_sum:
        return 0.0
    dvx = ovx - rvx
    dvy = ovy - rvy
    a = dvx * dvx + dvy * dvy
    if a <= 1e-12:
        return None
    b = 2.0 * (dx * dvx + dy * dvy)
    if b >= 0.0:
        return None
    disc = b * b - 4.0 * a * (dist_sq - rad_sum * rad_sum)
    if disc < 0.0:
        return None
    t = (-b - math.sqrt(disc)) / (2.0 * a)
    return t if t >= 0.0 else None


class CorridorPassingManager:
    """Stateful causal corridor assessment & proactive lateral replanning manager."""

    def __init__(
        self,
        world_cfg: WorldConfig,
        robot_cfg: RobotConfig,
        dwa_cfg: DWAConfig,
        obs_cfg: ObservationConfig = ObservationConfig(),
    ) -> None:
        self._world_cfg = world_cfg
        self._robot_cfg = robot_cfg
        self._dwa_cfg = dwa_cfg
        self._obs_cfg = obs_cfg
        self._obstruction_streak: int = 0
        self._open_space_streak: int = 0
        self._cooldown_s: float = 0.0
        self._replan_count: int = 0
        self._first_replan_time_s: float | None = None
        self._last_chosen_side: str | None = None
        self._last_assessment: CorridorAssessment | None = None
        self._last_candidates: list[CandidatePathEvaluation] = []

    def reset(self) -> None:
        self._obstruction_streak = 0
        self._open_space_streak = 0
        self._cooldown_s = 0.0
        self._replan_count = 0
        self._first_replan_time_s = None
        self._last_chosen_side = None
        self._last_assessment = None
        self._last_candidates = []

    @property
    def replan_count(self) -> int:
        return self._replan_count

    @property
    def first_replan_time_s(self) -> float | None:
        return self._first_replan_time_s

    @property
    def last_assessment(self) -> CorridorAssessment | None:
        return self._last_assessment

    @property
    def last_candidates(self) -> list[CandidatePathEvaluation]:
        return list(self._last_candidates)

    def assess_corridor(
        self,
        robot_state: RobotState,
        dt_s: float,
        current_path_world_m: list[tuple[float, float]],
        static_obstacles: list[StaticObstacle],
        observation_frame: ObservationFrame,
    ) -> CorridorAssessment:
        """Assess whether the current path ahead is obstructed and lateral passing is viable."""
        self._cooldown_s = max(0.0, self._cooldown_s - max(0.0, dt_s))
        braking_dist = (
            abs(robot_state.v_mps) * self._dwa_cfg.reaction_time_s
            + (robot_state.v_mps ** 2) / max(1e-6, 2.0 * self._dwa_cfg.max_brake_decel_mps2)
        )

        if not self._dwa_cfg.proactive_replan_enabled or len(current_path_world_m) < 2:
            out = CorridorAssessment(
                path_obstructed=False,
                obstructing_obstacle_id=None,
                min_path_clearance_m=math.inf,
                predicted_path_clearance_m=math.inf,
                estimated_ttc_s=None,
                braking_distance_m=braking_dist,
                lateral_clearance_left_m=0.0,
                lateral_clearance_right_m=0.0,
                trigger_replan=False,
                reason="DISABLED_OR_NO_PATH",
            )
            self._last_assessment = out
            return out

        local_pts, tangent = _remaining_path_ahead(
            robot_state.x_m,
            robot_state.y_m,
            current_path_world_m,
            self._dwa_cfg.proactive_path_lookahead_m,
        )
        if len(local_pts) < 2:
            out = CorridorAssessment(
                path_obstructed=False,
                obstructing_obstacle_id=None,
                min_path_clearance_m=math.inf,
                predicted_path_clearance_m=math.inf,
                estimated_ttc_s=None,
                braking_distance_m=braking_dist,
                lateral_clearance_left_m=0.0,
                lateral_clearance_right_m=0.0,
                trigger_replan=False,
                reason="NEAR_GOAL",
            )
            self._last_assessment = out
            return out

        # Check lateral wall confinement near robot to distinguish corridors/channels from open space
        nx, ny = -tangent[1], tangent[0]
        rob_left = _ray_static_clearance_m(robot_state.x_m, robot_state.y_m, nx, ny, self._world_cfg, static_obstacles)
        rob_right = _ray_static_clearance_m(robot_state.x_m, robot_state.y_m, -nx, -ny, self._world_cfg, static_obstacles)
        in_corridor = min(rob_left, rob_right) <= 2.0

        rvx = robot_state.v_mps * math.cos(robot_state.theta_rad)
        rvy = robot_state.v_mps * math.sin(robot_state.theta_rad)

        min_path_clear = math.inf
        min_pred_clear = math.inf
        min_ttc: float | None = None
        obstructing_obs: ObservedObstacle | None = None

        for obs in observation_frame.obstacles:
            if not obs.is_valid:
                continue
            dx_r = obs.x_m - robot_state.x_m
            dy_r = obs.y_m - robot_state.y_m
            dist_r = math.hypot(dx_r, dy_r)
            along = dx_r * tangent[0] + dy_r * tangent[1]
            if along <= 0.1 or dist_r > self._dwa_cfg.proactive_obs_dist_m:
                continue

            # Current clearance from obstacle to local path segments
            d_seg = min(
                _point_to_segment_distance_m(
                    obs.x_m, obs.y_m, local_pts[i][0], local_pts[i][1], local_pts[i + 1][0], local_pts[i + 1][1]
                )
                for i in range(len(local_pts) - 1)
            )
            cur_c = d_seg - self._robot_cfg.radius_m - obs.radius_m - obs.position_uncertainty_m
            if cur_c < min_path_clear:
                min_path_clear = cur_c

            # Causal constant-velocity predicted clearance over predict_time_s
            preds = obs.predict_causal_positions(
                observation_frame.sim_time_s,
                self._dwa_cfg.predict_dt_s,
                max(1, round(self._dwa_cfg.predict_time_s / self._dwa_cfg.predict_dt_s)),
                self._obs_cfg.uncertainty_growth_rate_mps,
            )
            pred_c = min(
                min(
                    _point_to_segment_distance_m(
                        px, py, local_pts[i][0], local_pts[i][1], local_pts[i + 1][0], local_pts[i + 1][1]
                    )
                    for i in range(len(local_pts) - 1)
                )
                - self._robot_cfg.radius_m
                - eff_r
                for px, py, eff_r in preds
            )
            if pred_c < min_pred_clear:
                min_pred_clear = pred_c

            obs_along_speed = obs.vx_mps * tangent[0] + obs.vy_mps * tangent[1]
            slower_or_blocking = obs_along_speed < (
                self._robot_cfg.max_linear_speed_mps - self._dwa_cfg.proactive_min_speed_diff_mps
            )
            if (
                (cur_c < self._dwa_cfg.proactive_path_clearance_threshold_m or pred_c < self._dwa_cfg.safety_margin_m + 0.15)
                and slower_or_blocking
            ):
                if obstructing_obs is None or cur_c <= min_path_clear + 1e-9:
                    obstructing_obs = obs

            ttc = _causal_ttc_s(
                robot_state.x_m, robot_state.y_m, rvx, rvy, self._robot_cfg.radius_m,
                obs.x_m, obs.y_m, obs.vx_mps, obs.vy_mps, obs.radius_m,
            )
            if ttc is not None and (min_ttc is None or ttc < min_ttc):
                min_ttc = ttc

        if obstructing_obs is None:
            self._obstruction_streak = 0
            self._open_space_streak = 0
            out = CorridorAssessment(
                path_obstructed=False,
                obstructing_obstacle_id=None,
                min_path_clearance_m=min_path_clear,
                predicted_path_clearance_m=min_pred_clear,
                estimated_ttc_s=min_ttc,
                braking_distance_m=braking_dist,
                lateral_clearance_left_m=0.0,
                lateral_clearance_right_m=0.0,
                trigger_replan=False,
                reason="PATH_CLEAR",
            )
            self._last_assessment = out
            return out

        # Measure available lateral passing space around the obstructing obstacle
        wall_left = _ray_static_clearance_m(
            obstructing_obs.x_m, obstructing_obs.y_m, nx, ny, self._world_cfg, static_obstacles
        )
        wall_right = _ray_static_clearance_m(
            obstructing_obs.x_m, obstructing_obs.y_m, -nx, -ny, self._world_cfg, static_obstacles
        )
        eff_r = obstructing_obs.radius_m + obstructing_obs.position_uncertainty_m
        lat_left = max(0.0, wall_left - eff_r)
        lat_right = max(0.0, wall_right - eff_r)
        required_gap = 2.0 * self._robot_cfg.radius_m + 2.0 * self._dwa_cfg.safety_margin_m
        lateral_passable = max(lat_left, lat_right) >= required_gap

        obs_along_v = obstructing_obs.vx_mps * tangent[0] + obstructing_obs.vy_mps * tangent[1]
        same_dir_lead = obs_along_v > 0.15

        if in_corridor and lateral_passable and robot_state.v_mps > 0.20:
            self._obstruction_streak += 1
            self._open_space_streak = 0
        elif (
            not in_corridor
            and self._dwa_cfg.proactive_open_space_enabled
            and lateral_passable
            and same_dir_lead
            and robot_state.v_mps > 0.20
        ):
            self._obstruction_streak = 0
            self._open_space_streak += 1
        else:
            self._obstruction_streak = 0
            self._open_space_streak = 0

        corridor_ready = self._obstruction_streak >= self._dwa_cfg.proactive_hysteresis_steps
        open_space_ready = (
            self._open_space_streak >= self._dwa_cfg.proactive_open_space_hysteresis_steps
        )
        should_trigger = (corridor_ready or open_space_ready) and self._cooldown_s <= 0.0
        reason = (
            (
                "OPEN_SPACE_OVERTAKE_TRIGGERED"
                if open_space_ready
                else "CORRIDOR_LATERAL_PASS_TRIGGERED"
            )
            if should_trigger
            else (
                "COOLDOWN_ACTIVE"
                if self._cooldown_s > 0.0 and (corridor_ready or open_space_ready)
                else ("NO_LATERAL_PASSING_SPACE" if not lateral_passable else "HYSTERESIS_ACCUMULATING")
            )
        )
        out = CorridorAssessment(
            path_obstructed=True,
            obstructing_obstacle_id=obstructing_obs.obstacle_id,
            min_path_clearance_m=min_path_clear,
            predicted_path_clearance_m=min_pred_clear,
            estimated_ttc_s=min_ttc,
            braking_distance_m=braking_dist,
            lateral_clearance_left_m=lat_left,
            lateral_clearance_right_m=lat_right,
            trigger_replan=should_trigger,
            reason=reason,
        )
        self._last_assessment = out
        return out

    def generate_and_validate_candidates(
        self,
        sim_time_s: float,
        robot_state: RobotState,
        dt_s: float,
        goal_m: tuple[float, float],
        base_occupancy_grid: np.ndarray,
        resolution_m: float,
        static_obstacles: list[StaticObstacle],
        observation_frame: ObservationFrame,
        dwa_eval_fn: Callable[[list[tuple[float, float]]], tuple[bool, float, float, float, float | None]],
    ) -> tuple[list[tuple[float, float]] | None, list[CandidatePathEvaluation]]:
        """Generate left/right/astar lateral passing candidates and validate each via
        static/dynamic geometry, continuity, DWA execution, and full braking safety.

        `dwa_eval_fn(candidate_path)` returns:
        `(dwa_safe, v_mps, omega_radps, predicted_min_clearance_m, min_braking_clearance_m)`.
        """
        assessment = self._last_assessment
        if assessment is None or not assessment.obstructing_obstacle_id:
            return (None, [])

        obs_target = next(
            (o for o in observation_frame.obstacles if o.is_valid and o.obstacle_id == assessment.obstructing_obstacle_id),
            None,
        )
        if obs_target is None:
            return (None, [])

        rows, cols = base_occupancy_grid.shape
        margin_m = self._dwa_cfg.safety_margin_m
        required_clearance_m = self._robot_cfg.radius_m + margin_m
        dx_g = goal_m[0] - robot_state.x_m
        dy_g = goal_m[1] - robot_state.y_m
        norm_g = math.hypot(dx_g, dy_g)
        tx, ty = (dx_g / norm_g, dy_g / norm_g) if norm_g > 1e-9 else (1.0, 0.0)
        nx, ny = -ty, tx

        def _plan_with_side_bias(side: str) -> list[tuple[float, float]]:
            grid_copy = np.array(base_occupancy_grid, dtype=np.uint8, copy=True)
            for obs in observation_frame.obstacles:
                if not obs.is_valid:
                    continue
                if obs.geometry is not None:
                    rasterize_geometry_onto_grid(grid_copy, obs.geometry, resolution_m)
                else:
                    for px, py, eff_r in obs.predict_causal_positions(
                        observation_frame.sim_time_s,
                        0.3,
                        4,
                        self._obs_cfg.uncertainty_growth_rate_mps,
                    ):
                        rasterize_geometry_onto_grid(
                            grid_copy,
                            CircleGeometry(px, py, eff_r),
                            resolution_m,
                        )
            if side in ("left", "right"):
                sign = -1.0 if side == "left" else 1.0  # block opposite side to force chosen side
                d_probe = obs_target.radius_m + 0.1
                while d_probe <= 2.5:
                    bx = obs_target.x_m + sign * nx * d_probe
                    by = obs_target.y_m + sign * ny * d_probe
                    if 0.0 <= bx < self._world_cfg.width_m and 0.0 <= by < self._world_cfg.height_m:
                        rr, cc = world_to_grid(bx, by, resolution_m, (rows, cols))
                        grid_copy[rr, cc] = 1
                    d_probe += resolution_m * 0.5

            inflated = inflate_occupancy_grid(
                grid_copy,
                resolution_m,
                self._robot_cfg.radius_m,
                margin_m,
                enforce_boundary_clearance=True,
            )
            start_cell = world_to_grid(robot_state.x_m, robot_state.y_m, resolution_m, inflated.shape)
            if inflated[start_cell]:
                sr, sc = start_cell
                search_r = math.ceil(required_clearance_m / resolution_m) + 2
                best_cell: tuple[int, int] | None = None
                best_d = math.inf
                for dr in range(-search_r, search_r + 1):
                    for dc in range(-search_r, search_r + 1):
                        nr, nc = sr + dr, sc + dc
                        if not (0 <= nr < rows and 0 <= nc < cols) or inflated[nr, nc]:
                            continue
                        cand_m = grid_to_world(nr, nc, resolution_m)
                        if not is_segment_collision_free(
                            (robot_state.x_m, robot_state.y_m),
                            cand_m,
                            grid_copy,
                            resolution_m,
                            max(0.0, self._robot_cfg.radius_m - 0.5 * resolution_m),
                            enforce_boundary_clearance=False,
                        ):
                            continue
                        d = math.hypot(cand_m[0] - robot_state.x_m, cand_m[1] - robot_state.y_m)
                        if d < best_d:
                            best_d = d
                            best_cell = (nr, nc)
                if best_cell is None:
                    return []
                start_cell = best_cell

            goal_cell = world_to_grid(goal_m[0], goal_m[1], resolution_m, inflated.shape)
            res = AStarPlanner(resolution_m, True).plan(
                inflated,
                start_cell,
                goal_cell,
                raw_grid=grid_copy,
                required_clearance_m=required_clearance_m,
                enforce_boundary_clearance=True,
            )
            return list(res.path_world_m) if res.success and res.path_world_m else []

        evaluations: list[CandidatePathEvaluation] = []
        accepted_list: list[CandidatePathEvaluation] = []
        base_inflated_grid = np.array(base_occupancy_grid, dtype=np.uint8, copy=True)

        for side in ("left", "right", "astar"):
            path = _plan_with_side_bias(side)
            if len(path) < 2:
                evaluations.append(
                    CandidatePathEvaluation(
                        side=side, path_world_m=[], geometric_min_clearance_m=-math.inf,
                        dwa_safe=False, dwa_v_mps=0.0, dwa_omega_radps=0.0,
                        dwa_predicted_clearance_m=-math.inf, dwa_braking_clearance_m=None,
                        goal_preserved=False, accepted=False, rejection_reason="NO_ASTAR_PATH",
                    )
                )
                continue

            goal_ok = math.hypot(path[-1][0] - goal_m[0], path[-1][1] - goal_m[1]) <= (
                self._dwa_cfg.goal_tolerance_m + resolution_m
            )
            if not goal_ok:
                evaluations.append(
                    CandidatePathEvaluation(
                        side=side, path_world_m=path, geometric_min_clearance_m=-math.inf,
                        dwa_safe=False, dwa_v_mps=0.0, dwa_omega_radps=0.0,
                        dwa_predicted_clearance_m=-math.inf, dwa_braking_clearance_m=None,
                        goal_preserved=False, accepted=False, rejection_reason="GOAL_NOT_PRESERVED",
                    )
                )
                continue

            # Check continuity from robot's current position to path[0] against static obstacles
            cont_ok = is_segment_collision_free(
                (robot_state.x_m, robot_state.y_m),
                path[0],
                base_inflated_grid,
                resolution_m,
                max(0.0, self._robot_cfg.radius_m - 0.5 * resolution_m),
                enforce_boundary_clearance=False,
            )
            if not cont_ok:
                evaluations.append(
                    CandidatePathEvaluation(
                        side=side, path_world_m=path, geometric_min_clearance_m=-math.inf,
                        dwa_safe=False, dwa_v_mps=0.0, dwa_omega_radps=0.0,
                        dwa_predicted_clearance_m=-math.inf, dwa_braking_clearance_m=None,
                        goal_preserved=True, accepted=False, rejection_reason="CONTINUITY_FAILED",
                    )
                )
                continue

            # Geometric clearance along local path ahead against static + causal observed obstacles
            geom_clear = math.inf
            for px, py in path[:12]:
                c_wall = min(
                    px - self._robot_cfg.radius_m,
                    self._world_cfg.width_m - px - self._robot_cfg.radius_m,
                    py - self._robot_cfg.radius_m,
                    self._world_cfg.height_m - py - self._robot_cfg.radius_m,
                )
                if c_wall < geom_clear:
                    geom_clear = c_wall
                for s in static_obstacles:
                    cx = max(s.x_m, min(px, s.x_m + s.width_m))
                    cy = max(s.y_m, min(py, s.y_m + s.height_m))
                    c_s = math.hypot(px - cx, py - cy) - self._robot_cfg.radius_m
                    if c_s < geom_clear:
                        geom_clear = c_s
                for obs in observation_frame.obstacles:
                    if not obs.is_valid:
                        continue
                    c_d = math.hypot(px - obs.x_m, py - obs.y_m) - self._robot_cfg.radius_m - obs.radius_m - obs.position_uncertainty_m
                    if c_d < geom_clear:
                        geom_clear = c_d

            if geom_clear < margin_m:
                evaluations.append(
                    CandidatePathEvaluation(
                        side=side, path_world_m=path, geometric_min_clearance_m=geom_clear,
                        dwa_safe=False, dwa_v_mps=0.0, dwa_omega_radps=0.0,
                        dwa_predicted_clearance_m=-math.inf, dwa_braking_clearance_m=None,
                        goal_preserved=True, accepted=False, rejection_reason="GEOMETRIC_MARGIN_VIOLATION",
                    )
                )
                continue

            dwa_safe, dv, dw, pred_c, brake_c = dwa_eval_fn(path)
            brake_ok = brake_c is None or brake_c >= margin_m
            accepted = dwa_safe and pred_c >= margin_m and brake_ok
            rej = None if accepted else ("BRAKING_MARGIN_VIOLATION" if not brake_ok else "DWA_UNSAFE")

            ev = CandidatePathEvaluation(
                side=side,
                path_world_m=path,
                geometric_min_clearance_m=geom_clear,
                dwa_safe=dwa_safe,
                dwa_v_mps=dv,
                dwa_omega_radps=dw,
                dwa_predicted_clearance_m=pred_c,
                dwa_braking_clearance_m=brake_c,
                goal_preserved=True,
                accepted=accepted,
                rejection_reason=rej,
            )
            evaluations.append(ev)
            if accepted:
                accepted_list.append(ev)

        self._last_candidates = evaluations
        if not accepted_list:
            return (None, evaluations)

        best = max(
            accepted_list,
            key=lambda e: (
                round(min(e.dwa_predicted_clearance_m, e.dwa_braking_clearance_m if e.dwa_braking_clearance_m is not None else math.inf), 2),
                1 if e.side == self._last_chosen_side else 0,
                round(e.geometric_min_clearance_m, 6),
            ),
        )
        self._last_chosen_side = best.side
        self._cooldown_s = self._dwa_cfg.proactive_replan_cooldown_s
        self._obstruction_streak = 0
        self._open_space_streak = 0
        self._replan_count += 1
        if self._first_replan_time_s is None:
            self._first_replan_time_s = sim_time_s
        return (best.path_world_m, evaluations)
