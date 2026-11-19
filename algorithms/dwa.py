"""Dynamic Window Approach (DWA) real-time local obstacle avoidance in SI units.

Implements velocity-space sampling, unicycle trajectory rollout, static/dynamic
obstacle clearance evaluation, multi-objective cost scoring, and optimal command
selection from scratch without external pathfinding or DWA libraries.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
import math
from typing import TYPE_CHECKING

from algorithms.astar import segment_to_square_distance_m
from algorithms.corridor_passing import CorridorPassingManager, build_static_occupancy_grid
from algorithms.observation import (
    ObservationFrame,
    ObservedObstacle,
    ObservedRobotState,
)
from config import DWAConfig, ObservationConfig, RobotConfig, WorldConfig
from simulation.dynamic_obstacles import DynamicObstacle
from simulation.entities import RobotState, StaticObstacle, wrap_angle_rad
from simulation.geometry import CircleGeometry, compute_geometry_distance
from simulation.safety import compute_static_obstacle_clearance

if TYPE_CHECKING:
    from simulation.objects import SimObject


@dataclass(frozen=True)
class DynamicWindow:
    """Reachable (v, omega) velocity bounds over one simulation step."""

    v_min_mps: float
    v_max_mps: float
    omega_min_radps: float
    omega_max_radps: float


class WaitingZoneState(str, Enum):
    """Predictive waiting-zone safety classification."""

    GATE_CLEAR_HOLD_SAFE = "GATE_CLEAR_HOLD_SAFE"
    GATE_CLEAR_HOLD_UNSAFE = "GATE_CLEAR_HOLD_UNSAFE"
    GATE_OCCUPIED_HOLD_SAFE = "GATE_OCCUPIED_HOLD_SAFE"
    HOLD_UNSAFE_EVADE = "HOLD_UNSAFE_EVADE"


class DeadlockState(str, Enum):
    """Explicit deadlock and fallback state machine classification."""

    NONE = "NONE"
    TEMPORARY_SAFE_WAIT = "TEMPORARY_SAFE_WAIT"
    PERSISTENT_FALLBACK = "PERSISTENT_FALLBACK"
    BLOCKED_GLOBAL_PATH = "BLOCKED_GLOBAL_PATH"
    NO_SAFE_MANEUVER = "NO_SAFE_MANEUVER"
    GENUINE_DEADLOCK = "GENUINE_DEADLOCK"


@dataclass(frozen=True)
class WaitingZoneAssessment:
    """Result of predictive waiting-zone evaluation over waiting_horizon_s."""

    state: WaitingZoneState
    gate_occupied: bool
    min_waiting_clearance_m: float
    threat_time_s: float | None = None


@dataclass(frozen=True)
class DWAResult:
    """Output of a single DWA planning cycle."""

    v_mps: float
    omega_radps: float
    score: float | None
    predicted_min_clearance_m: float | None
    selected_trajectory: list[tuple[float, float]] = field(default_factory=list)
    rejected_trajectories: list[list[tuple[float, float]]] = field(
        default_factory=list
    )
    candidate_trajectories: list[list[tuple[float, float]]] = field(
        default_factory=list
    )
    target_reached: bool = False
    safety_margin_satisfied: bool = True
    collision_free_exists: bool = True
    fallback_active: bool = False
    fallback_reason: str | None = None
    waiting_for_passage: bool = False
    blocked: bool = False
    status: str = "NORMAL"
    min_braking_clearance_m: float | None = None
    min_braking_clearance_time_s: float | None = None
    waiting_state: str = WaitingZoneState.GATE_CLEAR_HOLD_SAFE.value
    waiting_min_clearance_m: float | None = None
    holding_pose_m: tuple[float, float] | None = None
    reverse_active: bool = False
    fallback_priority: int | None = None
    fallback_duration_s: float = 0.0
    deadlock_state: str = DeadlockState.NONE.value
    proactive_replan_triggered: bool = False


def compute_stopping_distance_m(
    v_mps: float,
    max_brake_decel_mps2: float,
    reaction_time_s: float = 0.0,
) -> float:
    """Compute physical stopping distance: |v|*t_react + v^2 / (2 * a_brake)."""
    speed = abs(v_mps)
    if speed <= 0.0 or max_brake_decel_mps2 <= 0.0:
        return speed * max(0.0, reaction_time_s)
    return speed * max(0.0, reaction_time_s) + (speed * speed) / (
        2.0 * max_brake_decel_mps2
    )


def compute_dynamic_window(
    v_current_mps: float,
    omega_current_radps: float,
    dt_s: float,
    robot_cfg: RobotConfig,
    dwa_cfg: DWAConfig,
    allow_reverse: bool = False,
) -> DynamicWindow:
    """Compute the reachable dynamic window [v_min, v_max] x [omega_min, omega_max]."""
    min_v_bound = (
        dwa_cfg.min_reverse_speed_mps if allow_reverse else dwa_cfg.min_linear_speed_mps
    )
    v_min = max(
        min_v_bound,
        v_current_mps - dwa_cfg.max_linear_accel_mps2 * dt_s,
    )
    v_max = min(
        robot_cfg.max_linear_speed_mps,
        v_current_mps + dwa_cfg.max_linear_accel_mps2 * dt_s,
    )
    if v_min > v_max:
        v_min, v_max = v_max, v_min

    omega_min = max(
        -robot_cfg.max_angular_speed_radps,
        omega_current_radps - dwa_cfg.max_angular_accel_radps2 * dt_s,
    )
    omega_max = min(
        robot_cfg.max_angular_speed_radps,
        omega_current_radps + dwa_cfg.max_angular_accel_radps2 * dt_s,
    )
    if omega_min > omega_max:
        omega_min, omega_max = omega_max, omega_min

    return DynamicWindow(
        v_min_mps=v_min,
        v_max_mps=v_max,
        omega_min_radps=omega_min,
        omega_max_radps=omega_max,
    )


def _linspace(low: float, high: float, count: int) -> list[float]:
    if count <= 1 or math.isclose(low, high, abs_tol=1e-9):
        return [0.5 * (low + high)]
    step = (high - low) / (count - 1)
    return [low + i * step for i in range(count)]


def sample_velocities(
    window: DynamicWindow,
    linear_samples: int,
    angular_samples: int,
) -> list[tuple[float, float]]:
    """Sample candidate (v_mps, omega_radps) pairs inside the dynamic window."""
    vs = _linspace(window.v_min_mps, window.v_max_mps, linear_samples)
    omegas = _linspace(window.omega_min_radps, window.omega_max_radps, angular_samples)

    # Always include exact 0.0 rad/s if within bounds for clean straight tracking.
    if window.omega_min_radps <= 0.0 <= window.omega_max_radps:
        if all(abs(w) > 1e-6 for w in omegas):
            omegas.append(0.0)

    return [(v, w) for v in vs for w in omegas]


def predict_trajectory(
    robot_state: RobotState,
    v_mps: float,
    omega_radps: float,
    predict_time_s: float,
    predict_dt_s: float,
    first_step_dt_s: float | None = None,
) -> list[tuple[float, float, float]]:
    """Roll out unicycle kinematics over predict_time_s in SI units."""
    steps = max(1, round(predict_time_s / predict_dt_s))
    x = robot_state.x_m
    y = robot_state.y_m
    theta = robot_state.theta_rad
    traj: list[tuple[float, float, float]] = [(x, y, theta)]

    if first_step_dt_s is not None and 1e-9 < first_step_dt_s < predict_dt_s - 1e-9:
        sub_dts = [first_step_dt_s, predict_dt_s - first_step_dt_s]
        for sub_dt in sub_dts:
            mid_theta = theta + 0.5 * omega_radps * sub_dt
            x += v_mps * math.cos(mid_theta) * sub_dt
            y += v_mps * math.sin(mid_theta) * sub_dt
            theta = wrap_angle_rad(theta + omega_radps * sub_dt)
            traj.append((x, y, theta))
        rem_steps = steps - 1
    else:
        rem_steps = steps

    for _ in range(rem_steps):
        mid_theta = theta + 0.5 * omega_radps * predict_dt_s
        x += v_mps * math.cos(mid_theta) * predict_dt_s
        y += v_mps * math.sin(mid_theta) * predict_dt_s
        theta = wrap_angle_rad(theta + omega_radps * predict_dt_s)
        traj.append((x, y, theta))

    return traj


def _point_clearance(
    x_m: float,
    y_m: float,
    robot_radius_m: float,
    world_cfg: WorldConfig,
    static_obstacles: list[StaticObstacle],
    dyn_positions: list[tuple[float, float, float]],
    static_boxes: list[tuple[float, float, float, float]] | None = None,
) -> float:
    """Compute minimum surface-to-surface clearance at (x_m, y_m)."""
    min_c = min(
        x_m - robot_radius_m,
        world_cfg.width_m - x_m - robot_radius_m,
        y_m - robot_radius_m,
        world_cfg.height_m - y_m - robot_radius_m,
    )
    hypot = math.hypot
    if static_boxes is not None:
        thresh_s = min_c + robot_radius_m
        for ox0, oy0, ox1, oy1 in static_boxes:
            dx = ox0 - x_m if x_m < ox0 else (x_m - ox1 if x_m > ox1 else 0.0)
            if dx >= thresh_s:
                continue
            dy = oy0 - y_m if y_m < oy0 else (y_m - oy1 if y_m > oy1 else 0.0)
            if dy >= thresh_s:
                continue
            surf_c = hypot(dx, dy) - robot_radius_m
            if surf_c < min_c:
                min_c = surf_c
                thresh_s = min_c + robot_radius_m
    else:
        for obs in static_obstacles:
            surf_c, _, _ = compute_static_obstacle_clearance(x_m, y_m, robot_radius_m, obs)
            if surf_c < min_c:
                min_c = surf_c
    for ox, oy, orad in dyn_positions:
        thresh_d = min_c + robot_radius_m + orad
        dx = x_m - ox
        if dx >= thresh_d or dx <= -thresh_d:
            continue
        dy = y_m - oy
        if dy >= thresh_d or dy <= -thresh_d:
            continue
        surf_c = hypot(dx, dy) - robot_radius_m - orad
        if surf_c < min_c:
            min_c = surf_c
    return min_c


def _swept_circle_clearance_m(
    ax0: float,
    ay0: float,
    ax1: float,
    ay1: float,
    bx0: float,
    by0: float,
    bx1: float,
    by1: float,
    r_sum: float,
) -> float:
    """Continuous-time minimum surface-to-surface clearance between two linearly
    moving circles over s in [0, 1].
    """
    px = ax0 - bx0
    py = ay0 - by0
    rx = (ax1 - ax0) - (bx1 - bx0)
    ry = (ay1 - ay0) - (by1 - by0)
    denom = rx * rx + ry * ry
    if denom <= 1e-12:
        return math.hypot(px, py) - r_sum
    s = max(0.0, min(1.0, -(px * rx + py * ry) / denom))
    return math.hypot(px + s * rx, py + s * ry) - r_sum


def simulate_braking_trajectory(
    start_pose: tuple[float, float, float],
    v_mps: float,
    predict_dt_s: float,
    max_brake_decel_mps2: float,
    reaction_time_s: float = 0.0,
) -> tuple[list[tuple[float, float, float]], list[float]]:
    """Simulate a physically feasible deceleration trajectory to rest using
    the same midpoint unicycle integration as Robot.step().

    Returns (poses, relative_times_s) starting at relative time 0.0.
    """
    x, y, theta = start_pose
    poses: list[tuple[float, float, float]] = [(x, y, theta)]
    times: list[float] = [0.0]
    if predict_dt_s <= 0.0 or abs(v_mps) <= 1e-9 or max_brake_decel_mps2 <= 0.0:
        return poses, times

    t_elapsed = 0.0
    sign = 1.0 if v_mps >= 0.0 else -1.0
    speed = abs(v_mps)
    cos_t = math.cos(theta)
    sin_t = math.sin(theta)

    # Reaction phase at constant velocity
    react_rem = max(0.0, reaction_time_s)
    while react_rem > 1e-9:
        dt = min(predict_dt_s, react_rem)
        x += sign * speed * cos_t * dt
        y += sign * speed * sin_t * dt
        t_elapsed += dt
        react_rem -= dt
        poses.append((x, y, theta))
        times.append(t_elapsed)

    # Braking deceleration phase
    while speed > 1e-9:
        dt = min(predict_dt_s, speed / max_brake_decel_mps2)
        next_speed = max(0.0, speed - max_brake_decel_mps2 * dt)
        avg_speed = 0.5 * (speed + next_speed)
        x += sign * avg_speed * cos_t * dt
        y += sign * avg_speed * sin_t * dt
        t_elapsed += dt
        speed = next_speed
        poses.append((x, y, theta))
        times.append(t_elapsed)

    return poses, times


def _causal_const_vel_predictions(
    dynamic_obstacles: list[DynamicObstacle],
    dt_s: float,
    steps: int,
) -> list[list[tuple[float, float, float]]]:
    """Causal constant-velocity predictions using only observed (x, y, vx, vy, radius)."""
    out: list[list[tuple[float, float, float]]] = []
    for o in dynamic_obstacles:
        if not o.active:
            continue
        out.append(
            [
                (o.x_m + o.vx_mps * (s * dt_s), o.y_m + o.vy_mps * (s * dt_s), o.radius_m)
                for s in range(max(0, steps) + 1)
            ]
        )
    return out


def _build_causal_preds_from_frame(
    obs_frame: ObservationFrame,
    dt_s: float,
    steps: int,
    growth_rate_mps: float,
) -> list[list[tuple[float, float, float]]]:
    """Build causal circular obstacle predictions from an ObservationFrame."""
    return [
        obs.predict_causal_positions(obs_frame.sim_time_s, dt_s, steps, growth_rate_mps)
        for obs in obs_frame.obstacles
        if obs.is_valid and obs.geometry is None
    ]


def _swept_static_clearance_m(
    x0: float,
    y0: float,
    x1: float,
    y1: float,
    robot_radius_m: float,
    static_boxes: list[tuple[float, float, float, float]],
    current_best: float,
) -> float:
    """Compute continuous segment-to-AABB static clearance using segment_to_square_distance_m."""
    if not static_boxes:
        return current_best
    ax_min = x0 if x0 < x1 else x1
    ax_max = x1 if x0 < x1 else x0
    ay_min = y0 if y0 < y1 else y1
    ay_max = y1 if y0 < y1 else y0
    best = current_best
    p0 = (x0, y0)
    p1 = (x1, y1)
    for ox0, oy0, ox1, oy1 in static_boxes:
        bound = best + robot_radius_m
        if (
            ax_min - ox1 >= bound
            or ox0 - ax_max >= bound
            or ay_min - oy1 >= bound
            or oy0 - ay_max >= bound
        ):
            continue
        surf_c = segment_to_square_distance_m(p0, p1, ox0, oy0, ox1, oy1) - robot_radius_m
        if surf_c < best:
            best = surf_c
    return best


def evaluate_trajectory_with_braking(
    trajectory: list[tuple[float, float, float]],
    predict_dt_s: float,
    robot_radius_m: float,
    safety_margin_m: float,
    world_cfg: WorldConfig,
    static_obstacles: list[StaticObstacle],
    dynamic_obstacles: list[DynamicObstacle],
    v_mps: float = 0.0,
    max_brake_decel_mps2: float = 0.0,
    reaction_time_s: float = 0.0,
    precomputed_dyn_preds: list[tuple[float, list[tuple[float, float]]]] | None = None,
    sim_objects: list[SimObject] | None = None,
    simobj_pred_cache: dict[float, list[object]] | None = None,
    static_boxes: list[tuple[float, float, float, float]] | None = None,
    precomputed_dyn_at_steps: list[list[tuple[float, float, float]]] | None = None,
    traj_times_s: list[float] | None = None,
    dyn_pred_dt_s: float | None = None,
    stop_on_margin_violation: bool = False,
    causal_dyn_preds: list[list[tuple[float, float, float]]] | None = None,
    observed_sim_objects: list[ObservedObstacle] | None = None,
    obs_sim_time_s: float = 0.0,
    uncertainty_growth_rate_mps: float = 0.0,
    causal_stop_safety_enabled: bool = False,
    causal_reversal_horizon_s: float = 0.0,
    causal_straight_reversal_horizon_s: float = 0.0,
    causal_lateral_escape_threshold_m: float = 0.10,
) -> tuple[bool, float, float | None, float | None]:
    """Evaluate continuous swept clearance along both the prediction trajectory
    and the complete deceleration braking trajectory.

    Returns (is_safe, min_clearance_m, min_braking_clearance_m, min_braking_time_s).
    """
    min_clearance = math.inf
    if traj_times_s is None:
        traj_times = [i * predict_dt_s for i in range(len(trajectory))]
    else:
        traj_times = traj_times_s
    horizon_t = traj_times[-1] if traj_times else 0.0
    obs_dt = dyn_pred_dt_s if dyn_pred_dt_s is not None and dyn_pred_dt_s > 0.0 else predict_dt_s

    brake_poses: list[tuple[float, float, float]] = []
    brake_times: list[float] = []
    extra_brake_steps = 0
    if max_brake_decel_mps2 > 0.0 and abs(v_mps) > 1e-6 and trajectory:
        brake_poses, brake_times = simulate_braking_trajectory(
            trajectory[-1],
            v_mps,
            predict_dt_s,
            max_brake_decel_mps2,
            reaction_time_s,
        )
        if brake_times:
            extra_brake_steps = max(1, math.ceil(brake_times[-1] / max(1e-6, obs_dt)))

    horizon_obs_steps = max(0, math.ceil(horizon_t / max(1e-6, obs_dt)))
    total_needed_steps = horizon_obs_steps + extra_brake_steps
    if causal_dyn_preds is not None:
        c_dyn_preds = causal_dyn_preds
    elif precomputed_dyn_preds is not None:
        c_dyn_preds = [
            [(px, py, orad) for px, py in preds] for orad, preds in precomputed_dyn_preds
        ]
    else:
        c_dyn_preds = _causal_const_vel_predictions(
            dynamic_obstacles, obs_dt, total_needed_steps
        )

    active_sim_objects = [o for o in sim_objects if o.motion.active] if sim_objects else []
    obs_sim_list = [o for o in observed_sim_objects if o.is_valid] if observed_sim_objects else []
    has_sim = bool(obs_sim_list) or bool(active_sim_objects)
    sim_cache = simobj_pred_cache if simobj_pred_cache is not None else ({} if has_sim else None)

    def _sim_geoms_at(t_s: float) -> list[tuple[object, float]]:
        assert sim_cache is not None
        t_key = round(t_s, 9)
        geoms = sim_cache.get(t_key)
        if geoms is None:
            if obs_sim_list:
                geoms = [
                    obj.predict_causal_geometry_at(
                        obs_sim_time_s, t_s, uncertainty_growth_rate_mps
                    )
                    for obj in obs_sim_list
                ]
            else:
                geoms = [(obj.predict_geometry_at(t_s), 0.0) for obj in active_sim_objects]
            sim_cache[t_key] = geoms  # type: ignore[assignment]
        return geoms  # type: ignore[return-value]

    if static_boxes is None and static_obstacles:
        static_boxes = [
            (o.x_m, o.y_m, o.x_m + o.width_m, o.y_m + o.height_m)
            for o in static_obstacles
        ]

    # 1. Prediction horizon: point + continuous swept dynamic clearance
    prev_obs_idx = 0
    prev_sim_geoms: list[tuple[object, float]] | None = None
    horizon_safe = True
    for step_idx, (x_m, y_m, _) in enumerate(trajectory):
        t_pred = traj_times[step_idx]
        obs_idx = round(t_pred / max(1e-9, obs_dt))
        if precomputed_dyn_at_steps is not None and step_idx < len(precomputed_dyn_at_steps):
            dyn_at_step = precomputed_dyn_at_steps[step_idx]
        else:
            dyn_at_step = [
                preds[min(obs_idx, len(preds) - 1)]
                for preds in c_dyn_preds
            ]
        step_clear = _point_clearance(
            x_m,
            y_m,
            robot_radius_m,
            world_cfg,
            static_obstacles,
            dyn_at_step,
            static_boxes=static_boxes,
        )
        cur_sim_geoms: list[tuple[object, float]] | None = None
        if has_sim:
            robot_geom = CircleGeometry(x_m, y_m, robot_radius_m)
            cur_sim_geoms = _sim_geoms_at(t_pred)
            for obj_geom, pad_m in cur_sim_geoms:
                obj_clr = (
                    compute_geometry_distance(
                        robot_geom, obj_geom  # type: ignore[arg-type]
                    ).surface_clearance_m
                    - pad_m
                )
                if obj_clr < step_clear:
                    step_clear = obj_clr
        if step_clear < min_clearance:
            min_clearance = step_clear

        if step_idx > 0:
            px, py, _ = trajectory[step_idx - 1]
            min_clearance = _swept_static_clearance_m(
                px, py, x_m, y_m, robot_radius_m, static_boxes, min_clearance
            )
            ax_min = px if px < x_m else x_m
            ax_max = x_m if px < x_m else px
            ay_min = py if py < y_m else y_m
            ay_max = y_m if py < y_m else py
            for preds in c_dyn_preds:
                bx0, by0, orad0 = preds[min(prev_obs_idx, len(preds) - 1)]
                bx1, by1, orad1 = preds[min(obs_idx, len(preds) - 1)]
                r_sum = robot_radius_m + max(orad0, orad1)
                bound = min_clearance + r_sum
                if (
                    ax_min - (bx1 if bx1 > bx0 else bx0) >= bound
                    or (bx0 if bx0 < bx1 else bx1) - ax_max >= bound
                    or ay_min - (by1 if by1 > by0 else by0) >= bound
                    or (by0 if by0 < by1 else by1) - ay_max >= bound
                ):
                    continue
                swept_c = _swept_circle_clearance_m(
                    px, py, x_m, y_m, bx0, by0, bx1, by1, r_sum
                )
                if swept_c < min_clearance:
                    min_clearance = swept_c
            if has_sim and prev_sim_geoms is not None and cur_sim_geoms is not None:
                for (p_g, p_pad), (c_g, c_pad) in zip(prev_sim_geoms, cur_sim_geoms):
                    if isinstance(p_g, CircleGeometry) and isinstance(c_g, CircleGeometry):
                        swept_s = _swept_circle_clearance_m(
                            px,
                            py,
                            x_m,
                            y_m,
                            p_g.center_x_m,
                            p_g.center_y_m,
                            c_g.center_x_m,
                            c_g.center_y_m,
                            robot_radius_m + c_g.radius_m + max(p_pad, c_pad),
                        )
                        if swept_s < min_clearance:
                            min_clearance = swept_s
        prev_obs_idx = obs_idx
        prev_sim_geoms = cur_sim_geoms

        if min_clearance <= 0.0:
            return (False, min_clearance, None, None)
        if min_clearance < safety_margin_m:
            if step_idx == 0 or not stop_on_margin_violation:
                horizon_safe = False
            elif horizon_safe and step_idx >= 2:
                return (False, min_clearance, None, None)

    if not horizon_safe and stop_on_margin_violation:
        return (False, min_clearance, None, None)

    # 1b. Causal sudden-stop and bounded-reversal immediate braking safety check
    if (
        stop_on_margin_violation
        and causal_stop_safety_enabled
        and max_brake_decel_mps2 > 0.0
        and abs(v_mps) > 1e-6
        and len(trajectory) >= 2
        and c_dyn_preds
    ):
        imm_poses, imm_times = simulate_braking_trajectory(
            trajectory[1],
            v_mps,
            predict_dt_s,
            max_brake_decel_mps2,
            reaction_time_s,
        )
        if len(imm_poses) > 1:
            rx0, ry0, _ = trajectory[0]
            inv_obs_dt = 1.0 / max(1e-6, obs_dt)
            for preds in c_dyn_preds:
                if len(preds) < 2:
                    continue
                ox0, oy0, orad = preds[0]
                ox1, oy1, _ = preds[1]
                ovx = (ox1 - ox0) * inv_obs_dt
                ovy = (oy1 - oy0) * inv_obs_dt
                if math.hypot(ovx, ovy) <= 0.05:
                    continue
                r_sum = robot_radius_m + orad
                if math.hypot(rx0 - ox0, ry0 - oy0) - r_sum < safety_margin_m:
                    continue
                for b_idx in range(1, len(imm_poses)):
                    bx0, by0, _ = imm_poses[b_idx - 1]
                    bx1, by1, _ = imm_poses[b_idx]
                    if (
                        _swept_circle_clearance_m(
                            bx0, by0, bx1, by1, ox0, oy0, ox0, oy0, r_sum
                        )
                        < safety_margin_m
                    ):
                        return (False, min_clearance, None, None)
                    if causal_reversal_horizon_s > 0.0:
                        t0 = min(imm_times[b_idx - 1], causal_reversal_horizon_s)
                        t1 = min(imm_times[b_idx], causal_reversal_horizon_s)
                        rx_a = ox0 - ovx * t0
                        ry_a = oy0 - ovy * t0
                        rx_b = ox0 - ovx * t1
                        ry_b = oy0 - ovy * t1
                        if (
                            _swept_circle_clearance_m(
                                bx0, by0, bx1, by1, rx_a, ry_a, rx_b, ry_b, r_sum
                            )
                            < safety_margin_m
                        ):
                            return (False, min_clearance, None, None)
                if causal_reversal_horizon_s > imm_times[-1]:
                    obs_spd = math.hypot(ovx, ovy)
                    ux_o = ovx / obs_spd
                    uy_o = ovy / obs_spd
                    lat_start = abs((rx0 - ox0) * (-uy_o) + (ry0 - oy0) * ux_o)
                    lat_traj_end = abs((trajectory[-1][0] - ox0) * (-uy_o) + (trajectory[-1][1] - oy0) * ux_o)
                    eff_rev_h = (
                        causal_reversal_horizon_s
                        if lat_traj_end > lat_start + causal_lateral_escape_threshold_m
                        else max(causal_reversal_horizon_s, causal_straight_reversal_horizon_s)
                    )
                    sx, sy, _ = imm_poses[-1]
                    t_a = imm_times[-1]
                    t_b = eff_rev_h
                    if (
                        _swept_circle_clearance_m(
                            sx,
                            sy,
                            sx,
                            sy,
                            ox0 - ovx * t_a,
                            oy0 - ovy * t_a,
                            ox0 - ovx * t_b,
                            oy0 - ovy * t_b,
                            r_sum,
                        )
                        < safety_margin_m
                    ):
                        return (False, min_clearance, None, None)

    # 2. Complete braking trajectory validation
    min_brake_c: float | None = None
    min_brake_t: float | None = None
    if len(brake_poses) > 1:
        min_brake_c = math.inf
        min_brake_t = horizon_t

        def _interp_dyn(t_rel: float) -> list[tuple[float, float, float]]:
            idx_f = (horizon_t + t_rel) / max(1e-6, obs_dt)
            i0 = int(math.floor(idx_f))
            frac = idx_f - i0
            res_list: list[tuple[float, float, float]] = []
            for preds in c_dyn_preds:
                p0 = preds[min(i0, len(preds) - 1)]
                p1 = preds[min(i0 + 1, len(preds) - 1)]
                res_list.append(
                    (
                        p0[0] + frac * (p1[0] - p0[0]),
                        p0[1] + frac * (p1[1] - p0[1]),
                        p0[2] + frac * (p1[2] - p0[2]),
                    )
                )
            return res_list

        prev_dyn = _interp_dyn(brake_times[0])
        for b_idx in range(1, len(brake_poses)):
            bx0, by0, _ = brake_poses[b_idx - 1]
            bx1, by1, _ = brake_poses[b_idx]
            cur_dyn = _interp_dyn(brake_times[b_idx])

            c_pt = _point_clearance(
                bx1,
                by1,
                robot_radius_m,
                world_cfg,
                static_obstacles,
                cur_dyn,
                static_boxes=static_boxes,
            )
            if has_sim:
                robot_geom_b = CircleGeometry(bx1, by1, robot_radius_m)
                for obj_geom, pad_m in _sim_geoms_at(horizon_t + brake_times[b_idx]):
                    obj_clr = (
                        compute_geometry_distance(
                            robot_geom_b, obj_geom  # type: ignore[arg-type]
                        ).surface_clearance_m
                        - pad_m
                    )
                    if obj_clr < c_pt:
                        c_pt = obj_clr
            if c_pt < min_brake_c:
                min_brake_c = c_pt
                min_brake_t = horizon_t + brake_times[b_idx]

            min_brake_c = _swept_static_clearance_m(
                bx0, by0, bx1, by1, robot_radius_m, static_boxes, min_brake_c
            )
            bx_min = bx0 if bx0 < bx1 else bx1
            bx_max = bx1 if bx0 < bx1 else bx0
            by_min = by0 if by0 < by1 else by1
            by_max = by1 if by0 < by1 else by0
            for d_i in range(len(cur_dyn)):
                ox0, oy0, orad = prev_dyn[d_i]
                ox1, oy1, _ = cur_dyn[d_i]
                r_sum = robot_radius_m + orad
                bound = min_brake_c + r_sum
                if (
                    bx_min - (ox1 if ox1 > ox0 else ox0) >= bound
                    or (ox0 if ox0 < ox1 else ox1) - bx_max >= bound
                    or by_min - (oy1 if oy1 > oy0 else oy0) >= bound
                    or (oy0 if oy0 < oy1 else oy1) - by_max >= bound
                ):
                    continue
                swept_b = _swept_circle_clearance_m(
                    bx0, by0, bx1, by1, ox0, oy0, ox1, oy1, r_sum
                )
                if swept_b < min_brake_c:
                    min_brake_c = swept_b
                    min_brake_t = horizon_t + 0.5 * (
                        brake_times[b_idx - 1] + brake_times[b_idx]
                    )

            prev_dyn = cur_dyn

        if min_brake_c < min_clearance:
            min_clearance = min_brake_c
        if min_brake_c <= 0.0:
            return (False, min_clearance, min_brake_c, min_brake_t)
        if min_brake_c < safety_margin_m:
            return (False, min_clearance, min_brake_c, min_brake_t)

    return (horizon_safe, min_clearance, min_brake_c, min_brake_t)


@dataclass(frozen=True)
class TrajectorySafetyClassification:
    """Explicitly separated safety outcomes for a candidate trajectory."""

    safety_margin_compliant: bool
    collision_free: bool
    min_predicted_clearance_m: float
    braking_valid: bool
    min_braking_clearance_m: float | None = None
    min_braking_time_s: float | None = None


def classify_trajectory_safety(
    trajectory: list[tuple[float, float, float]],
    predict_dt_s: float,
    robot_radius_m: float,
    safety_margin_m: float,
    world_cfg: WorldConfig,
    static_obstacles: list[StaticObstacle],
    dynamic_obstacles: list[DynamicObstacle],
    v_mps: float = 0.0,
    max_brake_decel_mps2: float = 0.0,
    reaction_time_s: float = 0.0,
    sim_objects: list[SimObject] | None = None,
) -> TrajectorySafetyClassification:
    """Classify trajectory safety without letting early margin violations mask later collisions."""
    is_margin_safe, min_c, brake_c, brake_t = evaluate_trajectory_with_braking(
        trajectory,
        predict_dt_s,
        robot_radius_m,
        safety_margin_m,
        world_cfg,
        static_obstacles,
        dynamic_obstacles,
        v_mps=v_mps,
        max_brake_decel_mps2=max_brake_decel_mps2,
        reaction_time_s=reaction_time_s,
        sim_objects=sim_objects,
        stop_on_margin_violation=False,
    )
    collision_free = min_c > 0.0 and (brake_c is None or brake_c > 0.0)
    braking_valid = brake_c is None or brake_c >= safety_margin_m
    return TrajectorySafetyClassification(
        safety_margin_compliant=is_margin_safe,
        collision_free=collision_free,
        min_predicted_clearance_m=min_c,
        braking_valid=braking_valid,
        min_braking_clearance_m=brake_c,
        min_braking_time_s=brake_t,
    )


def evaluate_trajectory_clearance(
    trajectory: list[tuple[float, float, float]],
    predict_dt_s: float,
    robot_radius_m: float,
    safety_margin_m: float,
    world_cfg: WorldConfig,
    static_obstacles: list[StaticObstacle],
    dynamic_obstacles: list[DynamicObstacle],
    v_mps: float = 0.0,
    max_brake_decel_mps2: float = 0.0,
    reaction_time_s: float = 0.0,
    precomputed_dyn_preds: list[tuple[float, list[tuple[float, float]]]] | None = None,
) -> tuple[bool, float]:
    """Evaluate minimum surface-to-surface clearance along the predicted trajectory
    and complete braking deceleration trajectory.
    """
    is_safe, min_c, _, _ = evaluate_trajectory_with_braking(
        trajectory,
        predict_dt_s,
        robot_radius_m,
        safety_margin_m,
        world_cfg,
        static_obstacles,
        dynamic_obstacles,
        v_mps=v_mps,
        max_brake_decel_mps2=max_brake_decel_mps2,
        reaction_time_s=reaction_time_s,
        precomputed_dyn_preds=precomputed_dyn_preds,
    )
    return (is_safe, min_c)


def _select_lookahead_target(
    robot_x_m: float,
    robot_y_m: float,
    path_world_m: list[tuple[float, float]],
    lookahead_distance_m: float,
) -> tuple[float, float]:
    """Find a local target point along the global A* path ahead of the robot."""
    if len(path_world_m) == 1:
        return path_world_m[0]

    # Find closest waypoint on path
    closest_idx = 0
    closest_dist = math.inf
    for idx, (wx, wy) in enumerate(path_world_m):
        d = math.hypot(wx - robot_x_m, wy - robot_y_m)
        if d < closest_dist:
            closest_dist = d
            closest_idx = idx

    # Walk forward from closest_idx until >= lookahead_distance_m from robot
    for idx in range(closest_idx, len(path_world_m)):
        wx, wy = path_world_m[idx]
        if math.hypot(wx - robot_x_m, wy - robot_y_m) >= lookahead_distance_m:
            return (wx, wy)

    return path_world_m[-1]


def _min_distance_to_path_m(
    x_m: float, y_m: float, path_world_m: list[tuple[float, float]]
) -> float:
    best = math.inf
    for wx, wy in path_world_m:
        d = math.hypot(wx - x_m, wy - y_m)
        if d < best:
            best = d
    return best


def score_trajectory(
    trajectory: list[tuple[float, float, float]],
    v_mps: float,
    min_clearance_m: float,
    lookahead_target_m: tuple[float, float],
    path_world_m: list[tuple[float, float]],
    max_v_mps: float,
    dwa_cfg: DWAConfig,
) -> float:
    """Compute weighted multi-objective DWA score (higher is better)."""
    start_x, start_y, _ = trajectory[0]
    end_x, end_y, end_theta = trajectory[-1]

    # 1. Heading alignment toward local A* target (guard against overshoot flip near target)
    end_to_target = math.hypot(
        lookahead_target_m[0] - end_x, lookahead_target_m[1] - end_y
    )
    if end_to_target < 0.45:
        desired_theta = math.atan2(
            lookahead_target_m[1] - start_y, lookahead_target_m[0] - start_x
        )
    else:
        desired_theta = math.atan2(
            lookahead_target_m[1] - end_y, lookahead_target_m[0] - end_x
        )
    heading_err = abs(wrap_angle_rad(desired_theta - end_theta))
    heading_score = 1.0 - (heading_err / math.pi)

    # 2. Normalized obstacle clearance
    cap = max(1e-6, dwa_cfg.max_clearance_cap_m)
    clearance_score = min(max(0.0, min_clearance_m), cap) / cap

    # 3. Forward velocity preference
    velocity_score = v_mps / max(1e-6, max_v_mps)

    # 4. Proximity to A* global path
    path_dist = _min_distance_to_path_m(end_x, end_y, path_world_m)
    path_score = 1.0 - min(1.0, path_dist / 2.0)

    # 5. Progress toward local target (prevents v=0 from beating safe forward motion)
    start_dist = math.hypot(
        lookahead_target_m[0] - start_x, lookahead_target_m[1] - start_y
    )
    progress_score = max(
        -1.0, min(1.0, (start_dist - end_to_target) / max(0.25, start_dist))
    )
    standstill_penalty = -1.0 if (v_mps < 0.15 and start_dist > 0.35) else 0.0

    return (
        dwa_cfg.heading_weight * heading_score
        + dwa_cfg.clearance_weight * clearance_score
        + dwa_cfg.velocity_weight * velocity_score
        + dwa_cfg.path_weight * path_score
        + progress_score
        + standstill_penalty
    )


def evaluate_waiting_zone_safety(
    robot_state: RobotState,
    lookahead_target: tuple[float, float],
    world_cfg: WorldConfig,
    robot_cfg: RobotConfig,
    dwa_cfg: DWAConfig,
    static_obstacles: list[StaticObstacle],
    dynamic_obstacles: list[DynamicObstacle],
    sim_objects: list[SimObject] | None = None,
    observation_frame: ObservationFrame | None = None,
    uncertainty_growth_rate_mps: float = 0.02,
) -> WaitingZoneAssessment:
    """Evaluate whether a narrow gate/corridor ahead requires waiting AND whether
    the robot's current stationary holding pose remains safe over `waiting_horizon_s`.

    Uses only current observed positions, velocities, and causal motion models
    (never reading hidden future schedules).
    """
    dx = lookahead_target[0] - robot_state.x_m
    dy = lookahead_target[1] - robot_state.y_m
    dist = math.hypot(dx, dy)
    gate_occupied = False

    if dist > 1e-6:
        ux, uy = dx / dist, dy / dist
        px, py = -uy, ux

        gate_obs_list: list[tuple[float, float, float, float, float]] = (
            [
                (o.x_m, o.y_m, o.vx_mps, o.vy_mps, o.radius_m)
                for o in observation_frame.obstacles
                if o.is_valid and o.geometry is None
            ]
            if observation_frame is not None
            else [
                (o.x_m, o.y_m, o.vx_mps, o.vy_mps, o.radius_m)
                for o in dynamic_obstacles
                if o.active
            ]
        )
        for ox, oy, ovx, ovy, orad in gate_obs_list:
            rel_x = ox - robot_state.x_m
            rel_y = oy - robot_state.y_m
            along = rel_x * ux + rel_y * uy
            lat = abs(rel_x * px + rel_y * py)
            v_along = ovx * ux + ovy * uy
            if along <= 0.5 or along > 13.0 or lat > 1.2 or v_along >= -0.1:
                continue

            left_wall = math.inf
            right_wall = math.inf
            for probe_d in (0.2 * i for i in range(1, 21)):
                lx = ox + px * probe_d
                ly = oy + py * probe_d
                if (
                    lx < 0.0
                    or lx > world_cfg.width_m
                    or ly < 0.0
                    or ly > world_cfg.height_m
                    or any(
                        s.x_m <= lx <= s.x_m + s.width_m
                        and s.y_m <= ly <= s.y_m + s.height_m
                        for s in static_obstacles
                    )
                ):
                    left_wall = probe_d
                    break
            for probe_d in (0.2 * i for i in range(1, 21)):
                rx = ox - px * probe_d
                ry = oy - py * probe_d
                if (
                    rx < 0.0
                    or rx > world_cfg.width_m
                    or ry < 0.0
                    or ry > world_cfg.height_m
                    or any(
                        s.x_m <= rx <= s.x_m + s.width_m
                        and s.y_m <= ry <= s.y_m + s.height_m
                        for s in static_obstacles
                    )
                ):
                    right_wall = probe_d
                    break

            max_side_gap = max(left_wall, right_wall) - orad
            required_side_gap = (
                2.0 * robot_cfg.radius_m
                + 2.0 * dwa_cfg.safety_margin_m
                + dwa_cfg.corridor_passage_margin_m
            )
            bottleneck_ahead = False
            if max_side_gap >= required_side_gap and along > 1.0:
                # Check if a narrow gate/bottleneck lies between the robot and the oncoming obstacle
                for d_gate in (1.0 * k for k in range(2, min(9, int(along) + 1))):
                    gx = robot_state.x_m + ux * d_gate
                    gy = robot_state.y_m + uy * d_gate
                    g_left = math.inf
                    g_right = math.inf
                    for probe_d in (0.2 * i for i in range(1, 15)):
                        lx = gx + px * probe_d
                        ly = gy + py * probe_d
                        if any(
                            s.x_m <= lx <= s.x_m + s.width_m
                            and s.y_m <= ly <= s.y_m + s.height_m
                            for s in static_obstacles
                        ):
                            g_left = probe_d
                            break
                    for probe_d in (0.2 * i for i in range(1, 15)):
                        rx = gx - px * probe_d
                        ry = gy - py * probe_d
                        if any(
                            s.x_m <= rx <= s.x_m + s.width_m
                            and s.y_m <= ry <= s.y_m + s.height_m
                            for s in static_obstacles
                        ):
                            g_right = probe_d
                            break
                    if max(g_left, g_right) - orad < required_side_gap:
                        bottleneck_ahead = True
                        break

            if max_side_gap < required_side_gap or bottleneck_ahead:
                rob_left = min(
                    (
                        d
                        for d in (0.2 * i for i in range(1, 21))
                        if any(
                            s.x_m <= robot_state.x_m + px * d <= s.x_m + s.width_m
                            and s.y_m <= robot_state.y_m + py * d <= s.y_m + s.height_m
                            for s in static_obstacles
                        )
                    ),
                    default=math.inf,
                )
                rob_right = min(
                    (
                        d
                        for d in (0.2 * i for i in range(1, 21))
                        if any(
                            s.x_m <= robot_state.x_m - px * d <= s.x_m + s.width_m
                            and s.y_m <= robot_state.y_m - py * d <= s.y_m + s.height_m
                            for s in static_obstacles
                        )
                    ),
                    default=math.inf,
                )
                # Detect narrow gate occupancy early enough to maintain safe standoff
                # before reaching the gate entrance (along < 12.0 m).
                if min(rob_left, rob_right) > 1.6 and along < 12.0:
                    gate_occupied = True
                    break

    # Evaluate stationary holding pose over the complete waiting horizon
    w_dt = max(0.05, dwa_cfg.waiting_dt_s)
    w_steps = max(1, round(dwa_cfg.waiting_horizon_s / w_dt))
    if observation_frame is not None:
        c_dyn_preds = _build_causal_preds_from_frame(
            observation_frame, w_dt, w_steps, uncertainty_growth_rate_mps
        )
        obs_sim_list = [o for o in observation_frame.obstacles if o.is_valid and o.geometry is not None]
    else:
        c_dyn_preds = _causal_const_vel_predictions(dynamic_obstacles, w_dt, w_steps)
        obs_sim_list = []
    robot_geom = CircleGeometry(robot_state.x_m, robot_state.y_m, robot_cfg.radius_m)

    min_wait_c = _point_clearance(
        robot_state.x_m,
        robot_state.y_m,
        robot_cfg.radius_m,
        world_cfg,
        static_obstacles,
        [],
    )
    threat_time_s: float | None = None
    eff_threshold = dwa_cfg.safety_margin_m + dwa_cfg.waiting_uncertainty_pad_m

    for step_idx in range(w_steps + 1):
        t_s = step_idx * w_dt
        step_c = min_wait_c
        for preds in c_dyn_preds:
            ox, oy, orad = preds[min(step_idx, len(preds) - 1)]
            c = math.hypot(robot_state.x_m - ox, robot_state.y_m - oy) - robot_cfg.radius_m - orad
            if c < step_c:
                step_c = c
        if obs_sim_list and observation_frame is not None:
            for obj in obs_sim_list:
                o_geom, pad_m = obj.predict_causal_geometry_at(
                    observation_frame.sim_time_s, t_s, uncertainty_growth_rate_mps
                )
                c = compute_geometry_distance(robot_geom, o_geom).surface_clearance_m - pad_m
                if c < step_c:
                    step_c = c
        elif sim_objects:
            for obj in sim_objects:
                if not obj.motion.active:
                    continue
                c = compute_geometry_distance(
                    robot_geom, obj.predict_geometry_at(t_s)
                ).surface_clearance_m
                if c < step_c:
                    step_c = c
        if step_c < min_wait_c:
            min_wait_c = step_c
        if threat_time_s is None and step_c < eff_threshold:
            threat_time_s = t_s

    hold_safe = min_wait_c >= eff_threshold
    if gate_occupied:
        state = (
            WaitingZoneState.GATE_OCCUPIED_HOLD_SAFE
            if hold_safe
            else WaitingZoneState.HOLD_UNSAFE_EVADE
        )
    else:
        state = (
            WaitingZoneState.GATE_CLEAR_HOLD_SAFE
            if hold_safe
            else WaitingZoneState.GATE_CLEAR_HOLD_UNSAFE
        )
    return WaitingZoneAssessment(
        state=state,
        gate_occupied=gate_occupied,
        min_waiting_clearance_m=min_wait_c,
        threat_time_s=threat_time_s,
    )


@dataclass(frozen=True)
class HoldingManeuverResult:
    """Validated unicycle maneuver to a lateral holding pose."""

    holding_pose_m: tuple[float, float]
    v_cmd_mps: float
    omega_cmd_radps: float
    min_clearance_m: float
    min_swept_clearance_m: float
    min_braking_clearance_m: float | None
    min_braking_time_s: float | None
    trajectory_xy: list[tuple[float, float]]
    maneuver_type: str


def plan_lateral_holding_maneuver(
    robot_state: RobotState,
    lookahead_target: tuple[float, float],
    dt_s: float,
    world_cfg: WorldConfig,
    robot_cfg: RobotConfig,
    dwa_cfg: DWAConfig,
    static_obstacles: list[StaticObstacle],
    dynamic_obstacles: list[DynamicObstacle],
    sim_objects: list[SimObject] | None = None,
    observation_frame: ObservationFrame | None = None,
    uncertainty_growth_rate_mps: float = 0.02,
) -> HoldingManeuverResult | None:
    """Plan and validate a physically feasible unicycle maneuver to a left/right
    holding pose that remains >= safety_margin_m over the maneuver, braking path,
    and waiting horizon.
    """
    if dt_s <= 0.0:
        return None
    dx = lookahead_target[0] - robot_state.x_m
    dy = lookahead_target[1] - robot_state.y_m
    d = math.hypot(dx, dy)
    ux, uy = (
        (dx / d, dy / d)
        if d > 1e-6
        else (math.cos(robot_state.theta_rad), math.sin(robot_state.theta_rad))
    )
    px, py = -uy, ux

    lat = dwa_cfg.waiting_lateral_offset_m
    candidate_specs: list[tuple[str, tuple[float, float]]] = [
        ("CURVED_LATERAL_LEFT", (robot_state.x_m + px * lat, robot_state.y_m + py * lat)),
        ("CURVED_LATERAL_RIGHT", (robot_state.x_m - px * lat, robot_state.y_m - py * lat)),
        (
            "CURVED_LATERAL_BACK_LEFT",
            (robot_state.x_m + px * (0.85 * lat) - ux * 0.4, robot_state.y_m + py * (0.85 * lat) - uy * 0.4),
        ),
        (
            "CURVED_LATERAL_BACK_RIGHT",
            (robot_state.x_m - px * (0.85 * lat) - ux * 0.4, robot_state.y_m - py * (0.85 * lat) - uy * 0.4),
        ),
    ]

    static_boxes = [
        (o.x_m, o.y_m, o.x_m + o.width_m, o.y_m + o.height_m)
        for o in static_obstacles
    ]
    active_sim = [o for o in sim_objects if o.motion.active] if sim_objects else []
    obs_sim_list = (
        [o for o in observation_frame.obstacles if o.is_valid and o.geometry is not None]
        if observation_frame is not None
        else []
    )
    has_sim = bool(obs_sim_list) or bool(active_sim)

    sim_steps = max(1, round(dwa_cfg.waiting_horizon_s / dt_s))
    c_dyn_preds = (
        _build_causal_preds_from_frame(
            observation_frame, dt_s, sim_steps + 5, uncertainty_growth_rate_mps
        )
        if observation_frame is not None
        else _causal_const_vel_predictions(dynamic_obstacles, dt_s, sim_steps + 5)
    )

    max_a = dwa_cfg.max_linear_accel_mps2
    max_alpha = dwa_cfg.max_angular_accel_radps2
    v_max = robot_cfg.max_linear_speed_mps
    w_max = robot_cfg.max_angular_speed_radps

    best_result: HoldingManeuverResult | None = None
    best_score = -math.inf

    for mtype, target_xy in candidate_specs:
        tx, ty = target_xy
        # Quick static/boundary check on target pose itself
        target_static_c = _point_clearance(
            tx, ty, robot_cfg.radius_m, world_cfg, static_obstacles, [], static_boxes=static_boxes
        )
        if target_static_c < dwa_cfg.safety_margin_m - 1e-6:
            continue

        # Simulate closed-loop unicycle rollout using exact Robot.step acceleration limits
        x = robot_state.x_m
        y = robot_state.y_m
        th = robot_state.theta_rad
        v = robot_state.v_mps
        w = robot_state.omega_radps

        traj_xy: list[tuple[float, float]] = [(x, y)]
        first_cmd: tuple[float, float] | None = None
        min_inst = math.inf
        min_swept = math.inf
        min_brake_c: float | None = None
        min_brake_t: float | None = None
        valid = True
        arrived = False

        # Validate t=0 clearance
        dyn_0 = [preds[0] for preds in c_dyn_preds]
        c0 = _point_clearance(
            x, y, robot_cfg.radius_m, world_cfg, static_obstacles, dyn_0, static_boxes=static_boxes
        )
        if has_sim:
            rg0 = CircleGeometry(x, y, robot_cfg.radius_m)
            if obs_sim_list and observation_frame is not None:
                for obj in obs_sim_list:
                    og0, pad0 = obj.predict_causal_geometry_at(
                        observation_frame.sim_time_s, 0.0, uncertainty_growth_rate_mps
                    )
                    oc0 = compute_geometry_distance(rg0, og0).surface_clearance_m - pad0
                    if oc0 < c0:
                        c0 = oc0
            else:
                for obj in active_sim:
                    oc0 = compute_geometry_distance(rg0, obj.predict_geometry_at(0.0)).surface_clearance_m
                    if oc0 < c0:
                        c0 = oc0
        if c0 < dwa_cfg.safety_margin_m:
            continue
        min_inst = c0
        min_swept = c0

        for step_k in range(sim_steps):
            dist_to_target = math.hypot(tx - x, ty - y)
            if dist_to_target <= 0.15:
                arrived = True

            if arrived:
                v_des = 0.0
                w_des = 0.0
            else:
                heading_to_target = math.atan2(ty - y, tx - x)
                heading_err = wrap_angle_rad(heading_to_target - th)
                w_des = max(-w_max, min(w_max, 3.0 * heading_err))
                if abs(heading_err) > 0.75:
                    # Curved turn with controlled forward speed
                    v_des = min(0.75, v_max)
                else:
                    # Approach and decelerate smoothly to rest
                    v_brake_cap = math.sqrt(max(0.0, 2.0 * max_a * max(0.0, dist_to_target - 0.05)))
                    v_des = min(1.2, v_max, v_brake_cap)

            dv = max(-max_a * dt_s, min(max_a * dt_s, v_des - v))
            dw = max(-max_alpha * dt_s, min(max_alpha * dt_s, w_des - w))
            v = max(0.0, min(v_max, v + dv))
            w = max(-w_max, min(w_max, w + dw))

            if first_cmd is None:
                first_cmd = (v, w)

            px_r, py_r = x, y
            mid_th = th + 0.5 * w * dt_s
            x += v * math.cos(mid_th) * dt_s
            y += v * math.sin(mid_th) * dt_s
            th = wrap_angle_rad(th + w * dt_s)
            traj_xy.append((x, y))

            t_next = (step_k + 1) * dt_s
            dyn_cur = [
                preds[min(step_k + 1, len(preds) - 1)]
                for preds in c_dyn_preds
            ]
            c_pt = _point_clearance(
                x, y, robot_cfg.radius_m, world_cfg, static_obstacles, dyn_cur, static_boxes=static_boxes
            )
            if has_sim:
                rg = CircleGeometry(x, y, robot_cfg.radius_m)
                if obs_sim_list and observation_frame is not None:
                    for obj in obs_sim_list:
                        og, pad_m = obj.predict_causal_geometry_at(
                            observation_frame.sim_time_s, t_next, uncertainty_growth_rate_mps
                        )
                        oc = compute_geometry_distance(rg, og).surface_clearance_m - pad_m
                        if oc < c_pt:
                            c_pt = oc
                else:
                    for obj in active_sim:
                        oc = compute_geometry_distance(rg, obj.predict_geometry_at(t_next)).surface_clearance_m
                        if oc < c_pt:
                            c_pt = oc
            if c_pt < min_inst:
                min_inst = c_pt
            if c_pt < min_swept:
                min_swept = c_pt

            for preds in c_dyn_preds:
                bx0, by0, orad0 = preds[min(step_k, len(preds) - 1)]
                bx1, by1, orad1 = preds[min(step_k + 1, len(preds) - 1)]
                sc = _swept_circle_clearance_m(
                    px_r, py_r, x, y, bx0, by0, bx1, by1, robot_cfg.radius_m + max(orad0, orad1)
                )
                if sc < min_swept:
                    min_swept = sc

            if min_inst < dwa_cfg.safety_margin_m or min_swept < dwa_cfg.safety_margin_m:
                valid = False
                break

            # Validate braking trajectory from first step and peak speed step
            if step_k == 0 and abs(v) > 1e-6:
                b_poses, b_times = simulate_braking_trajectory(
                    (x, y, th),
                    v,
                    dwa_cfg.predict_dt_s,
                    dwa_cfg.max_brake_decel_mps2,
                    dwa_cfg.reaction_time_s,
                )
                if len(b_poses) > 1:
                    min_brake_c = math.inf
                    min_brake_t = t_next
                    for bi in range(1, len(b_poses)):
                        bx0, by0, _ = b_poses[bi - 1]
                        bx1, by1, _ = b_poses[bi]
                        tb = t_next + b_times[bi]
                        idx_b0 = min(round((t_next + b_times[bi - 1]) / dt_s), len(c_dyn_preds[0]) - 1) if c_dyn_preds else 0
                        idx_b1 = min(round(tb / dt_s), len(c_dyn_preds[0]) - 1) if c_dyn_preds else 0
                        dyn_b1 = [preds[idx_b1] for preds in c_dyn_preds]
                        cb = _point_clearance(
                            bx1, by1, robot_cfg.radius_m, world_cfg, static_obstacles, dyn_b1, static_boxes=static_boxes
                        )
                        if has_sim:
                            rgb = CircleGeometry(bx1, by1, robot_cfg.radius_m)
                            if obs_sim_list and observation_frame is not None:
                                for obj in obs_sim_list:
                                    ogb, padb = obj.predict_causal_geometry_at(
                                        observation_frame.sim_time_s, tb, uncertainty_growth_rate_mps
                                    )
                                    ocb = compute_geometry_distance(rgb, ogb).surface_clearance_m - padb
                                    if ocb < cb:
                                        cb = ocb
                            else:
                                for obj in active_sim:
                                    ocb = compute_geometry_distance(rgb, obj.predict_geometry_at(tb)).surface_clearance_m
                                    if ocb < cb:
                                        cb = ocb
                        if cb < min_brake_c:
                            min_brake_c = cb
                            min_brake_t = tb
                        for preds in c_dyn_preds:
                            ox0, oy0, orad0 = preds[idx_b0]
                            ox1, oy1, orad1 = preds[idx_b1]
                            sb = _swept_circle_clearance_m(
                                bx0, by0, bx1, by1, ox0, oy0, ox1, oy1, robot_cfg.radius_m + max(orad0, orad1)
                            )
                            if sb < min_brake_c:
                                min_brake_c = sb
                                min_brake_t = tb
                        if min_brake_c < dwa_cfg.safety_margin_m:
                            valid = False
                            break
                if not valid:
                    break

        if not valid or first_cmd is None or (abs(first_cmd[0]) < 1e-6 and abs(first_cmd[1]) < 1e-6):
            continue

        min_overall = min(min_inst, min_swept, min_brake_c if min_brake_c is not None else math.inf)
        if min_overall < dwa_cfg.safety_margin_m:
            continue

        final_dist = math.hypot(tx - x, ty - y)
        sc = min_overall - 0.20 * final_dist
        if sc > best_score:
            best_score = sc
            best_result = HoldingManeuverResult(
                holding_pose_m=(tx, ty),
                v_cmd_mps=first_cmd[0],
                omega_cmd_radps=first_cmd[1],
                min_clearance_m=min_overall,
                min_swept_clearance_m=min_swept,
                min_braking_clearance_m=min_brake_c,
                min_braking_time_s=min_brake_t,
                trajectory_xy=traj_xy,
                maneuver_type=mtype,
            )

    return best_result


def select_safe_holding_pose(
    robot_state: RobotState,
    lookahead_target: tuple[float, float],
    dt_s: float,
    world_cfg: WorldConfig,
    robot_cfg: RobotConfig,
    dwa_cfg: DWAConfig,
    static_obstacles: list[StaticObstacle],
    dynamic_obstacles: list[DynamicObstacle],
    sim_objects: list[SimObject] | None = None,
    observation_frame: ObservationFrame | None = None,
    uncertainty_growth_rate_mps: float = 0.0,
) -> tuple[tuple[float, float], tuple[float, float], float] | None:
    """Search for a safe reachable holding pose via lateral offsets.

    Returns (holding_pose_xy, (v_cmd, omega_cmd), min_clearance_m) or None.
    """
    res = plan_lateral_holding_maneuver(
        robot_state,
        lookahead_target,
        dt_s,
        world_cfg,
        robot_cfg,
        dwa_cfg,
        static_obstacles,
        dynamic_obstacles,
        sim_objects=sim_objects,
        observation_frame=observation_frame,
        uncertainty_growth_rate_mps=uncertainty_growth_rate_mps,
    )
    if res is None:
        return None
    return (res.holding_pose_m, (res.v_cmd_mps, res.omega_cmd_radps), res.min_clearance_m)


class DWAPlanner:
    """Real-time Dynamic Window Approach local planner tracking a global A* path."""

    def __init__(
        self,
        world_cfg: WorldConfig,
        robot_cfg: RobotConfig,
        dwa_cfg: DWAConfig,
        observation_cfg: ObservationConfig = ObservationConfig(),
    ) -> None:
        self._world_cfg = world_cfg
        self._robot_cfg = robot_cfg
        self._dwa_cfg = dwa_cfg
        self._observation_cfg = observation_cfg
        self._path_world_m: list[tuple[float, float]] = []
        self._target_reached: bool = False
        self._safety_events: list[str] = []
        self._safety_margin_fallback_count: int = 0
        self._collision_risk_fallback_count: int = 0
        self._stalled_time_s: float = 0.0
        self._fallback_duration_s: float = 0.0
        self._best_goal_dist_m: float = math.inf
        self._progress_timer_s: float = 0.0
        self._blocked: bool = False
        self._deadlock_state: DeadlockState = DeadlockState.NONE
        self._corridor_manager = CorridorPassingManager(
            world_cfg, robot_cfg, dwa_cfg, observation_cfg
        )
        self._proactive_replan_enabled_for_eval: bool = True
        self._static_grid_cache: object | None = None
        self._static_grid_sig: tuple[int, ...] | None = None

    @property
    def corridor_manager(self) -> CorridorPassingManager:
        """Return the causal corridor assessment & passing manager."""
        return self._corridor_manager

    def _get_static_grid(self, static_obstacles: list[StaticObstacle]) -> object:
        sig = tuple(
            hash((o.x_m, o.y_m, o.width_m, o.height_m)) for o in static_obstacles
        )
        if self._static_grid_cache is None or self._static_grid_sig != sig:
            self._static_grid_cache = build_static_occupancy_grid(
                self._world_cfg, static_obstacles
            )
            self._static_grid_sig = sig
        return self._static_grid_cache

    @property
    def has_path(self) -> bool:
        """Return True if a global A* path is loaded."""
        return len(self._path_world_m) > 0

    @property
    def target_reached(self) -> bool:
        """Return True once the robot is within goal tolerance."""
        return self._target_reached

    @property
    def blocked(self) -> bool:
        """Return True if DWA has detected an impassable blockage / deadlock."""
        return self._blocked

    @property
    def deadlock_state(self) -> DeadlockState:
        """Return the current explicit deadlock/fallback classification."""
        return self._deadlock_state

    @property
    def fallback_duration_s(self) -> float:
        """Return continuous time spent in fallback without goal progress."""
        return self._fallback_duration_s

    @property
    def safety_events(self) -> list[str]:
        """Return recorded safety-fallback event tags."""
        return list(self._safety_events)

    @property
    def fallback_count(self) -> int:
        """Return total number of safety-fallback activations."""
        return self._safety_margin_fallback_count + self._collision_risk_fallback_count

    @property
    def safety_margin_fallback_count(self) -> int:
        """Return count of fallbacks where safety margin was violated but collision-free trajectory existed."""
        return self._safety_margin_fallback_count

    @property
    def collision_risk_fallback_count(self) -> int:
        """Return count of fallbacks where all candidate trajectories predicted physical collision."""
        return self._collision_risk_fallback_count

    def _reset_fallback_tracking(self) -> None:
        self._stalled_time_s = 0.0
        self._fallback_duration_s = 0.0
        self._best_goal_dist_m = math.inf
        self._progress_timer_s = 0.0
        self._blocked = False
        self._deadlock_state = DeadlockState.NONE

    def set_path(self, path_world_m: list[tuple[float, float]]) -> None:
        """Set the global A* reference path in SI meters."""
        self._path_world_m = list(path_world_m)
        self._target_reached = False
        self._safety_events.clear()
        self._safety_margin_fallback_count = 0
        self._collision_risk_fallback_count = 0
        self._reset_fallback_tracking()

    def clear(self) -> None:
        """Clear the loaded reference path and safety event counters."""
        self._path_world_m = []
        self._target_reached = False
        self._safety_events.clear()
        self._safety_margin_fallback_count = 0
        self._collision_risk_fallback_count = 0
        self._reset_fallback_tracking()

    def reset(self) -> None:
        """Reset goal-reached state and safety event counters while preserving loaded path."""
        self._target_reached = False
        self._safety_events.clear()
        self._safety_margin_fallback_count = 0
        self._collision_risk_fallback_count = 0
        self._reset_fallback_tracking()

    def _should_wait_before_narrow_passage(
        self,
        robot_state: RobotState,
        lookahead_target: tuple[float, float],
        static_obstacles: list[StaticObstacle],
        dynamic_obstacles: list[DynamicObstacle],
    ) -> bool:
        """Return True if an oncoming dynamic obstacle occupies a narrow passage ahead
        with insufficient side clearance to pass safely, and the robot can wait outside.
        """
        if not static_obstacles or not dynamic_obstacles:
            return False

        dx = lookahead_target[0] - robot_state.x_m
        dy = lookahead_target[1] - robot_state.y_m
        dist = math.hypot(dx, dy)
        if dist <= 1e-6:
            return False
        ux, uy = dx / dist, dy / dist
        # Perpendicular unit vector
        px, py = -uy, ux

        for obs in dynamic_obstacles:
            if not obs.active:
                continue
            rel_x = obs.x_m - robot_state.x_m
            rel_y = obs.y_m - robot_state.y_m
            along = rel_x * ux + rel_y * uy
            lat = abs(rel_x * px + rel_y * py)
            # Only consider oncoming obstacles ahead along the path
            v_along = obs.vx_mps * ux + obs.vy_mps * uy
            if along <= 0.5 or along > 13.0 or lat > 1.2 or v_along >= -0.1:
                continue

            # Measure lateral clearance to static obstacles / boundaries at obstacle's position
            left_wall = math.inf
            right_wall = math.inf
            for probe_d in [0.2 * i for i in range(1, 21)]:
                lx = obs.x_m + px * probe_d
                ly = obs.y_m + py * probe_d
                if (
                    lx < 0.0
                    or lx > self._world_cfg.width_m
                    or ly < 0.0
                    or ly > self._world_cfg.height_m
                    or any(
                        s.x_m <= lx <= s.x_m + s.width_m
                        and s.y_m <= ly <= s.y_m + s.height_m
                        for s in static_obstacles
                    )
                ):
                    left_wall = min(left_wall, probe_d)
                    break
            for probe_d in [0.2 * i for i in range(1, 21)]:
                rx = obs.x_m - px * probe_d
                ry = obs.y_m - py * probe_d
                if (
                    rx < 0.0
                    or rx > self._world_cfg.width_m
                    or ry < 0.0
                    or ry > self._world_cfg.height_m
                    or any(
                        s.x_m <= rx <= s.x_m + s.width_m
                        and s.y_m <= ry <= s.y_m + s.height_m
                        for s in static_obstacles
                    )
                ):
                    right_wall = min(right_wall, probe_d)
                    break

            max_side_gap = max(left_wall, right_wall) - obs.radius_m
            required_side_gap = (
                2.0 * self._robot_cfg.radius_m
                + 2.0 * self._dwa_cfg.safety_margin_m
                + self._dwa_cfg.corridor_passage_margin_m
            )
            if max_side_gap < required_side_gap:
                # Check if robot is still in wider/safe staging area or far enough to wait
                rob_left = min(
                    (
                        d
                        for d in [0.2 * i for i in range(1, 21)]
                        if any(
                            s.x_m <= robot_state.x_m + px * d <= s.x_m + s.width_m
                            and s.y_m <= robot_state.y_m + py * d <= s.y_m + s.height_m
                            for s in static_obstacles
                        )
                    ),
                    default=math.inf,
                )
                rob_right = min(
                    (
                        d
                        for d in [0.2 * i for i in range(1, 21)]
                        if any(
                            s.x_m <= robot_state.x_m - px * d <= s.x_m + s.width_m
                            and s.y_m <= robot_state.y_m - py * d <= s.y_m + s.height_m
                            for s in static_obstacles
                        )
                    ),
                    default=math.inf,
                )
                # Wait only if robot is still outside the narrow walls and obstacle is within 9.5 m
                if min(rob_left, rob_right) > 1.6 and along < 9.5:
                    return True
        return False

    def compute_command(
        self,
        robot_state: RobotState,
        dt_s: float,
        static_obstacles: list[StaticObstacle],
        dynamic_obstacles: list[DynamicObstacle],
        sim_objects: list[SimObject] | None = None,
        observation_frame: ObservationFrame | None = None,
    ) -> DWAResult:
        """Select optimal collision-free (v_mps, omega_radps) via DWA."""
        if not self._path_world_m or dt_s <= 0.0:
            return DWAResult(0.0, 0.0, None, None)

        eff_robot = (
            RobotState(
                x_m=observation_frame.robot.x_m,
                y_m=observation_frame.robot.y_m,
                theta_rad=observation_frame.robot.theta_rad,
                v_mps=observation_frame.robot.v_mps,
                omega_radps=observation_frame.robot.omega_radps,
            )
            if observation_frame is not None
            else robot_state
        )

        goal_x, goal_y = self._path_world_m[-1]
        if (
            math.hypot(goal_x - eff_robot.x_m, goal_y - eff_robot.y_m)
            <= self._dwa_cfg.goal_tolerance_m
        ):
            self._target_reached = True
            return DWAResult(0.0, 0.0, None, None, target_reached=True)

        window = compute_dynamic_window(
            eff_robot.v_mps,
            eff_robot.omega_radps,
            dt_s,
            self._robot_cfg,
            self._dwa_cfg,
        )
        candidates = sample_velocities(
            window,
            self._dwa_cfg.linear_samples,
            self._dwa_cfg.angular_samples,
        )
        lookahead_target = _select_lookahead_target(
            eff_robot.x_m,
            eff_robot.y_m,
            self._path_world_m,
            self._dwa_cfg.lookahead_distance_m,
        )

        goal_dist = math.hypot(goal_x - eff_robot.x_m, goal_y - eff_robot.y_m)
        if goal_dist < self._best_goal_dist_m - self._dwa_cfg.progress_epsilon_m:
            self._best_goal_dist_m = goal_dist
            self._progress_timer_s = 0.0
        else:
            self._progress_timer_s += dt_s

        waiting_assessment = evaluate_waiting_zone_safety(
            robot_state=eff_robot,
            lookahead_target=lookahead_target,
            world_cfg=self._world_cfg,
            robot_cfg=self._robot_cfg,
            dwa_cfg=self._dwa_cfg,
            static_obstacles=static_obstacles,
            dynamic_obstacles=dynamic_obstacles,
            sim_objects=sim_objects,
            observation_frame=observation_frame,
            uncertainty_growth_rate_mps=self._observation_cfg.uncertainty_growth_rate_mps,
        )

        if waiting_assessment.state == WaitingZoneState.GATE_OCCUPIED_HOLD_SAFE:
            # Wait safely outside/before the narrow passage using physically feasible braking
            desired_theta = math.atan2(
                lookahead_target[1] - eff_robot.y_m,
                lookahead_target[0] - eff_robot.x_m,
            )
            err = wrap_angle_rad(desired_theta - eff_robot.theta_rad)
            wait_omega = max(
                window.omega_min_radps,
                min(window.omega_max_radps, 2.0 * err),
            )
            self._stalled_time_s = 0.0
            self._fallback_duration_s = 0.0
            self._blocked = False
            self._deadlock_state = DeadlockState.TEMPORARY_SAFE_WAIT
            return DWAResult(
                v_mps=window.v_min_mps,
                omega_radps=wait_omega,
                score=None,
                predicted_min_clearance_m=waiting_assessment.min_waiting_clearance_m,
                selected_trajectory=[],
                rejected_trajectories=[],
                candidate_trajectories=[],
                target_reached=False,
                safety_margin_satisfied=True,
                collision_free_exists=True,
                fallback_active=False,
                fallback_reason="WAITING_FOR_CORRIDOR_CLEARANCE",
                waiting_for_passage=True,
                blocked=False,
                status="WAITING",
                waiting_state=waiting_assessment.state.value,
                waiting_min_clearance_m=waiting_assessment.min_waiting_clearance_m,
                fallback_priority=1,
                fallback_duration_s=0.0,
                deadlock_state=self._deadlock_state.value,
            )

        proactive_triggered = False
        if self._proactive_replan_enabled_for_eval and self._dwa_cfg.proactive_replan_enabled:
            eff_obs_frame = observation_frame
            if eff_obs_frame is None:
                eff_obs_frame = ObservationFrame(
                    sim_time_s=0.0,
                    robot=ObservedRobotState(
                        x_m=eff_robot.x_m,
                        y_m=eff_robot.y_m,
                        theta_rad=eff_robot.theta_rad,
                        v_mps=eff_robot.v_mps,
                        omega_radps=eff_robot.omega_radps,
                        timestamp_s=0.0,
                    ),
                    obstacles=[
                        ObservedObstacle(
                            obstacle_id=o.obstacle_id,
                            x_m=o.x_m,
                            y_m=o.y_m,
                            radius_m=o.radius_m,
                            vx_mps=o.vx_mps,
                            vy_mps=o.vy_mps,
                            heading_rad=math.atan2(o.vy_mps, o.vx_mps)
                            if math.hypot(o.vx_mps, o.vy_mps) > 1e-6
                            else 0.0,
                            velocity_valid=True,
                            timestamp_s=0.0,
                            observation_age_s=0.0,
                            position_uncertainty_m=0.0,
                            velocity_uncertainty_mps=0.0,
                            is_valid=True,
                        )
                        for o in dynamic_obstacles
                        if o.active
                    ],
                )
            assessment = self._corridor_manager.assess_corridor(
                eff_robot,
                dt_s,
                self._path_world_m,
                static_obstacles,
                eff_obs_frame,
            )
            if assessment.trigger_replan:
                static_grid = self._get_static_grid(static_obstacles)

                def _eval_cand_path(
                    cand_path: list[tuple[float, float]],
                ) -> tuple[bool, float, float, float, float | None]:
                    temp_dwa = DWAPlanner(
                        self._world_cfg,
                        self._robot_cfg,
                        self._dwa_cfg,
                        self._observation_cfg,
                    )
                    temp_dwa._proactive_replan_enabled_for_eval = False
                    temp_dwa.set_path(cand_path)
                    r = temp_dwa.compute_command(
                        eff_robot,
                        dt_s,
                        static_obstacles,
                        dynamic_obstacles,
                        sim_objects=sim_objects,
                        observation_frame=eff_obs_frame,
                    )
                    return (
                        (not r.fallback_active) and r.safety_margin_satisfied,
                        r.v_mps,
                        r.omega_radps,
                        r.predicted_min_clearance_m if r.predicted_min_clearance_m is not None else -math.inf,
                        r.min_braking_clearance_m,
                    )

                best_path, _ = self._corridor_manager.generate_and_validate_candidates(
                    eff_obs_frame.sim_time_s,
                    eff_robot,
                    dt_s,
                    (goal_x, goal_y),
                    static_grid,  # type: ignore[arg-type]
                    self._world_cfg.grid_resolution_m,
                    static_obstacles,
                    eff_obs_frame,
                    _eval_cand_path,
                )
                if best_path is not None:
                    self._path_world_m = list(best_path)
                    proactive_triggered = True
                    lookahead_target = _select_lookahead_target(
                        eff_robot.x_m,
                        eff_robot.y_m,
                        self._path_world_m,
                        self._dwa_cfg.lookahead_distance_m,
                    )

        pred_steps = max(1, round(self._dwa_cfg.predict_time_s / self._dwa_cfg.predict_dt_s))
        has_first_substep = 1e-9 < dt_s < self._dwa_cfg.predict_dt_s - 1e-9
        first_step_dt = dt_s if has_first_substep else None
        traj_times_s: list[float] = [0.0]
        if has_first_substep:
            traj_times_s.append(dt_s)
        for s_idx in range(1, pred_steps + 1):
            traj_times_s.append(s_idx * self._dwa_cfg.predict_dt_s)
        dyn_pred_dt_s = dt_s if has_first_substep else self._dwa_cfg.predict_dt_s

        max_stop_time = self._dwa_cfg.reaction_time_s + (
            self._robot_cfg.max_linear_speed_mps / max(1e-6, self._dwa_cfg.max_brake_decel_mps2)
        )
        horizon_obs_steps = max(0, math.ceil(traj_times_s[-1] / max(1e-6, dyn_pred_dt_s)))
        max_stop_obs_steps = max(1, math.ceil(max_stop_time / max(1e-6, dyn_pred_dt_s)))
        growth_rate = self._observation_cfg.uncertainty_growth_rate_mps
        if observation_frame is not None:
            causal_dyn_preds = _build_causal_preds_from_frame(
                observation_frame,
                dyn_pred_dt_s,
                horizon_obs_steps + max_stop_obs_steps,
                growth_rate,
            )
            observed_sim_objects = [
                o for o in observation_frame.obstacles if o.is_valid and o.geometry is not None
            ]
            obs_sim_t = observation_frame.sim_time_s
        else:
            causal_dyn_preds = _causal_const_vel_predictions(
                dynamic_obstacles,
                dyn_pred_dt_s,
                horizon_obs_steps + max_stop_obs_steps,
            )
            observed_sim_objects = None
            obs_sim_t = 0.0

        static_boxes = [
            (o.x_m, o.y_m, o.x_m + o.width_m, o.y_m + o.height_m)
            for o in static_obstacles
        ]
        precomputed_dyn_at_steps = [
            [
                preds[min(round(t_s / max(1e-9, dyn_pred_dt_s)), len(preds) - 1)]
                for preds in causal_dyn_preds
            ]
            for t_s in traj_times_s
        ]
        active_sim_objects = (
            [o for o in sim_objects if o.motion.active]
            if (sim_objects and observation_frame is None)
            else []
        )
        simobj_pred_cache: dict[float, list[object]] = {}
        if observed_sim_objects:
            for t_s in traj_times_s:
                simobj_pred_cache[round(t_s, 9)] = [
                    obj.predict_causal_geometry_at(obs_sim_t, t_s, growth_rate)
                    for obj in observed_sim_objects
                ]
        elif active_sim_objects:
            for t_s in traj_times_s:
                simobj_pred_cache[round(t_s, 9)] = [
                    (obj.predict_geometry_at(t_s), 0.0) for obj in active_sim_objects
                ]

        best_score = -math.inf
        best_cmd: tuple[float, float] | None = None
        best_traj_xy: list[tuple[float, float]] = []
        best_clearance: float | None = None
        best_brake_clearance: float | None = None
        best_brake_time: float | None = None

        evaluated_candidates: list[
            tuple[float, float, float, list[tuple[float, float]], float | None, float | None]
        ] = []

        candidate_xy_list: list[list[tuple[float, float]]] = []
        rejected_xy_list: list[list[tuple[float, float]]] = []

        for v, omega in candidates:
            traj = predict_trajectory(
                eff_robot,
                v,
                omega,
                self._dwa_cfg.predict_time_s,
                self._dwa_cfg.predict_dt_s,
                first_step_dt_s=first_step_dt,
            )
            traj_xy = [(p[0], p[1]) for p in traj]
            candidate_xy_list.append(traj_xy)

            is_safe, min_clear, brake_clear, brake_time = evaluate_trajectory_with_braking(
                traj,
                self._dwa_cfg.predict_dt_s,
                self._robot_cfg.radius_m,
                self._dwa_cfg.safety_margin_m,
                self._world_cfg,
                static_obstacles,
                dynamic_obstacles,
                v_mps=v,
                max_brake_decel_mps2=self._dwa_cfg.max_brake_decel_mps2,
                reaction_time_s=self._dwa_cfg.reaction_time_s,
                sim_objects=active_sim_objects,
                simobj_pred_cache=simobj_pred_cache,
                static_boxes=static_boxes,
                precomputed_dyn_at_steps=precomputed_dyn_at_steps,
                traj_times_s=traj_times_s,
                dyn_pred_dt_s=dyn_pred_dt_s,
                stop_on_margin_violation=True,
                causal_dyn_preds=causal_dyn_preds,
                observed_sim_objects=observed_sim_objects,
                obs_sim_time_s=obs_sim_t,
                uncertainty_growth_rate_mps=growth_rate,
                causal_stop_safety_enabled=self._dwa_cfg.causal_stop_safety_enabled,
                causal_reversal_horizon_s=self._dwa_cfg.causal_reversal_horizon_s,
                causal_straight_reversal_horizon_s=self._dwa_cfg.causal_straight_reversal_horizon_s,
                causal_lateral_escape_threshold_m=self._dwa_cfg.causal_lateral_escape_threshold_m,
            )
            evaluated_candidates.append(
                (v, omega, min_clear, traj_xy, brake_clear, brake_time)
            )
            if not is_safe:
                rejected_xy_list.append(traj_xy)
                continue

            sc = score_trajectory(
                traj,
                v,
                min_clear,
                lookahead_target,
                self._path_world_m,
                self._robot_cfg.max_linear_speed_mps,
                self._dwa_cfg,
            )
            if sc > best_score:
                best_score = sc
                best_cmd = (v, omega)
                best_traj_xy = traj_xy
                best_clearance = min_clear
                best_brake_clearance = brake_clear
                best_brake_time = brake_time

        if best_cmd is None:
            self._fallback_duration_s += dt_s
            robot_geom_now = CircleGeometry(
                eff_robot.x_m, eff_robot.y_m, self._robot_cfg.radius_m
            )
            dyn_now = [preds[0] for preds in causal_dyn_preds]
            start_clear = _point_clearance(
                eff_robot.x_m,
                eff_robot.y_m,
                self._robot_cfg.radius_m,
                self._world_cfg,
                static_obstacles,
                dyn_now,
            )
            if observed_sim_objects:
                for obj in observed_sim_objects:
                    oc = compute_geometry_distance(
                        robot_geom_now,
                        obj.predict_causal_geometry_at(obs_sim_t, 0.0, growth_rate),  # type: ignore[arg-type]
                    ).surface_clearance_m
                    if oc < start_clear:
                        start_clear = oc
            elif active_sim_objects:
                for obj in active_sim_objects:
                    oc = compute_geometry_distance(
                        robot_geom_now, obj.geometry()
                    ).surface_clearance_m
                    if oc < start_clear:
                        start_clear = oc

            def _traj_end_clearance(traj_pts: list[tuple[float, float]]) -> float:
                ex, ey = traj_pts[-1]
                end_obs_idx = round(traj_times_s[-1] / max(1e-9, dyn_pred_dt_s))
                dyn_end = [
                    preds[min(end_obs_idx, len(preds) - 1)]
                    for preds in causal_dyn_preds
                ]
                ec = _point_clearance(
                    ex,
                    ey,
                    self._robot_cfg.radius_m,
                    self._world_cfg,
                    static_obstacles,
                    dyn_end,
                )
                if observed_sim_objects or active_sim_objects:
                    eg = CircleGeometry(ex, ey, self._robot_cfg.radius_m)
                    t_end = pred_steps * self._dwa_cfg.predict_dt_s
                    t_end_key = round(t_end, 9)
                    end_geoms = simobj_pred_cache.get(t_end_key)
                    if end_geoms is None:
                        if observed_sim_objects:
                            end_geoms = [
                                obj.predict_causal_geometry_at(obs_sim_t, t_end, growth_rate)
                                for obj in observed_sim_objects
                            ]
                        else:
                            end_geoms = [
                                (obj.predict_geometry_at(t_end), 0.0)
                                for obj in active_sim_objects
                            ]
                        simobj_pred_cache[t_end_key] = end_geoms
                    for obj_geom, pad_m in end_geoms:
                        oc = (
                            compute_geometry_distance(
                                eg, obj_geom  # type: ignore[arg-type]
                            ).surface_clearance_m
                            - pad_m
                        )
                        if oc < ec:
                            ec = oc
                return ec

            # Re-evaluate collision-free candidates with explicit recovery check
            # when already below safety_margin_m (Critical Rule: never reject
            # recovery solely because t=0 is below 0.15 m, while never worsening clearance).
            collision_free = [c for c in evaluated_candidates if c[2] > 0.0]
            recovery_candidates: list[
                tuple[float, float, float, list[tuple[float, float]], float | None, float | None, float]
            ] = []
            for v_c, w_c, clr_c, txy_c, bc_c, bt_c in collision_free:
                end_c = _traj_end_clearance(txy_c)
                if start_clear < self._dwa_cfg.safety_margin_m:
                    if clr_c >= start_clear - 1e-6 and end_c > start_clear + 1e-3:
                        recovery_candidates.append((v_c, w_c, clr_c, txy_c, bc_c, bt_c, end_c))

            chosen_cmd: tuple[float, float] | None = None
            chosen_clear: float | None = None
            chosen_traj: list[tuple[float, float]] = []
            chosen_brake_c: float | None = None
            chosen_brake_t: float | None = None
            chosen_priority: int = 5
            chosen_reason: str = "PHYSICAL_COLLISION_RISK"
            chosen_holding_pose: tuple[float, float] | None = None
            reverse_used = False
            cf_exists = len(collision_free) > 0

            # Priority 1: Safe standstill or controlled braking when holding pose is safe
            if (
                waiting_assessment.state
                in (
                    WaitingZoneState.GATE_OCCUPIED_HOLD_SAFE,
                    WaitingZoneState.GATE_CLEAR_HOLD_SAFE,
                )
                and waiting_assessment.min_waiting_clearance_m
                >= self._dwa_cfg.safety_margin_m
                and collision_free
            ):
                brake_cf = [
                    c for c in collision_free if abs(c[0] - window.v_min_mps) <= 1e-9
                ]
                if brake_cf:
                    p1 = max(brake_cf, key=lambda c: (round(c[2], 6), -abs(c[1])))
                    chosen_cmd = (p1[0], p1[1])
                    chosen_clear = p1[2]
                    chosen_traj = p1[3]
                    chosen_brake_c = p1[4]
                    chosen_brake_t = p1[5]
                    chosen_priority = 1
                    chosen_reason = "SAFE_STANDSTILL_BRAKING"

            # Priority 2: Safe clearance-increasing / highest-clearance maneuver
            if chosen_cmd is None and recovery_candidates:
                if start_clear < self._dwa_cfg.safety_margin_m:
                    p2 = max(
                        recovery_candidates,
                        key=lambda c: (round(c[2], 6), round(c[6], 5), -abs(c[1])),
                    )
                else:
                    p2 = max(
                        recovery_candidates,
                        key=lambda c: (round(c[2], 6), -abs(c[0]), -abs(c[1])),
                    )
                chosen_cmd = (p2[0], p2[1])
                chosen_clear = p2[2]
                chosen_traj = p2[3]
                chosen_brake_c = p2[4]
                chosen_brake_t = p2[5]
                chosen_priority = 2
                chosen_reason = "CLEARANCE_INCREASING_RECOVERY"
            def _find_reverse_candidates(
                require_margin_safe: bool = False,
            ) -> list[
                tuple[float, float, float, list[tuple[float, float]], float | None, float | None, float, bool]
            ]:
                rev_window = compute_dynamic_window(
                    eff_robot.v_mps,
                    eff_robot.omega_radps,
                    dt_s,
                    self._robot_cfg,
                    self._dwa_cfg,
                    allow_reverse=True,
                )
                rev_samples = [
                    (v_r, w_r)
                    for v_r, w_r in sample_velocities(
                        rev_window,
                        self._dwa_cfg.linear_samples,
                        self._dwa_cfg.angular_samples,
                    )
                    if v_r < -1e-6
                ]
                out: list[
                    tuple[float, float, float, list[tuple[float, float]], float | None, float | None, float, bool]
                ] = []
                for v_r, w_r in rev_samples:
                    r_traj = predict_trajectory(
                        eff_robot,
                        v_r,
                        w_r,
                        self._dwa_cfg.predict_time_s,
                        self._dwa_cfg.predict_dt_s,
                        first_step_dt_s=first_step_dt,
                    )
                    r_xy = [(p[0], p[1]) for p in r_traj]
                    r_safe, r_min_c, r_bc, r_bt = evaluate_trajectory_with_braking(
                        r_traj,
                        self._dwa_cfg.predict_dt_s,
                        self._robot_cfg.radius_m,
                        self._dwa_cfg.safety_margin_m,
                        self._world_cfg,
                        static_obstacles,
                        dynamic_obstacles,
                        v_mps=v_r,
                        max_brake_decel_mps2=self._dwa_cfg.max_brake_decel_mps2,
                        reaction_time_s=self._dwa_cfg.reaction_time_s,
                        sim_objects=active_sim_objects,
                        simobj_pred_cache=simobj_pred_cache,
                        static_boxes=static_boxes,
                        precomputed_dyn_at_steps=precomputed_dyn_at_steps,
                        traj_times_s=traj_times_s,
                        dyn_pred_dt_s=dyn_pred_dt_s,
                        stop_on_margin_violation=False,
                        causal_dyn_preds=causal_dyn_preds,
                        observed_sim_objects=observed_sim_objects,
                        obs_sim_time_s=obs_sim_t,
                        uncertainty_growth_rate_mps=growth_rate,
                    )
                    if r_min_c <= 0.0:
                        continue
                    if require_margin_safe:
                        if r_safe and (r_bc is None or r_bc >= self._dwa_cfg.safety_margin_m):
                            r_end_c = _traj_end_clearance(r_xy)
                            out.append((v_r, w_r, r_min_c, r_xy, r_bc, r_bt, r_end_c, True))
                    else:
                        r_end_c = _traj_end_clearance(r_xy)
                        if (
                            r_safe
                            or (
                                start_clear < self._dwa_cfg.safety_margin_m
                                and r_min_c >= start_clear - 1e-6
                                and r_end_c > start_clear + 1e-3
                            )
                            or (r_end_c > start_clear + 1e-3)
                        ):
                            out.append((v_r, w_r, r_min_c, r_xy, r_bc, r_bt, r_end_c, r_safe))
                return out

            # Conditional Priority 3 override when holding is unsafe: prefer a genuinely
            # margin-safe (>= 0.15 m) reverse recovery before accepting a margin-violating
            # forward fallback in Priority 2, provided the reverse end pose also remains
            # collision-free over the waiting horizon.
            if (
                chosen_cmd is None
                and waiting_assessment.state == WaitingZoneState.GATE_CLEAR_HOLD_UNSAFE
            ):
                safe_rev_all = _find_reverse_candidates(require_margin_safe=True)
                safe_rev: list[
                    tuple[float, float, float, list[tuple[float, float]], float | None, float | None, float, bool]
                ] = []
                if safe_rev_all:
                    w_dt = max(0.05, self._dwa_cfg.waiting_dt_s)
                    w_steps = max(1, round(self._dwa_cfg.waiting_horizon_s / w_dt))
                    w_dyn_preds = (
                        _build_causal_preds_from_frame(
                            observation_frame,
                            w_dt,
                            w_steps,
                            growth_rate,
                        )
                        if observation_frame is not None
                        else _causal_const_vel_predictions(dynamic_obstacles, w_dt, w_steps)
                    )
                    t_end_r = traj_times_s[-1]
                    for cand in safe_rev_all:
                        rx, ry = cand[3][-1]
                        wait_ok = True
                        for step_w in range(w_steps + 1):
                            if step_w * w_dt < t_end_r - 1e-6:
                                continue
                            for preds in w_dyn_preds:
                                ox, oy, orad = preds[min(step_w, len(preds) - 1)]
                                if (
                                    math.hypot(rx - ox, ry - oy)
                                    - self._robot_cfg.radius_m
                                    - orad
                                    < self._dwa_cfg.safety_margin_m
                                ):
                                    wait_ok = False
                                    break
                            if not wait_ok:
                                break
                        if wait_ok:
                            safe_rev.append(cand)
                if safe_rev:
                    cf_exists = True
                    p3_safe = max(
                        safe_rev,
                        key=lambda c: (round(c[2], 6), round(c[6], 5), -abs(c[1])),
                    )
                    chosen_cmd = (p3_safe[0], p3_safe[1])
                    chosen_clear = p3_safe[2]
                    chosen_traj = p3_safe[3]
                    chosen_brake_c = p3_safe[4]
                    chosen_brake_t = p3_safe[5]
                    chosen_priority = 3
                    chosen_reason = "REVERSE_RECOVERY"
                    reverse_used = True

            # Conditional Priority 4 lateral holding pose evaluation when GATE_CLEAR_HOLD_UNSAFE
            # and no margin-safe reverse recovery exists, before Priority 2 margin-violating fallback.
            if (
                chosen_cmd is None
                and waiting_assessment.state == WaitingZoneState.GATE_CLEAR_HOLD_UNSAFE
            ):
                lat_hold = plan_lateral_holding_maneuver(
                    eff_robot,
                    lookahead_target,
                    dt_s,
                    self._world_cfg,
                    self._robot_cfg,
                    self._dwa_cfg,
                    static_obstacles,
                    dynamic_obstacles,
                    sim_objects=active_sim_objects,
                    observation_frame=observation_frame,
                    uncertainty_growth_rate_mps=growth_rate,
                )
                if lat_hold is not None:
                    chosen_holding_pose = lat_hold.holding_pose_m
                    chosen_cmd = (lat_hold.v_cmd_mps, lat_hold.omega_cmd_radps)
                    chosen_clear = lat_hold.min_clearance_m
                    chosen_traj = lat_hold.trajectory_xy
                    chosen_brake_c = lat_hold.min_braking_clearance_m
                    chosen_brake_t = lat_hold.min_braking_time_s
                    chosen_priority = 4
                    chosen_reason = "LATERAL_HOLDING_POSE"
                    cf_exists = True
                    reverse_used = False

            if (
                chosen_cmd is None
                and collision_free
                and waiting_assessment.state != WaitingZoneState.HOLD_UNSAFE_EVADE
                and start_clear >= self._dwa_cfg.safety_margin_m
            ):
                non_accel_cf = [
                    c for c in collision_free if c[0] <= eff_robot.v_mps + 1e-6
                ]
                cf_pool = non_accel_cf if non_accel_cf else collision_free
                p2_cf = max(
                    cf_pool,
                    key=lambda c: (
                        1 if (c[4] is None or c[4] > 0.0) else 0,
                        round(c[2], 4),
                        -max(0.0, c[0]),
                        -abs(c[0]),
                        -abs(c[1]),
                    ),
                )
                chosen_cmd = (p2_cf[0], p2_cf[1])
                chosen_clear = p2_cf[2]
                chosen_traj = p2_cf[3]
                chosen_brake_c = p2_cf[4]
                chosen_brake_t = p2_cf[5]
                chosen_priority = 2
                chosen_reason = "SAFETY_MARGIN_VIOLATION"

            # Priority 3: Reverse recovery (when gate-evade is required, or no forward
            # collision-free/recovery candidate exists)
            if chosen_cmd is None:
                rev_valid = _find_reverse_candidates(require_margin_safe=False)

                if rev_valid:
                    cf_exists = True
                    p3 = max(
                        rev_valid,
                        key=lambda c: (1 if c[7] else 0, round(c[6], 5), round(c[2], 5)),
                    )
                    chosen_cmd = (p3[0], p3[1])
                    chosen_clear = p3[2]
                    chosen_traj = p3[3]
                    chosen_brake_c = p3[4]
                    chosen_brake_t = p3[5]
                    chosen_priority = 3
                    chosen_reason = "REVERSE_RECOVERY"
                    reverse_used = True

            # Priority 4: Alternative safe holding pose + persistent fallback / deadlock
            if chosen_cmd is None and waiting_assessment.state in (
                WaitingZoneState.HOLD_UNSAFE_EVADE,
                WaitingZoneState.GATE_CLEAR_HOLD_UNSAFE,
            ):
                hold_choice = select_safe_holding_pose(
                    eff_robot,
                    lookahead_target,
                    dt_s,
                    self._world_cfg,
                    self._robot_cfg,
                    self._dwa_cfg,
                    static_obstacles,
                    dynamic_obstacles,
                    sim_objects=sim_objects,
                    observation_frame=observation_frame,
                    uncertainty_growth_rate_mps=growth_rate,
                )
                if hold_choice is not None:
                    chosen_holding_pose, chosen_cmd, chosen_clear = hold_choice
                    chosen_priority = 4
                    chosen_reason = "ALTERNATIVE_HOLDING_POSE"
                    cf_exists = True
                    reverse_used = chosen_cmd[0] < -1e-6

            # If still no Priority 1-4 recovery command, fall back to highest-clearance
            # collision-free standstill/braking or Priority 5 emergency braking.
            if chosen_cmd is None and collision_free:
                non_accel_cf = [
                    c for c in collision_free if c[0] <= eff_robot.v_mps + 1e-6
                ]
                cf_pool = non_accel_cf if non_accel_cf else collision_free
                p_cf = max(
                    cf_pool,
                    key=lambda c: (
                        1 if (c[4] is None or c[4] > 0.0) else 0,
                        round(c[2], 4),
                        -max(0.0, c[0]),
                        -abs(c[0]),
                        -abs(c[1]),
                    ),
                )
                chosen_cmd = (p_cf[0], p_cf[1])
                chosen_clear = p_cf[2]
                chosen_traj = p_cf[3]
                chosen_brake_c = p_cf[4]
                chosen_brake_t = p_cf[5]
                chosen_priority = 1 if abs(p_cf[0]) < 0.05 else 2
                chosen_reason = "SAFETY_MARGIN_VIOLATION"

            if chosen_cmd is not None:
                if abs(chosen_cmd[0]) < 0.05:
                    self._stalled_time_s += dt_s
                else:
                    self._stalled_time_s = 0.0

                persistent_deadlock = (
                    self._fallback_duration_s >= self._dwa_cfg.fallback_replan_timeout_s
                    and self._progress_timer_s >= self._dwa_cfg.fallback_replan_timeout_s
                )
                self._blocked = (
                    self._stalled_time_s >= self._dwa_cfg.deadlock_timeout_s
                    or persistent_deadlock
                )
                self._deadlock_state = (
                    DeadlockState.PERSISTENT_FALLBACK
                    if self._blocked
                    else DeadlockState.NONE
                )
                self._safety_margin_fallback_count += 1
                self._safety_events.append(chosen_reason)
                margin_ok = (
                    chosen_clear is not None
                    and chosen_clear >= self._dwa_cfg.safety_margin_m
                )
                return DWAResult(
                    v_mps=chosen_cmd[0],
                    omega_radps=chosen_cmd[1],
                    score=None,
                    predicted_min_clearance_m=chosen_clear,
                    selected_trajectory=chosen_traj,
                    rejected_trajectories=rejected_xy_list,
                    candidate_trajectories=candidate_xy_list,
                    target_reached=False,
                    safety_margin_satisfied=margin_ok,
                    collision_free_exists=cf_exists,
                    fallback_active=True,
                    fallback_reason=chosen_reason,
                    blocked=self._blocked,
                    status="BLOCKED" if self._blocked else "FALLBACK",
                    min_braking_clearance_m=chosen_brake_c,
                    min_braking_clearance_time_s=chosen_brake_t,
                    waiting_state=waiting_assessment.state.value,
                    waiting_min_clearance_m=waiting_assessment.min_waiting_clearance_m,
                    holding_pose_m=chosen_holding_pose,
                    reverse_active=reverse_used,
                    fallback_priority=chosen_priority,
                    fallback_duration_s=self._fallback_duration_s,
                    deadlock_state=self._deadlock_state.value,
                )

            # Priority 5: All forward & reverse candidates predict physical collision.
            braking_candidates = [
                c
                for c in evaluated_candidates
                if abs(c[0] - window.v_min_mps) <= 1e-9
            ]
            if braking_candidates:
                chosen_brake = max(
                    braking_candidates,
                    key=lambda c: (c[2], abs(c[1])),
                )
                brake_v = chosen_brake[0]
                brake_omega = chosen_brake[1]
                brake_clear = chosen_brake[2]
            else:
                brake_v = window.v_min_mps
                brake_omega = 0.0
                brake_clear = None

            self._stalled_time_s += dt_s
            self._blocked = True
            self._deadlock_state = DeadlockState.NO_SAFE_MANEUVER
            self._collision_risk_fallback_count += 1
            self._safety_events.append("PHYSICAL_COLLISION_RISK")
            return DWAResult(
                v_mps=brake_v,
                omega_radps=brake_omega,
                score=None,
                predicted_min_clearance_m=brake_clear,
                selected_trajectory=[],
                rejected_trajectories=rejected_xy_list,
                candidate_trajectories=candidate_xy_list,
                target_reached=False,
                safety_margin_satisfied=False,
                collision_free_exists=False,
                fallback_active=True,
                fallback_reason="PHYSICAL_COLLISION_RISK",
                blocked=True,
                status="BLOCKED",
                waiting_state=waiting_assessment.state.value,
                waiting_min_clearance_m=waiting_assessment.min_waiting_clearance_m,
                holding_pose_m=None,
                reverse_active=False,
                fallback_priority=5,
                fallback_duration_s=self._fallback_duration_s,
                deadlock_state=self._deadlock_state.value,
            )

        self._fallback_duration_s = 0.0
        self._deadlock_state = DeadlockState.NONE
        if abs(best_cmd[0]) < 0.05:
            self._stalled_time_s += dt_s
        else:
            self._stalled_time_s = 0.0
        self._blocked = self._stalled_time_s >= self._dwa_cfg.deadlock_timeout_s
        return DWAResult(
            v_mps=best_cmd[0],
            omega_radps=best_cmd[1],
            score=best_score,
            predicted_min_clearance_m=best_clearance,
            selected_trajectory=best_traj_xy,
            rejected_trajectories=rejected_xy_list,
            candidate_trajectories=candidate_xy_list,
            target_reached=False,
            safety_margin_satisfied=True,
            collision_free_exists=True,
            fallback_active=False,
            fallback_reason=None,
            blocked=self._blocked,
            status="BLOCKED" if self._blocked else "NORMAL",
            min_braking_clearance_m=best_brake_clearance,
            min_braking_clearance_time_s=best_brake_time,
            waiting_state=waiting_assessment.state.value,
            waiting_min_clearance_m=waiting_assessment.min_waiting_clearance_m,
            holding_pose_m=None,
            reverse_active=False,
            fallback_priority=None,
            fallback_duration_s=0.0,
            deadlock_state=self._deadlock_state.value,
            proactive_replan_triggered=proactive_triggered,
        )


def replan_global_path_around_obstacles(
    base_occupancy_grid: object,
    resolution_m: float,
    robot_x_m: float,
    robot_y_m: float,
    goal_x_m: float,
    goal_y_m: float,
    robot_radius_m: float,
    safety_margin_m: float,
    dynamic_obstacles: list[DynamicObstacle],
    allow_diagonal: bool = True,
    enforce_boundary_clearance: bool = True,
    sim_objects: list[SimObject] | None = None,
    observation_frame: ObservationFrame | None = None,
) -> tuple[bool, list[tuple[float, float]]]:
    """Trigger a controlled A* global replan preserving static inflation,
    segment-level safety validation, and geometry-aware rasterization of active
    dynamic obstacles and universal SimObjects (using current observed state only).
    """
    import numpy as np
    from algorithms.astar import (
        AStarPlanner,
        grid_to_world,
        inflate_occupancy_grid,
        is_segment_collision_free,
        rasterize_geometry_onto_grid,
        world_to_grid,
    )

    grid_copy = np.array(base_occupancy_grid, dtype=np.uint8, copy=True)
    rows, cols = grid_copy.shape

    if observation_frame is not None:
        for obs in observation_frame.obstacles:
            if not obs.is_valid:
                continue
            if obs.geometry is not None:
                rasterize_geometry_onto_grid(grid_copy, obs.geometry, resolution_m)
            else:
                rasterize_geometry_onto_grid(
                    grid_copy,
                    CircleGeometry(
                        obs.x_m,
                        obs.y_m,
                        obs.radius_m + obs.position_uncertainty_m,
                    ),
                    resolution_m,
                )
    else:
        for obs in dynamic_obstacles:
            if not obs.active:
                continue
            rasterize_geometry_onto_grid(
                grid_copy,
                CircleGeometry(obs.x_m, obs.y_m, obs.radius_m),
                resolution_m,
            )

        if sim_objects:
            for obj in sim_objects:
                if not obj.motion.active:
                    continue
                rasterize_geometry_onto_grid(grid_copy, obj.geometry(), resolution_m)

    inflated = inflate_occupancy_grid(
        grid_copy,
        resolution_m,
        robot_radius_m,
        safety_margin_m,
        enforce_boundary_clearance=enforce_boundary_clearance,
    )
    start_cell = world_to_grid(robot_x_m, robot_y_m, resolution_m, inflated.shape)
    if inflated[start_cell]:
        sr, sc = start_cell
        search_r = math.ceil((robot_radius_m + safety_margin_m) / resolution_m) + 2
        best_cell: tuple[int, int] | None = None
        best_dist = math.inf
        for dr in range(-search_r, search_r + 1):
            for dc in range(-search_r, search_r + 1):
                nr, nc = sr + dr, sc + dc
                if not (0 <= nr < rows and 0 <= nc < cols) or inflated[nr, nc]:
                    continue
                cand_m = grid_to_world(nr, nc, resolution_m)
                if not is_segment_collision_free(
                    (robot_x_m, robot_y_m),
                    cand_m,
                    grid_copy,
                    resolution_m,
                    max(0.0, robot_radius_m - 0.5 * resolution_m),
                    enforce_boundary_clearance=False,
                ):
                    continue
                d = math.hypot(cand_m[0] - robot_x_m, cand_m[1] - robot_y_m)
                if d < best_dist:
                    best_dist = d
                    best_cell = (nr, nc)
        if best_cell is None:
            return (False, [])
        start_cell = best_cell

    goal_cell = world_to_grid(goal_x_m, goal_y_m, resolution_m, inflated.shape)
    required_clearance_m = robot_radius_m + safety_margin_m

    result = AStarPlanner(resolution_m, allow_diagonal).plan(
        inflated,
        start_cell,
        goal_cell,
        raw_grid=grid_copy,
        required_clearance_m=required_clearance_m,
        enforce_boundary_clearance=enforce_boundary_clearance,
    )
    if result.success and result.path_world_m:
        return (True, list(result.path_world_m))
    return (False, [])
