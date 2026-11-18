"""Causal sensor observation, state estimation, and prediction pipeline (SI units).

Strictly separates ground-truth simulation state (including hidden scenario schedules)
from controller-visible timestamped observations. Velocity is estimated causally from
successive valid observations without reading future waypoints, reversal timers, or
STOP_AND_GO schedules.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
import math
from typing import TYPE_CHECKING

from config import ObservationConfig
from simulation.entities import RobotState, wrap_angle_rad
from simulation.geometry import (
    CapsuleGeometry,
    CircleGeometry,
    Geometry,
    OrientedRectGeometry,
    PolygonGeometry,
)

if TYPE_CHECKING:
    from simulation.dynamic_obstacles import DynamicObstacle
    from simulation.objects import SimObject


@dataclass(frozen=True)
class ObservedRobotState:
    """Controller-visible robot state at observation timestamp."""

    x_m: float
    y_m: float
    theta_rad: float
    v_mps: float
    omega_radps: float
    timestamp_s: float
    observation_age_s: float = 0.0
    position_uncertainty_m: float = 0.0


@dataclass(frozen=True)
class ObservedObstacle:
    """Controller-visible obstacle observation with causal state estimate & uncertainty."""

    obstacle_id: str
    x_m: float
    y_m: float
    radius_m: float
    vx_mps: float
    vy_mps: float
    heading_rad: float
    velocity_valid: bool
    timestamp_s: float
    observation_age_s: float
    position_uncertainty_m: float
    velocity_uncertainty_mps: float
    is_valid: bool
    geometry: Geometry | None = None
    angular_speed_radps: float = 0.0

    def predict_causal_positions(
        self,
        current_time_s: float,
        dt_s: float,
        steps: int,
        growth_rate_mps: float = 0.02,
    ) -> list[tuple[float, float, float]]:
        """Predict (x_m, y_m, effective_radius_m) using only observed state & constant velocity.

        Propagates from `self.timestamp_s` to `current_time_s + step * dt_s`.
        Uncertainty grows monotonically with observation age and prediction horizon.
        """
        out: list[tuple[float, float, float]] = []
        base_age = max(0.0, current_time_s - self.timestamp_s)
        for s_idx in range(max(0, steps) + 1):
            t_horizon = s_idx * dt_s
            dt_total = base_age + t_horizon
            px = self.x_m + self.vx_mps * dt_total
            py = self.y_m + self.vy_mps * dt_total
            eff_r = (
                self.radius_m
                + self.position_uncertainty_m
                + self.velocity_uncertainty_mps * dt_total
                + growth_rate_mps * t_horizon
            )
            out.append((px, py, eff_r))
        return out

    def predict_causal_geometry_at(
        self,
        current_time_s: float,
        t_offset_s: float,
        growth_rate_mps: float = 0.02,
    ) -> tuple[Geometry, float]:
        """Predict (geometry, uncertainty_pad_m) causally from latest observation."""
        base_age = max(0.0, current_time_s - self.timestamp_s)
        t_h = max(0.0, t_offset_s)
        dt_total = base_age + t_h
        dx = self.vx_mps * dt_total
        dy = self.vy_mps * dt_total
        d_th = self.angular_speed_radps * dt_total
        pad_m = (
            self.position_uncertainty_m
            + self.velocity_uncertainty_mps * dt_total
            + growth_rate_mps * t_h
        )
        g = self.geometry
        if g is None or isinstance(g, CircleGeometry):
            return (
                CircleGeometry(self.x_m + dx, self.y_m + dy, self.radius_m),
                pad_m,
            )
        if isinstance(g, OrientedRectGeometry):
            return (
                OrientedRectGeometry(
                    g.center_x_m + dx,
                    g.center_y_m + dy,
                    g.width_m,
                    g.length_m,
                    wrap_angle_rad(g.heading_rad + d_th),
                ),
                pad_m,
            )
        if isinstance(g, CapsuleGeometry):
            return (
                CapsuleGeometry(
                    g.center_x_m + dx,
                    g.center_y_m + dy,
                    g.length_m,
                    g.radius_m,
                    wrap_angle_rad(g.heading_rad + d_th),
                ),
                pad_m,
            )
        if isinstance(g, PolygonGeometry):
            cos_a = math.cos(d_th)
            sin_a = math.sin(d_th)
            cx = g.center_x_m
            cy = g.center_y_m
            verts = tuple(
                (
                    cx + dx + (vx - cx) * cos_a - (vy - cy) * sin_a,
                    cy + dy + (vx - cx) * sin_a + (vy - cy) * cos_a,
                )
                for vx, vy in g.vertices_m
            )
            return (PolygonGeometry(verts), pad_m)
        return (CircleGeometry(self.x_m + dx, self.y_m + dy, self.radius_m), pad_m)


@dataclass(frozen=True)
class ObservationFrame:
    """Complete controller-visible observation snapshot at `sim_time_s`."""

    sim_time_s: float
    robot: ObservedRobotState
    obstacles: list[ObservedObstacle]
    stale_obstacle_ids: list[str] = field(default_factory=list)
    estimation_time_ms: float = 0.0


@dataclass
class _TrackedObstacleHistory:
    last_x_m: float
    last_y_m: float
    last_radius_m: float
    last_timestamp_s: float
    est_vx_mps: float = 0.0
    est_vy_mps: float = 0.0
    est_heading_rad: float = 0.0
    est_angular_speed_radps: float = 0.0
    velocity_valid: bool = False
    last_geometry: Geometry | None = None


class ObstacleStateEstimator:
    """Causal finite-difference / EMA velocity estimator from timestamped observations."""

    def __init__(self, cfg: ObservationConfig) -> None:
        self._cfg = cfg
        self._tracks: dict[str, _TrackedObstacleHistory] = {}

    def reset(self) -> None:
        """Clear all tracked obstacle histories."""
        self._tracks.clear()

    def update(
        self,
        obstacle_id: str,
        x_m: float,
        y_m: float,
        radius_m: float,
        obs_timestamp_s: float,
        current_time_s: float,
        geometry: Geometry | None = None,
        heading_rad: float | None = None,
    ) -> ObservedObstacle | None:
        """Update track with a new timestamped observation; reject invalid timestamps."""
        if (
            math.isnan(obs_timestamp_s)
            or math.isinf(obs_timestamp_s)
            or obs_timestamp_s < 0.0
            or obs_timestamp_s > current_time_s + 1e-9
        ):
            return None

        age_s = max(0.0, current_time_s - obs_timestamp_s)
        is_valid = age_s <= self._cfg.max_observation_age_s + 1e-9
        prev = self._tracks.get(obstacle_id)

        if prev is None:
            # Newly detected obstacle: do NOT invent velocity from hidden ground truth.
            h0 = heading_rad if heading_rad is not None else 0.0
            track = _TrackedObstacleHistory(
                last_x_m=x_m,
                last_y_m=y_m,
                last_radius_m=radius_m,
                last_timestamp_s=obs_timestamp_s,
                est_vx_mps=0.0,
                est_vy_mps=0.0,
                est_heading_rad=h0,
                est_angular_speed_radps=0.0,
                velocity_valid=False,
                last_geometry=geometry,
            )
            self._tracks[obstacle_id] = track
            return ObservedObstacle(
                obstacle_id=obstacle_id,
                x_m=x_m,
                y_m=y_m,
                radius_m=radius_m,
                vx_mps=0.0,
                vy_mps=0.0,
                heading_rad=h0,
                velocity_valid=False,
                timestamp_s=obs_timestamp_s,
                observation_age_s=age_s,
                position_uncertainty_m=self._cfg.position_uncertainty_m,
                velocity_uncertainty_mps=self._cfg.initial_velocity_uncertainty_mps,
                is_valid=is_valid,
                geometry=geometry,
                angular_speed_radps=0.0,
            )

        if obs_timestamp_s < prev.last_timestamp_s - 1e-9:
            # Out-of-order / regressive timestamp rejected
            return None

        dt_obs = obs_timestamp_s - prev.last_timestamp_s
        if dt_obs > 1e-9:
            raw_vx = (x_m - prev.last_x_m) / dt_obs
            raw_vy = (y_m - prev.last_y_m) / dt_obs
            alpha = max(0.0, min(1.0, self._cfg.velocity_ema_alpha))
            if not prev.velocity_valid:
                est_vx, est_vy = raw_vx, raw_vy
            else:
                est_vx = alpha * raw_vx + (1.0 - alpha) * prev.est_vx_mps
                est_vy = alpha * raw_vy + (1.0 - alpha) * prev.est_vy_mps

            est_omega = 0.0
            if heading_rad is not None:
                d_h = wrap_angle_rad(heading_rad - prev.est_heading_rad)
                est_omega = d_h / dt_obs
                est_h = heading_rad
            else:
                est_h = math.atan2(est_vy, est_vx) if math.hypot(est_vx, est_vy) > 1e-6 else prev.est_heading_rad

            prev.last_x_m = x_m
            prev.last_y_m = y_m
            prev.last_radius_m = radius_m
            prev.last_timestamp_s = obs_timestamp_s
            prev.est_vx_mps = est_vx
            prev.est_vy_mps = est_vy
            prev.est_heading_rad = est_h
            prev.est_angular_speed_radps = est_omega
            prev.velocity_valid = True
            prev.last_geometry = geometry
        else:
            # Duplicate timestamp: avoid division by zero, keep existing state
            prev.last_x_m = x_m
            prev.last_y_m = y_m
            prev.last_radius_m = radius_m
            prev.last_geometry = geometry

        vel_unc = (
            self._cfg.velocity_uncertainty_mps
            if prev.velocity_valid
            else self._cfg.initial_velocity_uncertainty_mps
        )
        return ObservedObstacle(
            obstacle_id=obstacle_id,
            x_m=prev.last_x_m,
            y_m=prev.last_y_m,
            radius_m=prev.last_radius_m,
            vx_mps=prev.est_vx_mps,
            vy_mps=prev.est_vy_mps,
            heading_rad=prev.est_heading_rad,
            velocity_valid=prev.velocity_valid,
            timestamp_s=prev.last_timestamp_s,
            observation_age_s=age_s,
            position_uncertainty_m=self._cfg.position_uncertainty_m,
            velocity_uncertainty_mps=vel_unc,
            is_valid=is_valid,
            geometry=prev.last_geometry,
            angular_speed_radps=prev.est_angular_speed_radps,
        )

    def get_if_missing(
        self, obstacle_id: str, current_time_s: float
    ) -> ObservedObstacle | None:
        """Handle temporarily missing obstacle: return valid aged track or mark stale."""
        prev = self._tracks.get(obstacle_id)
        if prev is None:
            return None
        age_s = max(0.0, current_time_s - prev.last_timestamp_s)
        is_valid = age_s <= self._cfg.max_observation_age_s + 1e-9
        vel_unc = (
            self._cfg.velocity_uncertainty_mps
            if prev.velocity_valid
            else self._cfg.initial_velocity_uncertainty_mps
        )
        return ObservedObstacle(
            obstacle_id=obstacle_id,
            x_m=prev.last_x_m,
            y_m=prev.last_y_m,
            radius_m=prev.last_radius_m,
            vx_mps=prev.est_vx_mps if is_valid else 0.0,
            vy_mps=prev.est_vy_mps if is_valid else 0.0,
            heading_rad=prev.est_heading_rad,
            velocity_valid=prev.velocity_valid and is_valid,
            timestamp_s=prev.last_timestamp_s,
            observation_age_s=age_s,
            position_uncertainty_m=self._cfg.position_uncertainty_m,
            velocity_uncertainty_mps=vel_unc,
            is_valid=is_valid,
            geometry=prev.last_geometry,
            angular_speed_radps=prev.est_angular_speed_radps if is_valid else 0.0,
        )


@dataclass
class _RawSnapshot:
    timestamp_s: float
    robot: RobotState
    raw_obstacles: list[tuple[str, float, float, float, Geometry | None, float | None]]


class ObservationPipeline:
    """Causal sensor observation pipeline isolating controller from ground truth."""

    def __init__(self, cfg: ObservationConfig = ObservationConfig()) -> None:
        self._cfg = cfg
        self._estimator = ObstacleStateEstimator(cfg)
        self._buffer: deque[_RawSnapshot] = deque(maxlen=64)

    def reset(self) -> None:
        """Reset estimator and latency buffer."""
        self._estimator.reset()
        self._buffer.clear()

    def observe(
        self,
        sim_time_s: float,
        robot_state: RobotState,
        dynamic_obstacles: list[DynamicObstacle],
        sim_objects: list[SimObject] | None = None,
        visible_ids: set[str] | None = None,
    ) -> ObservationFrame:
        """Sample ground-truth state into a causal ObservationFrame (with optional latency)."""
        raw_obs: list[tuple[str, float, float, float, Geometry | None, float | None]] = []
        for o in dynamic_obstacles:
            if not o.active:
                continue
            if visible_ids is not None and o.obstacle_id not in visible_ids:
                continue
            raw_obs.append((o.obstacle_id, o.x_m, o.y_m, o.radius_m, None, None))
        if sim_objects:
            for obj in sim_objects:
                if not obj.motion.active:
                    continue
                if visible_ids is not None and obj.object_id not in visible_ids:
                    continue
                g = obj.geometry()
                r_approx = g.radius_m if isinstance(g, CircleGeometry) else 0.0
                raw_obs.append(
                    (
                        obj.object_id,
                        obj.motion.x_m,
                        obj.motion.y_m,
                        r_approx,
                        g,
                        obj.motion.heading_rad,
                    )
                )

        snap = _RawSnapshot(
            timestamp_s=sim_time_s,
            robot=RobotState(
                x_m=robot_state.x_m,
                y_m=robot_state.y_m,
                theta_rad=robot_state.theta_rad,
                v_mps=robot_state.v_mps,
                omega_radps=robot_state.omega_radps,
            ),
            raw_obstacles=raw_obs,
        )
        self._buffer.append(snap)

        # Select delayed snapshot according to configured observation_latency_s
        target_t = sim_time_s - max(0.0, self._cfg.observation_latency_s)
        chosen = self._buffer[0]
        for item in self._buffer:
            if item.timestamp_s <= target_t + 1e-9:
                chosen = item
            else:
                break

        obs_t = chosen.timestamp_s
        age_s = max(0.0, sim_time_s - obs_t)
        obs_robot = ObservedRobotState(
            x_m=chosen.robot.x_m,
            y_m=chosen.robot.y_m,
            theta_rad=chosen.robot.theta_rad,
            v_mps=chosen.robot.v_mps,
            omega_radps=chosen.robot.omega_radps,
            timestamp_s=obs_t,
            observation_age_s=age_s,
            position_uncertainty_m=self._cfg.position_uncertainty_m,
        )

        valid_obstacles: list[ObservedObstacle] = []
        stale_ids: list[str] = []
        seen_ids: set[str] = set()
        for oid, ox, oy, orad, ogeom, ohead in chosen.raw_obstacles:
            seen_ids.add(oid)
            est = self._estimator.update(
                oid, ox, oy, orad, obs_t, sim_time_s, geometry=ogeom, heading_rad=ohead
            )
            if est is None:
                continue
            if est.is_valid:
                valid_obstacles.append(est)
            else:
                stale_ids.append(oid)

        # Check any previously tracked obstacles missing from current frame
        for oid in list(self._estimator._tracks.keys()):
            if oid not in seen_ids:
                aged = self._estimator.get_if_missing(oid, sim_time_s)
                if aged is not None:
                    if aged.is_valid:
                        valid_obstacles.append(aged)
                    else:
                        stale_ids.append(oid)

        return ObservationFrame(
            sim_time_s=sim_time_s,
            robot=obs_robot,
            obstacles=valid_obstacles,
            stale_obstacle_ids=stale_ids,
        )
