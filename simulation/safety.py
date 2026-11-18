"""Real-time geometric safety clearance, TTC, and event analytics in SI units.

This module is strictly independent of Pygame and computes all clearances
from exact obstacle geometries (axis-aligned rectangles and circles).
"""

from dataclasses import dataclass
from enum import Enum, auto
import math

from config import SafetyThresholdConfig
from simulation.dynamic_obstacles import DynamicObstacle
from simulation.entities import RobotState, StaticObstacle


class SafetyLevel(Enum):
    """Surface-to-surface clearance classification categories."""

    SAFE = auto()       # clearance > 0.30 m
    CAUTION = auto()    # 0.15 m < clearance <= 0.30 m
    CRITICAL = auto()   # 0.0 m < clearance <= 0.15 m
    COLLISION = auto()  # clearance <= 0.0 m


@dataclass(frozen=True)
class SafetySnapshot:
    """Instantaneous safety measurement relative to the nearest obstacle."""

    nearest_obstacle_id: str | None
    nearest_obstacle_type: str | None  # "SABİT" or "HAREKETLİ"
    nearest_point_on_obstacle_m: tuple[float, float] | None
    center_to_center_distance_m: float | None
    surface_clearance_m: float | None
    required_safety_margin_m: float
    safety_level: SafetyLevel
    ttc_s: float | None


@dataclass(frozen=True)
class SafetyEvent:
    """Timestamped safety event entry for the analytics log."""

    timestamp_s: float
    message: str
    level: SafetyLevel

    def format_line(self) -> str:
        """Format as MM:SS.cc – MESAJ."""
        mins = int(self.timestamp_s // 60)
        secs = self.timestamp_s - mins * 60
        return f"{mins:02d}:{secs:05.2f} – {self.message}"


def classify_safety_level(
    surface_clearance_m: float | None,
    thresholds: SafetyThresholdConfig,
) -> SafetyLevel:
    """Classify surface-to-surface clearance into SAFE/CAUTION/CRITICAL/COLLISION."""
    if surface_clearance_m is None:
        return SafetyLevel.SAFE
    if surface_clearance_m <= thresholds.critical_clearance_m:
        return SafetyLevel.COLLISION
    if surface_clearance_m <= thresholds.caution_clearance_m:
        return SafetyLevel.CRITICAL
    if surface_clearance_m <= thresholds.safe_clearance_m:
        return SafetyLevel.CAUTION
    return SafetyLevel.SAFE


def compute_static_obstacle_clearance(
    robot_x_m: float,
    robot_y_m: float,
    robot_radius_m: float,
    obstacle: StaticObstacle,
) -> tuple[float, float, tuple[float, float]]:
    """Compute (surface_clearance_m, center_to_center_m, closest_point_m) for a rectangle."""
    closest_x = max(obstacle.x_m, min(robot_x_m, obstacle.x_m + obstacle.width_m))
    closest_y = max(obstacle.y_m, min(robot_y_m, obstacle.y_m + obstacle.height_m))

    dist_to_rect = math.hypot(robot_x_m - closest_x, robot_y_m - closest_y)
    surface_clearance_m = dist_to_rect - robot_radius_m

    obs_cx = obstacle.x_m + 0.5 * obstacle.width_m
    obs_cy = obstacle.y_m + 0.5 * obstacle.height_m
    center_dist_m = math.hypot(robot_x_m - obs_cx, robot_y_m - obs_cy)
    return (surface_clearance_m, center_dist_m, (closest_x, closest_y))


def compute_dynamic_obstacle_clearance(
    robot_x_m: float,
    robot_y_m: float,
    robot_radius_m: float,
    obstacle: DynamicObstacle,
) -> tuple[float, float, tuple[float, float]]:
    """Compute (surface_clearance_m, center_to_center_m, closest_point_m) for a circle."""
    dx = robot_x_m - obstacle.x_m
    dy = robot_y_m - obstacle.y_m
    center_dist_m = math.hypot(dx, dy)
    surface_clearance_m = center_dist_m - robot_radius_m - obstacle.radius_m

    if center_dist_m > 1e-9:
        ux = dx / center_dist_m
        uy = dy / center_dist_m
        closest_pt = (
            obstacle.x_m + obstacle.radius_m * ux,
            obstacle.y_m + obstacle.radius_m * uy,
        )
    else:
        closest_pt = (obstacle.x_m, obstacle.y_m)

    return (surface_clearance_m, center_dist_m, closest_pt)


def compute_ttc_s(
    robot_state: RobotState,
    closest_point_m: tuple[float, float],
    surface_clearance_m: float,
    obs_vx_mps: float = 0.0,
    obs_vy_mps: float = 0.0,
) -> float | None:
    """Compute Time-To-Collision (seconds) along the line to the nearest obstacle point.

    Returns None when already colliding (clearance <= 0) or when closing speed <= 0.
    """
    if surface_clearance_m <= 0.0:
        return None

    dx = closest_point_m[0] - robot_state.x_m
    dy = closest_point_m[1] - robot_state.y_m
    dist = math.hypot(dx, dy)
    if dist <= 1e-9:
        return None

    nx = dx / dist
    ny = dy / dist

    rob_vx = robot_state.v_mps * math.cos(robot_state.theta_rad)
    rob_vy = robot_state.v_mps * math.sin(robot_state.theta_rad)

    rel_vx = rob_vx - obs_vx_mps
    rel_vy = rob_vy - obs_vy_mps
    closing_speed = rel_vx * nx + rel_vy * ny
    if closing_speed <= 1e-4:
        return None

    return surface_clearance_m / closing_speed


class SafetyTracker:
    """Tracks real-time safety snapshots, minimum clearance, counters, and event logs."""

    def __init__(self, thresholds: SafetyThresholdConfig) -> None:
        self._thresholds = thresholds
        self._min_clearance_m: float | None = None
        self._warning_count: int = 0
        self._collision_count: int = 0
        self._near_miss_count: int = 0
        self._prev_level: SafetyLevel = SafetyLevel.SAFE
        self._clearance_history: list[tuple[float, float]] = []
        self._event_log: list[SafetyEvent] = []
        self._latest_snapshot: SafetySnapshot | None = None

    @property
    def min_clearance_m(self) -> float | None:
        """Return minimum observed surface clearance in meters."""
        return self._min_clearance_m

    @property
    def warning_count(self) -> int:
        """Return number of distinct safety warning episodes."""
        return self._warning_count

    @property
    def collision_count(self) -> int:
        """Return number of distinct physical collision episodes."""
        return self._collision_count

    @property
    def near_miss_count(self) -> int:
        """Return number of near-miss episodes (0 < clearance <= 0.15 m)."""
        return self._near_miss_count

    @property
    def clearance_history(self) -> list[tuple[float, float]]:
        """Return list of (sim_time_s, surface_clearance_m) samples."""
        return self._clearance_history

    @property
    def event_log(self) -> list[SafetyEvent]:
        """Return chronological list of recent safety events."""
        return self._event_log

    @property
    def latest_snapshot(self) -> SafetySnapshot | None:
        """Return the most recent SafetySnapshot."""
        return self._latest_snapshot

    def reset(self) -> None:
        """Reset all safety statistics, history, and event logs."""
        self._min_clearance_m = None
        self._warning_count = 0
        self._collision_count = 0
        self._near_miss_count = 0
        self._prev_level = SafetyLevel.SAFE
        self._clearance_history.clear()
        self._event_log.clear()
        self._latest_snapshot = None

    def log_event(self, timestamp_s: float, message: str, level: SafetyLevel) -> None:
        """Append a timestamped event to the bounded log."""
        self._event_log.append(
            SafetyEvent(timestamp_s=timestamp_s, message=message, level=level)
        )
        max_entries = self._thresholds.event_log_max_entries
        if len(self._event_log) > max_entries:
            self._event_log = self._event_log[-max_entries:]

    def evaluate(
        self,
        sim_time_s: float,
        robot_state: RobotState,
        robot_radius_m: float,
        static_obstacles: list[StaticObstacle],
        dynamic_obstacles: list[DynamicObstacle],
        required_safety_margin_m: float,
        record_history: bool = True,
    ) -> SafetySnapshot:
        """Compute exact safety snapshot from current world geometry and update stats."""
        best_clearance: float | None = None
        best_center_dist: float | None = None
        best_id: str | None = None
        best_type: str | None = None
        best_point: tuple[float, float] | None = None
        best_obs_vel: tuple[float, float] = (0.0, 0.0)

        for idx, obs in enumerate(static_obstacles, start=1):
            surf_c, center_d, pt = compute_static_obstacle_clearance(
                robot_state.x_m, robot_state.y_m, robot_radius_m, obs
            )
            if best_clearance is None or surf_c < best_clearance:
                best_clearance = surf_c
                best_center_dist = center_d
                best_id = f"SABİT-{idx}"
                best_type = "SABİT"
                best_point = pt
                best_obs_vel = (0.0, 0.0)

        for obs in dynamic_obstacles:
            if not obs.active:
                continue
            surf_c, center_d, pt = compute_dynamic_obstacle_clearance(
                robot_state.x_m, robot_state.y_m, robot_radius_m, obs
            )
            if best_clearance is None or surf_c < best_clearance:
                best_clearance = surf_c
                best_center_dist = center_d
                best_id = obs.obstacle_id
                best_type = "HAREKETLİ"
                best_point = pt
                best_obs_vel = (obs.vx_mps, obs.vy_mps)

        level = classify_safety_level(best_clearance, self._thresholds)
        ttc_s: float | None = None
        if best_clearance is not None and best_point is not None:
            ttc_s = compute_ttc_s(
                robot_state,
                best_point,
                best_clearance,
                obs_vx_mps=best_obs_vel[0],
                obs_vy_mps=best_obs_vel[1],
            )

        snapshot = SafetySnapshot(
            nearest_obstacle_id=best_id,
            nearest_obstacle_type=best_type,
            nearest_point_on_obstacle_m=best_point,
            center_to_center_distance_m=best_center_dist,
            surface_clearance_m=best_clearance,
            required_safety_margin_m=required_safety_margin_m,
            safety_level=level,
            ttc_s=ttc_s,
        )
        self._latest_snapshot = snapshot

        if record_history and best_clearance is not None:
            if self._min_clearance_m is None or best_clearance < self._min_clearance_m:
                self._min_clearance_m = best_clearance

            self._clearance_history.append((sim_time_s, best_clearance))
            if len(self._clearance_history) > self._thresholds.history_max_samples:
                self._clearance_history = self._clearance_history[
                    -self._thresholds.history_max_samples :
                ]

            self._update_counters_and_events(sim_time_s, level)

        return snapshot

    def _update_counters_and_events(
        self, sim_time_s: float, level: SafetyLevel
    ) -> None:
        """Detect rising-edge transitions between safety levels and log events."""
        if level != self._prev_level:
            if level == SafetyLevel.CAUTION and self._prev_level == SafetyLevel.SAFE:
                self._warning_count += 1
                self.log_event(sim_time_s, "ENGEL YAKLAŞIYOR", level)
            elif level == SafetyLevel.CRITICAL:
                if self._prev_level == SafetyLevel.SAFE:
                    self._warning_count += 1
                if self._prev_level != SafetyLevel.COLLISION:
                    self._near_miss_count += 1
                self.log_event(sim_time_s, "GÜVENLİK PAYI İHLAL EDİLDİ", level)
            elif level == SafetyLevel.COLLISION:
                self._collision_count += 1
                self.log_event(sim_time_s, "FİZİKSEL ÇARPIŞMA", level)

        self._prev_level = level
