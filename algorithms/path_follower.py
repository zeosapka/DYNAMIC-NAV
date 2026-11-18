"""Autonomous global A* waypoint-following controller in SI units.

This is a basic global path-tracking controller (not DWA) that rotates toward
the next A* waypoint, drives forward when aligned, and stops at the goal.
Strictly independent of Pygame.
"""

import math

from config import FollowerConfig
from simulation.entities import RobotState, wrap_angle_rad


def _ramp_toward(current: float, target: float, max_delta: float) -> float:
    """Move current toward target by at most max_delta."""
    if max_delta <= 0.0:
        return target
    diff = target - current
    if abs(diff) <= max_delta:
        return target
    return current + math.copysign(max_delta, diff)


class WaypointFollower:
    """Tracks a sequence of 2D world waypoints (x_m, y_m) using unicycle commands."""

    def __init__(self, config: FollowerConfig) -> None:
        self._config = config
        self._waypoints: list[tuple[float, float]] = []
        self._current_index: int = 0
        self._target_reached: bool = False
        self._last_v_mps: float = 0.0
        self._last_omega_radps: float = 0.0

    @property
    def has_path(self) -> bool:
        """Return True if a non-empty path is loaded."""
        return len(self._waypoints) > 0

    @property
    def current_index(self) -> int:
        """Return index of the active target waypoint."""
        return self._current_index

    @property
    def target_reached(self) -> bool:
        """Return True once the robot has reached the final goal waypoint."""
        return self._target_reached

    @property
    def waypoints(self) -> list[tuple[float, float]]:
        """Return the loaded waypoint list in meters."""
        return self._waypoints

    def set_path(self, path_world_m: list[tuple[float, float]]) -> None:
        """Load a new A* waypoint path and reset tracking state."""
        self._waypoints = list(path_world_m)
        self._current_index = 1 if len(self._waypoints) > 1 else 0
        self._target_reached = False
        self._last_v_mps = 0.0
        self._last_omega_radps = 0.0

    def clear(self) -> None:
        """Clear the loaded path and stop."""
        self._waypoints = []
        self._current_index = 0
        self._target_reached = False
        self._last_v_mps = 0.0
        self._last_omega_radps = 0.0

    def reset(self) -> None:
        """Reset tracking to the start of the current path."""
        self._current_index = 1 if len(self._waypoints) > 1 else 0
        self._target_reached = False
        self._last_v_mps = 0.0
        self._last_omega_radps = 0.0

    def reset_velocity_ramp(self) -> None:
        """Zero internal ramped velocities (e.g., on PAUSE or safety stop)."""
        self._last_v_mps = 0.0
        self._last_omega_radps = 0.0

    def compute_command(
        self,
        robot_state: RobotState,
        max_v_mps: float,
        max_omega_radps: float,
        dt_s: float,
    ) -> tuple[float, float]:
        """Compute smooth (v_mps, omega_radps) to follow the next waypoint."""
        if not self._waypoints or self._target_reached or dt_s <= 0.0:
            return (0.0, 0.0)

        # Advance through intermediate waypoints within tolerance.
        while self._current_index < len(self._waypoints) - 1:
            wx, wy = self._waypoints[self._current_index]
            dist_wp = math.hypot(wx - robot_state.x_m, wy - robot_state.y_m)
            if dist_wp <= self._config.waypoint_tolerance_m:
                self._current_index += 1
            else:
                break

        goal_x, goal_y = self._waypoints[-1]
        dist_to_goal = math.hypot(goal_x - robot_state.x_m, goal_y - robot_state.y_m)
        if (
            self._current_index == len(self._waypoints) - 1
            and dist_to_goal <= self._config.goal_tolerance_m
        ):
            self._target_reached = True
            self._last_v_mps = 0.0
            self._last_omega_radps = 0.0
            return (0.0, 0.0)

        target_x, target_y = self._waypoints[self._current_index]
        desired_theta = math.atan2(target_y - robot_state.y_m, target_x - robot_state.x_m)
        heading_error = wrap_angle_rad(desired_theta - robot_state.theta_rad)

        raw_omega = max(
            -max_omega_radps,
            min(max_omega_radps, self._config.angular_gain * heading_error),
        )

        thresh = self._config.rotate_in_place_threshold_rad
        if abs(heading_error) >= thresh:
            raw_v = 0.0
        else:
            alignment_scale = 1.0 - (abs(heading_error) / thresh)
            raw_v = max_v_mps * max(0.0, alignment_scale)
            if self._current_index == len(self._waypoints) - 1:
                # Smooth deceleration into the final goal.
                raw_v = min(raw_v, max(0.25, dist_to_goal * 2.5))

        v_cmd = _ramp_toward(
            self._last_v_mps,
            raw_v,
            self._config.max_linear_accel_mps2 * dt_s,
        )
        omega_cmd = _ramp_toward(
            self._last_omega_radps,
            raw_omega,
            self._config.max_angular_accel_radps2 * dt_s,
        )
        self._last_v_mps = v_cmd
        self._last_omega_radps = omega_cmd
        return (v_cmd, omega_cmd)
