"""Dynamic obstacle models, movement behaviors, and continuous-time collision detection.

All quantities are expressed in SI units (meters, seconds, m/s).
This module is strictly independent of Pygame.
"""

from dataclasses import dataclass
from enum import Enum, auto
import math


class MovementBehavior(Enum):
    """Supported deterministic movement behaviors for dynamic obstacles."""

    HEAD_ON = auto()
    CROSSING = auto()
    CORRIDOR = auto()
    STOP_AND_GO = auto()


@dataclass(frozen=True)
class DynamicCollisionReport:
    """Collision and safety-margin status for a single dynamic obstacle over a step."""

    obstacle_id: str
    min_center_distance_m: float
    physical_overlap: bool
    safety_warning: bool
    time_of_closest_approach_fraction: float


class DynamicObstacle:
    """Moving circular obstacle with deterministic, delta-time-based kinematics."""

    def __init__(
        self,
        obstacle_id: str,
        x_m: float,
        y_m: float,
        vx_mps: float,
        vy_mps: float,
        radius_m: float,
        behavior: MovementBehavior,
        active: bool = True,
        waypoint_a_m: tuple[float, float] | None = None,
        waypoint_b_m: tuple[float, float] | None = None,
        reverse_at_bounds: bool = True,
        move_duration_s: float = 2.0,
        stop_duration_s: float = 1.5,
    ) -> None:
        if radius_m <= 0.0:
            raise ValueError("Dynamic obstacle radius_m must be strictly positive.")
        if move_duration_s < 0.0 or stop_duration_s < 0.0:
            raise ValueError("Durations must be non-negative.")

        self.obstacle_id = obstacle_id
        self.x_m = x_m
        self.y_m = y_m
        self.vx_mps = vx_mps
        self.vy_mps = vy_mps
        self.radius_m = radius_m
        self.behavior = behavior
        self.active = active

        self.waypoint_a_m = waypoint_a_m
        self.waypoint_b_m = waypoint_b_m
        self.reverse_at_bounds = reverse_at_bounds
        self.move_duration_s = move_duration_s
        self.stop_duration_s = stop_duration_s

        self._initial_x_m = x_m
        self._initial_y_m = y_m
        self._initial_vx_mps = vx_mps
        self._initial_vy_mps = vy_mps
        self._base_vx_mps = vx_mps
        self._base_vy_mps = vy_mps
        self._phase_time_s: float = 0.0
        self._stopped_now: bool = False

    @property
    def speed_mps(self) -> float:
        """Return instantaneous scalar speed in m/s."""
        return math.hypot(self.vx_mps, self.vy_mps)

    @property
    def is_stopped(self) -> bool:
        """Return True when currently paused (e.g., during STOP_AND_GO stop phase)."""
        return self._stopped_now

    def reset(self) -> None:
        """Restore obstacle to its initial deterministic state."""
        self.x_m = self._initial_x_m
        self.y_m = self._initial_y_m
        self.vx_mps = self._initial_vx_mps
        self.vy_mps = self._initial_vy_mps
        self._base_vx_mps = self._initial_vx_mps
        self._base_vy_mps = self._initial_vy_mps
        self._phase_time_s = 0.0
        self._stopped_now = False

    def predict_positions(
        self, dt_s: float, steps: int
    ) -> list[tuple[float, float]]:
        """Predict future (x_m, y_m) positions over `steps` intervals of `dt_s`
        using only current instantaneous state and configured motion model.
        """
        positions: list[tuple[float, float]] = [(self.x_m, self.y_m)]
        if not self.active or dt_s <= 0.0 or steps <= 0:
            return positions + [(self.x_m, self.y_m)] * max(0, steps)

        ghost = DynamicObstacle(
            obstacle_id=self.obstacle_id,
            x_m=self.x_m,
            y_m=self.y_m,
            vx_mps=self.vx_mps,
            vy_mps=self.vy_mps,
            radius_m=self.radius_m,
            behavior=self.behavior,
            active=self.active,
            waypoint_a_m=self.waypoint_a_m,
            waypoint_b_m=self.waypoint_b_m,
            reverse_at_bounds=self.reverse_at_bounds,
            move_duration_s=self.move_duration_s,
            stop_duration_s=self.stop_duration_s,
        )
        ghost._base_vx_mps = self._base_vx_mps
        ghost._base_vy_mps = self._base_vy_mps
        ghost._phase_time_s = self._phase_time_s
        ghost._stopped_now = self._stopped_now

        for _ in range(steps):
            ghost.step(dt_s)
            positions.append((ghost.x_m, ghost.y_m))
        return positions

    def step(self, dt_s: float) -> tuple[float, float, float, float]:
        """Advance obstacle position by dt_s seconds.

        Returns (prev_x_m, prev_y_m, new_x_m, new_y_m) for swept collision checks.
        """
        prev_x, prev_y = self.x_m, self.y_m
        if not self.active or dt_s <= 0.0:
            return (prev_x, prev_y, self.x_m, self.y_m)

        if self.behavior == MovementBehavior.STOP_AND_GO:
            active_dt_s = self._advance_stop_and_go_timer(dt_s)
            if active_dt_s > 0.0:
                self._integrate_motion(
                    active_dt_s, self._base_vx_mps, self._base_vy_mps
                )
            if self._stopped_now:
                self.vx_mps = 0.0
                self.vy_mps = 0.0
            else:
                self.vx_mps = self._base_vx_mps
                self.vy_mps = self._base_vy_mps
        else:
            self._stopped_now = False
            self._integrate_motion(dt_s, self.vx_mps, self.vy_mps)

        return (prev_x, prev_y, self.x_m, self.y_m)

    def _advance_stop_and_go_timer(self, dt_s: float) -> float:
        """Advance STOP_AND_GO cycle timer and return exact moving time within dt_s.

        Uses exact O(1) cycle reduction modulo ``cycle_s = move_duration_s + stop_duration_s``
        and interval-overlap integration to eliminate IEEE-754 non-progress loops near
        transition boundaries and preserve precision at large simulation timestamps.
        Numerical tolerance: ``1e-9 s`` boundary snap matches ``step()`` transition
        semantics while preventing sub-nanosecond floating-point phase drift.
        """
        if math.isnan(dt_s) or math.isinf(dt_s) or dt_s <= 0.0:
            return 0.0

        cycle_s = self.move_duration_s + self.stop_duration_s
        if cycle_s <= 0.0 or self.move_duration_s <= 0.0:
            self._stopped_now = True
            return 0.0
        if self.stop_duration_s <= 0.0:
            self._phase_time_s = (self._phase_time_s + dt_s) % max(1e-9, self.move_duration_s)
            self._stopped_now = False
            return dt_s

        phase = self._phase_time_s % cycle_s
        if cycle_s - phase < 1e-9:
            phase = 0.0

        full_cycles = math.floor(dt_s / cycle_s)
        rem = dt_s - full_cycles * cycle_s
        # Overlap of [phase, phase + rem] with [0, move_duration_s] and [cycle_s, cycle_s + move_duration_s]
        p_end = phase + rem
        overlap_1 = max(0.0, min(p_end, self.move_duration_s) - phase)
        overlap_2 = max(0.0, min(p_end, cycle_s + self.move_duration_s) - max(phase, cycle_s))
        moving_dt = full_cycles * self.move_duration_s + overlap_1 + overlap_2

        final_phase = p_end % cycle_s
        if cycle_s - final_phase < 1e-9:
            final_phase = 0.0
        self._phase_time_s = final_phase
        self._stopped_now = final_phase >= (self.move_duration_s - 1e-9)
        return moving_dt

    def _integrate_motion(self, dt_s: float, vx_mps: float, vy_mps: float) -> None:
        """Integrate linear motion with optional bounded waypoint reflection/clamping."""
        if self.waypoint_a_m is None or self.waypoint_b_m is None:
            self.x_m += vx_mps * dt_s
            self.y_m += vy_mps * dt_s
            return

        ax, ay = self.waypoint_a_m
        bx, by = self.waypoint_b_m
        seg_dx = bx - ax
        seg_dy = by - ay
        length_m = math.hypot(seg_dx, seg_dy)
        if length_m <= 1e-9:
            return

        ux = seg_dx / length_m
        uy = seg_dy / length_m

        # Current 1D coordinate along segment [0, length_m] and signed speed.
        s = (self.x_m - ax) * ux + (self.y_m - ay) * uy
        v_signed = vx_mps * ux + vy_mps * uy
        s_next = s + v_signed * dt_s

        if self.reverse_at_bounds:
            # Triangle-wave folding into [0, length_m] handles arbitrary dt_s.
            period = 2.0 * length_m
            s_mod = s_next % period
            if s_mod > length_m:
                s_final = period - s_mod
                flipped = True
            else:
                s_final = s_mod
                flipped = False

            # Determine whether parity of reflections flipped direction relative to v_signed.
            k = math.floor(s_next / length_m)
            parity_flipped = (k % 2 != 0) if s_next >= 0.0 else ((-k) % 2 != 0)
            if parity_flipped:
                v_signed = -v_signed
            elif flipped and s_next < 0.0:
                v_signed = -v_signed

            self.x_m = ax + s_final * ux
            self.y_m = ay + s_final * uy
            new_vx = v_signed * ux
            new_vy = v_signed * uy
            self.vx_mps = new_vx
            self.vy_mps = new_vy
            self._base_vx_mps = new_vx
            self._base_vy_mps = new_vy
        else:
            if s_next <= 0.0:
                self.x_m, self.y_m = ax, ay
                self.vx_mps = 0.0
                self.vy_mps = 0.0
                self._base_vx_mps = 0.0
                self._base_vy_mps = 0.0
            elif s_next >= length_m:
                self.x_m, self.y_m = bx, by
                self.vx_mps = 0.0
                self.vy_mps = 0.0
                self._base_vx_mps = 0.0
                self._base_vy_mps = 0.0
            else:
                self.x_m = ax + s_next * ux
                self.y_m = ay + s_next * uy


def swept_circle_min_distance_m(
    r0_m: tuple[float, float],
    r1_m: tuple[float, float],
    o0_m: tuple[float, float],
    o1_m: tuple[float, float],
) -> tuple[float, float]:
    """Compute exact minimum center-to-center distance over linear step t in [0, 1].

    Uses relative motion D(t) = (r0 - o0) + t * ((r1 - r0) - (o1 - o0)).
    Returns (min_distance_m, t_star) where t_star in [0, 1].
    """
    d0x = r0_m[0] - o0_m[0]
    d0y = r0_m[1] - o0_m[1]
    vrx = (r1_m[0] - r0_m[0]) - (o1_m[0] - o0_m[0])
    vry = (r1_m[1] - r0_m[1]) - (o1_m[1] - o0_m[1])

    v_sq = vrx * vrx + vry * vry
    if v_sq <= 1e-15:
        return (math.hypot(d0x, d0y), 0.0)

    t_star = -(d0x * vrx + d0y * vry) / v_sq
    t_star = max(0.0, min(1.0, t_star))
    min_dx = d0x + t_star * vrx
    min_dy = d0y + t_star * vry
    return (math.hypot(min_dx, min_dy), t_star)


def evaluate_dynamic_collisions(
    robot_prev_m: tuple[float, float],
    robot_curr_m: tuple[float, float],
    robot_radius_m: float,
    safety_margin_m: float,
    obstacle_transitions: list[tuple[DynamicObstacle, tuple[float, float], tuple[float, float]]],
) -> list[DynamicCollisionReport]:
    """Evaluate swept physical overlap and safety-margin warnings for all active obstacles."""
    reports: list[DynamicCollisionReport] = []
    for obs, obs_prev_m, obs_curr_m in obstacle_transitions:
        if not obs.active:
            continue
        min_dist_m, t_star = swept_circle_min_distance_m(
            robot_prev_m, robot_curr_m, obs_prev_m, obs_curr_m
        )
        physical_limit_m = robot_radius_m + obs.radius_m
        required_limit_m = physical_limit_m + safety_margin_m

        reports.append(
            DynamicCollisionReport(
                obstacle_id=obs.obstacle_id,
                min_center_distance_m=min_dist_m,
                physical_overlap=min_dist_m <= physical_limit_m + 1e-9,
                safety_warning=min_dist_m <= required_limit_m + 1e-9,
                time_of_closest_approach_fraction=t_star,
            )
        )
    return reports
