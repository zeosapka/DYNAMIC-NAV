"""Simulation entities: mobile robot kinematics and static obstacles."""

from dataclasses import dataclass
import math

from config import RobotConfig, StaticObstacleConfig


def wrap_angle_rad(angle_rad: float) -> float:
    """Normalize an angle in radians to the half-open interval [-pi, pi)."""
    return (angle_rad + math.pi) % (2.0 * math.pi) - math.pi


@dataclass
class RobotState:
    """Instantaneous kinematic state of the robot in SI units."""

    x_m: float
    y_m: float
    theta_rad: float
    v_mps: float = 0.0
    omega_radps: float = 0.0


@dataclass(frozen=True)
class StaticObstacle:
    """Axis-aligned rectangular static obstacle in world coordinates (meters).

    (x_m, y_m) is the bottom-left corner in world space.
    """

    x_m: float
    y_m: float
    width_m: float
    height_m: float

    @classmethod
    def from_config(cls, cfg: StaticObstacleConfig) -> "StaticObstacle":
        """Construct a StaticObstacle from configuration."""
        return cls(
            x_m=cfg.x_m,
            y_m=cfg.y_m,
            width_m=cfg.width_m,
            height_m=cfg.height_m,
        )

    def intersects_circle(self, cx_m: float, cy_m: float, radius_m: float) -> bool:
        """Return True if a circle centered at (cx_m, cy_m) overlaps this rectangle."""
        closest_x = max(self.x_m, min(cx_m, self.x_m + self.width_m))
        closest_y = max(self.y_m, min(cy_m, self.y_m + self.height_m))
        dx = cx_m - closest_x
        dy = cy_m - closest_y
        return (dx * dx + dy * dy) < (radius_m * radius_m)


class Robot:
    """2D unicycle-model mobile robot operating in SI units."""

    def __init__(self, config: RobotConfig) -> None:
        self._config = config
        self._initial_x_m = config.initial_x_m
        self._initial_y_m = config.initial_y_m
        self._initial_theta_rad = wrap_angle_rad(config.initial_theta_rad)
        self._state = RobotState(
            x_m=self._initial_x_m,
            y_m=self._initial_y_m,
            theta_rad=self._initial_theta_rad,
        )
        self._cmd_v_mps: float = 0.0
        self._cmd_omega_radps: float = 0.0

    @property
    def config(self) -> RobotConfig:
        """Return the robot configuration."""
        return self._config

    @property
    def state(self) -> RobotState:
        """Return the current robot state."""
        return self._state

    @property
    def radius_m(self) -> float:
        """Return the robot collision/visual radius in meters."""
        return self._config.radius_m

    def set_initial_position(self, x_m: float, y_m: float) -> None:
        """Update the robot start position in meters and move the robot there."""
        self._initial_x_m = x_m
        self._initial_y_m = y_m
        self.reset()

    def set_initial_pose(self, x_m: float, y_m: float, theta_rad: float) -> None:
        """Update the robot start pose (x_m, y_m, theta_rad) and reset the robot."""
        self._initial_x_m = x_m
        self._initial_y_m = y_m
        self._initial_theta_rad = wrap_angle_rad(theta_rad)
        self.reset()

    def reset(self) -> None:
        """Reset robot pose and velocities to initial configured values."""
        self._state = RobotState(
            x_m=self._initial_x_m,
            y_m=self._initial_y_m,
            theta_rad=self._initial_theta_rad,
            v_mps=0.0,
            omega_radps=0.0,
        )
        self._cmd_v_mps = 0.0
        self._cmd_omega_radps = 0.0

    def set_velocity_command(self, v_mps: float, omega_radps: float) -> None:
        """Clamp and store the desired linear (m/s) and angular (rad/s) velocities."""
        max_v = self._config.max_linear_speed_mps
        max_w = self._config.max_angular_speed_radps
        self._cmd_v_mps = max(-max_v, min(max_v, v_mps))
        self._cmd_omega_radps = max(-max_w, min(max_w, omega_radps))

    def step(
        self,
        dt_s: float,
        world_width_m: float,
        world_height_m: float,
        obstacles: list[StaticObstacle],
    ) -> None:
        """Advance robot kinematics by dt_s seconds with boundary and obstacle checks."""
        if dt_s <= 0.0:
            return

        v = self._cmd_v_mps
        omega = self._cmd_omega_radps

        new_theta = wrap_angle_rad(self._state.theta_rad + omega * dt_s)
        # Midpoint heading integration reduces discretization drift during turns.
        mid_theta = self._state.theta_rad + 0.5 * omega * dt_s
        proposed_x = self._state.x_m + v * math.cos(mid_theta) * dt_s
        proposed_y = self._state.y_m + v * math.sin(mid_theta) * dt_s

        r = self._config.radius_m
        proposed_x = max(r, min(world_width_m - r, proposed_x))
        proposed_y = max(r, min(world_height_m - r, proposed_y))

        # Axis-separated collision resolution allows smooth sliding along obstacle walls.
        final_x = self._state.x_m
        final_y = self._state.y_m

        if not self._collides(proposed_x, final_y, obstacles):
            final_x = proposed_x
        if not self._collides(final_x, proposed_y, obstacles):
            final_y = proposed_y

        self._state = RobotState(
            x_m=final_x,
            y_m=final_y,
            theta_rad=new_theta,
            v_mps=v,
            omega_radps=omega,
        )

    def _collides(
        self,
        x_m: float,
        y_m: float,
        obstacles: list[StaticObstacle],
    ) -> bool:
        """Return True if the robot circle at (x_m, y_m) intersects any obstacle."""
        r = self._config.radius_m
        return any(obs.intersects_circle(x_m, y_m, r) for obs in obstacles)
