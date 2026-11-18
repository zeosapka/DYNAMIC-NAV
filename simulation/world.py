"""2D grid-based simulation world managing state, robot, static and dynamic obstacles."""

from enum import Enum, auto
import math
import numpy as np

from config import RobotConfig, SimulationConfig, WorldConfig
from simulation.dynamic_obstacles import (
    DynamicCollisionReport,
    DynamicObstacle,
    evaluate_dynamic_collisions,
)
from simulation.entities import Robot, StaticObstacle
from simulation.geometry import geometry_bounding_box_m, geometry_overlaps_cell
from simulation.objects import MotionBehaviorType, ObjectCategory, ObjectEditorManager, SimObject
from simulation.scenarios import ScenarioDefinition


class SimulationState(Enum):
    """Execution lifecycle states for the simulator."""

    READY = auto()
    RUNNING = auto()
    PAUSED = auto()


class SimulationWorld:
    """Encapsulates the 2D metric world, occupancy grid, robot, and dynamic obstacles."""

    def __init__(
        self,
        world_config: WorldConfig,
        robot_config: RobotConfig,
        sim_config: SimulationConfig,
        safety_margin_m: float = 0.15,
    ) -> None:
        self._world_config = world_config
        self._sim_config = sim_config
        self._safety_margin_m = safety_margin_m
        self._obstacles: list[StaticObstacle] = [
            StaticObstacle.from_config(obs) for obs in world_config.static_obstacles
        ]
        self._dynamic_obstacles: list[DynamicObstacle] = []
        self._object_editor = ObjectEditorManager()
        self._collision_reports: list[DynamicCollisionReport] = []
        self._active_scenario_name: str | None = None

        self._robot = Robot(robot_config)
        self._occupancy_grid = self._build_occupancy_grid()
        self._sim_time_s: float = 0.0
        self._state = (
            SimulationState.RUNNING if sim_config.auto_start else SimulationState.READY
        )

    @property
    def world_config(self) -> WorldConfig:
        """Return the world configuration."""
        return self._world_config

    @property
    def robot(self) -> Robot:
        """Return the mobile robot instance."""
        return self._robot

    @property
    def obstacles(self) -> list[StaticObstacle]:
        """Return the list of static obstacles."""
        return self._obstacles

    @property
    def dynamic_obstacles(self) -> list[DynamicObstacle]:
        """Return the list of dynamic obstacles."""
        return self._dynamic_obstacles

    @property
    def object_editor(self) -> ObjectEditorManager:
        """Return the interactive universal object editor manager."""
        return self._object_editor

    @property
    def sim_objects(self) -> list[SimObject]:
        """Return all universal simulation objects."""
        return self._object_editor.objects

    def rebuild_occupancy_grid(self) -> None:
        """Rebuild the static occupancy grid from StaticObstacles and static SimObjects."""
        self._occupancy_grid = self._build_occupancy_grid()

    @property
    def collision_reports(self) -> list[DynamicCollisionReport]:
        """Return the latest dynamic obstacle collision & safety reports."""
        return self._collision_reports

    @property
    def has_physical_collision(self) -> bool:
        """Return True if any active dynamic obstacle physically overlapped the robot."""
        return any(r.physical_overlap for r in self._collision_reports)

    @property
    def has_safety_warning(self) -> bool:
        """Return True if any active dynamic obstacle violated the required safety margin."""
        return any(r.safety_warning for r in self._collision_reports)

    @property
    def active_scenario_name(self) -> str | None:
        """Return the name of the currently loaded scenario, if any."""
        return self._active_scenario_name

    @property
    def occupancy_grid(self) -> np.ndarray:
        """Return the 2D static binary occupancy grid (0 = free, 1 = occupied)."""
        return self._occupancy_grid

    @property
    def sim_time_s(self) -> float:
        """Return elapsed simulation time in seconds."""
        return self._sim_time_s

    @property
    def state(self) -> SimulationState:
        """Return the current simulation lifecycle state."""
        return self._state

    def start(self) -> None:
        """Start or resume simulation stepping."""
        self._state = SimulationState.RUNNING

    def pause(self) -> None:
        """Pause simulation stepping if currently running."""
        if self._state == SimulationState.RUNNING:
            self._state = SimulationState.PAUSED

    def toggle_pause(self) -> None:
        """Toggle between RUNNING and PAUSED/READY."""
        if self._state == SimulationState.RUNNING:
            self._state = SimulationState.PAUSED
        else:
            self._state = SimulationState.RUNNING

    def reset(self) -> None:
        """Reset simulation time, robot pose, dynamic obstacles, and return to READY."""
        self._robot.reset()
        for obs in self._dynamic_obstacles:
            obs.reset()
        self._object_editor.reset_all()
        self._collision_reports = []
        self._sim_time_s = 0.0
        self._state = SimulationState.READY

    def set_dynamic_obstacles(self, obstacles: list[DynamicObstacle]) -> None:
        """Replace the active list of dynamic obstacles without altering the static grid."""
        self._dynamic_obstacles = list(obstacles)
        self._collision_reports = []

    def load_scenario(self, scenario: ScenarioDefinition) -> None:
        """Load a deterministic scenario into the world and reset simulation clock."""
        self._active_scenario_name = scenario.name
        self._obstacles = [
            StaticObstacle.from_config(obs) for obs in scenario.static_obstacles
        ]
        self._occupancy_grid = self._build_occupancy_grid()
        self._robot.set_initial_pose(
            scenario.robot_start_m[0],
            scenario.robot_start_m[1],
            scenario.robot_theta_rad,
        )
        self._dynamic_obstacles = [
            spec.instantiate() for spec in scenario.dynamic_obstacles
        ]
        self._object_editor.clear()
        for obj_spec in getattr(scenario, "sim_objects", ()):
            obj = self._object_editor.add_object(
                ObjectCategory(obj_spec.category),
                obj_spec.x_m,
                obj_spec.y_m,
                obj_spec.heading_rad,
            )
            obj.object_id = obj_spec.object_id
            self._object_editor.update_properties(
                obj.object_id,
                radius_m=obj_spec.radius_m,
                linear_speed_mps=obj_spec.linear_speed_mps,
                acceleration_mps2=obj_spec.acceleration_mps2,
                behavior=MotionBehaviorType(obj_spec.behavior),
            )
            obj.motion.min_speed_mps = obj_spec.min_speed_mps
            obj._initial_motion.min_speed_mps = obj_spec.min_speed_mps
        self._object_editor.select_by_id(None)
        self._collision_reports = []
        self._sim_time_s = 0.0
        self._state = SimulationState.READY

    def add_static_obstacle(self, obstacle: StaticObstacle) -> None:
        """Add a static obstacle and rebuild the occupancy grid."""
        self._obstacles.append(obstacle)
        self._occupancy_grid = self._build_occupancy_grid()

    def remove_static_obstacle_at(self, x_m: float, y_m: float) -> bool:
        """Remove the topmost static obstacle containing (x_m, y_m); return True if removed."""
        for idx in range(len(self._obstacles) - 1, -1, -1):
            obs = self._obstacles[idx]
            if (
                obs.x_m <= x_m <= obs.x_m + obs.width_m
                and obs.y_m <= y_m <= obs.y_m + obs.height_m
            ):
                self._obstacles.pop(idx)
                self._occupancy_grid = self._build_occupancy_grid()
                return True
        return False

    def step(self, dt_s: float) -> None:
        """Advance the simulation by dt_s seconds when in RUNNING state."""
        if self._state != SimulationState.RUNNING or dt_s <= 0.0:
            return

        clamped_dt = min(dt_s, self._sim_config.max_dt_s)
        robot_prev = (self._robot.state.x_m, self._robot.state.y_m)

        self._robot.step(
            dt_s=clamped_dt,
            world_width_m=self._world_config.width_m,
            world_height_m=self._world_config.height_m,
            obstacles=self._obstacles,
        )
        robot_curr = (self._robot.state.x_m, self._robot.state.y_m)

        transitions: list[
            tuple[DynamicObstacle, tuple[float, float], tuple[float, float]]
        ] = []
        for obs in self._dynamic_obstacles:
            px, py, nx, ny = obs.step(clamped_dt)
            transitions.append((obs, (px, py), (nx, ny)))

        self._object_editor.step_all(clamped_dt)

        self._collision_reports = evaluate_dynamic_collisions(
            robot_prev_m=robot_prev,
            robot_curr_m=robot_curr,
            robot_radius_m=self._robot.radius_m,
            safety_margin_m=self._safety_margin_m,
            obstacle_transitions=transitions,
        )
        self._sim_time_s += clamped_dt

    def _build_occupancy_grid(self) -> np.ndarray:
        """Discretize static obstacles onto a 2D NumPy occupancy grid (rows=Y, cols=X)."""
        res = self._world_config.grid_resolution_m
        cols = math.ceil(self._world_config.width_m / res)
        rows = math.ceil(self._world_config.height_m / res)
        grid = np.zeros((rows, cols), dtype=np.uint8)

        for obs in self._obstacles:
            col_start = max(0, math.floor(obs.x_m / res))
            col_end = min(cols, math.ceil((obs.x_m + obs.width_m) / res))
            row_start = max(0, math.floor(obs.y_m / res))
            row_end = min(rows, math.ceil((obs.y_m + obs.height_m) / res))
            grid[row_start:row_end, col_start:col_end] = 1

        for obj in self._object_editor.objects:
            if obj.category != ObjectCategory.STATIC_OBSTACLE or not obj.motion.active:
                continue
            geom = obj.geometry()
            min_x, min_y, max_x, max_y = geometry_bounding_box_m(geom)
            c0 = max(0, math.floor(min_x / res))
            c1 = min(cols - 1, math.ceil(max_x / res) - 1)
            r0 = max(0, math.floor(min_y / res))
            r1 = min(rows - 1, math.ceil(max_y / res) - 1)
            for r in range(r0, r1 + 1):
                cy0 = r * res
                cy1 = cy0 + res
                for c in range(c0, c1 + 1):
                    cx0 = c * res
                    if geometry_overlaps_cell(geom, cx0, cy0, cx0 + res, cy1):
                        grid[r, c] = 1

        return grid
