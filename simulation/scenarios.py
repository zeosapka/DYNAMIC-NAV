"""Deterministic scenario definitions and management for DYNAMIC NAV.

All scenario parameters are specified in SI units (meters, radians, m/s, seconds)
and are strictly reproducible without random generation.
"""

from dataclasses import dataclass

from config import StaticObstacleConfig
from simulation.dynamic_obstacles import DynamicObstacle, MovementBehavior


@dataclass(frozen=True)
class DynamicObstacleSpec:
    """Immutable specification used to instantiate a DynamicObstacle."""

    obstacle_id: str
    x_m: float
    y_m: float
    vx_mps: float
    vy_mps: float
    radius_m: float
    behavior: MovementBehavior
    active: bool = True
    waypoint_a_m: tuple[float, float] | None = None
    waypoint_b_m: tuple[float, float] | None = None
    reverse_at_bounds: bool = True
    move_duration_s: float = 2.0
    stop_duration_s: float = 1.5

    def instantiate(self) -> DynamicObstacle:
        """Create a fresh DynamicObstacle instance from this specification."""
        return DynamicObstacle(
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


@dataclass(frozen=True)
class SimObjectSpec:
    """Immutable specification for a universal SimObject in a scenario."""

    object_id: str
    category: str
    x_m: float
    y_m: float
    heading_rad: float = 0.0
    linear_speed_mps: float = 0.0
    acceleration_mps2: float = 0.0
    min_speed_mps: float = 0.0
    radius_m: float = 0.45
    behavior: str = "STATIONARY"


@dataclass(frozen=True)
class ScenarioDefinition:
    """Complete deterministic benchmark scenario specification."""

    name: str
    behavior: MovementBehavior
    description: str
    robot_start_m: tuple[float, float]
    robot_theta_rad: float
    goal_m: tuple[float, float]
    static_obstacles: tuple[StaticObstacleConfig, ...]
    dynamic_obstacles: tuple[DynamicObstacleSpec, ...]
    sim_objects: tuple[SimObjectSpec, ...] = ()


SCENARIO_CATALOG: dict[str, ScenarioDefinition] = {
    "HEAD-ON": ScenarioDefinition(
        name="HEAD-ON",
        behavior=MovementBehavior.HEAD_ON,
        description="Hareketli engel y=7.5m boyunca robota karşıdan yaklaşır.",
        robot_start_m=(2.0, 7.5),
        robot_theta_rad=0.0,
        goal_m=(18.0, 7.5),
        static_obstacles=(
            StaticObstacleConfig(x_m=8.0, y_m=2.0, width_m=3.0, height_m=2.0),
            StaticObstacleConfig(x_m=8.0, y_m=11.0, width_m=3.0, height_m=2.0),
        ),
        dynamic_obstacles=(
            DynamicObstacleSpec(
                obstacle_id="DYN-HEAD-1",
                x_m=16.0,
                y_m=7.5,
                vx_mps=-1.2,
                vy_mps=0.0,
                radius_m=0.45,
                behavior=MovementBehavior.HEAD_ON,
                waypoint_a_m=(3.5, 7.5),
                waypoint_b_m=(16.5, 7.5),
                reverse_at_bounds=True,
            ),
        ),
    ),
    "CROSSING": ScenarioDefinition(
        name="CROSSING",
        behavior=MovementBehavior.CROSSING,
        description="İki hareketli engel robotun beklenen rotasını çapraz keser.",
        robot_start_m=(2.0, 2.5),
        robot_theta_rad=0.55,
        goal_m=(18.0, 12.5),
        static_obstacles=(
            StaticObstacleConfig(x_m=4.0, y_m=9.0, width_m=2.5, height_m=2.5),
            StaticObstacleConfig(x_m=13.5, y_m=3.0, width_m=2.5, height_m=2.5),
        ),
        dynamic_obstacles=(
            DynamicObstacleSpec(
                obstacle_id="DYN-CROSS-1",
                x_m=12.0,
                y_m=3.0,
                vx_mps=-0.8,
                vy_mps=1.2,
                radius_m=0.40,
                behavior=MovementBehavior.CROSSING,
                waypoint_a_m=(12.0, 3.0),
                waypoint_b_m=(6.0, 12.0),
                reverse_at_bounds=True,
            ),
            DynamicObstacleSpec(
                obstacle_id="DYN-CROSS-2",
                x_m=15.0,
                y_m=12.0,
                vx_mps=-0.9,
                vy_mps=-1.35,
                radius_m=0.40,
                behavior=MovementBehavior.CROSSING,
                waypoint_a_m=(15.0, 12.0),
                waypoint_b_m=(9.0, 3.0),
                reverse_at_bounds=True,
            ),
        ),
    ),
    "CORRIDOR": ScenarioDefinition(
        name="CORRIDOR",
        behavior=MovementBehavior.CORRIDOR,
        description="Hareketli engel dar koridor sınırları içinde devriye gezer.",
        robot_start_m=(2.0, 7.5),
        robot_theta_rad=0.0,
        goal_m=(18.0, 7.5),
        static_obstacles=(
            StaticObstacleConfig(x_m=5.0, y_m=4.0, width_m=10.0, height_m=2.0),
            StaticObstacleConfig(x_m=5.0, y_m=9.0, width_m=10.0, height_m=2.0),
        ),
        dynamic_obstacles=(
            DynamicObstacleSpec(
                obstacle_id="DYN-CORR-1",
                x_m=14.0,
                y_m=7.5,
                vx_mps=-1.0,
                vy_mps=0.0,
                radius_m=0.40,
                behavior=MovementBehavior.CORRIDOR,
                waypoint_a_m=(5.6, 7.5),
                waypoint_b_m=(14.4, 7.5),
                reverse_at_bounds=True,
            ),
        ),
    ),
    "STOP-AND-GO": ScenarioDefinition(
        name="STOP-AND-GO",
        behavior=MovementBehavior.STOP_AND_GO,
        description="Hareketli engel rota boyunca periyodik olarak durur ve devam eder.",
        robot_start_m=(2.0, 7.5),
        robot_theta_rad=0.0,
        goal_m=(18.0, 7.5),
        static_obstacles=(
            StaticObstacleConfig(x_m=6.0, y_m=10.5, width_m=2.5, height_m=2.0),
            StaticObstacleConfig(x_m=12.0, y_m=2.5, width_m=2.5, height_m=2.0),
        ),
        dynamic_obstacles=(
            DynamicObstacleSpec(
                obstacle_id="DYN-SG-1",
                x_m=10.0,
                y_m=3.0,
                vx_mps=0.0,
                vy_mps=1.4,
                radius_m=0.45,
                behavior=MovementBehavior.STOP_AND_GO,
                waypoint_a_m=(10.0, 3.0),
                waypoint_b_m=(10.0, 12.0),
                reverse_at_bounds=True,
                move_duration_s=2.0,
                stop_duration_s=1.5,
            ),
        ),
    ),
    "NARROW-GATE": ScenarioDefinition(
        name="NARROW-GATE",
        behavior=MovementBehavior.CORRIDOR,
        description=(
            "2.60 m genişliğinde dar statik geçit (x=9.0..11.0m, y=6.2..8.8m; "
            "robot çapı 0.70m + 2*0.15m marj sonrası net 1.60m boşluk) ve geçide "
            "yaklaşan devriye engeli (DYN-GATE-1, v=-0.8 m/s, r=0.40m)."
        ),
        robot_start_m=(2.0, 7.5),
        robot_theta_rad=0.0,
        goal_m=(18.0, 7.5),
        static_obstacles=(
            StaticObstacleConfig(x_m=9.0, y_m=0.0, width_m=2.0, height_m=6.2),
            StaticObstacleConfig(x_m=9.0, y_m=8.8, width_m=2.0, height_m=6.2),
        ),
        dynamic_obstacles=(
            DynamicObstacleSpec(
                obstacle_id="DYN-GATE-1",
                x_m=14.0,
                y_m=7.5,
                vx_mps=-0.8,
                vy_mps=0.0,
                radius_m=0.40,
                behavior=MovementBehavior.CORRIDOR,
                waypoint_a_m=(6.5, 7.5),
                waypoint_b_m=(15.5, 7.5),
                reverse_at_bounds=True,
            ),
        ),
    ),
    "MULTIPLE-DYNAMIC-OBSTACLES": ScenarioDefinition(
        name="MULTIPLE-DYNAMIC-OBSTACLES",
        behavior=MovementBehavior.CROSSING,
        description=(
            "Dört eşzamanlı deterministik dinamik engel: karşıdan gelen (DYN-MULTI-1), "
            "iki çapraz kesen (DYN-MULTI-2, DYN-MULTI-3) ve bir dur-kalk (DYN-MULTI-4)."
        ),
        robot_start_m=(2.0, 7.5),
        robot_theta_rad=0.0,
        goal_m=(18.0, 7.5),
        static_obstacles=(
            StaticObstacleConfig(x_m=7.0, y_m=2.0, width_m=2.0, height_m=2.0),
            StaticObstacleConfig(x_m=12.0, y_m=11.0, width_m=2.0, height_m=2.0),
        ),
        dynamic_obstacles=(
            DynamicObstacleSpec(
                obstacle_id="DYN-MULTI-1",
                x_m=16.0,
                y_m=7.5,
                vx_mps=-1.1,
                vy_mps=0.0,
                radius_m=0.40,
                behavior=MovementBehavior.HEAD_ON,
                waypoint_a_m=(4.0, 7.5),
                waypoint_b_m=(16.5, 7.5),
                reverse_at_bounds=True,
            ),
            DynamicObstacleSpec(
                obstacle_id="DYN-MULTI-2",
                x_m=7.5,
                y_m=3.0,
                vx_mps=0.0,
                vy_mps=1.3,
                radius_m=0.40,
                behavior=MovementBehavior.CROSSING,
                waypoint_a_m=(7.5, 2.5),
                waypoint_b_m=(7.5, 12.5),
                reverse_at_bounds=True,
            ),
            DynamicObstacleSpec(
                obstacle_id="DYN-MULTI-3",
                x_m=11.5,
                y_m=12.0,
                vx_mps=0.0,
                vy_mps=-1.2,
                radius_m=0.40,
                behavior=MovementBehavior.CROSSING,
                waypoint_a_m=(11.5, 2.5),
                waypoint_b_m=(11.5, 12.5),
                reverse_at_bounds=True,
            ),
            DynamicObstacleSpec(
                obstacle_id="DYN-MULTI-4",
                x_m=14.5,
                y_m=4.5,
                vx_mps=0.0,
                vy_mps=1.0,
                radius_m=0.40,
                behavior=MovementBehavior.STOP_AND_GO,
                waypoint_a_m=(14.5, 4.0),
                waypoint_b_m=(14.5, 11.0),
                reverse_at_bounds=True,
                move_duration_s=2.0,
                stop_duration_s=1.5,
            ),
        ),
    ),
    "SUDDEN-STOP": ScenarioDefinition(
        name="SUDDEN-STOP",
        behavior=MovementBehavior.STOP_AND_GO,
        description=(
            "Önde giden engel (DYN-SUDDEN-1, v=+1.2 m/s, r=0.45m) t=2.5s anında "
            "x=10.5m konumunda 3.5s boyunca ani duruş yapar."
        ),
        robot_start_m=(2.0, 7.5),
        robot_theta_rad=0.0,
        goal_m=(18.0, 7.5),
        static_obstacles=(
            StaticObstacleConfig(x_m=6.0, y_m=2.0, width_m=8.0, height_m=2.0),
            StaticObstacleConfig(x_m=6.0, y_m=11.0, width_m=8.0, height_m=2.0),
        ),
        dynamic_obstacles=(
            DynamicObstacleSpec(
                obstacle_id="DYN-SUDDEN-1",
                x_m=7.5,
                y_m=7.5,
                vx_mps=1.2,
                vy_mps=0.0,
                radius_m=0.45,
                behavior=MovementBehavior.STOP_AND_GO,
                waypoint_a_m=(4.0, 7.5),
                waypoint_b_m=(16.5, 7.5),
                reverse_at_bounds=True,
                move_duration_s=2.5,
                stop_duration_s=3.5,
            ),
        ),
    ),
    "SUDDEN-STOP-FINITE-DECEL": ScenarioDefinition(
        name="SUDDEN-STOP-FINITE-DECEL",
        behavior=MovementBehavior.STOP_AND_GO,
        description=(
            "Sonlu yavaşlama (DECELERATING): x=9.0m, v0=1.2 m/s, a=-2.0 m/s^2; "
            "0.60 s ve 0.36 m fren mesafesi sonunda x=9.36 m konumunda durur."
        ),
        robot_start_m=(2.0, 7.5),
        robot_theta_rad=0.0,
        goal_m=(18.0, 7.5),
        static_obstacles=(
            StaticObstacleConfig(x_m=6.0, y_m=2.0, width_m=8.0, height_m=2.0),
            StaticObstacleConfig(x_m=6.0, y_m=11.0, width_m=8.0, height_m=2.0),
        ),
        dynamic_obstacles=(),
        sim_objects=(
            SimObjectSpec(
                object_id="OBJ-FINITE-DECEL-1",
                category="DYNAMIC_OBSTACLE",
                x_m=9.0,
                y_m=7.5,
                heading_rad=0.0,
                linear_speed_mps=1.2,
                acceleration_mps2=2.0,
                min_speed_mps=0.0,
                radius_m=0.45,
                behavior="DECELERATING",
            ),
        ),
    ),
}


class ScenarioManager:
    """Provides deterministic lookup and instantiation of named scenarios."""

    @staticmethod
    def list_scenarios() -> list[str]:
        """Return ordered list of available scenario names."""
        return list(SCENARIO_CATALOG.keys())

    @staticmethod
    def get_scenario(name: str) -> ScenarioDefinition:
        """Return ScenarioDefinition by name (case-insensitive)."""
        key = name.upper()
        if key not in SCENARIO_CATALOG:
            raise KeyError(
                f"Unknown scenario '{name}'. Available: {list(SCENARIO_CATALOG.keys())}"
            )
        return SCENARIO_CATALOG[key]
