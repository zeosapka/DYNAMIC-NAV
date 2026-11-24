"""Deterministic scenario definitions and management for DYNAMIC NAV.

All scenario parameters are specified in SI units (meters, radians, m/s, seconds)
and are strictly reproducible without random generation.
"""

from dataclasses import dataclass
import json
import math
from pathlib import Path

from config import RobotConfig, StaticObstacleConfig, WorldConfig
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

    @staticmethod
    def load_json(
        json_path: str | Path,
        world_cfg: WorldConfig = WorldConfig(),
        robot_cfg: RobotConfig = RobotConfig(),
    ) -> ScenarioDefinition:
        """Load and validate a custom ScenarioDefinition from a JSON file."""
        return load_scenario_from_json(json_path, world_cfg=world_cfg, robot_cfg=robot_cfg)


def _point_in_or_near_aabb(
    px: float,
    py: float,
    obs: StaticObstacleConfig,
    margin_m: float,
) -> bool:
    cx = max(obs.x_m, min(px, obs.x_m + obs.width_m))
    cy = max(obs.y_m, min(py, obs.y_m + obs.height_m))
    return math.hypot(px - cx, py - cy) < margin_m


def _validate_point_in_world(
    name: str,
    x_m: float,
    y_m: float,
    world_cfg: WorldConfig,
    clearance_m: float,
) -> None:
    if not (math.isfinite(x_m) and math.isfinite(y_m)):
        raise ValueError(f"{name} coordinates must be finite: ({x_m}, {y_m}).")
    if (
        x_m < clearance_m
        or x_m > world_cfg.width_m - clearance_m
        or y_m < clearance_m
        or y_m > world_cfg.height_m - clearance_m
    ):
        raise ValueError(
            f"{name} ({x_m}, {y_m}) is out of world bounds [0, {world_cfg.width_m}] x [0, {world_cfg.height_m}] "
            f"with clearance {clearance_m} m."
        )


def load_scenario_from_json(
    json_path: str | Path,
    world_cfg: WorldConfig = WorldConfig(),
    robot_cfg: RobotConfig = RobotConfig(),
) -> ScenarioDefinition:
    """Parse and validate a custom scenario JSON file into a ScenarioDefinition (SI units)."""
    path = Path(json_path)
    raw = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(raw, dict):
        raise ValueError("Scenario JSON root must be an object.")

    name = str(raw.get("name", path.stem)).strip() or path.stem
    description = str(raw.get("description", "Custom JSON scenario"))
    start_raw = raw.get("robot_start_m")
    goal_raw = raw.get("goal_m")
    if not (isinstance(start_raw, (list, tuple)) and len(start_raw) == 2):
        raise ValueError("robot_start_m must be a 2-element [x_m, y_m] list.")
    if not (isinstance(goal_raw, (list, tuple)) and len(goal_raw) == 2):
        raise ValueError("goal_m must be a 2-element [x_m, y_m] list.")

    sx, sy = float(start_raw[0]), float(start_raw[1])
    gx, gy = float(goal_raw[0]), float(goal_raw[1])
    theta = float(raw.get("robot_theta_rad", 0.0))
    if not math.isfinite(theta):
        raise ValueError(f"robot_theta_rad must be finite: {theta}.")

    _validate_point_in_world("robot_start_m", sx, sy, world_cfg, robot_cfg.radius_m)
    _validate_point_in_world("goal_m", gx, gy, world_cfg, robot_cfg.radius_m)

    static_list: list[StaticObstacleConfig] = []
    for idx, obs_raw in enumerate(raw.get("static_obstacles", [])):
        ox = float(obs_raw["x_m"])
        oy = float(obs_raw["y_m"])
        ow = float(obs_raw["width_m"])
        oh = float(obs_raw["height_m"])
        if not all(math.isfinite(v) for v in (ox, oy, ow, oh)) or ow <= 0.0 or oh <= 0.0:
            raise ValueError(f"static_obstacles[{idx}] has invalid dimensions or non-finite values.")
        if ox < 0.0 or oy < 0.0 or ox + ow > world_cfg.width_m or oy + oh > world_cfg.height_m:
            raise ValueError(f"static_obstacles[{idx}] exceeds world bounds.")
        sobs = StaticObstacleConfig(x_m=ox, y_m=oy, width_m=ow, height_m=oh)
        if _point_in_or_near_aabb(sx, sy, sobs, robot_cfg.radius_m):
            raise ValueError(f"robot_start_m overlaps static_obstacles[{idx}].")
        if _point_in_or_near_aabb(gx, gy, sobs, robot_cfg.radius_m):
            raise ValueError(f"goal_m overlaps static_obstacles[{idx}].")
        static_list.append(sobs)

    dyn_list: list[DynamicObstacleSpec] = []
    max_obs_speed = 5.0
    for idx, d_raw in enumerate(raw.get("dynamic_obstacles", [])):
        oid = str(d_raw.get("obstacle_id", f"DYN-CUSTOM-{idx + 1}"))
        dx = float(d_raw["x_m"])
        dy = float(d_raw["y_m"])
        vx = float(d_raw.get("vx_mps", 0.0))
        vy = float(d_raw.get("vy_mps", 0.0))
        rad = float(d_raw.get("radius_m", 0.35))
        if not all(math.isfinite(v) for v in (dx, dy, vx, vy, rad)) or rad <= 0.0:
            raise ValueError(f"dynamic_obstacles[{idx}] has invalid radius or non-finite values.")
        if math.hypot(vx, vy) > max_obs_speed:
            raise ValueError(
                f"dynamic_obstacles[{idx}] speed {math.hypot(vx, vy):.2f} m/s exceeds {max_obs_speed} m/s."
            )
        _validate_point_in_world(f"dynamic_obstacles[{idx}]", dx, dy, world_cfg, rad)
        if math.hypot(dx - sx, dy - sy) <= robot_cfg.radius_m + rad:
            raise ValueError(f"dynamic_obstacles[{idx}] overlaps robot_start_m.")

        beh_name = str(d_raw.get("behavior", "HEAD_ON")).upper()
        try:
            beh = MovementBehavior[beh_name]
        except KeyError as exc:
            raise ValueError(f"Unknown dynamic obstacle behavior '{beh_name}'.") from exc

        wp_a = d_raw.get("waypoint_a_m")
        wp_b = d_raw.get("waypoint_b_m")
        wp_a_t = (float(wp_a[0]), float(wp_a[1])) if wp_a is not None else None
        wp_b_t = (float(wp_b[0]), float(wp_b[1])) if wp_b is not None else None
        if wp_a_t is not None:
            _validate_point_in_world(f"dynamic_obstacles[{idx}].waypoint_a_m", wp_a_t[0], wp_a_t[1], world_cfg, rad)
        if wp_b_t is not None:
            _validate_point_in_world(f"dynamic_obstacles[{idx}].waypoint_b_m", wp_b_t[0], wp_b_t[1], world_cfg, rad)

        dyn_list.append(
            DynamicObstacleSpec(
                obstacle_id=oid,
                x_m=dx,
                y_m=dy,
                vx_mps=vx,
                vy_mps=vy,
                radius_m=rad,
                behavior=beh,
                active=bool(d_raw.get("active", True)),
                waypoint_a_m=wp_a_t,
                waypoint_b_m=wp_b_t,
                reverse_at_bounds=bool(d_raw.get("reverse_at_bounds", True)),
                move_duration_s=float(d_raw.get("move_duration_s", 2.0)),
                stop_duration_s=float(d_raw.get("stop_duration_s", 1.5)),
            )
        )

    top_beh_name = str(raw.get("behavior", dyn_list[0].behavior.name if dyn_list else "HEAD_ON")).upper()
    try:
        top_beh = MovementBehavior[top_beh_name]
    except KeyError as exc:
        raise ValueError(f"Unknown scenario behavior '{top_beh_name}'.") from exc

    return ScenarioDefinition(
        name=name,
        behavior=top_beh,
        description=description,
        robot_start_m=(sx, sy),
        robot_theta_rad=theta,
        goal_m=(gx, gy),
        static_obstacles=tuple(static_list),
        dynamic_obstacles=tuple(dyn_list),
    )
