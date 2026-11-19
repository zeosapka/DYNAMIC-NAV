"""Centralized configuration for the DYNAMIC NAV simulator.

All physical quantities are expressed in SI units (meters, seconds, radians, m/s, rad/s).
Screen dimensions and UI layout parameters are expressed in pixels.
"""

from dataclasses import dataclass, field


@dataclass(frozen=True)
class StaticObstacleConfig:
    """Axis-aligned rectangular static obstacle in world SI coordinates (meters)."""

    x_m: float
    y_m: float
    width_m: float
    height_m: float


@dataclass(frozen=True)
class WindowConfig:
    """Pygame window and layout configuration (pixels & frames per second)."""

    width: int = 1440
    height: int = 900
    fps: int = 60
    title: str = "DYNAMIC NAV — Otonom Mobil Robot Navigasyon Simülatörü"
    resizable: bool = False
    sidebar_width: int = 340
    viewport_padding: int = 14
    top_bar_height: int = 104
    left_rail_width: int = 232
    right_inspector_width: int = 316
    bottom_dock_height: int = 140
    demo_speed_presets: tuple[float, ...] = (0.5, 1.0, 1.5, 2.0)
    demo_event_duration_s: float = 2.8


@dataclass(frozen=True)
class ThemeConfig:
    """Dark engineering UI color palette (RGB tuples)."""

    background: tuple[int, int, int] = (15, 20, 27)
    viewport_bg: tuple[int, int, int] = (19, 25, 34)
    sidebar_bg: tuple[int, int, int] = (22, 29, 39)
    panel_bg: tuple[int, int, int] = (28, 37, 49)
    border: tuple[int, int, int] = (46, 60, 79)
    grid_minor: tuple[int, int, int] = (29, 39, 54)
    grid_major: tuple[int, int, int] = (45, 61, 84)
    axis_text: tuple[int, int, int] = (130, 148, 170)

    text_primary: tuple[int, int, int] = (230, 237, 243)
    text_secondary: tuple[int, int, int] = (148, 163, 184)
    accent: tuple[int, int, int] = (56, 189, 248)

    robot_fill: tuple[int, int, int] = (34, 211, 238)
    robot_outline: tuple[int, int, int] = (165, 243, 252)
    robot_heading: tuple[int, int, int] = (245, 158, 11)

    obstacle_fill: tuple[int, int, int] = (71, 85, 105)
    obstacle_border: tuple[int, int, int] = (239, 68, 68)
    inflated_cell: tuple[int, int, int] = (55, 42, 52)

    start_marker: tuple[int, int, int] = (56, 189, 248)
    goal_marker: tuple[int, int, int] = (34, 197, 94)
    explored_node: tuple[int, int, int] = (30, 64, 95)
    path_line: tuple[int, int, int] = (59, 130, 246)
    path_node: tuple[int, int, int] = (96, 165, 250)

    dyn_obs_fill: tuple[int, int, int] = (249, 115, 22)
    dyn_obs_outline: tuple[int, int, int] = (254, 215, 170)
    dyn_obs_vector: tuple[int, int, int] = (253, 224, 71)
    dyn_obs_warning: tuple[int, int, int] = (245, 158, 11)
    dyn_obs_collision: tuple[int, int, int] = (239, 68, 68)

    safety_safe: tuple[int, int, int] = (34, 197, 94)
    safety_caution: tuple[int, int, int] = (250, 204, 21)
    safety_critical: tuple[int, int, int] = (249, 115, 22)
    safety_collision: tuple[int, int, int] = (239, 68, 68)
    nav_blue: tuple[int, int, int] = (56, 189, 248)

    status_ready: tuple[int, int, int] = (56, 189, 248)
    status_running: tuple[int, int, int] = (34, 197, 94)
    status_paused: tuple[int, int, int] = (245, 158, 11)

    button_bg: tuple[int, int, int] = (36, 48, 64)
    button_hover: tuple[int, int, int] = (51, 68, 90)
    button_active: tuple[int, int, int] = (14, 116, 144)
    button_disabled: tuple[int, int, int] = (28, 35, 45)
    button_text_disabled: tuple[int, int, int] = (90, 102, 118)

    adas_topbar_bg: tuple[int, int, int] = (13, 18, 28)
    adas_rail_bg: tuple[int, int, int] = (17, 24, 36)
    adas_card_bg: tuple[int, int, int] = (24, 33, 48)
    adas_card_border: tuple[int, int, int] = (44, 59, 82)
    adas_selected: tuple[int, int, int] = (56, 189, 248)
    adas_measure_line: tuple[int, int, int] = (168, 85, 247)
    adas_chart_line: tuple[int, int, int] = (34, 211, 238)
    adas_vehicle_car: tuple[int, int, int] = (59, 130, 246)
    adas_vehicle_truck: tuple[int, int, int] = (139, 92, 246)
    adas_vehicle_bus: tuple[int, int, int] = (234, 179, 8)
    adas_vehicle_forklift: tuple[int, int, int] = (245, 158, 11)
    adas_pedestrian: tuple[int, int, int] = (16, 185, 129)
    adas_cyclist: tuple[int, int, int] = (6, 182, 212)
    adas_custom: tuple[int, int, int] = (236, 72, 153)


@dataclass(frozen=True)
class WorldConfig:
    """2D grid-based simulation environment parameters in SI units (meters)."""

    width_m: float = 20.0
    height_m: float = 15.0
    grid_resolution_m: float = 0.2
    minor_grid_interval_m: float = 1.0
    major_grid_interval_m: float = 5.0
    static_obstacles: tuple[StaticObstacleConfig, ...] = (
        StaticObstacleConfig(x_m=5.0, y_m=3.0, width_m=2.0, height_m=5.0),
        StaticObstacleConfig(x_m=10.0, y_m=8.0, width_m=3.0, height_m=2.0),
        StaticObstacleConfig(x_m=14.0, y_m=2.0, width_m=2.0, height_m=4.0),
        StaticObstacleConfig(x_m=3.0, y_m=10.0, width_m=4.0, height_m=2.0),
        StaticObstacleConfig(x_m=15.0, y_m=10.0, width_m=2.0, height_m=3.0),
    )


@dataclass(frozen=True)
class RobotConfig:
    """Differential-drive / unicycle mobile robot parameters in SI units."""

    initial_x_m: float = 2.0
    initial_y_m: float = 2.0
    initial_theta_rad: float = 0.0
    radius_m: float = 0.35
    max_linear_speed_mps: float = 3.0
    max_angular_speed_radps: float = 2.8


@dataclass(frozen=True)
class SimulationConfig:
    """Simulation timing and execution parameters."""

    max_dt_s: float = 0.05
    auto_start: bool = False


@dataclass(frozen=True)
class PlannerConfig:
    """Global A* path planner configuration in SI units."""

    allow_diagonal: bool = True
    safety_margin_m: float = 0.15
    enforce_boundary_clearance: bool = True
    default_goal_x_m: float = 18.0
    default_goal_y_m: float = 13.0
    interactive_obstacle_size_m: float = 1.0


@dataclass(frozen=True)
class FollowerConfig:
    """Autonomous global waypoint-following controller parameters in SI units."""

    waypoint_tolerance_m: float = 0.22
    goal_tolerance_m: float = 0.15
    angular_gain: float = 3.0
    rotate_in_place_threshold_rad: float = 0.9
    max_linear_accel_mps2: float = 4.0
    max_angular_accel_radps2: float = 6.0


@dataclass(frozen=True)
class SafetyThresholdConfig:
    """Surface-to-surface clearance visualization & analytics thresholds (SI meters)."""

    safe_clearance_m: float = 0.30
    caution_clearance_m: float = 0.15
    critical_clearance_m: float = 0.0
    near_miss_threshold_m: float = 0.15
    history_max_samples: int = 240
    event_log_max_entries: int = 8


@dataclass(frozen=True)
class DWAConfig:
    """Dynamic Window Approach (DWA) local obstacle avoidance parameters in SI units."""

    min_linear_speed_mps: float = 0.0
    max_linear_accel_mps2: float = 5.0
    max_angular_accel_radps2: float = 16.0
    max_brake_decel_mps2: float = 5.0
    reaction_time_s: float = 0.10
    linear_samples: int = 7
    angular_samples: int = 17
    predict_time_s: float = 1.2
    predict_dt_s: float = 0.1
    heading_weight: float = 1.3
    clearance_weight: float = 0.8
    velocity_weight: float = 1.8
    path_weight: float = 1.0
    lookahead_distance_m: float = 1.8
    goal_tolerance_m: float = 0.25
    safety_margin_m: float = 0.15
    max_clearance_cap_m: float = 0.65
    corridor_passage_margin_m: float = 0.35
    deadlock_timeout_s: float = 2.5
    replan_cooldown_s: float = 2.0
    waiting_horizon_s: float = 2.5
    waiting_dt_s: float = 0.1
    waiting_uncertainty_pad_m: float = 0.05
    waiting_lateral_offset_m: float = 1.4
    min_reverse_speed_mps: float = -0.6
    fallback_replan_timeout_s: float = 1.5
    progress_epsilon_m: float = 0.10
    proactive_replan_enabled: bool = True
    proactive_path_lookahead_m: float = 3.0
    proactive_path_clearance_threshold_m: float = 0.25
    proactive_obs_dist_m: float = 2.8
    proactive_min_speed_diff_mps: float = 0.25
    proactive_hysteresis_steps: int = 3
    proactive_replan_cooldown_s: float = 2.5
    proactive_min_clearance_improvement_m: float = 0.10
    proactive_open_space_enabled: bool = True
    proactive_open_space_hysteresis_steps: int = 8
    causal_stop_safety_enabled: bool = True
    causal_reversal_horizon_s: float = 0.35
    causal_straight_reversal_horizon_s: float = 0.85
    causal_lateral_escape_threshold_m: float = 0.10


@dataclass(frozen=True)
class ObjectDefaultSpec:
    """Default SI dimensions, dynamics, and safety values for an object category."""

    width_m: float
    length_m: float
    radius_m: float
    default_speed_mps: float
    max_speed_mps: float
    max_accel_mps2: float
    safety_margin_m: float


@dataclass(frozen=True)
class ObjectDefaultsConfig:
    """Configurable SI defaults for all supported universal object categories."""

    static_obstacle: ObjectDefaultSpec = ObjectDefaultSpec(1.0, 1.0, 0.5, 0.0, 0.0, 0.0, 0.15)
    dynamic_obstacle: ObjectDefaultSpec = ObjectDefaultSpec(0.9, 0.9, 0.45, 1.2, 3.0, 3.0, 0.25)
    passenger_car: ObjectDefaultSpec = ObjectDefaultSpec(1.85, 4.40, 1.10, 2.0, 8.0, 4.0, 0.40)
    truck: ObjectDefaultSpec = ObjectDefaultSpec(2.45, 7.50, 1.40, 1.5, 6.0, 2.5, 0.50)
    bus: ObjectDefaultSpec = ObjectDefaultSpec(2.50, 9.00, 1.45, 1.5, 6.0, 2.5, 0.50)
    forklift: ObjectDefaultSpec = ObjectDefaultSpec(1.15, 2.20, 0.70, 1.2, 3.5, 2.5, 0.30)
    pedestrian: ObjectDefaultSpec = ObjectDefaultSpec(0.55, 0.55, 0.28, 1.2, 2.5, 2.0, 0.35)
    cyclist: ObjectDefaultSpec = ObjectDefaultSpec(0.65, 1.75, 0.45, 2.5, 6.0, 3.0, 0.40)
    custom: ObjectDefaultSpec = ObjectDefaultSpec(1.20, 1.80, 0.60, 1.0, 5.0, 3.0, 0.25)


@dataclass(frozen=True)
class EditorConfig:
    """Interactive object editor and measurement tool parameters."""

    selection_tolerance_m: float = 0.45
    min_dimension_m: float = 0.20
    max_dimension_m: float = 14.0
    min_radius_m: float = 0.15
    max_radius_m: float = 5.0
    position_nudge_m: float = 0.25
    angle_nudge_rad: float = 0.2617993877991494  # 15 deg
    size_nudge_m: float = 0.20
    default_safety_margin_m: float = 0.15
    default_ttc_warning_s: float = 3.0


@dataclass(frozen=True)
class ExperimentConfig:
    """Reproducible Phase 5 benchmark runner and metric configuration in SI units."""

    dt_s: float = 0.05
    max_sim_time_s: float = 60.0
    safety_margin_m: float = 0.15
    stop_speed_threshold_mps: float = 0.05
    goal_tolerance_m: float = 0.25
    export_dir: str = "analysis/results"
    benchmark_scenarios: tuple[str, ...] = (
        "HEAD-ON",
        "CROSSING",
        "NARROW-GATE",
        "MULTIPLE-DYNAMIC-OBSTACLES",
        "SUDDEN-STOP",
    )


@dataclass(frozen=True)
class ObservationConfig:
    """Causal sensor observation, state estimation, and uncertainty configuration (SI units)."""

    observation_latency_s: float = 0.0
    max_observation_age_s: float = 0.50
    position_uncertainty_m: float = 0.0
    velocity_uncertainty_mps: float = 0.0
    initial_velocity_uncertainty_mps: float = 0.25
    uncertainty_growth_rate_mps: float = 0.02  # 0.05 m over 2.5 s waiting horizon
    velocity_ema_alpha: float = 1.0


@dataclass(frozen=True)
class ProjectConfig:
    """Top-level configuration container for the entire application."""

    window: WindowConfig = field(default_factory=WindowConfig)
    theme: ThemeConfig = field(default_factory=ThemeConfig)
    world: WorldConfig = field(default_factory=WorldConfig)
    robot: RobotConfig = field(default_factory=RobotConfig)
    simulation: SimulationConfig = field(default_factory=SimulationConfig)
    planner: PlannerConfig = field(default_factory=PlannerConfig)
    follower: FollowerConfig = field(default_factory=FollowerConfig)
    safety: SafetyThresholdConfig = field(default_factory=SafetyThresholdConfig)
    dwa: DWAConfig = field(default_factory=DWAConfig)
    object_defaults: ObjectDefaultsConfig = field(default_factory=ObjectDefaultsConfig)
    editor: EditorConfig = field(default_factory=EditorConfig)
    experiment: ExperimentConfig = field(default_factory=ExperimentConfig)
    observation: ObservationConfig = field(default_factory=ObservationConfig)


DEFAULT_CONFIG = ProjectConfig()
