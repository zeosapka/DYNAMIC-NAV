"""Navigation and obstacle avoidance algorithms package."""

from algorithms.astar import (
    AStarPlanner,
    AStarResult,
    grid_to_world,
    inflate_occupancy_grid,
    is_segment_collision_free,
    manhattan_heuristic_m,
    octile_heuristic_m,
    segment_to_square_distance_m,
    world_to_grid,
)
from algorithms.corridor_passing import (
    CandidatePathEvaluation,
    CorridorAssessment,
    CorridorPassingManager,
)
from algorithms.dwa import (
    DWAPlanner,
    DWAResult,
    DeadlockState,
    DynamicWindow,
    WaitingZoneAssessment,
    WaitingZoneState,
    compute_dynamic_window,
    compute_stopping_distance_m,
    evaluate_trajectory_clearance,
    evaluate_trajectory_with_braking,
    evaluate_waiting_zone_safety,
    predict_trajectory,
    replan_global_path_around_obstacles,
    sample_velocities,
    score_trajectory,
    select_safe_holding_pose,
    simulate_braking_trajectory,
)
from algorithms.dwa_interface import DWATelemetryPlaceholder
from algorithms.observation import (
    ObservationFrame,
    ObservationPipeline,
    ObservedObstacle,
    ObservedRobotState,
    ObstacleStateEstimator,
)
from algorithms.path_follower import WaypointFollower

__all__ = [
    "AStarPlanner",
    "AStarResult",
    "DWAPlanner",
    "DWAResult",
    "DynamicWindow",
    "DWATelemetryPlaceholder",
    "ObservationFrame",
    "ObservationPipeline",
    "ObservedObstacle",
    "ObservedRobotState",
    "ObstacleStateEstimator",
    "WaypointFollower",
    "compute_dynamic_window",
    "compute_stopping_distance_m",
    "evaluate_trajectory_clearance",
    "evaluate_trajectory_with_braking",
    "grid_to_world",
    "inflate_occupancy_grid",
    "is_segment_collision_free",
    "manhattan_heuristic_m",
    "octile_heuristic_m",
    "predict_trajectory",
    "replan_global_path_around_obstacles",
    "sample_velocities",
    "score_trajectory",
    "segment_to_square_distance_m",
    "simulate_braking_trajectory",
    "world_to_grid",
]
