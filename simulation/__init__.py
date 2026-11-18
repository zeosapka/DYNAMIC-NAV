"""Simulation package for DYNAMIC NAV.

Contains pure-math, framework-agnostic 2D world modeling, SI coordinate transforms,
robot kinematics, static/dynamic obstacle models, and deterministic scenarios.
"""

from simulation.coordinates import CoordinateTransform, ScreenRect
from simulation.dynamic_obstacles import (
    DynamicCollisionReport,
    DynamicObstacle,
    MovementBehavior,
    evaluate_dynamic_collisions,
    swept_circle_min_distance_m,
)
from simulation.entities import Robot, RobotState, StaticObstacle
from simulation.safety import (
    SafetyEvent,
    SafetyLevel,
    SafetySnapshot,
    SafetyTracker,
    classify_safety_level,
    compute_dynamic_obstacle_clearance,
    compute_static_obstacle_clearance,
    compute_ttc_s,
)
from simulation.scenarios import ScenarioDefinition, ScenarioManager
from simulation.world import SimulationState, SimulationWorld

__all__ = [
    "CoordinateTransform",
    "ScreenRect",
    "DynamicCollisionReport",
    "DynamicObstacle",
    "MovementBehavior",
    "evaluate_dynamic_collisions",
    "swept_circle_min_distance_m",
    "Robot",
    "RobotState",
    "StaticObstacle",
    "SafetyEvent",
    "SafetyLevel",
    "SafetySnapshot",
    "SafetyTracker",
    "classify_safety_level",
    "compute_dynamic_obstacle_clearance",
    "compute_static_obstacle_clearance",
    "compute_ttc_s",
    "ScenarioDefinition",
    "ScenarioManager",
    "SimulationState",
    "SimulationWorld",
]
