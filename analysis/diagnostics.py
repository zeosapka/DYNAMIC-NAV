"""Optional per-step diagnostic trace exporter for Phase 5.1 failure analysis.

Records robot state, dynamic obstacle states, DWA commands/fallbacks/scores,
braking clearance, stopping distance, replanning state, and safety clearances
without modifying any navigation or collision algorithm behavior.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
from typing import Any

from algorithms.astar import AStarPlanner, inflate_occupancy_grid, world_to_grid
from algorithms.dwa import (
    DWAPlanner,
    compute_stopping_distance_m,
    replan_global_path_around_obstacles,
)
from algorithms.observation import ObservationPipeline
from analysis.metrics import compute_instantaneous_clearance_m, compute_swept_clearance_m
from config import DEFAULT_CONFIG, ProjectConfig
from simulation.dynamic_obstacles import DynamicObstacle
from simulation.scenarios import ScenarioManager
from simulation.world import SimulationWorld


@dataclass(frozen=True)
class DiagnosticStepRecord:
    """Complete per-step snapshot for failure analysis."""

    step: int
    sim_time_s: float
    robot_x_m: float
    robot_y_m: float
    robot_theta_rad: float
    robot_v_mps: float
    robot_omega_radps: float
    stopping_distance_m: float
    obstacles: list[dict[str, Any]]
    dwa_v_mps: float
    dwa_omega_radps: float
    dwa_score: float | None
    predicted_min_clearance_m: float | None
    min_braking_clearance_m: float | None
    min_braking_clearance_time_s: float | None
    fallback_active: bool
    fallback_reason: str | None
    collision_free_exists: bool
    blocked: bool
    dwa_status: str
    candidate_count: int
    rejected_count: int
    inst_clearance_m: float
    swept_clearance_m: float
    replan_attempts: int
    replan_successes: int
    replan_triggered: bool
    waiting_state: str = "GATE_CLEAR_HOLD_SAFE"
    waiting_min_clearance_m: float | None = None
    holding_pose_m: tuple[float, float] | None = None
    reverse_active: bool = False
    fallback_priority: int | None = None
    fallback_duration_s: float = 0.0
    deadlock_state: str = "NONE"

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def run_diagnostic_trace(
    scenario_name: str,
    config: ProjectConfig = DEFAULT_CONFIG,
) -> tuple[str, list[DiagnosticStepRecord]]:
    """Run `scenario_name` with ASTAR_DWA and collect full per-step diagnostic trace."""
    exp_cfg = config.experiment
    dt_s = exp_cfg.dt_s
    max_steps = max(1, round(exp_cfg.max_sim_time_s / dt_s))

    scenario = ScenarioManager.get_scenario(scenario_name)
    world = SimulationWorld(
        config.world,
        config.robot,
        config.simulation,
        safety_margin_m=exp_cfg.safety_margin_m,
    )
    world.load_scenario(scenario)

    res_m = config.world.grid_resolution_m
    inflated = inflate_occupancy_grid(
        world.occupancy_grid,
        res_m,
        config.robot.radius_m,
        config.planner.safety_margin_m,
        enforce_boundary_clearance=config.planner.enforce_boundary_clearance,
    )
    start_cell = world_to_grid(
        scenario.robot_start_m[0], scenario.robot_start_m[1], res_m, inflated.shape
    )
    goal_cell = world_to_grid(
        scenario.goal_m[0], scenario.goal_m[1], res_m, inflated.shape
    )
    astar_res = AStarPlanner(res_m, config.planner.allow_diagonal).plan(
        inflated,
        start_cell,
        goal_cell,
        raw_grid=world.occupancy_grid,
        required_clearance_m=config.robot.radius_m + config.planner.safety_margin_m,
        enforce_boundary_clearance=config.planner.enforce_boundary_clearance,
    )
    if not astar_res.success or not astar_res.path_world_m:
        return ("PLANNING_FAILED", [])

    dwa = DWAPlanner(config.world, config.robot, config.dwa, observation_cfg=config.observation)
    dwa.set_path(astar_res.path_world_m)
    obs_pipeline = ObservationPipeline(config.observation)
    world.start()

    cooldown_s = 0.0
    replan_attempts = 0
    replan_successes = 0
    sim_time_s = 0.0
    goal_x, goal_y = scenario.goal_m
    records: list[DiagnosticStepRecord] = []
    termination = "TIMEOUT"

    for step_idx in range(1, max_steps + 1):
        cooldown_s = max(0.0, cooldown_s - dt_s)
        obs_frame = obs_pipeline.observe(
            sim_time_s,
            world.robot.state,
            world.dynamic_obstacles,
            sim_objects=world.sim_objects,
        )
        dwa_res = dwa.compute_command(
            world.robot.state,
            dt_s,
            world.obstacles,
            world.dynamic_obstacles,
            sim_objects=world.sim_objects,
            observation_frame=obs_frame,
        )
        world.robot.set_velocity_command(dwa_res.v_mps, dwa_res.omega_radps)

        replan_triggered = False
        if dwa_res.blocked and cooldown_s <= 0.0:
            replan_triggered = True
            replan_attempts += 1
            cooldown_s = config.dwa.replan_cooldown_s
            ok_replan, new_path = replan_global_path_around_obstacles(
                base_occupancy_grid=world.occupancy_grid,
                resolution_m=res_m,
                robot_x_m=obs_frame.robot.x_m,
                robot_y_m=obs_frame.robot.y_m,
                goal_x_m=goal_x,
                goal_y_m=goal_y,
                robot_radius_m=config.robot.radius_m,
                safety_margin_m=config.planner.safety_margin_m,
                dynamic_obstacles=world.dynamic_obstacles,
                allow_diagonal=config.planner.allow_diagonal,
                enforce_boundary_clearance=config.planner.enforce_boundary_clearance,
                sim_objects=world.sim_objects,
                observation_frame=obs_frame,
            )
            if ok_replan and new_path:
                replan_successes += 1
                dwa.set_path(new_path)

        prev_pos = (world.robot.state.x_m, world.robot.state.y_m)
        world.robot.step(
            dt_s=dt_s,
            world_width_m=world.world_config.width_m,
            world_height_m=world.world_config.height_m,
            obstacles=world.obstacles,
        )
        curr_pos = (world.robot.state.x_m, world.robot.state.y_m)

        transitions: list[
            tuple[DynamicObstacle, tuple[float, float], tuple[float, float]]
        ] = []
        for obs in world.dynamic_obstacles:
            px, py, nx, ny = obs.step(dt_s)
            transitions.append((obs, (px, py), (nx, ny)))
        world.object_editor.step_all(dt_s)
        sim_time_s += dt_s

        inst_c = compute_instantaneous_clearance_m(
            curr_pos[0],
            curr_pos[1],
            world.robot.radius_m,
            world.world_config,
            world.obstacles,
            world.dynamic_obstacles,
            world.sim_objects,
        )
        swept_c = compute_swept_clearance_m(
            prev_pos,
            curr_pos,
            world.robot.radius_m,
            world.world_config,
            world.obstacles,
            transitions,
            world.sim_objects,
        )

        rs = world.robot.state
        stop_d = compute_stopping_distance_m(
            rs.v_mps, config.dwa.max_brake_decel_mps2, config.dwa.reaction_time_s
        )
        obs_snap = [
            {
                "id": o.obstacle_id,
                "x_m": o.x_m,
                "y_m": o.y_m,
                "vx_mps": o.vx_mps,
                "vy_mps": o.vy_mps,
                "radius_m": o.radius_m,
                "behavior": o.behavior.value,
            }
            for o in world.dynamic_obstacles
        ]
        records.append(
            DiagnosticStepRecord(
                step=step_idx,
                sim_time_s=sim_time_s,
                robot_x_m=rs.x_m,
                robot_y_m=rs.y_m,
                robot_theta_rad=rs.theta_rad,
                robot_v_mps=rs.v_mps,
                robot_omega_radps=rs.omega_radps,
                stopping_distance_m=stop_d,
                obstacles=obs_snap,
                dwa_v_mps=dwa_res.v_mps,
                dwa_omega_radps=dwa_res.omega_radps,
                dwa_score=dwa_res.score,
                predicted_min_clearance_m=dwa_res.predicted_min_clearance_m,
                min_braking_clearance_m=dwa_res.min_braking_clearance_m,
                min_braking_clearance_time_s=dwa_res.min_braking_clearance_time_s,
                fallback_active=dwa_res.fallback_active,
                fallback_reason=dwa_res.fallback_reason,
                collision_free_exists=dwa_res.collision_free_exists,
                blocked=dwa_res.blocked,
                dwa_status=dwa_res.status,
                candidate_count=len(dwa_res.candidate_trajectories),
                rejected_count=len(dwa_res.rejected_trajectories),
                inst_clearance_m=inst_c,
                swept_clearance_m=swept_c,
                replan_attempts=replan_attempts,
                replan_successes=replan_successes,
                replan_triggered=replan_triggered,
                waiting_state=dwa_res.waiting_state,
                waiting_min_clearance_m=dwa_res.waiting_min_clearance_m,
                holding_pose_m=dwa_res.holding_pose_m,
                reverse_active=dwa_res.reverse_active,
                fallback_priority=dwa_res.fallback_priority,
                fallback_duration_s=dwa_res.fallback_duration_s,
                deadlock_state=dwa_res.deadlock_state,
            )
        )

        if min(inst_c, swept_c) <= 0.0:
            termination = "COLLISION_ABORT"
            break
        if math.hypot(goal_x - rs.x_m, goal_y - rs.y_m) <= exp_cfg.goal_tolerance_m:
            termination = "GOAL_REACHED"
            break

    return (termination, records)


def export_diagnostic_trace_json(
    scenario_name: str,
    termination: str,
    records: list[DiagnosticStepRecord],
    output_path: str | Path,
) -> Path:
    """Export per-step diagnostic trace to JSON."""
    path = Path(output_path)
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "scenario_name": scenario_name,
        "termination_reason": termination,
        "steps": [r.to_dict() for r in records],
    }
    path.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    return path
