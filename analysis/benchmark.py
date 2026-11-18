"""Reproducible benchmark runner and raw CSV/JSON exporter for Phase 5.

Compares ASTAR_FOLLOWER (A* + WaypointFollower without local avoidance) and
ASTAR_DWA (A* + DWA with local avoidance and automatic global replanning)
under identical initial conditions, timesteps, and termination rules.
"""

from __future__ import annotations

import csv
import json
import math
from pathlib import Path
import time
from typing import Sequence

from algorithms.astar import AStarPlanner, inflate_occupancy_grid, world_to_grid
from algorithms.dwa import DWAPlanner, replan_global_path_around_obstacles
from algorithms.observation import ObservationPipeline
from algorithms.path_follower import WaypointFollower
from analysis.metrics import (
    RunMetrics,
    RunMetricsTracker,
    compute_instantaneous_clearance_m,
    compute_swept_clearance_m,
)
from config import DEFAULT_CONFIG, ProjectConfig
from simulation.dynamic_obstacles import DynamicObstacle
from simulation.scenarios import ScenarioManager
from simulation.world import SimulationWorld

CONTROLLER_FOLLOWER = "ASTAR_FOLLOWER"
CONTROLLER_DWA = "ASTAR_DWA"


def run_single_experiment(
    scenario_name: str,
    controller_id: str,
    config: ProjectConfig = DEFAULT_CONFIG,
) -> RunMetrics:
    """Execute one deterministic benchmark run for `(scenario_name, controller_id)`."""
    if controller_id not in (CONTROLLER_FOLLOWER, CONTROLLER_DWA):
        raise ValueError(f"Unsupported controller_id '{controller_id}'.")

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
    required_clearance_m = config.robot.radius_m + config.planner.safety_margin_m
    astar_res = AStarPlanner(res_m, config.planner.allow_diagonal).plan(
        inflated,
        start_cell,
        goal_cell,
        raw_grid=world.occupancy_grid,
        required_clearance_m=required_clearance_m,
        enforce_boundary_clearance=config.planner.enforce_boundary_clearance,
    )

    is_dwa = controller_id == CONTROLLER_DWA
    tracker = RunMetricsTracker(
        scenario_name=scenario.name,
        controller_id=controller_id,
        dt_s=dt_s,
        safety_margin_m=exp_cfg.safety_margin_m,
        stop_speed_threshold_mps=exp_cfg.stop_speed_threshold_mps,
        is_dwa=is_dwa,
    )

    if not astar_res.success or not astar_res.path_world_m:
        return tracker.finalize(
            goal_reached=False,
            termination_reason="PLANNING_FAILED",
            sim_time_s=0.0,
            final_speed_mps=0.0,
        )

    follower: WaypointFollower | None = None
    dwa: DWAPlanner | None = None
    obs_pipeline: ObservationPipeline | None = None
    if is_dwa:
        dwa = DWAPlanner(config.world, config.robot, config.dwa, observation_cfg=config.observation)
        dwa.set_path(astar_res.path_world_m)
        obs_pipeline = ObservationPipeline(config.observation)
    else:
        follower = WaypointFollower(config.follower)
        follower.set_path(astar_res.path_world_m)

    world.start()
    cooldown_s = 0.0
    sim_time_s = 0.0
    goal_x, goal_y = scenario.goal_m
    termination_reason = "TIMEOUT"
    goal_reached = False

    for _ in range(max_steps):
        if is_dwa:
            assert dwa is not None
            assert obs_pipeline is not None
            cooldown_s = max(0.0, cooldown_s - dt_s)
            t0 = time.perf_counter()
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
            elapsed_ms = (time.perf_counter() - t0) * 1000.0
            tracker.record_dwa_call(elapsed_ms, dwa_res.blocked)
            if dwa_res.proactive_replan_triggered:
                tracker.record_replan(True)
            world.robot.set_velocity_command(dwa_res.v_mps, dwa_res.omega_radps)

            if dwa_res.blocked and cooldown_s <= 0.0:
                cooldown_s = config.dwa.replan_cooldown_s
                st = world.robot.state
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
                tracker.record_replan(ok_replan and bool(new_path))
                if ok_replan and new_path:
                    dwa.set_path(new_path)
        else:
            assert follower is not None
            v_cmd, w_cmd = follower.compute_command(
                world.robot.state,
                config.robot.max_linear_speed_mps,
                config.robot.max_angular_speed_radps,
                dt_s,
            )
            world.robot.set_velocity_command(v_cmd, w_cmd)

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

        dist_to_goal = math.hypot(goal_x - curr_pos[0], goal_y - curr_pos[1])
        reached_now = dist_to_goal <= exp_cfg.goal_tolerance_m
        tracker.record_step(
            sim_time_s=sim_time_s,
            prev_pos_m=prev_pos,
            curr_pos_m=curr_pos,
            speed_mps=world.robot.state.v_mps,
            inst_clearance_m=inst_c,
            swept_clearance_m=swept_c,
            is_goal_step=reached_now,
        )

        if tracker.has_collided:
            termination_reason = "COLLISION_ABORT"
            goal_reached = False
            break
        if reached_now:
            termination_reason = "GOAL_REACHED"
            goal_reached = True
            break

    return tracker.finalize(
        goal_reached=goal_reached,
        termination_reason=termination_reason,
        sim_time_s=sim_time_s,
        final_speed_mps=world.robot.state.v_mps,
    )


def run_benchmark_suite(
    scenarios: Sequence[str] | None = None,
    controllers: Sequence[str] = (CONTROLLER_FOLLOWER, CONTROLLER_DWA),
    config: ProjectConfig = DEFAULT_CONFIG,
) -> list[RunMetrics]:
    """Run all `(scenario, controller)` pairs and return raw per-run results."""
    sc_list = list(scenarios) if scenarios is not None else list(config.experiment.benchmark_scenarios)
    results: list[RunMetrics] = []
    for sc_name in sc_list:
        for ctrl_id in controllers:
            results.append(run_single_experiment(sc_name, ctrl_id, config))
    return results


def export_results_json(results: Sequence[RunMetrics], output_path: str | Path) -> Path:
    """Export raw per-run metrics to structured JSON (preserving None as null)."""
    path = Path(output_path)
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "schema_version": "5.0",
        "runs": [r.to_dict() for r in results],
    }
    path.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    return path


def export_results_csv(results: Sequence[RunMetrics], output_path: str | Path) -> Path:
    """Export raw per-run metrics to CSV (formatting None DWA timing fields as N/A)."""
    path = Path(output_path)
    path.parent.mkdir(parents=True, exist_ok=True)
    if not results:
        path.write_text("", encoding="utf-8")
        return path

    fieldnames = list(results[0].to_dict().keys())
    with path.open("w", newline="", encoding="utf-8") as fh:
        writer = csv.DictWriter(fh, fieldnames=fieldnames)
        writer.writeheader()
        for r in results:
            row = r.to_dict()
            for k in ("avg_dwa_time_ms", "max_dwa_time_ms", "p95_dwa_time_ms"):
                if row[k] is None:
                    row[k] = "N/A"
            writer.writerow(row)
    return path
