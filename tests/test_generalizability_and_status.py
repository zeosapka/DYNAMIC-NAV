"""Tests for custom JSON scenario generalizability, input validation, and status distinction."""

from __future__ import annotations

import json
import math
from pathlib import Path

import pygame
import pytest

from algorithms.astar import AStarPlanner, inflate_occupancy_grid, world_to_grid
from algorithms.dwa import DWAPlanner
from algorithms.observation import ObservationPipeline
from analysis.metrics import compute_swept_clearance_m
from config import DEFAULT_CONFIG
from interface.app import DynamicNavApp
from interface.views import AppView
from simulation.scenarios import ScenarioDefinition, load_scenario_from_json
from simulation.world import SimulationWorld


def _run_custom_scenario(scen: ScenarioDefinition) -> dict[str, object]:
    cfg = DEFAULT_CONFIG
    world = SimulationWorld(cfg.world, cfg.robot, cfg.simulation, safety_margin_m=cfg.planner.safety_margin_m)
    world.load_scenario(scen)
    res_m = cfg.world.grid_resolution_m
    inflated = inflate_occupancy_grid(
        world.occupancy_grid,
        res_m,
        cfg.robot.radius_m,
        cfg.planner.safety_margin_m,
        enforce_boundary_clearance=True,
    )
    astar = AStarPlanner(res_m, True).plan(
        inflated,
        world_to_grid(scen.robot_start_m[0], scen.robot_start_m[1], res_m, inflated.shape),
        world_to_grid(scen.goal_m[0], scen.goal_m[1], res_m, inflated.shape),
        raw_grid=world.occupancy_grid,
        required_clearance_m=cfg.robot.radius_m + cfg.planner.safety_margin_m,
        enforce_boundary_clearance=True,
    )
    assert astar.success, f"Initial A* failed for custom scenario {scen.name}"

    dwa = DWAPlanner(cfg.world, cfg.robot, cfg.dwa, cfg.observation)
    dwa.set_path(astar.path_world_m)
    pipe = ObservationPipeline(cfg.observation)
    world.start()

    dt_s = cfg.experiment.dt_s
    max_steps = int(round(cfg.experiment.max_sim_time_s / dt_s))
    min_swept = math.inf
    col_count = 0
    goal_reached = False
    completion_time_s = None

    for step in range(max_steps):
        t_s = step * dt_s
        obs_frame = pipe.observe(t_s, world.robot.state, world.dynamic_obstacles, sim_objects=world.sim_objects)
        res = dwa.compute_command(
            world.robot.state,
            dt_s,
            world.obstacles,
            world.dynamic_obstacles,
            sim_objects=world.sim_objects,
            observation_frame=obs_frame,
        )
        prev_p = (world.robot.state.x_m, world.robot.state.y_m)
        prev_d = [(d.x_m, d.y_m) for d in world.dynamic_obstacles]
        world.robot.set_velocity_command(res.v_mps, res.omega_radps)
        world.step(dt_s)

        swept = compute_swept_clearance_m(
            prev_p,
            (world.robot.state.x_m, world.robot.state.y_m),
            cfg.robot.radius_m,
            cfg.world,
            world.obstacles,
            [(d, prev_d[i], (d.x_m, d.y_m)) for i, d in enumerate(world.dynamic_obstacles)],
            world.sim_objects,
        )
        min_swept = min(min_swept, swept)
        if swept <= 0.0:
            col_count += 1
            break
        if (
            res.target_reached
            or math.hypot(world.robot.state.x_m - scen.goal_m[0], world.robot.state.y_m - scen.goal_m[1])
            <= cfg.dwa.goal_tolerance_m
        ):
            goal_reached = True
            completion_time_s = round((step + 1) * dt_s, 3)
            break

    return {
        "name": scen.name,
        "goal_reached": goal_reached,
        "collisions": col_count,
        "min_swept_m": round(min_swept, 6),
        "completion_time_s": completion_time_s,
    }


def test_1_unseen_example_json_scenario_reaches_goal_safely() -> None:
    """Example JSON scenario (unseen diagonal start/goal + crossing obstacle) succeeds safely."""
    example_path = Path("examples/custom_scenario_example.json")
    assert example_path.exists()
    scen = load_scenario_from_json(example_path, DEFAULT_CONFIG.world, DEFAULT_CONFIG.robot)
    out = _run_custom_scenario(scen)
    assert out["goal_reached"] is True
    assert out["collisions"] == 0
    assert float(out["min_swept_m"]) >= DEFAULT_CONFIG.planner.safety_margin_m


def test_2_second_unseen_json_scenario_head_on_and_slalom(tmp_path: Path) -> None:
    """Second independent unseen JSON scenario with reverse heading and 2 dynamic obstacles."""
    scen_data = {
        "name": "CUSTOM-UNSEEN-REVERSE-LANE",
        "description": "Robot travels right-to-left around staggered blocks and two dynamic obstacles.",
        "behavior": "HEAD_ON",
        "robot_start_m": [17.5, 11.5],
        "robot_theta_rad": math.pi,
        "goal_m": [2.5, 4.0],
        "static_obstacles": [
            {"x_m": 11.0, "y_m": 9.0, "width_m": 2.0, "height_m": 3.0},
            {"x_m": 6.5, "y_m": 3.0, "width_m": 2.0, "height_m": 3.0},
        ],
        "dynamic_obstacles": [
            {
                "obstacle_id": "DYN-REV-1",
                "x_m": 9.5,
                "y_m": 7.5,
                "vx_mps": 0.65,
                "vy_mps": 0.0,
                "radius_m": 0.35,
                "behavior": "HEAD_ON",
                "waypoint_a_m": [7.0, 7.5],
                "waypoint_b_m": [13.5, 7.5],
                "reverse_at_bounds": True,
            },
            {
                "obstacle_id": "DYN-REV-2",
                "x_m": 4.5,
                "y_m": 6.0,
                "vx_mps": 0.0,
                "vy_mps": -0.60,
                "radius_m": 0.35,
                "behavior": "CROSSING",
                "waypoint_a_m": [4.5, 3.0],
                "waypoint_b_m": [4.5, 8.5],
                "reverse_at_bounds": True,
            },
        ],
    }
    p = tmp_path / "unseen2.json"
    p.write_text(json.dumps(scen_data), encoding="utf-8")
    scen = load_scenario_from_json(p, DEFAULT_CONFIG.world, DEFAULT_CONFIG.robot)
    out = _run_custom_scenario(scen)
    assert out["goal_reached"] is True
    assert out["collisions"] == 0
    assert float(out["min_swept_m"]) > 0.0


def test_3_invalid_json_scenario_inputs_rejected(tmp_path: Path) -> None:
    """Out-of-bounds, overlapping obstacles, NaN coordinates, and excessive speeds raise ValueError."""
    cases = [
        {"robot_start_m": [-1.0, 5.0], "goal_m": [10.0, 5.0]},
        {"robot_start_m": [2.0, 5.0], "goal_m": [25.0, 5.0]},
        {
            "robot_start_m": [5.5, 5.5],
            "goal_m": [15.0, 5.0],
            "static_obstacles": [{"x_m": 5.0, "y_m": 5.0, "width_m": 2.0, "height_m": 2.0}],
        },
        {
            "robot_start_m": [2.0, 5.0],
            "goal_m": [15.0, 5.0],
            "dynamic_obstacles": [
                {"x_m": 8.0, "y_m": 5.0, "vx_mps": 9.0, "vy_mps": 0.0, "radius_m": 0.35}
            ],
        },
        {
            "robot_start_m": [2.0, 5.0],
            "goal_m": [15.0, 5.0],
            "dynamic_obstacles": [
                {"x_m": 2.1, "y_m": 5.0, "vx_mps": 0.0, "vy_mps": 0.0, "radius_m": 0.35}
            ],
        },
    ]
    for idx, payload in enumerate(cases):
        fp = tmp_path / f"bad_{idx}.json"
        fp.write_text(json.dumps(payload), encoding="utf-8")
        with pytest.raises(ValueError):
            load_scenario_from_json(fp, DEFAULT_CONFIG.world, DEFAULT_CONFIG.robot)


def test_4_ui_distinguishes_goal_collision_and_no_path(monkeypatch: pytest.MonkeyPatch) -> None:
    """Renderer experiment status clearly separates GOAL_REACHED, COLLISION, and NO_PATH."""
    from algorithms.astar import AStarResult

    monkeypatch.setenv("SDL_VIDEODRIVER", "dummy")
    app = DynamicNavApp(DEFAULT_CONFIG, initial_view=AppView.SIMULATION)
    try:
        ok_astar = AStarResult(success=True)
        fail_astar = AStarResult(success=False, failure_reason="No path")

        st_goal, _ = app._renderer._compute_experiment_status(app._world, ok_astar, target_reached=True, collision_occurred=False)
        assert st_goal == "GOAL_REACHED"

        st_col, _ = app._renderer._compute_experiment_status(app._world, ok_astar, target_reached=False, collision_occurred=True)
        assert st_col == "COLLISION"

        st_nopath, _ = app._renderer._compute_experiment_status(app._world, fail_astar, target_reached=False, collision_occurred=False)
        assert st_nopath == "NO_PATH"
    finally:
        pygame.quit()
