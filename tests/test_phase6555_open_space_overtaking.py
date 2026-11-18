"""Phase 6.5.5.5 — Open-Space Proactive Overtaking & Safe Following Tests."""

from __future__ import annotations

from dataclasses import replace
import math

import pytest

from algorithms.astar import AStarPlanner, inflate_occupancy_grid, world_to_grid
from algorithms.corridor_passing import CorridorPassingManager, build_static_occupancy_grid
from algorithms.dwa import DWAPlanner
from algorithms.observation import ObservationPipeline
from analysis.metrics import compute_swept_clearance_m
from config import DEFAULT_CONFIG, DWAConfig, ProjectConfig
from simulation.dynamic_obstacles import DynamicObstacle, MovementBehavior
from simulation.entities import RobotState, StaticObstacle
from simulation.scenarios import ScenarioManager
from simulation.world import SimulationWorld


def _run_scenario(scen, cfg: ProjectConfig) -> tuple[bool, bool, float, int]:
    w = SimulationWorld(cfg.world, cfg.robot, cfg.simulation, safety_margin_m=cfg.planner.safety_margin_m)
    w.load_scenario(scen)
    res_m = cfg.world.grid_resolution_m
    inf = inflate_occupancy_grid(w.occupancy_grid, res_m, cfg.robot.radius_m, cfg.planner.safety_margin_m, enforce_boundary_clearance=True)
    ar = AStarPlanner(res_m, True).plan(
        inf,
        world_to_grid(scen.robot_start_m[0], scen.robot_start_m[1], res_m, inf.shape),
        world_to_grid(scen.goal_m[0], scen.goal_m[1], res_m, inf.shape),
        raw_grid=w.occupancy_grid,
        required_clearance_m=cfg.robot.radius_m + cfg.planner.safety_margin_m,
        enforce_boundary_clearance=True,
    )
    assert ar.success
    dwa = DWAPlanner(cfg.world, cfg.robot, cfg.dwa, cfg.observation)
    dwa.set_path(ar.path_world_m)
    pipe = ObservationPipeline(cfg.observation)
    w.start()
    min_swept = math.inf
    proactive = 0
    for step in range(800):
        f = pipe.observe(step * 0.05, w.robot.state, w.dynamic_obstacles)
        res = dwa.compute_command(w.robot.state, 0.05, w.obstacles, w.dynamic_obstacles, observation_frame=f)
        if res.proactive_replan_triggered:
            proactive += 1
        prev_p = (w.robot.state.x_m, w.robot.state.y_m)
        prev_d = [(d.x_m, d.y_m) for d in w.dynamic_obstacles]
        w.robot.set_velocity_command(res.v_mps, res.omega_radps)
        w.step(0.05)
        trans = [(d, prev_d[i], (d.x_m, d.y_m)) for i, d in enumerate(w.dynamic_obstacles)]
        sc = compute_swept_clearance_m(prev_p, (w.robot.state.x_m, w.robot.state.y_m), cfg.robot.radius_m, cfg.world, w.obstacles, trans, w.sim_objects)
        min_swept = min(min_swept, sc)
        if w.has_physical_collision or res.target_reached:
            break
    return (dwa.target_reached, w.has_physical_collision, min_swept, proactive)


def test_1_narrow_gate_l1_rd_01_01_resolved() -> None:
    """Exact NARROW-GATE__L1-RD-01__01 (max_linear_speed=2.2) reaches goal with 0 collisions and >= 0.15 m clearance."""
    scen = ScenarioManager.get_scenario("NARROW-GATE")
    cfg = replace(DEFAULT_CONFIG, robot=replace(DEFAULT_CONFIG.robot, max_linear_speed_mps=2.2))
    reached, coll, min_swept, proactive = _run_scenario(scen, cfg)
    assert reached is True
    assert coll is False
    assert min_swept >= cfg.planner.safety_margin_m
    assert proactive >= 1


def test_2_narrow_gate_l1_sg_01_01_resolved() -> None:
    """Exact NARROW-GATE__L1-SG-01__01 (gate width -0.20 m) reaches goal with 0 collisions and >= 0.15 m clearance."""
    base = ScenarioManager.get_scenario("NARROW-GATE")
    s0, s1 = base.static_obstacles[0], base.static_obstacles[1]
    statics = (
        replace(s0, y_m=s0.y_m + 0.10),
        replace(s1, y_m=s1.y_m - 0.10),
    )
    scen = replace(base, static_obstacles=statics)
    reached, coll, min_swept, proactive = _run_scenario(scen, DEFAULT_CONFIG)
    assert reached is True
    assert coll is False
    assert min_swept >= DEFAULT_CONFIG.planner.safety_margin_m
    assert proactive >= 1


def test_3_open_space_proactive_overtaking_triggers() -> None:
    """Slower same-direction lead obstacle in open space triggers OPEN_SPACE_OVERTAKE_TRIGGERED after hysteresis."""
    cfg = DEFAULT_CONFIG
    mgr = CorridorPassingManager(cfg.world, cfg.robot, cfg.dwa, cfg.observation)
    path = [(float(x), 7.5) for x in range(2, 19)]
    robot = RobotState(x_m=6.0, y_m=7.5, theta_rad=0.0, v_mps=0.8, omega_radps=0.0)
    obs = DynamicObstacle("LEAD", x_m=7.5, y_m=7.5, vx_mps=0.8, vy_mps=0.0, radius_m=0.40, behavior=MovementBehavior.HEAD_ON)
    pipe = ObservationPipeline(cfg.observation)

    for step in range(cfg.dwa.proactive_open_space_hysteresis_steps + 1):
        f = pipe.observe(step * 0.05, robot, [obs])
        ass = mgr.assess_corridor(robot, 0.05, path, [], f)
        if step < cfg.dwa.proactive_open_space_hysteresis_steps:
            assert ass.trigger_replan is False
        else:
            assert ass.trigger_replan is True
            assert ass.reason == "OPEN_SPACE_OVERTAKE_TRIGGERED"
        obs.step(0.05)
        robot = RobotState(x_m=robot.x_m + 0.8 * 0.05, y_m=7.5, theta_rad=0.0, v_mps=0.8, omega_radps=0.0)


def test_4_insufficient_lateral_clearance_blocks_overtaking() -> None:
    """When walls block lateral space in open-approach channel, overtaking is not triggered."""
    cfg = DEFAULT_CONFIG
    mgr = CorridorPassingManager(cfg.world, cfg.robot, cfg.dwa, cfg.observation)
    narrow_walls = [
        StaticObstacle(x_m=5.0, y_m=4.0, width_m=10.0, height_m=2.8),
        StaticObstacle(x_m=5.0, y_m=8.2, width_m=10.0, height_m=2.8),
    ]
    path = [(float(x), 7.5) for x in range(2, 19)]
    robot = RobotState(x_m=8.0, y_m=7.5, theta_rad=0.0, v_mps=0.8, omega_radps=0.0)
    obs = DynamicObstacle("LEAD", x_m=9.5, y_m=7.5, vx_mps=0.8, vy_mps=0.0, radius_m=0.40, behavior=MovementBehavior.HEAD_ON)
    pipe = ObservationPipeline(cfg.observation)
    for step in range(10):
        f = pipe.observe(step * 0.05, robot, [obs])
        ass = mgr.assess_corridor(robot, 0.05, path, narrow_walls, f)
        assert ass.trigger_replan is False


def test_5_safe_following_when_overtaking_unavailable() -> None:
    """With proactive_open_space_enabled=False, robot maintains safe following without crashing."""
    cfg = replace(DEFAULT_CONFIG, dwa=replace(DEFAULT_CONFIG.dwa, proactive_open_space_enabled=False))
    dwa = DWAPlanner(cfg.world, cfg.robot, cfg.dwa, cfg.observation)
    dwa.set_path([(2.0, 7.5), (18.0, 7.5)])
    robot = RobotState(x_m=6.0, y_m=7.5, theta_rad=0.0, v_mps=0.8, omega_radps=0.0)
    obs = DynamicObstacle("LEAD", x_m=7.5, y_m=7.5, vx_mps=0.8, vy_mps=0.0, radius_m=0.40, behavior=MovementBehavior.HEAD_ON)
    pipe = ObservationPipeline(cfg.observation)
    for step in range(10):
        f = pipe.observe(step * 0.05, robot, [obs])
        res = dwa.compute_command(robot, 0.05, [], [obs], observation_frame=f)
        assert res.proactive_replan_triggered is False
        assert res.safety_margin_satisfied is True
        obs.step(0.05)
        robot = RobotState(x_m=robot.x_m + res.v_mps * 0.05, y_m=7.5, theta_rad=0.0, v_mps=res.v_mps, omega_radps=0.0)


def test_6_dynamic_obstacle_reversal_without_hidden_schedule(monkeypatch: pytest.MonkeyPatch) -> None:
    """Open-space overtaking never calls DynamicObstacle.predict_positions()."""
    def _forbidden(*args: object, **kwargs: object) -> object:
        raise AssertionError("predict_positions() must never be called!")

    monkeypatch.setattr(DynamicObstacle, "predict_positions", _forbidden)
    cfg = DEFAULT_CONFIG
    dwa = DWAPlanner(cfg.world, cfg.robot, cfg.dwa, cfg.observation)
    dwa.set_path([(2.0, 7.5), (18.0, 7.5)])
    pipe = ObservationPipeline(cfg.observation)
    robot = RobotState(x_m=6.0, y_m=7.5, theta_rad=0.0, v_mps=0.8, omega_radps=0.0)
    obs = DynamicObstacle("LEAD", x_m=7.5, y_m=7.5, vx_mps=0.8, vy_mps=0.0, radius_m=0.40, behavior=MovementBehavior.CORRIDOR, waypoint_a_m=(4.0, 7.5), waypoint_b_m=(10.0, 7.5))
    for step in range(12):
        f = pipe.observe(step * 0.05, robot, [obs])
        res = dwa.compute_command(robot, 0.05, [], [obs], observation_frame=f)
        assert math.isfinite(res.v_mps)
        obs.step(0.05)


def test_7_braking_safety_rejects_unsafe_open_space_candidate() -> None:
    """Open-space candidate failing braking safety margin must be rejected."""
    cfg = DEFAULT_CONFIG
    mgr = CorridorPassingManager(cfg.world, cfg.robot, cfg.dwa, cfg.observation)
    grid = build_static_occupancy_grid(cfg.world, [])
    path = [(float(x), 7.5) for x in range(2, 19)]
    robot = RobotState(x_m=6.0, y_m=7.5, theta_rad=0.0, v_mps=0.8, omega_radps=0.0)
    obs = DynamicObstacle("LEAD", x_m=7.5, y_m=7.5, vx_mps=0.8, vy_mps=0.0, radius_m=0.40, behavior=MovementBehavior.HEAD_ON)
    pipe = ObservationPipeline(cfg.observation)
    for step in range(cfg.dwa.proactive_open_space_hysteresis_steps):
        f = pipe.observe(step * 0.05, robot, [obs])
        mgr.assess_corridor(robot, 0.05, path, [], f)
        obs.step(0.05)
        robot = RobotState(x_m=robot.x_m + 0.8 * 0.05, y_m=7.5, theta_rad=0.0, v_mps=0.8, omega_radps=0.0)

    best, evals = mgr.generate_and_validate_candidates(
        0.40, robot, 0.05, (18.0, 7.5), grid, cfg.world.grid_resolution_m, [], f,
        lambda _p: (True, 0.8, 0.0, 0.30, 0.05),
    )
    assert best is None
    assert all(not e.accepted for e in evals)
