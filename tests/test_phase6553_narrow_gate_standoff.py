"""Phase 6.5.5.3 — NARROW-GATE Safe Standoff & Fallback Arbitration Regression Tests."""

from __future__ import annotations

from dataclasses import replace
import math

import pytest

from algorithms.astar import AStarPlanner, inflate_occupancy_grid, world_to_grid
from algorithms.dwa import DWAPlanner, WaitingZoneState
from algorithms.observation import ObservationPipeline
from analysis.metrics import compute_swept_clearance_m
from config import DEFAULT_CONFIG
from simulation.dynamic_obstacles import DynamicObstacle, MovementBehavior
from simulation.entities import RobotState, StaticObstacle
from simulation.scenarios import ScenarioManager
from simulation.world import SimulationWorld


def test_1_narrow_gate_l1_ic_01_02_exact_failure_resolved() -> None:
    """Exact NARROW-GATE__L1-IC-01__02 perturbation now reaches goal with 0 collisions and 0 margin violations."""
    cfg = DEFAULT_CONFIG
    base_scen = ScenarioManager.get_scenario("NARROW-GATE")
    scen = replace(
        base_scen,
        robot_start_m=(base_scen.robot_start_m[0] + 0.30, base_scen.robot_start_m[1] - 0.30),
    )
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
    for step in range(600):
        f = pipe.observe(step * 0.05, w.robot.state, w.dynamic_obstacles)
        res = dwa.compute_command(w.robot.state, 0.05, w.obstacles, w.dynamic_obstacles, observation_frame=f)
        prev_p = (w.robot.state.x_m, w.robot.state.y_m)
        prev_d = [(d.x_m, d.y_m) for d in w.dynamic_obstacles]
        w.robot.set_velocity_command(res.v_mps, res.omega_radps)
        w.step(0.05)
        assert w.has_physical_collision is False
        trans = [(d, prev_d[i], (d.x_m, d.y_m)) for i, d in enumerate(w.dynamic_obstacles)]
        sc = compute_swept_clearance_m(prev_p, (w.robot.state.x_m, w.robot.state.y_m), cfg.robot.radius_m, cfg.world, w.obstacles, trans, w.sim_objects)
        min_swept = min(min_swept, sc)
        if res.target_reached:
            break

    assert dwa.target_reached is True
    assert min_swept >= cfg.planner.safety_margin_m


def test_2_unsafe_p2_standstill_selects_safe_p3_reverse() -> None:
    """When GATE_CLEAR_HOLD_UNSAFE and safe reverse exists, Priority 3 REVERSE_RECOVERY is chosen."""
    cfg = DEFAULT_CONFIG
    dwa = DWAPlanner(cfg.world, cfg.robot, cfg.dwa, cfg.observation)
    dwa.set_path([(7.5, 7.5), (15.0, 7.5)])
    robot = RobotState(x_m=7.5, y_m=7.5, theta_rad=0.0, v_mps=0.0, omega_radps=0.0)
    obs = DynamicObstacle("OBS-REV", x_m=8.6, y_m=7.5, vx_mps=-0.20, vy_mps=0.0, radius_m=0.40, behavior=MovementBehavior.HEAD_ON)
    res = dwa.compute_command(robot, 0.05, [], [obs])
    assert res.fallback_active is True
    assert res.fallback_priority == 3
    assert res.v_mps < 0.0


def test_3_unsafe_p2_standstill_selects_safe_p4_lateral_holding() -> None:
    """When GATE_CLEAR_HOLD_UNSAFE and reverse is blocked/unsafe, Priority 4 LATERAL_HOLDING_POSE is chosen."""
    cfg = DEFAULT_CONFIG
    dwa = DWAPlanner(cfg.world, cfg.robot, cfg.dwa, cfg.observation)
    dwa.set_path([(5.0, 7.5), (15.0, 7.5)])
    rear_wall = StaticObstacle(x_m=3.6, y_m=6.5, width_m=0.8, height_m=2.0)
    robot = RobotState(x_m=5.0, y_m=7.5, theta_rad=0.0, v_mps=0.5, omega_radps=0.0)
    obs = DynamicObstacle("OBS-FAST", x_m=7.2, y_m=7.5, vx_mps=-1.2, vy_mps=0.0, radius_m=0.40, behavior=MovementBehavior.HEAD_ON)
    res = dwa.compute_command(robot, 0.05, [rear_wall], [obs])
    assert res.waiting_state == WaitingZoneState.GATE_CLEAR_HOLD_UNSAFE.value
    assert res.fallback_active is True
    assert res.fallback_priority == 4
    assert res.holding_pose_m is not None


def test_4_safe_p2_standstill_preserves_waiting_behavior() -> None:
    """When holding pose is safe over waiting_horizon_s, robot waits safely (Priority 1)."""
    cfg = DEFAULT_CONFIG
    dwa = DWAPlanner(cfg.world, cfg.robot, cfg.dwa, cfg.observation)
    dwa.set_path([(4.5, 7.5), (15.0, 7.5)])
    statics = [
        StaticObstacle(x_m=8.0, y_m=0.0, width_m=2.0, height_m=6.6),
        StaticObstacle(x_m=8.0, y_m=8.4, width_m=2.0, height_m=6.6),
    ]
    robot = RobotState(x_m=4.5, y_m=7.5, theta_rad=0.0, v_mps=0.0, omega_radps=0.0)
    obs = DynamicObstacle("DYN-GATE", x_m=9.5, y_m=7.5, vx_mps=-0.8, vy_mps=0.0, radius_m=0.40, behavior=MovementBehavior.CORRIDOR)
    res = dwa.compute_command(robot, 0.05, statics, [obs])
    assert res.waiting_for_passage is True
    assert res.waiting_state == WaitingZoneState.GATE_OCCUPIED_HOLD_SAFE.value
    assert res.v_mps == 0.0


def test_5_no_feasible_escape_reports_blocked_honestly() -> None:
    """When boxed in on all sides, DWA reports BLOCKED (Priority 5 PHYSICAL_COLLISION_RISK)."""
    cfg = DEFAULT_CONFIG
    dwa = DWAPlanner(cfg.world, cfg.robot, cfg.dwa, cfg.observation)
    dwa.set_path([(7.5, 7.5), (15.0, 7.5)])
    statics = [
        StaticObstacle(x_m=6.0, y_m=6.2, width_m=3.0, height_m=0.6),
        StaticObstacle(x_m=6.0, y_m=8.2, width_m=3.0, height_m=0.6),
        StaticObstacle(x_m=6.0, y_m=6.8, width_m=0.8, height_m=1.4),
    ]
    robot = RobotState(x_m=7.5, y_m=7.5, theta_rad=0.0, v_mps=0.0, omega_radps=0.0)
    obs = DynamicObstacle("OBS-BOX", x_m=8.3, y_m=7.5, vx_mps=-0.8, vy_mps=0.0, radius_m=0.40, behavior=MovementBehavior.HEAD_ON)
    res = dwa.compute_command(robot, 0.05, statics, [obs])
    assert res.blocked is True
    assert res.fallback_priority == 5
    assert res.fallback_reason == "PHYSICAL_COLLISION_RISK"


def test_6_multiple_approaching_obstacles_standoff() -> None:
    """Multiple approaching obstacles in front trigger unsafe holding evasion rather than standstill."""
    cfg = DEFAULT_CONFIG
    dwa = DWAPlanner(cfg.world, cfg.robot, cfg.dwa, cfg.observation)
    dwa.set_path([(5.0, 7.5), (15.0, 7.5)])
    robot = RobotState(x_m=5.0, y_m=7.5, theta_rad=0.0, v_mps=0.4, omega_radps=0.0)
    obs1 = DynamicObstacle("OBS-1", x_m=7.2, y_m=7.5, vx_mps=-1.0, vy_mps=0.0, radius_m=0.40, behavior=MovementBehavior.HEAD_ON)
    obs2 = DynamicObstacle("OBS-2", x_m=7.8, y_m=7.9, vx_mps=-0.9, vy_mps=0.0, radius_m=0.35, behavior=MovementBehavior.HEAD_ON)
    res = dwa.compute_command(robot, 0.05, [], [obs1, obs2])
    assert res.fallback_active is True
    assert res.fallback_priority in (3, 4)


def test_7_causal_isolation_from_hidden_schedules(monkeypatch: pytest.MonkeyPatch) -> None:
    """Gate standoff and fallback arbitration never call DynamicObstacle.predict_positions()."""
    def _forbidden(*args: object, **kwargs: object) -> object:
        raise AssertionError("predict_positions() must never be called!")

    monkeypatch.setattr(DynamicObstacle, "predict_positions", _forbidden)
    cfg = DEFAULT_CONFIG
    dwa = DWAPlanner(cfg.world, cfg.robot, cfg.dwa, cfg.observation)
    dwa.set_path([(2.0, 7.5), (18.0, 7.5)])
    pipe = ObservationPipeline(cfg.observation)
    robot = RobotState(x_m=4.5, y_m=7.5, theta_rad=0.0, v_mps=0.5, omega_radps=0.0)
    obs = DynamicObstacle("OBS-ISO", x_m=9.5, y_m=7.5, vx_mps=-0.8, vy_mps=0.0, radius_m=0.40, behavior=MovementBehavior.CORRIDOR, waypoint_a_m=(8.0, 7.5), waypoint_b_m=(14.0, 7.5))
    f = pipe.observe(0.0, robot, [obs])
    res = dwa.compute_command(robot, 0.05, [], [obs], observation_frame=f)
    assert math.isfinite(res.v_mps)
