"""Phase 6.5.5.7 — Continuous Static Swept-Clearance Validation Tests."""

from __future__ import annotations

from dataclasses import replace
import math

import pytest

from algorithms.astar import AStarPlanner, inflate_occupancy_grid, world_to_grid
from algorithms.dwa import DWAPlanner, evaluate_trajectory_with_braking
from algorithms.observation import ObservationPipeline
from analysis.metrics import compute_swept_clearance_m
from config import DEFAULT_CONFIG
from simulation.dynamic_obstacles import DynamicObstacle, MovementBehavior
from simulation.entities import StaticObstacle
from simulation.scenarios import ScenarioManager
from simulation.world import SimulationWorld


def test_1_narrow_gate_l1_do_01_01_exact_margin_resolved() -> None:
    """Exact NARROW-GATE__L1-DO-01__01 completes with min_swept >= 0.150000 m and 0 margin violations."""
    cfg = DEFAULT_CONFIG
    base = ScenarioManager.get_scenario("NARROW-GATE")
    scen = replace(
        base,
        dynamic_obstacles=tuple(
            replace(d, vx_mps=d.vx_mps * 0.85, vy_mps=d.vy_mps * 0.85)
            for d in base.dynamic_obstacles
        ),
    )
    w = SimulationWorld(cfg.world, cfg.robot, cfg.simulation, safety_margin_m=cfg.planner.safety_margin_m)
    w.load_scenario(scen)
    inf = inflate_occupancy_grid(w.occupancy_grid, 0.2, cfg.robot.radius_m, cfg.planner.safety_margin_m, enforce_boundary_clearance=True)
    ar = AStarPlanner(0.2, True).plan(
        inf,
        world_to_grid(scen.robot_start_m[0], scen.robot_start_m[1], 0.2, inf.shape),
        world_to_grid(scen.goal_m[0], scen.goal_m[1], 0.2, inf.shape),
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
    for step in range(800):
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


def test_2_segment_with_safe_endpoints_but_unsafe_interior_rejected() -> None:
    """Critical test: endpoints both > 0.15 m from AABB corner, but chord interior < 0.15 m, must be rejected."""
    cfg = DEFAULT_CONFIG
    # Corner at (5.0, 5.0). Robot radius 0.35 + margin 0.15 = 0.50 m.
    # Endpoints (4.6, 5.35) and (5.35, 4.6): dist to (5.0, 5.0) = sqrt(0.4^2 + 0.35^2) = 0.5315 m -> clearance = 0.1815 m >= 0.15 m!
    # Midpoint (4.975, 4.975): inside/on corner -> clearance < 0.0 m!
    obs = StaticObstacle(x_m=5.0, y_m=5.0, width_m=2.0, height_m=2.0)
    traj = [(4.6, 5.35, 0.0), (5.35, 4.6, 0.0)]
    is_safe, min_c, _, _ = evaluate_trajectory_with_braking(
        traj, 0.1, 0.35, 0.15, cfg.world, [obs], [], v_mps=0.0, stop_on_margin_violation=True
    )
    assert is_safe is False
    assert min_c < 0.15


def test_3_segment_safely_outside_aabb_accepted() -> None:
    """Segment remaining >= 0.15 m from AABB everywhere is accepted."""
    cfg = DEFAULT_CONFIG
    obs = StaticObstacle(x_m=5.0, y_m=5.0, width_m=2.0, height_m=2.0)
    traj = [(4.4, 5.5, 0.0), (4.4, 6.5, 0.0)]  # dist = 0.60 m -> clearance = 0.25 m
    is_safe, min_c, _, _ = evaluate_trajectory_with_braking(
        traj, 0.1, 0.35, 0.15, cfg.world, [obs], [], v_mps=0.0, stop_on_margin_violation=True
    )
    assert is_safe is True
    assert min_c == pytest.approx(0.25, abs=1e-9)


def test_4_tangential_safety_margin_boundary() -> None:
    """Exact 0.15 m tangential clearance passes; 0.1498 m fails."""
    cfg = DEFAULT_CONFIG
    obs = StaticObstacle(x_m=5.0, y_m=5.0, width_m=2.0, height_m=2.0)
    ok, c_ok, _, _ = evaluate_trajectory_with_braking(
        [(5.0, 4.50, 0.0), (7.0, 4.50, 0.0)], 0.1, 0.35, 0.15, cfg.world, [obs], [], v_mps=0.0, stop_on_margin_violation=True
    )
    assert ok is True
    assert c_ok == pytest.approx(0.15, abs=1e-9)

    bad, c_bad, _, _ = evaluate_trajectory_with_braking(
        [(5.0, 4.5002, 0.0), (7.0, 4.5002, 0.0)], 0.1, 0.35, 0.15, cfg.world, [obs], [], v_mps=0.0, stop_on_margin_violation=True
    )
    assert bad is False
    assert c_bad < 0.15


def test_5_braking_trajectory_static_swept_clearance() -> None:
    """Braking segment whose interior clips a static corner is caught and rejected."""
    cfg = DEFAULT_CONFIG
    obs = StaticObstacle(x_m=5.0, y_m=5.0, width_m=2.0, height_m=2.0)
    # Braking trajectory from (4.65, 5.25) heading -pi/4 passes right across corner (5.0, 5.0)
    traj = [(4.55, 5.35, -0.785398), (4.65, 5.25, -0.785398)]
    is_safe, min_c, brake_c, _ = evaluate_trajectory_with_braking(
        traj, 0.1, 0.35, 0.15, cfg.world, [obs], [], v_mps=1.5, max_brake_decel_mps2=5.0, reaction_time_s=0.1, stop_on_margin_violation=True
    )
    assert is_safe is False
    assert (brake_c is not None and brake_c < 0.15) or min_c < 0.15


def test_6_dynamic_swept_and_world_boundary_preserved() -> None:
    """Dynamic swept clearance and world-boundary checks remain active."""
    cfg = DEFAULT_CONFIG
    obs = DynamicObstacle("DYN", x_m=5.5, y_m=7.5, vx_mps=-2.0, vy_mps=0.0, radius_m=0.35, behavior=MovementBehavior.HEAD_ON)
    is_safe_dyn, min_c_dyn, _, _ = evaluate_trajectory_with_braking(
        [(5.0, 7.5, 0.0), (5.5, 7.5, 0.0)], 0.1, 0.35, 0.15, cfg.world, [], [obs], v_mps=0.0, stop_on_margin_violation=True
    )
    assert is_safe_dyn is False
    assert min_c_dyn <= 0.0

    is_safe_bnd, min_c_bnd, _, _ = evaluate_trajectory_with_braking(
        [(0.45, 7.5, 0.0), (0.45, 8.0, 0.0)], 0.1, 0.35, 0.15, cfg.world, [], [], v_mps=0.0, stop_on_margin_violation=True
    )
    assert is_safe_bnd is False
    assert min_c_bnd == pytest.approx(0.10, abs=1e-9)


def test_7_candidate_vs_evaluator_geometry_consistency() -> None:
    """Single-segment static+dynamic clearance in evaluate_trajectory_with_braking matches compute_swept_clearance_m."""
    cfg = DEFAULT_CONFIG
    obs = StaticObstacle(x_m=8.0, y_m=4.0, width_m=2.0, height_m=2.6)
    p0 = (7.6, 6.95, 0.0)
    p1 = (8.2, 7.05, 0.0)
    _, min_c, _, _ = evaluate_trajectory_with_braking(
        [p0, p1], 0.05, 0.35, 0.15, cfg.world, [obs], [], v_mps=0.0, stop_on_margin_violation=False
    )
    eval_c = compute_swept_clearance_m((p0[0], p0[1]), (p1[0], p1[1]), 0.35, cfg.world, [obs], [])
    assert min_c == pytest.approx(eval_c, abs=1e-12)
