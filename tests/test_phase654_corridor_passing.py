"""Phase 6.5.4 — Predictive Corridor Passing & Proactive Lateral Replanning Tests."""

from __future__ import annotations

import math
from pathlib import Path

import pytest

from algorithms.corridor_passing import (
    CorridorPassingManager,
    build_static_occupancy_grid,
)
from algorithms.dwa import DWAPlanner
from algorithms.observation import ObservationFrame, ObservationPipeline, ObservedObstacle, ObservedRobotState
from config import DEFAULT_CONFIG, DWAConfig
from simulation.dynamic_obstacles import DynamicObstacle, MovementBehavior
from simulation.entities import RobotState, StaticObstacle


def _corridor_statics() -> list[StaticObstacle]:
    return [
        StaticObstacle(x_m=5.0, y_m=4.0, width_m=10.0, height_m=2.0),
        StaticObstacle(x_m=5.0, y_m=9.0, width_m=10.0, height_m=2.0),
    ]


def _make_obs(
    obs_id: str = "DYN-CORR-1",
    x_m: float = 10.0,
    y_m: float = 7.5,
    vx_mps: float = 1.0,
    vy_mps: float = 0.0,
    radius_m: float = 0.40,
    waypoint_b_m: tuple[float, float] = (14.4, 7.5),
) -> DynamicObstacle:
    return DynamicObstacle(
        obstacle_id=obs_id,
        x_m=x_m,
        y_m=y_m,
        vx_mps=vx_mps,
        vy_mps=vy_mps,
        radius_m=radius_m,
        behavior=MovementBehavior.CORRIDOR,
        active=True,
        waypoint_a_m=(5.6, 7.5),
        waypoint_b_m=waypoint_b_m,
        reverse_at_bounds=True,
    )


def test_1_causal_detection_of_path_obstruction() -> None:
    """CorridorPassingManager detects path obstruction causally after hysteresis."""
    cfg = DEFAULT_CONFIG
    mgr = CorridorPassingManager(cfg.world, cfg.robot, cfg.dwa, cfg.observation)
    statics = _corridor_statics()
    path = [(2.0, 7.5), (10.0, 7.5), (18.0, 7.5)]
    robot = RobotState(x_m=8.0, y_m=7.5, theta_rad=0.0, v_mps=1.0, omega_radps=0.0)
    obs = _make_obs(x_m=9.5, y_m=7.5, vx_mps=1.0)
    pipe = ObservationPipeline(cfg.observation)

    for step in range(cfg.dwa.proactive_hysteresis_steps):
        t_s = step * 0.05
        frame = pipe.observe(t_s, robot, [obs])
        ass = mgr.assess_corridor(robot, 0.05, path, statics, frame)
        assert ass.path_obstructed is True
        assert ass.obstructing_obstacle_id == "DYN-CORR-1"
        if step < cfg.dwa.proactive_hysteresis_steps - 1:
            assert ass.trigger_replan is False
        else:
            assert ass.trigger_replan is True


def test_2_lateral_path_generation_and_goal_preservation() -> None:
    """Validates lateral A* candidate generation around observed obstacle & goal preservation."""
    cfg = DEFAULT_CONFIG
    mgr = CorridorPassingManager(cfg.world, cfg.robot, cfg.dwa, cfg.observation)
    statics = _corridor_statics()
    grid = build_static_occupancy_grid(cfg.world, statics)
    path = [(2.0, 7.5), (10.0, 7.5), (18.0, 7.5)]
    robot = RobotState(x_m=8.0, y_m=7.5, theta_rad=0.0, v_mps=1.0, omega_radps=0.0)
    obs = _make_obs(x_m=9.6, y_m=7.5, vx_mps=1.0)
    pipe = ObservationPipeline(cfg.observation)

    for step in range(cfg.dwa.proactive_hysteresis_steps):
        frame = pipe.observe(step * 0.05, robot, [obs])
        mgr.assess_corridor(robot, 0.05, path, statics, frame)
        obs.step(0.05)
        robot = RobotState(x_m=robot.x_m + 1.0 * 0.05, y_m=7.5, theta_rad=0.0, v_mps=1.0, omega_radps=0.0)

    def _eval_fn(cand: list[tuple[float, float]]) -> tuple[bool, float, float, float, float | None]:
        dwa = DWAPlanner(cfg.world, cfg.robot, cfg.dwa, cfg.observation)
        dwa._proactive_replan_enabled_for_eval = False
        dwa.set_path(cand)
        r = dwa.compute_command(robot, 0.05, statics, [obs], observation_frame=frame)
        return (not r.fallback_active and r.safety_margin_satisfied, r.v_mps, r.omega_radps, r.predicted_min_clearance_m or -math.inf, r.min_braking_clearance_m)

    best_path, evals = mgr.generate_and_validate_candidates(
        0.10, robot, 0.05, (18.0, 7.5), grid, cfg.world.grid_resolution_m, statics, frame, _eval_fn
    )
    assert best_path is not None
    assert math.hypot(best_path[-1][0] - 18.0, best_path[-1][1] - 7.5) <= cfg.dwa.goal_tolerance_m + cfg.world.grid_resolution_m
    assert any(abs(p[1] - 7.5) >= 0.6 for p in best_path)
    assert any(e.accepted for e in evals)


def test_3_replanning_without_waiting_for_deadlock_timeout() -> None:
    """DWAPlanner triggers proactive lateral replanning while cruising (blocked=False)."""
    cfg = DEFAULT_CONFIG
    dwa = DWAPlanner(cfg.world, cfg.robot, cfg.dwa, cfg.observation)
    dwa.set_path([(2.0, 7.5), (10.0, 7.5), (18.0, 7.5)])
    statics = _corridor_statics()
    robot = RobotState(x_m=8.0, y_m=7.5, theta_rad=0.0, v_mps=1.0, omega_radps=0.0)
    obs = _make_obs(x_m=9.6, y_m=7.5, vx_mps=1.0)
    pipe = ObservationPipeline(cfg.observation)

    triggered = False
    for step in range(cfg.dwa.proactive_hysteresis_steps + 2):
        frame = pipe.observe(step * 0.05, robot, [obs])
        res = dwa.compute_command(robot, 0.05, statics, [obs], observation_frame=frame)
        assert res.blocked is False
        if res.proactive_replan_triggered:
            triggered = True
            break
        obs.step(0.05)
        robot = RobotState(x_m=robot.x_m + 1.0 * 0.05, y_m=7.5, theta_rad=0.0, v_mps=1.0, omega_radps=0.0)
    assert triggered is True


def test_4_braking_safety_rejects_unsafe_candidates() -> None:
    """Candidates failing DWA/braking safety margin must be rejected."""
    cfg = DEFAULT_CONFIG
    mgr = CorridorPassingManager(cfg.world, cfg.robot, cfg.dwa, cfg.observation)
    statics = _corridor_statics()
    grid = build_static_occupancy_grid(cfg.world, statics)
    path = [(2.0, 7.5), (10.0, 7.5), (18.0, 7.5)]
    robot = RobotState(x_m=8.0, y_m=7.5, theta_rad=0.0, v_mps=1.0, omega_radps=0.0)
    obs = _make_obs(x_m=9.6, y_m=7.5, vx_mps=1.0)
    pipe = ObservationPipeline(cfg.observation)
    for step in range(cfg.dwa.proactive_hysteresis_steps):
        frame = pipe.observe(step * 0.05, robot, [obs])
        mgr.assess_corridor(robot, 0.05, path, statics, frame)
        obs.step(0.05)
        robot = RobotState(x_m=robot.x_m + 1.0 * 0.05, y_m=7.5, theta_rad=0.0, v_mps=1.0, omega_radps=0.0)

    def _unsafe_braking_eval(_cand: list[tuple[float, float]]) -> tuple[bool, float, float, float, float | None]:
        # Geometric path may be clear, but braking clearance violates 0.15 m margin
        return (True, 1.0, 0.0, 0.30, 0.05)

    best_path, evals = mgr.generate_and_validate_candidates(
        0.10, robot, 0.05, (18.0, 7.5), grid, cfg.world.grid_resolution_m, statics, frame, _unsafe_braking_eval
    )
    assert best_path is None
    assert all(not e.accepted for e in evals)
    assert any(e.rejection_reason == "BRAKING_MARGIN_VIOLATION" for e in evals)


def test_5_no_hidden_schedule_access(monkeypatch: pytest.MonkeyPatch) -> None:
    """CorridorPassingManager must never call predict_positions() or reference schedule fields."""
    def _forbidden(*args: object, **kwargs: object) -> object:
        raise AssertionError("predict_positions() must never be called!")

    monkeypatch.setattr(DynamicObstacle, "predict_positions", _forbidden)

    root = Path(__file__).resolve().parent.parent
    text = (root / "algorithms" / "corridor_passing.py").read_text(encoding="utf-8")
    for tok in ["predict_positions", "_phase_time_s", "move_duration_s", "stop_duration_s", "waypoint_a_m", "waypoint_b_m", "reverse_at_bounds"]:
        assert tok not in text


def test_6_no_unnecessary_replanning_in_unobstructed_open_space() -> None:
    """Unobstructed open space must not trigger proactive corridor replanning."""
    cfg = DEFAULT_CONFIG
    mgr = CorridorPassingManager(cfg.world, cfg.robot, cfg.dwa, cfg.observation)
    path = [(2.0, 7.5), (18.0, 7.5)]
    robot = RobotState(x_m=5.0, y_m=7.5, theta_rad=0.0, v_mps=1.5, omega_radps=0.0)
    obs = _make_obs(x_m=15.0, y_m=2.0, vx_mps=0.0)
    pipe = ObservationPipeline(cfg.observation)

    for step in range(10):
        frame = pipe.observe(step * 0.05, robot, [obs])
        ass = mgr.assess_corridor(robot, 0.05, path, [], frame)
        assert ass.trigger_replan is False


def test_7_hysteresis_and_cooldown_behavior() -> None:
    """Trigger requires hysteresis steps and respects cooldown after a replan."""
    cfg = DEFAULT_CONFIG
    dwa_cfg = DWAConfig(proactive_hysteresis_steps=3, proactive_replan_cooldown_s=2.0)
    mgr = CorridorPassingManager(cfg.world, cfg.robot, dwa_cfg, cfg.observation)
    statics = _corridor_statics()
    grid = build_static_occupancy_grid(cfg.world, statics)
    path = [(2.0, 7.5), (10.0, 7.5), (18.0, 7.5)]
    robot = RobotState(x_m=8.0, y_m=7.5, theta_rad=0.0, v_mps=1.0, omega_radps=0.0)
    obs = _make_obs(x_m=9.6, y_m=7.5, vx_mps=1.0)
    pipe = ObservationPipeline(cfg.observation)

    for step in range(2):
        frame = pipe.observe(step * 0.05, robot, [obs])
        assert mgr.assess_corridor(robot, 0.05, path, statics, frame).trigger_replan is False

    frame = pipe.observe(0.10, robot, [obs])
    assert mgr.assess_corridor(robot, 0.05, path, statics, frame).trigger_replan is True
    mgr.generate_and_validate_candidates(
        0.10, robot, 0.05, (18.0, 7.5), grid, cfg.world.grid_resolution_m, statics, frame,
        lambda _p: (True, 1.0, 0.0, 0.25, 0.25),
    )
    # Immediately after replan, cooldown must suppress re-triggering
    for step in range(3, 10):
        frame = pipe.observe(step * 0.05, robot, [obs])
        ass = mgr.assess_corridor(robot, 0.05, path, statics, frame)
        assert ass.trigger_replan is False


def test_8_safe_behavior_when_both_lateral_paths_blocked() -> None:
    """When walls leave insufficient lateral passing clearance, do not force a maneuver."""
    cfg = DEFAULT_CONFIG
    mgr = CorridorPassingManager(cfg.world, cfg.robot, cfg.dwa, cfg.observation)
    # Narrow 1.4 m corridor: gap around r=0.40 obstacle at y=7.5 is only 0.30 m per side (< 1.00 m required)
    narrow_statics = [
        StaticObstacle(x_m=5.0, y_m=4.0, width_m=10.0, height_m=2.8),  # top=6.8
        StaticObstacle(x_m=5.0, y_m=8.2, width_m=10.0, height_m=2.8),  # bot=8.2
    ]
    path = [(2.0, 7.5), (10.0, 7.5), (18.0, 7.5)]
    robot = RobotState(x_m=8.0, y_m=7.5, theta_rad=0.0, v_mps=1.0, omega_radps=0.0)
    obs = _make_obs(x_m=9.5, y_m=7.5, vx_mps=1.0)
    pipe = ObservationPipeline(cfg.observation)

    for step in range(5):
        frame = pipe.observe(step * 0.05, robot, [obs])
        ass = mgr.assess_corridor(robot, 0.05, path, narrow_statics, frame)
        assert ass.trigger_replan is False
        assert ass.reason in ("NO_LATERAL_PASSING_SPACE", "PATH_CLEAR")


def test_9_direction_change_after_observation_remains_causal() -> None:
    """Two worlds identical up to t=0.10 s but with different future reversal produce identical decisions."""
    cfg = DEFAULT_CONFIG
    statics = _corridor_statics()
    path = [(2.0, 7.5), (10.0, 7.5), (18.0, 7.5)]
    robot = RobotState(x_m=8.0, y_m=7.5, theta_rad=0.0, v_mps=1.0, omega_radps=0.0)

    obs_a = _make_obs(x_m=9.6, y_m=7.5, vx_mps=1.0, waypoint_b_m=(9.72, 7.5))
    obs_b = _make_obs(x_m=9.6, y_m=7.5, vx_mps=1.0, waypoint_b_m=(14.4, 7.5))
    dwa_a = DWAPlanner(cfg.world, cfg.robot, cfg.dwa, cfg.observation)
    dwa_b = DWAPlanner(cfg.world, cfg.robot, cfg.dwa, cfg.observation)
    dwa_a.set_path(path)
    dwa_b.set_path(path)
    pipe_a = ObservationPipeline(cfg.observation)
    pipe_b = ObservationPipeline(cfg.observation)

    for step in range(3):
        t_s = step * 0.05
        fa = pipe_a.observe(t_s, robot, [obs_a])
        fb = pipe_b.observe(t_s, robot, [obs_b])
        ra = dwa_a.compute_command(robot, 0.05, statics, [obs_a], observation_frame=fa)
        rb = dwa_b.compute_command(robot, 0.05, statics, [obs_b], observation_frame=fb)
        assert ra.v_mps == rb.v_mps
        assert ra.omega_radps == rb.omega_radps
        assert ra.proactive_replan_triggered == rb.proactive_replan_triggered
        obs_a.step(0.05)
        obs_b.step(0.05)
