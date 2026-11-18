"""Phase 6.5.2 — Causal Observation Pipeline & Future-Information Isolation Tests."""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import pytest

from algorithms.dwa import DWAPlanner, replan_global_path_around_obstacles
from algorithms.observation import (
    ObservationFrame,
    ObservationPipeline,
    ObservedObstacle,
    ObstacleStateEstimator,
)
from config import DEFAULT_CONFIG, ObservationConfig
from simulation.dynamic_obstacles import DynamicObstacle, MovementBehavior
from simulation.entities import RobotState, StaticObstacle
from simulation.geometry import OrientedRectGeometry


def _make_obstacle(
    obs_id: str = "OBS-1",
    x_m: float = 4.0,
    y_m: float = 2.0,
    vx_mps: float = 1.0,
    vy_mps: float = 0.0,
    radius_m: float = 0.35,
    behavior: MovementBehavior = MovementBehavior.CROSSING,
    move_duration_s: float = 1.0,
    stop_duration_s: float = 1.0,
    waypoint_a_m: tuple[float, float] = (2.0, 2.0),
    waypoint_b_m: tuple[float, float] = (6.0, 2.0),
    phase_time_s: float = 0.0,
) -> DynamicObstacle:
    obs = DynamicObstacle(
        obstacle_id=obs_id,
        x_m=x_m,
        y_m=y_m,
        vx_mps=vx_mps,
        vy_mps=vy_mps,
        radius_m=radius_m,
        behavior=behavior,
        active=True,
        waypoint_a_m=waypoint_a_m,
        waypoint_b_m=waypoint_b_m,
        reverse_at_bounds=True,
        move_duration_s=move_duration_s,
        stop_duration_s=stop_duration_s,
    )
    obs._phase_time_s = phase_time_s
    return obs


def test_1_no_future_schedule_access_during_dwa(monkeypatch: pytest.MonkeyPatch) -> None:
    """DWA + ObservationPipeline must never call DynamicObstacle.predict_positions()."""
    def _forbidden_predict(*args: object, **kwargs: object) -> object:
        raise AssertionError("DynamicObstacle.predict_positions() must never be called!")

    monkeypatch.setattr(DynamicObstacle, "predict_positions", _forbidden_predict)

    cfg = DEFAULT_CONFIG
    pipeline = ObservationPipeline(cfg.observation)
    dwa = DWAPlanner(cfg.world, cfg.robot, cfg.dwa, observation_cfg=cfg.observation)
    dwa.set_path([(1.0, 2.0), (8.0, 2.0)])

    robot = RobotState(x_m=1.0, y_m=2.0, theta_rad=0.0, v_mps=0.5, omega_radps=0.0)
    obs = _make_obstacle(x_m=4.5, y_m=2.0, vx_mps=-0.8, vy_mps=0.0)

    for step in range(5):
        t_s = step * 0.05
        frame = pipeline.observe(t_s, robot, [obs], sim_objects=[])
        res = dwa.compute_command(
            robot,
            0.05,
            [],
            [obs],
            sim_objects=[],
            observation_frame=frame,
        )
        assert math.isfinite(res.v_mps)
        assert math.isfinite(res.omega_radps)
        obs.step(0.05)


def test_2_identical_observed_history_identical_control() -> None:
    """Obstacles with identical observed state/history but different hidden future schedules
    must produce bit-for-bit identical observations and DWA control commands.
    """
    cfg = DEFAULT_CONFIG
    robot = RobotState(x_m=1.5, y_m=3.0, theta_rad=0.0, v_mps=0.6, omega_radps=0.0)

    # Same current position & velocity, totally different hidden future schedules
    obs_a = _make_obstacle(
        obs_id="DYN-1",
        x_m=4.0,
        y_m=3.0,
        vx_mps=-1.0,
        vy_mps=0.0,
        behavior=MovementBehavior.CROSSING,
        waypoint_a_m=(3.95, 3.0),  # about to reverse immediately on next step!
        waypoint_b_m=(8.0, 3.0),
    )
    obs_b = _make_obstacle(
        obs_id="DYN-1",
        x_m=4.0,
        y_m=3.0,
        vx_mps=-1.0,
        vy_mps=0.0,
        behavior=MovementBehavior.STOP_AND_GO,
        move_duration_s=0.05,
        stop_duration_s=5.0,
        phase_time_s=0.049,  # about to stop immediately on next step!
        waypoint_a_m=(0.5, 3.0),
        waypoint_b_m=(8.0, 3.0),
    )

    pipe_a = ObservationPipeline(cfg.observation)
    pipe_b = ObservationPipeline(cfg.observation)
    dwa_a = DWAPlanner(cfg.world, cfg.robot, cfg.dwa, observation_cfg=cfg.observation)
    dwa_b = DWAPlanner(cfg.world, cfg.robot, cfg.dwa, observation_cfg=cfg.observation)
    path = [(1.5, 3.0), (9.0, 3.0)]
    dwa_a.set_path(path)
    dwa_b.set_path(path)

    # Feed two identical observations (t=0.0 and t=0.05 with same displacement)
    for t_s, dx in [(0.0, 0.0), (0.05, -0.05)]:
        obs_a.x_m = 4.0 + dx
        obs_b.x_m = 4.0 + dx
        frame_a = pipe_a.observe(t_s, robot, [obs_a], sim_objects=[])
        frame_b = pipe_b.observe(t_s, robot, [obs_b], sim_objects=[])
        res_a = dwa_a.compute_command(robot, 0.05, [], [obs_a], observation_frame=frame_a)
        res_b = dwa_b.compute_command(robot, 0.05, [], [obs_b], observation_frame=frame_b)

        assert res_a.v_mps == res_b.v_mps
        assert res_a.omega_radps == res_b.omega_radps
        assert res_a.predicted_min_clearance_m == res_b.predicted_min_clearance_m
        assert res_a.selected_trajectory == res_b.selected_trajectory


def test_3_new_obstacle_appearance_starts_with_zero_velocity() -> None:
    """First observation must NOT copy ground-truth velocity; it starts with v=0 and initial uncertainty."""
    obs_cfg = ObservationConfig(initial_velocity_uncertainty_mps=0.35)
    pipeline = ObservationPipeline(obs_cfg)
    robot = RobotState(x_m=1.0, y_m=1.0, theta_rad=0.0)
    obs = _make_obstacle(obs_id="NEW-1", x_m=5.0, y_m=2.0, vx_mps=2.5, vy_mps=-1.2)

    frame = pipeline.observe(0.0, robot, [obs])
    assert len(frame.obstacles) == 1
    o = frame.obstacles[0]
    assert o.vx_mps == 0.0
    assert o.vy_mps == 0.0
    assert o.velocity_valid is False
    assert o.velocity_uncertainty_mps == pytest.approx(0.35)


def test_4_causal_velocity_estimation_from_two_observations() -> None:
    """Two successive timestamped observations yield accurate causal finite-difference velocity."""
    obs_cfg = ObservationConfig(velocity_ema_alpha=1.0, velocity_uncertainty_mps=0.05)
    pipeline = ObservationPipeline(obs_cfg)
    robot = RobotState(x_m=1.0, y_m=1.0, theta_rad=0.0)
    obs = _make_obstacle(obs_id="VEL-1", x_m=3.0, y_m=4.0, vx_mps=1.5, vy_mps=-0.5)

    pipeline.observe(0.0, robot, [obs])
    obs.x_m += 1.5 * 0.10
    obs.y_m += -0.5 * 0.10
    frame2 = pipeline.observe(0.10, robot, [obs])

    o = frame2.obstacles[0]
    assert o.velocity_valid is True
    assert o.vx_mps == pytest.approx(1.5, abs=1e-9)
    assert o.vy_mps == pytest.approx(-0.5, abs=1e-9)
    assert o.velocity_uncertainty_mps == pytest.approx(0.05)


def test_5_observation_latency_propagation() -> None:
    """Delayed observations record observation_age_s >= latency and propagate causally."""
    obs_cfg = ObservationConfig(observation_latency_s=0.20, velocity_ema_alpha=1.0)
    pipeline = ObservationPipeline(obs_cfg)
    robot = RobotState(x_m=1.0, y_m=1.0, theta_rad=0.0)
    obs = _make_obstacle(obs_id="LAT-1", x_m=2.0, y_m=2.0, vx_mps=1.0, vy_mps=0.0)

    # Step ground truth and observe at t=0.0, 0.1, 0.2, 0.3
    frames: list[ObservationFrame] = []
    for i in range(4):
        t_s = i * 0.10
        obs.x_m = 2.0 + 1.0 * t_s
        frames.append(pipeline.observe(t_s, robot, [obs]))

    latest = frames[-1]
    assert latest.sim_time_s == pytest.approx(0.30)
    o = latest.obstacles[0]
    assert o.timestamp_s <= 0.10 + 1e-9
    assert o.observation_age_s >= 0.20 - 1e-9
    # Causal prediction at current time (0.30) should propagate from delayed timestamp (0.10)
    preds = o.predict_causal_positions(latest.sim_time_s, 0.10, 1, growth_rate_mps=0.02)
    assert preds[0][0] == pytest.approx(o.x_m + o.vx_mps * (latest.sim_time_s - o.timestamp_s), abs=1e-6)


def test_6_stale_observation_rejection_and_marking() -> None:
    """Stale tracks and invalid/out-of-order timestamps must be rejected/marked invalid."""
    obs_cfg = ObservationConfig(max_observation_age_s=0.25)
    est = ObstacleStateEstimator(obs_cfg)

    # Invalid timestamps rejected
    assert est.update("A", 1.0, 1.0, 0.3, -0.1, 0.5) is None
    assert est.update("A", 1.0, 1.0, 0.3, math.nan, 0.5) is None
    assert est.update("A", 1.0, 1.0, 0.3, 0.6, 0.5) is None  # future timestamp

    # Valid update at t=0.1, then regressive timestamp 0.05 rejected
    assert est.update("A", 1.0, 1.0, 0.3, 0.1, 0.1) is not None
    assert est.update("A", 1.2, 1.0, 0.3, 0.05, 0.2) is None

    # Disappearance via ObservationPipeline marks obstacle stale after max_observation_age_s
    pipe = ObservationPipeline(obs_cfg)
    robot = RobotState(x_m=0.0, y_m=0.0, theta_rad=0.0)
    obs = _make_obstacle(obs_id="STALE-1", x_m=3.0, y_m=3.0)
    pipe.observe(0.0, robot, [obs], visible_ids={"STALE-1"})
    f_stale = pipe.observe(0.40, robot, [obs], visible_ids=set())
    assert "STALE-1" in f_stale.stale_obstacle_ids
    assert all(not o.is_valid for o in f_stale.obstacles if o.obstacle_id == "STALE-1")


def test_7_uncertainty_growth_with_age_and_horizon() -> None:
    """Effective radius and geometry uncertainty pad grow monotonically with age and horizon."""
    obs = ObservedObstacle(
        obstacle_id="UNC-1",
        x_m=3.0,
        y_m=3.0,
        radius_m=0.4,
        vx_mps=1.0,
        vy_mps=0.0,
        heading_rad=0.0,
        velocity_valid=True,
        timestamp_s=1.0,
        observation_age_s=0.2,
        position_uncertainty_m=0.05,
        velocity_uncertainty_mps=0.10,
        is_valid=True,
        geometry=OrientedRectGeometry(3.0, 3.0, 0.8, 1.2, 0.0),
    )
    preds = obs.predict_causal_positions(current_time_s=1.2, dt_s=0.1, steps=5, growth_rate_mps=0.02)
    radii = [p[2] for p in preds]
    assert all(radii[i] < radii[i + 1] for i in range(len(radii) - 1))

    _, pad0 = obs.predict_causal_geometry_at(current_time_s=1.2, t_offset_s=0.0, growth_rate_mps=0.02)
    _, pad5 = obs.predict_causal_geometry_at(current_time_s=1.2, t_offset_s=0.5, growth_rate_mps=0.02)
    assert pad5 > pad0


def test_8_stop_and_go_schedule_isolation() -> None:
    """Stopped STOP_AND_GO obstacle is predicted stationary until observed motion resumes."""
    cfg = DEFAULT_CONFIG
    pipe = ObservationPipeline(cfg.observation)
    robot = RobotState(x_m=0.5, y_m=2.0, theta_rad=0.0)
    obs = _make_obstacle(
        obs_id="SG-1",
        x_m=4.0,
        y_m=2.0,
        vx_mps=0.0,
        vy_mps=0.0,
        behavior=MovementBehavior.STOP_AND_GO,
        move_duration_s=1.0,
        stop_duration_s=0.15,
        phase_time_s=1.10,  # stopped, about to resume in 0.05s
    )

    # Step 0 & 1 while still stationary
    f0 = pipe.observe(0.0, robot, [obs])
    f1 = pipe.observe(0.04, robot, [obs])
    assert f1.obstacles[0].vx_mps == pytest.approx(0.0)
    preds = f1.obstacles[0].predict_causal_positions(0.04, 0.1, 5, cfg.observation.uncertainty_growth_rate_mps)
    assert all(p[0] == pytest.approx(4.0) for p in preds)


def test_9_replan_uses_only_observed_obstacles(monkeypatch: pytest.MonkeyPatch) -> None:
    """replan_global_path_around_obstacles with observation_frame ignores hidden schedule & invalid obs."""
    def _forbidden_predict(*args: object, **kwargs: object) -> object:
        raise AssertionError("predict_positions must not be called during replan!")

    monkeypatch.setattr(DynamicObstacle, "predict_positions", _forbidden_predict)

    grid = np.zeros((40, 40), dtype=np.uint8)
    obs = _make_obstacle(obs_id="REP-1", x_m=3.0, y_m=2.0, vx_mps=0.0, vy_mps=0.0, radius_m=0.5)
    pipe = ObservationPipeline(DEFAULT_CONFIG.observation)
    robot = RobotState(x_m=1.0, y_m=2.0, theta_rad=0.0)
    frame = pipe.observe(0.0, robot, [obs])

    ok, path = replan_global_path_around_obstacles(
        base_occupancy_grid=grid,
        resolution_m=0.2,
        robot_x_m=1.0,
        robot_y_m=2.0,
        goal_x_m=6.0,
        goal_y_m=2.0,
        robot_radius_m=0.25,
        safety_margin_m=0.15,
        dynamic_obstacles=[obs],
        observation_frame=frame,
    )
    assert ok is True
    assert len(path) >= 2


def test_10_source_code_isolation_audit() -> None:
    """Controller modules must contain zero references to DynamicObstacle.predict_positions or schedule internals."""
    root = Path(__file__).resolve().parent.parent
    forbidden = [
        "predict_positions",
        "_phase_time_s",
        "move_duration_s",
        "stop_duration_s",
        "waypoint_a_m",
        "waypoint_b_m",
        "reverse_at_bounds",
    ]
    for rel_path in ["algorithms/dwa.py", "algorithms/astar.py", "algorithms/observation.py"]:
        text = (root / rel_path).read_text(encoding="utf-8")
        for token in forbidden:
            assert token not in text, f"Forbidden schedule token '{token}' found in {rel_path}"


def test_11_end_to_end_hidden_reversal_schedule() -> None:
    """End-to-end: hidden reversal schedule cannot be anticipated before reversal;
    once reversal occurs in ground truth, causal estimator tracks the new velocity.
    """
    cfg = DEFAULT_CONFIG
    pipe_rev = ObservationPipeline(ObservationConfig(velocity_ema_alpha=1.0))
    pipe_const = ObservationPipeline(ObservationConfig(velocity_ema_alpha=1.0))
    dwa_rev = DWAPlanner(cfg.world, cfg.robot, cfg.dwa)
    dwa_const = DWAPlanner(cfg.world, cfg.robot, cfg.dwa)
    path = [(1.0, 2.0), (9.0, 2.0)]
    dwa_rev.set_path(path)
    dwa_const.set_path(path)

    robot = RobotState(x_m=1.0, y_m=2.0, theta_rad=0.0, v_mps=0.5, omega_radps=0.0)
    obs_rev = _make_obstacle(
        obs_id="REV-1",
        x_m=4.5,
        y_m=2.0,
        vx_mps=1.0,
        vy_mps=0.0,
        waypoint_a_m=(2.0, 2.0),
        waypoint_b_m=(4.70, 2.0),
    )
    obs_const = _make_obstacle(
        obs_id="REV-1",
        x_m=4.5,
        y_m=2.0,
        vx_mps=1.0,
        vy_mps=0.0,
        waypoint_a_m=(2.0, 2.0),
        waypoint_b_m=(12.0, 2.0),
    )

    # Before reversal (t=0.0, 0.05, 0.10, 0.15): both worlds match identically
    dt = 0.05
    for step in range(4):
        t_s = step * dt
        f_r = pipe_rev.observe(t_s, robot, [obs_rev])
        f_c = pipe_const.observe(t_s, robot, [obs_const])
        res_r = dwa_rev.compute_command(robot, dt, [], [obs_rev], observation_frame=f_r)
        res_c = dwa_const.compute_command(robot, dt, [], [obs_const], observation_frame=f_c)
        assert res_r.v_mps == res_c.v_mps
        assert res_r.omega_radps == res_c.omega_radps
        obs_rev.step(dt)
        obs_const.step(dt)

    # Step a couple more times so obs_rev bounces off 4.70 and moves left
    for step in range(4, 8):
        t_s = step * dt
        f_r = pipe_rev.observe(t_s, robot, [obs_rev])
        obs_rev.step(dt)

    assert f_r.obstacles[0].vx_mps < 0.0
