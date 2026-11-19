"""Unit tests for CorridorPassingManager state machine, multi-obstacle tracking, and ABORT_HOLD braking."""

from __future__ import annotations

from dataclasses import replace
import math

from algorithms.corridor_passing import (
    CorridorPassingManager,
    PassingState,
    build_static_occupancy_grid,
)
from algorithms.dwa import DWAPlanner
from algorithms.observation import ObservationFrame, ObservedObstacle, ObservedRobotState
from config import DEFAULT_CONFIG
from simulation.entities import RobotState, StaticObstacle
from simulation.geometry import CircleGeometry, OrientedRectGeometry


def _make_obs(
    obs_id: str,
    x_m: float,
    y_m: float,
    vx_mps: float = 0.0,
    vy_mps: float = 0.0,
    radius_m: float = 0.35,
    sim_time_s: float = 0.0,
    geom: CircleGeometry | OrientedRectGeometry | None = None,
) -> ObservedObstacle:
    return ObservedObstacle(
        obstacle_id=obs_id,
        x_m=x_m,
        y_m=y_m,
        radius_m=radius_m,
        vx_mps=vx_mps,
        vy_mps=vy_mps,
        heading_rad=0.0,
        velocity_valid=True,
        timestamp_s=sim_time_s,
        observation_age_s=0.0,
        position_uncertainty_m=0.0,
        velocity_uncertainty_mps=0.0,
        is_valid=True,
        geometry=geom,
    )


def _make_frame(
    sim_time_s: float,
    robot: RobotState,
    obstacles: list[ObservedObstacle],
) -> ObservationFrame:
    return ObservationFrame(
        sim_time_s=sim_time_s,
        robot=ObservedRobotState(
            x_m=robot.x_m,
            y_m=robot.y_m,
            theta_rad=robot.theta_rad,
            v_mps=robot.v_mps,
            omega_radps=robot.omega_radps,
            timestamp_s=sim_time_s,
        ),
        obstacles=obstacles,
    )


def test_1_state_machine_armed_active_abort_recover_idle() -> None:
    """Verify IDLE -> PASSING_ARMED -> PASSING_ACTIVE -> ABORT_HOLD -> PASSING_ARMED -> IDLE."""
    cfg = DEFAULT_CONFIG
    dwa_cfg = replace(cfg.dwa, proactive_hysteresis_steps=1, proactive_replan_cooldown_s=0.0)
    mgr = CorridorPassingManager(cfg.world, cfg.robot, dwa_cfg, cfg.observation)
    static_obs = [
        StaticObstacle(0.0, 5.5, 20.0, 0.5),
        StaticObstacle(0.0, 9.5, 20.0, 0.5),
    ]
    grid = build_static_occupancy_grid(cfg.world, static_obs)
    path = [(2.0, 7.5), (10.0, 7.5), (18.0, 7.5)]
    robot = RobotState(2.0, 7.5, 0.0, 0.8, 0.0)
    obs = _make_obs("dyn_1", 4.0, 7.5, vx_mps=0.1, sim_time_s=0.0)
    frame = _make_frame(0.0, robot, [obs])

    assess = mgr.assess_corridor(robot, 0.05, path, static_obs, frame)
    assert assess.trigger_replan
    assert mgr.passing_state == PassingState.IDLE

    accepted_path, _ = mgr.generate_and_validate_candidates(
        0.0, robot, 0.05, (18.0, 7.5), grid, cfg.world.grid_resolution_m, static_obs, frame,
        lambda _p: (True, 0.6, 0.2, 0.4, 0.4),
    )
    assert accepted_path is not None
    assert mgr.passing_state == PassingState.PASSING_ARMED

    # Robot engages laterally -> PASSING_ACTIVE
    robot_lat = RobotState(2.8, 7.5 + dwa_cfg.passing_lateral_engage_offset_m + 0.05, 0.0, 0.8, 0.0)
    frame_lat = _make_frame(0.5, robot_lat, [_make_obs("dyn_1", 4.05, 7.5, vx_mps=0.1, sim_time_s=0.5)])
    assess_lat = mgr.assess_corridor(robot_lat, 0.05, accepted_path, static_obs, frame_lat)
    assert mgr.passing_state == PassingState.PASSING_ACTIVE
    assert assess_lat.passing_state == PassingState.PASSING_ACTIVE.value

    # All candidates rejected while PASSING_ACTIVE -> ABORT_HOLD
    rejected_path, _ = mgr.generate_and_validate_candidates(
        0.6, robot_lat, 0.05, (18.0, 7.5), grid, cfg.world.grid_resolution_m, static_obs, frame_lat,
        lambda _p: (False, 0.0, 0.0, -0.1, -0.1),
    )
    assert rejected_path is None
    assert mgr.passing_state == PassingState.ABORT_HOLD

    # New safe path accepted while obstacle still unpassed -> PASSING_ARMED (not IDLE or PASSING_ACTIVE)
    recovered_path, _ = mgr.generate_and_validate_candidates(
        1.0, robot_lat, 0.05, (18.0, 7.5), grid, cfg.world.grid_resolution_m, static_obs, frame_lat,
        lambda _p: (True, 0.5, 0.1, 0.35, 0.35),
    )
    assert recovered_path is not None
    assert mgr.passing_state == PassingState.PASSING_ARMED

    # Robot passes obstacle geometrically along locked tangent -> IDLE
    robot_past = RobotState(
        4.05 + 0.35 + cfg.robot.radius_m + dwa_cfg.safety_margin_m + dwa_cfg.passing_completion_margin_m + 0.2,
        7.8, 0.0, 0.8, 0.0,
    )
    frame_past = _make_frame(3.0, robot_past, [_make_obs("dyn_1", 4.05, 7.5, vx_mps=0.0, sim_time_s=3.0)])
    assess_past = mgr.assess_corridor(robot_past, 0.05, recovered_path, static_obs, frame_past)
    assert mgr.passing_state == PassingState.IDLE
    assert assess_past.passing_state == PassingState.IDLE.value
    assert len(mgr.active_obstacles) == 0


def test_2_multi_obstacle_different_speeds_and_lateral_positions() -> None:
    """PASSING_ACTIVE must persist until ALL tracked obstacles are geometrically passed."""
    cfg = DEFAULT_CONFIG
    dwa_cfg = replace(cfg.dwa, proactive_hysteresis_steps=1, proactive_replan_cooldown_s=0.0)
    mgr = CorridorPassingManager(cfg.world, cfg.robot, dwa_cfg, cfg.observation)
    static_obs = [
        StaticObstacle(0.0, 5.5, 20.0, 0.5),
        StaticObstacle(0.0, 9.5, 20.0, 0.5),
    ]
    grid = build_static_occupancy_grid(cfg.world, static_obs)
    path = [(2.0, 7.5), (10.0, 7.5), (18.0, 7.5)]
    robot = RobotState(2.0, 7.5, 0.0, 0.8, 0.0)
    obs1 = _make_obs("obs_near", 3.5, 7.6, vx_mps=0.1, sim_time_s=0.0)
    obs2 = _make_obs("obs_far", 5.4, 7.3, vx_mps=-0.2, sim_time_s=0.0, geom=OrientedRectGeometry(5.4, 7.3, 0.6, 0.8, 0.0))
    frame = _make_frame(0.0, robot, [obs1, obs2])

    mgr.assess_corridor(robot, 0.05, path, static_obs, frame)
    accepted_path, _ = mgr.generate_and_validate_candidates(
        0.0, robot, 0.05, (18.0, 7.5), grid, cfg.world.grid_resolution_m, static_obs, frame,
        lambda _p: (True, 0.6, 0.2, 0.4, 0.4),
    )
    assert accepted_path is not None

    # Move past obs_near, engage laterally, but obs_far is still ahead
    robot_mid = RobotState(4.65, 7.9, 0.0, 0.8, 0.0)
    frame_mid = _make_frame(1.0, robot_mid, [
        _make_obs("obs_near", 3.5, 7.6, vx_mps=0.0, sim_time_s=1.0),
        _make_obs("obs_far", 5.4, 7.3, vx_mps=-0.1, sim_time_s=1.0, geom=OrientedRectGeometry(5.4, 7.3, 0.6, 0.8, 0.0)),
    ])
    mgr.assess_corridor(robot_mid, 0.05, accepted_path, static_obs, frame_mid)
    assert mgr.passing_state == PassingState.PASSING_ACTIVE
    assert mgr.active_obstacles["obs_near"].geom_passed is True
    assert mgr.active_obstacles["obs_far"].geom_passed is False

    # Now pass obs_far as well -> transitions to IDLE and clears active set
    robot_done = RobotState(7.0, 7.9, 0.0, 0.8, 0.0)
    frame_done = _make_frame(2.0, robot_done, [
        _make_obs("obs_near", 3.5, 7.6, vx_mps=0.0, sim_time_s=2.0),
        _make_obs("obs_far", 5.3, 7.3, vx_mps=0.0, sim_time_s=2.0, geom=OrientedRectGeometry(5.3, 7.3, 0.6, 0.8, 0.0)),
    ])
    mgr.assess_corridor(robot_done, 0.05, accepted_path, static_obs, frame_done)
    assert mgr.passing_state == PassingState.IDLE
    assert len(mgr.active_obstacles) == 0


def test_3_observation_loss_grows_uncertainty_and_reset_clears() -> None:
    """Losing an obstacle from observation_frame must preserve state and grow bounded uncertainty."""
    cfg = DEFAULT_CONFIG
    dwa_cfg = replace(cfg.dwa, proactive_hysteresis_steps=1, proactive_replan_cooldown_s=0.0)
    mgr = CorridorPassingManager(cfg.world, cfg.robot, dwa_cfg, cfg.observation)
    static_obs = [
        StaticObstacle(0.0, 5.5, 20.0, 0.5),
        StaticObstacle(0.0, 9.5, 20.0, 0.5),
    ]
    grid = build_static_occupancy_grid(cfg.world, static_obs)
    path = [(2.0, 7.5), (18.0, 7.5)]
    robot = RobotState(2.0, 7.5, 0.0, 0.8, 0.0)
    frame = _make_frame(0.0, robot, [_make_obs("dyn_lost", 4.0, 7.5, vx_mps=0.0, sim_time_s=0.0)])

    mgr.assess_corridor(robot, 0.05, path, static_obs, frame)
    accepted_path, _ = mgr.generate_and_validate_candidates(
        0.0, robot, 0.05, (18.0, 7.5), grid, cfg.world.grid_resolution_m, static_obs, frame,
        lambda _p: (True, 0.6, 0.2, 0.4, 0.4),
    )
    assert accepted_path is not None

    # Obstacle disappears from observation_frame while robot is still behind it
    robot_wait = RobotState(2.5, 7.8, 0.0, 0.8, 0.0)
    empty_frame = _make_frame(0.5, robot_wait, [])
    mgr.assess_corridor(robot_wait, 0.5, accepted_path, static_obs, empty_frame)
    rec = mgr.active_obstacles["dyn_lost"]
    assert rec.observation_lost is True
    assert rec.position_uncertainty_m > 0.0
    assert rec.position_uncertainty_m <= dwa_cfg.passing_max_lost_uncertainty_m
    assert mgr.passing_state == PassingState.PASSING_ACTIVE

    mgr.reset()
    assert mgr.passing_state == PassingState.IDLE
    assert len(mgr.active_obstacles) == 0


def test_4_dwa_abort_hold_brakes_with_is_safe_and_handles_negative_v() -> None:
    """ABORT_HOLD must brake toward 0 using only is_safe=True candidates, never accelerate."""
    cfg = DEFAULT_CONFIG
    planner = DWAPlanner(cfg.world, cfg.robot, cfg.dwa, cfg.observation)
    planner.set_path([(2.0, 7.5), (18.0, 7.5)])

    # Force ABORT_HOLD in corridor manager
    planner._corridor_manager._passing_state = PassingState.ABORT_HOLD
    planner._corridor_manager._locked_tangent = (1.0, 0.0)
    planner._corridor_manager._locked_origin_m = (2.0, 7.5)
    planner._corridor_manager._cooldown_s = 1.0  # keep in ABORT_HOLD without replanning

    robot_fwd = RobotState(3.0, 7.5, 0.0, 0.8, 0.0)
    frame_fwd = _make_frame(1.0, robot_fwd, [_make_obs("d1", 9.0, 7.5, 0.0, 0.0, sim_time_s=1.0)])
    res_fwd = planner.compute_command(
        robot_fwd, 0.1, [], [], observation_frame=frame_fwd,
    )
    assert res_fwd.fallback_active
    assert res_fwd.fallback_reason == "ABORT_HOLD_BRAKING"
    assert res_fwd.v_mps <= robot_fwd.v_mps + 1e-6

    # Negative current velocity must brake toward 0.0, not grow reverse speed
    robot_rev = RobotState(3.0, 7.5, 0.0, -0.4, 0.0)
    frame_rev = _make_frame(1.1, robot_rev, [_make_obs("d1", 9.0, 7.5, 0.0, 0.0, sim_time_s=1.1)])
    res_rev = planner.compute_command(
        robot_rev, 0.1, [], [], observation_frame=frame_rev,
    )
    assert res_rev.fallback_active
    assert res_rev.fallback_reason == "ABORT_HOLD_BRAKING"
    assert res_rev.v_mps >= robot_rev.v_mps - 1e-6
    assert res_rev.v_mps <= 1e-6
    assert math.isfinite(res_rev.v_mps)


def test_5_causal_reversal_bypasses_cooldown_preserving_lock() -> None:
    """When an active obstacle reverses (+ -> -) during PASSING_ACTIVE, cooldown is bypassed."""
    cfg = DEFAULT_CONFIG
    dwa_cfg = replace(cfg.dwa, proactive_hysteresis_steps=1, proactive_replan_cooldown_s=2.5)
    mgr = CorridorPassingManager(cfg.world, cfg.robot, dwa_cfg, cfg.observation)
    static_obs = [
        StaticObstacle(0.0, 5.5, 20.0, 0.5),
        StaticObstacle(0.0, 9.5, 20.0, 0.5),
    ]
    grid = build_static_occupancy_grid(cfg.world, static_obs)
    path = [(2.0, 7.5), (10.0, 7.5), (18.0, 7.5)]
    robot = RobotState(2.0, 7.5, 0.0, 0.8, 0.0)
    obs_away = _make_obs("dyn_rev", 4.0, 7.5, vx_mps=1.15, sim_time_s=0.0)
    frame0 = _make_frame(0.0, robot, [obs_away])

    mgr.assess_corridor(robot, 0.05, path, static_obs, frame0)
    accepted_path, _ = mgr.generate_and_validate_candidates(
        0.0, robot, 0.05, (18.0, 7.5), grid, cfg.world.grid_resolution_m, static_obs, frame0,
        lambda _p: (True, 0.6, 0.2, 0.4, 0.4),
    )
    assert accepted_path is not None

    # Engage laterally while obstacle still moves away -> PASSING_ACTIVE & cooldown active
    robot_lat = RobotState(2.8, 7.5 + dwa_cfg.passing_lateral_engage_offset_m + 0.05, 0.0, 0.8, 0.0)
    frame_lat = _make_frame(0.2, robot_lat, [_make_obs("dyn_rev", 4.2, 7.5, vx_mps=1.15, sim_time_s=0.2)])
    assess_cd = mgr.assess_corridor(robot_lat, 0.05, path, static_obs, frame_lat)
    assert mgr.passing_state == PassingState.PASSING_ACTIVE
    assert assess_cd.trigger_replan is False
    assert assess_cd.reason == "COOLDOWN_ACTIVE"

    # Obstacle reverses direction -> cooldown bypassed, PASSING_ACTIVE & obstacle lock preserved
    frame_rev = _make_frame(0.3, robot_lat, [_make_obs("dyn_rev", 4.1, 7.5, vx_mps=-1.15, sim_time_s=0.3)])
    assess_rev = mgr.assess_corridor(robot_lat, 0.05, path, static_obs, frame_rev)
    assert mgr.passing_state == PassingState.PASSING_ACTIVE
    assert "dyn_rev" in mgr.active_obstacles
    assert assess_rev.trigger_replan is True
    assert assess_rev.reason == "CORRIDOR_LATERAL_PASS_TRIGGERED"
