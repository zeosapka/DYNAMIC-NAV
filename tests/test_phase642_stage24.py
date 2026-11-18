"""Phase 6.4 Stage 2.4 regression tests for predictive lateral holding pose recovery."""

from __future__ import annotations

from config import DEFAULT_CONFIG
from algorithms.dwa import (
    DWAPlanner,
    WaitingZoneState,
    plan_lateral_holding_maneuver,
)
from simulation.dynamic_obstacles import DynamicObstacle
from simulation.entities import RobotState, StaticObstacle


def _rear_wall_and_front_threat():
    """Robot at (5.0, 7.5), rear wall blocking reverse (>=0.20m clear), approaching obstacle ahead."""
    state = RobotState(x_m=5.0, y_m=7.5, theta_rad=0.0, v_mps=0.0, omega_radps=0.0)
    rear_wall = StaticObstacle(x_m=4.0, y_m=6.5, width_m=0.45, height_m=2.0)
    # Approaches at -0.60 m/s from x=6.60 (0.80 m clear at t=0) -> holding still drops below
    # 0.15 m within 1.1 s (GATE_CLEAR_HOLD_UNSAFE), rear wall blocks reverse, leaving lateral escape.
    front_obs = DynamicObstacle(
        obstacle_id="DYN-FRONT",
        x_m=6.60,
        y_m=7.5,
        vx_mps=-0.60,
        vy_mps=0.0,
        radius_m=0.45,
        behavior="CONSTANT_VELOCITY",
    )
    return state, rear_wall, front_obs


def test_gate_clear_hold_unsafe_selects_safe_left_holding_pose() -> None:
    """1. No safe reverse + right side blocked -> select safe left holding pose."""
    dwa = DWAPlanner(DEFAULT_CONFIG.world, DEFAULT_CONFIG.robot, DEFAULT_CONFIG.dwa)
    dwa.set_path([(5.0, 7.5), (15.0, 7.5)])
    state, rear_wall, front_obs = _rear_wall_and_front_threat()
    right_wall = StaticObstacle(x_m=4.0, y_m=5.5, width_m=3.0, height_m=1.2)

    res = dwa.compute_command(state, 0.05, [rear_wall, right_wall], [front_obs])
    assert res.waiting_state == WaitingZoneState.GATE_CLEAR_HOLD_UNSAFE.value
    assert res.fallback_priority == 4
    assert res.fallback_reason == "LATERAL_HOLDING_POSE"
    assert res.holding_pose_m is not None
    assert res.holding_pose_m[1] > state.y_m
    assert res.predicted_min_clearance_m is not None
    assert res.predicted_min_clearance_m >= DEFAULT_CONFIG.dwa.safety_margin_m


def test_gate_clear_hold_unsafe_selects_safe_right_holding_pose() -> None:
    """2. No safe reverse + left side blocked -> select safe right holding pose."""
    dwa = DWAPlanner(DEFAULT_CONFIG.world, DEFAULT_CONFIG.robot, DEFAULT_CONFIG.dwa)
    dwa.set_path([(5.0, 7.5), (15.0, 7.5)])
    state, rear_wall, front_obs = _rear_wall_and_front_threat()
    left_wall = StaticObstacle(x_m=4.0, y_m=8.3, width_m=3.0, height_m=1.2)

    res = dwa.compute_command(state, 0.05, [rear_wall, left_wall], [front_obs])
    assert res.waiting_state == WaitingZoneState.GATE_CLEAR_HOLD_UNSAFE.value
    assert res.fallback_priority == 4
    assert res.fallback_reason == "LATERAL_HOLDING_POSE"
    assert res.holding_pose_m is not None
    assert res.holding_pose_m[1] < state.y_m
    assert res.predicted_min_clearance_m is not None
    assert res.predicted_min_clearance_m >= DEFAULT_CONFIG.dwa.safety_margin_m


def test_both_holding_poses_unsafe_preserves_priority2_behavior() -> None:
    """3. When both left and right holding poses are blocked, Priority 2 fallback is preserved."""
    dwa = DWAPlanner(DEFAULT_CONFIG.world, DEFAULT_CONFIG.robot, DEFAULT_CONFIG.dwa)
    dwa.set_path([(5.0, 7.5), (15.0, 7.5)])
    state, rear_wall, front_obs = _rear_wall_and_front_threat()
    left_wall = StaticObstacle(x_m=4.0, y_m=8.3, width_m=3.0, height_m=1.2)
    right_wall = StaticObstacle(x_m=4.0, y_m=5.5, width_m=3.0, height_m=1.2)

    res = dwa.compute_command(state, 0.05, [rear_wall, left_wall, right_wall], [front_obs])
    assert res.waiting_state == WaitingZoneState.GATE_CLEAR_HOLD_UNSAFE.value
    assert res.fallback_reason != "LATERAL_HOLDING_POSE"
    assert res.fallback_priority in (1, 2, 5)


def test_safe_final_pose_with_unsafe_intermediate_trajectory_is_rejected() -> None:
    """4. Geometrically safe final poses (y=8.9, 6.1) must be rejected if intermediate path hits obstacles."""
    state = RobotState(x_m=5.0, y_m=7.5, theta_rad=0.0, v_mps=0.0, omega_radps=0.0)
    # Thin horizontal walls at y=8.1 and y=6.8 leave final poses y=8.9 and y=6.1 clear,
    # but block the intermediate lateral trajectory.
    mid_top = StaticObstacle(x_m=4.0, y_m=8.0, width_m=3.0, height_m=0.35)
    mid_bot = StaticObstacle(x_m=4.0, y_m=6.65, width_m=3.0, height_m=0.35)
    res = plan_lateral_holding_maneuver(
        state, (15.0, 7.5), 0.05, DEFAULT_CONFIG.world, DEFAULT_CONFIG.robot, DEFAULT_CONFIG.dwa,
        [mid_top, mid_bot], [],
    )
    assert res is None


def test_safe_trajectory_with_unsafe_braking_path_is_rejected() -> None:
    """5. A lateral maneuver whose initial braking trajectory violates 0.15 m must be rejected."""
    state = RobotState(x_m=5.0, y_m=7.5, theta_rad=0.0, v_mps=1.5, omega_radps=0.0)
    # Static wall right ahead at x=6.05: clearance at t=0 is 0.70 m,
    # but at v=1.5 m/s braking distance + reaction (0.30 m) + first step (0.075 m) leaves < 0.15 m
    # while turning laterally.
    front_wall = StaticObstacle(x_m=5.70, y_m=6.0, width_m=0.5, height_m=3.0)
    res = plan_lateral_holding_maneuver(
        state, (15.0, 7.5), 0.05, DEFAULT_CONFIG.world, DEFAULT_CONFIG.robot, DEFAULT_CONFIG.dwa,
        [front_wall], [],
    )
    assert res is None


def test_static_obstacle_blocking_lateral_maneuver_is_rejected() -> None:
    """6. Static obstacles directly on both lateral holding poses reject both poses."""
    state = RobotState(x_m=5.0, y_m=7.5, theta_rad=0.0, v_mps=0.0, omega_radps=0.0)
    top_block = StaticObstacle(x_m=4.2, y_m=8.4, width_m=1.6, height_m=1.0)
    bot_block = StaticObstacle(x_m=4.2, y_m=5.6, width_m=1.6, height_m=1.0)
    res = plan_lateral_holding_maneuver(
        state, (15.0, 7.5), 0.05, DEFAULT_CONFIG.world, DEFAULT_CONFIG.robot, DEFAULT_CONFIG.dwa,
        [top_block, bot_block], [],
    )
    assert res is None


def test_dynamic_obstacle_blocking_lateral_maneuver_is_rejected() -> None:
    """7. Dynamic obstacles moving through both lateral channels reject the holding maneuver."""
    state = RobotState(x_m=5.0, y_m=7.5, theta_rad=0.0, v_mps=0.0, omega_radps=0.0)
    dyn_top = DynamicObstacle("D-TOP", x_m=5.0, y_m=8.8, vx_mps=0.0, vy_mps=0.0, radius_m=0.45, behavior="CONSTANT_VELOCITY")
    dyn_bot = DynamicObstacle("D-BOT", x_m=5.0, y_m=6.2, vx_mps=0.0, vy_mps=0.0, radius_m=0.45, behavior="CONSTANT_VELOCITY")
    res = plan_lateral_holding_maneuver(
        state, (15.0, 7.5), 0.05, DEFAULT_CONFIG.world, DEFAULT_CONFIG.robot, DEFAULT_CONFIG.dwa,
        [], [dyn_top, dyn_bot],
    )
    assert res is None


def test_holding_pose_becoming_unsafe_during_waiting_horizon_is_rejected() -> None:
    """8. A holding pose that is safe at arrival (t=1.5 s) but swept at t=2.2 s must be rejected."""
    state = RobotState(x_m=5.0, y_m=7.5, theta_rad=0.0, v_mps=0.0, omega_radps=0.0)
    # Left & right obstacles approach at 1.0 m/s from x=7.2 so they arrive at x=5.0 at t=2.2 s (< 2.5 s)
    lat_top = DynamicObstacle("D-WAIT-TOP", x_m=7.2, y_m=8.9, vx_mps=-1.0, vy_mps=0.0, radius_m=0.45, behavior="CONSTANT_VELOCITY")
    lat_bot = DynamicObstacle("D-WAIT-BOT", x_m=7.2, y_m=6.1, vx_mps=-1.0, vy_mps=0.0, radius_m=0.45, behavior="CONSTANT_VELOCITY")
    res = plan_lateral_holding_maneuver(
        state, (15.0, 7.5), 0.05, DEFAULT_CONFIG.world, DEFAULT_CONFIG.robot, DEFAULT_CONFIG.dwa,
        [], [lat_top, lat_bot],
    )
    assert res is None


def test_safe_reverse_recovery_takes_precedence_over_lateral_holding() -> None:
    """9. When a genuinely safe reverse candidate exists, REVERSE_RECOVERY (Priority 3) takes precedence."""
    dwa = DWAPlanner(DEFAULT_CONFIG.world, DEFAULT_CONFIG.robot, DEFAULT_CONFIG.dwa)
    dwa.set_path([(5.0, 7.5), (15.0, 7.5)])
    state = RobotState(x_m=5.0, y_m=7.5, theta_rad=0.0, v_mps=0.0, omega_radps=0.0)
    obs = DynamicObstacle(
        obstacle_id="DYN-REV-FIRST",
        x_m=5.98,
        y_m=7.5,
        vx_mps=-0.06,
        vy_mps=0.0,
        radius_m=0.45,
        behavior="CONSTANT_VELOCITY",
    )
    res = dwa.compute_command(state, 0.05, [], [obs])
    assert res.waiting_state == WaitingZoneState.GATE_CLEAR_HOLD_UNSAFE.value
    assert res.fallback_priority == 3
    assert res.fallback_reason == "REVERSE_RECOVERY"
    assert res.reverse_active is True


def test_normal_dwa_outside_gate_clear_hold_unsafe_remains_unchanged() -> None:
    """10. Normal open-space DWA operation selects a forward trajectory with fallback_active=False."""
    dwa = DWAPlanner(DEFAULT_CONFIG.world, DEFAULT_CONFIG.robot, DEFAULT_CONFIG.dwa)
    dwa.set_path([(2.0, 7.5), (18.0, 7.5)])
    state = RobotState(x_m=2.0, y_m=7.5, theta_rad=0.0, v_mps=0.5, omega_radps=0.0)
    res = dwa.compute_command(state, 0.05, [], [])
    assert res.fallback_active is False
    assert res.fallback_priority is None
    assert res.v_mps > 0.5
    assert res.waiting_state == WaitingZoneState.GATE_CLEAR_HOLD_SAFE.value
