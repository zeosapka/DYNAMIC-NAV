"""Automated unit & integration tests for Phase 6.2:
Predictive Safe Waiting, Reverse Recovery, DWA Fallback & Deadlock Detection.
"""

import pytest

from algorithms.dwa import (
    DWAPlanner,
    DeadlockState,
    WaitingZoneState,
    compute_dynamic_window,
    evaluate_trajectory_with_braking,
    evaluate_waiting_zone_safety,
    predict_trajectory,
    sample_velocities,
)
from analysis.benchmark import CONTROLLER_DWA, run_single_experiment
from config import DEFAULT_CONFIG
from simulation.dynamic_obstacles import DynamicObstacle, MovementBehavior
from simulation.entities import RobotState, StaticObstacle


# ---------------------------------------------------------------------------
# A. Waiting-zone classification
# ---------------------------------------------------------------------------
def test_waiting_zone_gate_clear_hold_safe() -> None:
    robot = RobotState(2.0, 7.5, 0.0)
    obs = DynamicObstacle("OBS-FAR", 15.0, 2.0, 0.0, 0.0, 0.40, MovementBehavior.HEAD_ON)
    res = evaluate_waiting_zone_safety(
        robot, (6.0, 7.5), DEFAULT_CONFIG.world, DEFAULT_CONFIG.robot, DEFAULT_CONFIG.dwa, [], [obs]
    )
    assert res.state == WaitingZoneState.GATE_CLEAR_HOLD_SAFE
    assert res.gate_occupied is False


def test_waiting_zone_gate_clear_hold_unsafe() -> None:
    robot = RobotState(5.0, 7.5, 0.0)
    # No static gate, but oncoming obstacle will sweep right through (5.0, 7.5) within 2.5s
    obs = DynamicObstacle("OBS-ONCOMING", 7.0, 7.5, -1.2, 0.0, 0.45, MovementBehavior.HEAD_ON)
    res = evaluate_waiting_zone_safety(
        robot, (9.0, 7.5), DEFAULT_CONFIG.world, DEFAULT_CONFIG.robot, DEFAULT_CONFIG.dwa, [], [obs]
    )
    assert res.state == WaitingZoneState.GATE_CLEAR_HOLD_UNSAFE
    assert res.gate_occupied is False
    assert res.threat_time_s is not None and res.threat_time_s <= 2.5


def test_waiting_zone_gate_occupied_hold_safe() -> None:
    # Gate at x=9..11, y=6.2..8.8; obstacle inside gate moving slowly and stopping at x=9.2 (away from robot at x=6.0)
    walls = [
        StaticObstacle(9.0, 0.0, 2.0, 6.2),
        StaticObstacle(9.0, 8.8, 2.0, 6.2),
    ]
    obs = DynamicObstacle(
        "OBS-GATE-STOP",
        10.5,
        7.5,
        -0.5,
        0.0,
        0.40,
        MovementBehavior.CORRIDOR,
        waypoint_a_m=(9.2, 7.5),
        waypoint_b_m=(14.0, 7.5),
        reverse_at_bounds=True,
    )
    res = evaluate_waiting_zone_safety(
        RobotState(6.0, 7.5, 0.0),
        (9.5, 7.5),
        DEFAULT_CONFIG.world,
        DEFAULT_CONFIG.robot,
        DEFAULT_CONFIG.dwa,
        walls,
        [obs],
    )
    assert res.state == WaitingZoneState.GATE_OCCUPIED_HOLD_SAFE
    assert res.gate_occupied is True


def test_waiting_zone_hold_unsafe_evade() -> None:
    walls = [
        StaticObstacle(9.0, 0.0, 2.0, 6.2),
        StaticObstacle(9.0, 8.8, 2.0, 6.2),
    ]
    # Gate occupied AND obstacle patrol crosses directly through robot holding pose at x=7.6
    obs = DynamicObstacle(
        "OBS-GATE-CROSS",
        9.5,
        7.5,
        -1.2,
        0.0,
        0.40,
        MovementBehavior.CORRIDOR,
        waypoint_a_m=(6.0, 7.5),
        waypoint_b_m=(14.0, 7.5),
        reverse_at_bounds=True,
    )
    res = evaluate_waiting_zone_safety(
        RobotState(7.6, 7.5, 0.0),
        (9.5, 7.5),
        DEFAULT_CONFIG.world,
        DEFAULT_CONFIG.robot,
        DEFAULT_CONFIG.dwa,
        walls,
        [obs],
    )
    assert res.state == WaitingZoneState.HOLD_UNSAFE_EVADE
    assert res.gate_occupied is True


# ---------------------------------------------------------------------------
# B. Reverse motion
# ---------------------------------------------------------------------------
def test_reverse_window_respects_acceleration_and_bounds() -> None:
    dt = 0.05
    # From v=0.0, max_linear_accel=5.0 -> v_min = max(-0.6, 0 - 5*0.05) = -0.25
    win = compute_dynamic_window(0.0, 0.0, dt, DEFAULT_CONFIG.robot, DEFAULT_CONFIG.dwa, allow_reverse=True)
    assert win.v_min_mps == pytest.approx(-0.25)
    samples = sample_velocities(win, DEFAULT_CONFIG.dwa.linear_samples, DEFAULT_CONFIG.dwa.angular_samples)
    assert any(v < 0.0 for v, _ in samples)

    # From v=1.0, v_min = max(-0.6, 1.0 - 0.25) = 0.75 (cannot jump instantaneously to negative)
    win_fast = compute_dynamic_window(1.0, 0.0, dt, DEFAULT_CONFIG.robot, DEFAULT_CONFIG.dwa, allow_reverse=True)
    assert win_fast.v_min_mps == pytest.approx(0.75)


def test_reverse_braking_rejected_near_rear_obstacle_or_boundary() -> None:
    # Robot at x=0.45 facing +X (theta=0), rear wall at x=0.0 -> clearance behind is 0.10 m
    state = RobotState(0.45, 7.5, 0.0, v_mps=-0.2)
    traj = predict_trajectory(state, v_mps=-0.5, omega_radps=0.0, predict_time_s=1.0, predict_dt_s=0.1)
    is_safe, min_c, _, _ = evaluate_trajectory_with_braking(
        traj,
        0.1,
        0.35,
        0.15,
        DEFAULT_CONFIG.world,
        [],
        [],
        v_mps=-0.5,
        max_brake_decel_mps2=5.0,
        reaction_time_s=0.1,
    )
    assert is_safe is False
    assert min_c < 0.15


# ---------------------------------------------------------------------------
# C. Fallback priority & existing margin violation rule
# ---------------------------------------------------------------------------
def test_fallback_prefers_standstill_when_gate_occupied_hold_safe() -> None:
    planner = DWAPlanner(DEFAULT_CONFIG.world, DEFAULT_CONFIG.robot, DEFAULT_CONFIG.dwa)
    planner.set_path([(6.0, 7.5), (18.0, 7.5)])
    walls = [
        StaticObstacle(9.0, 0.0, 2.0, 6.2),
        StaticObstacle(9.0, 8.8, 2.0, 6.2),
    ]
    obs = DynamicObstacle(
        "OBS-SAFE-WAIT",
        10.5,
        7.5,
        -0.5,
        0.0,
        0.40,
        MovementBehavior.CORRIDOR,
        waypoint_a_m=(9.2, 7.5),
        waypoint_b_m=(14.0, 7.5),
        reverse_at_bounds=True,
    )
    res = planner.compute_command(RobotState(6.0, 7.5, 0.0), 0.05, walls, [obs])
    assert res.waiting_for_passage is True
    assert res.v_mps == pytest.approx(0.0)
    assert res.deadlock_state == DeadlockState.TEMPORARY_SAFE_WAIT.value


def test_fallback_recovery_from_existing_margin_violation_reports_honestly() -> None:
    planner = DWAPlanner(DEFAULT_CONFIG.world, DEFAULT_CONFIG.robot, DEFAULT_CONFIG.dwa)
    # Path leads down/away from obstacle at (8.5, 7.5)
    planner.set_path([(7.6, 7.5), (7.6, 4.0), (16.0, 4.0)])
    # Obstacle at (8.5, 7.5) r=0.45, robot at (7.63, 7.5) -> center dist 0.87, surface clr = 0.07m (< 0.15m, > 0)
    obs = DynamicObstacle("OBS-CLOSE", 8.5, 7.5, 0.0, 0.0, 0.45, MovementBehavior.HEAD_ON)
    res = planner.compute_command(RobotState(7.63, 7.5, -1.57, v_mps=0.0), 0.05, [], [obs])
    assert res.collision_free_exists is True
    assert res.safety_margin_satisfied is False  # reported honestly!
    assert res.v_mps >= 0.0 or res.reverse_active is True
    assert res.predicted_min_clearance_m is not None and res.predicted_min_clearance_m > 0.0


# ---------------------------------------------------------------------------
# D. Deadlock detection
# ---------------------------------------------------------------------------
def test_temporary_safe_wait_does_not_trigger_deadlock_while_persistent_fallback_does() -> None:
    planner = DWAPlanner(DEFAULT_CONFIG.world, DEFAULT_CONFIG.robot, DEFAULT_CONFIG.dwa)
    planner.set_path([(6.0, 7.5), (18.0, 7.5)])
    walls = [
        StaticObstacle(9.0, 0.0, 2.0, 6.2),
        StaticObstacle(9.0, 8.8, 2.0, 6.2),
    ]
    obs_wait = DynamicObstacle(
        "OBS-WAIT",
        10.5,
        7.5,
        -0.5,
        0.0,
        0.40,
        MovementBehavior.CORRIDOR,
        waypoint_a_m=(9.2, 7.5),
        waypoint_b_m=(14.0, 7.5),
        reverse_at_bounds=True,
    )
    for _ in range(60):  # 3.0 s > fallback_replan_timeout_s
        r = planner.compute_command(RobotState(6.0, 7.5, 0.0), 0.05, walls, [obs_wait])
        assert r.blocked is False
        assert r.deadlock_state == DeadlockState.TEMPORARY_SAFE_WAIT.value

    # Now enclose the robot in a dead-end box so persistent fallback triggers deadlock
    planner2 = DWAPlanner(DEFAULT_CONFIG.world, DEFAULT_CONFIG.robot, DEFAULT_CONFIG.dwa)
    planner2.set_path([(6.0, 7.5), (18.0, 7.5)])
    plug = [
        StaticObstacle(6.48, 6.0, 1.0, 3.0),
        StaticObstacle(4.52, 6.0, 1.0, 3.0),
        StaticObstacle(5.5, 6.0, 1.0, 1.05),
        StaticObstacle(5.5, 7.95, 1.0, 1.05),
    ]
    blocked_seen = False
    for _ in range(60):
        r2 = planner2.compute_command(RobotState(6.0, 7.5, 0.0), 0.05, plug, [])
        if r2.blocked:
            blocked_seen = True
            break
    assert blocked_seen is True


# ---------------------------------------------------------------------------
# E. Integration scenarios & determinism
# ---------------------------------------------------------------------------
@pytest.mark.parametrize(
    "scenario_name,require_zero_margin_viol",
    [
        ("NARROW-GATE", True),
        ("SUDDEN-STOP-FINITE-DECEL", False),
        ("HEAD-ON", False),
        ("CROSSING", False),
        ("CORRIDOR", False),
        ("MULTIPLE-DYNAMIC-OBSTACLES", False),
        ("STOP-AND-GO", False),
        ("SUDDEN-STOP", False),
    ],
)
def test_phase62_integration_scenarios_zero_collisions_and_determinism(
    scenario_name: str, require_zero_margin_viol: bool
) -> None:
    r1 = run_single_experiment(scenario_name, CONTROLLER_DWA)
    r2 = run_single_experiment(scenario_name, CONTROLLER_DWA)

    assert r1.physical_collision_episode_count == 0
    assert r1.goal_reached is True
    if require_zero_margin_viol:
        assert r1.safety_margin_violation_step_count == 0

    # Deterministic repeatability of physical metrics
    assert r1.completion_time_s == r2.completion_time_s
    assert r1.min_instantaneous_clearance_m == pytest.approx(r2.min_instantaneous_clearance_m)
    assert r1.min_swept_clearance_m == pytest.approx(r2.min_swept_clearance_m)
    assert r1.path_length_m == pytest.approx(r2.path_length_m)
