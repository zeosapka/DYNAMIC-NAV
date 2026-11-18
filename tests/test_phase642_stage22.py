"""Phase 6.4 Stage 2.2 regression tests for conditional reverse recovery and collision classification integrity."""

from __future__ import annotations

from config import DEFAULT_CONFIG
from algorithms.dwa import (
    DWAPlanner,
    WaitingZoneState,
    classify_trajectory_safety,
    evaluate_trajectory_with_braking,
    predict_trajectory,
)
from simulation.dynamic_obstacles import DynamicObstacle
from simulation.entities import RobotState, StaticObstacle
from simulation.objects import ObjectCategory, create_default_object


def test_gate_clear_hold_unsafe_prefers_safe_reverse_over_margin_violating_forward() -> None:
    """When GATE_CLEAR_HOLD_UNSAFE and a >= 0.15 m reverse exists while forward violates margin, choose reverse."""
    dwa = DWAPlanner(DEFAULT_CONFIG.world, DEFAULT_CONFIG.robot, DEFAULT_CONFIG.dwa)
    dwa.set_path([(5.0, 7.5), (15.0, 7.5)])
    # Robot stopped near a wall/obstacle ahead that will approach slowly enough that
    # reversing stays >= 0.15 m over the waiting horizon, while forward violates 0.15 m.
    state = RobotState(x_m=5.0, y_m=7.5, theta_rad=0.0, v_mps=0.0, omega_radps=0.0)
    # Obstacle at x=5.96 moving slowly left at -0.25 m/s:
    # t=0 clearance = 5.96 - 5.0 - 0.35 - 0.45 = 0.16 m (>= 0.15 m)
    # Holding still becomes unsafe within 0.1 s (GATE_CLEAR_HOLD_UNSAFE), forward violates 0.15 m,
    # while reversing at -0.50 m/s stays >= 0.15 m over both 1.0 s + braking and 2.5 s waiting horizon.
    obs = DynamicObstacle(
        obstacle_id="DYN-TEST-REV",
        x_m=5.98,
        y_m=7.5,
        vx_mps=-0.06,
        vy_mps=0.0,
        radius_m=0.45,
        behavior="CONSTANT_VELOCITY",
    )
    res = dwa.compute_command(state, 0.05, [], [obs])
    assert res.waiting_state == WaitingZoneState.GATE_CLEAR_HOLD_UNSAFE.value
    assert res.reverse_active is True
    assert res.v_mps < 0.0
    assert res.fallback_priority == 3
    assert res.fallback_reason == "REVERSE_RECOVERY"
    assert res.predicted_min_clearance_m is not None
    assert res.predicted_min_clearance_m >= DEFAULT_CONFIG.dwa.safety_margin_m


def test_gate_clear_hold_unsafe_retains_fallback_when_no_safe_reverse_exists() -> None:
    """When GATE_CLEAR_HOLD_UNSAFE and no safe reverse exists (rear blocked), retain Priority 2 fallback."""
    dwa = DWAPlanner(DEFAULT_CONFIG.world, DEFAULT_CONFIG.robot, DEFAULT_CONFIG.dwa)
    dwa.set_path([(5.0, 7.5), (15.0, 7.5)])
    state = RobotState(x_m=5.0, y_m=7.5, theta_rad=0.0, v_mps=0.0, omega_radps=0.0)
    # Rear wall blocks reverse, front obstacle approaches slowly
    rear_wall = StaticObstacle(x_m=4.0, y_m=6.0, width_m=0.60, height_m=3.0)
    obs = DynamicObstacle(
        obstacle_id="DYN-TEST-NOREV",
        x_m=5.96,
        y_m=7.5,
        vx_mps=-0.25,
        vy_mps=0.0,
        radius_m=0.45,
        behavior="CONSTANT_VELOCITY",
    )
    res = dwa.compute_command(state, 0.05, [rear_wall], [obs])
    assert res.waiting_state == WaitingZoneState.GATE_CLEAR_HOLD_UNSAFE.value
    assert res.reverse_active is False
    assert res.v_mps >= 0.0


def test_early_margin_violation_followed_by_collision_is_classified_colliding() -> None:
    """An early < 0.15 m margin violation must not mask a later <= 0.0 m physical collision."""
    state = RobotState(x_m=5.0, y_m=7.5, theta_rad=0.0, v_mps=1.0, omega_radps=0.0)
    # Wall at x=5.55 -> clearance at t=0 is 0.20 m, at t=0.10 s (v=1.0) is 0.10 m (< 0.15),
    # and at t=0.30 s is -0.10 m (physical collision).
    wall = StaticObstacle(x_m=5.55, y_m=4.0, width_m=2.0, height_m=7.0)
    traj = predict_trajectory(state, 1.0, 0.0, 1.0, 0.10, first_step_dt_s=0.05)

    is_safe, min_c, _, _ = evaluate_trajectory_with_braking(
        traj,
        0.10,
        DEFAULT_CONFIG.robot.radius_m,
        DEFAULT_CONFIG.dwa.safety_margin_m,
        DEFAULT_CONFIG.world,
        [wall],
        [],
        v_mps=1.0,
        max_brake_decel_mps2=DEFAULT_CONFIG.dwa.max_brake_decel_mps2,
        reaction_time_s=DEFAULT_CONFIG.dwa.reaction_time_s,
    )
    assert is_safe is False
    assert min_c <= 0.0

    cls = classify_trajectory_safety(
        traj,
        0.10,
        DEFAULT_CONFIG.robot.radius_m,
        DEFAULT_CONFIG.dwa.safety_margin_m,
        DEFAULT_CONFIG.world,
        [wall],
        [],
        v_mps=1.0,
        max_brake_decel_mps2=DEFAULT_CONFIG.dwa.max_brake_decel_mps2,
        reaction_time_s=DEFAULT_CONFIG.dwa.reaction_time_s,
    )
    assert cls.safety_margin_compliant is False
    assert cls.collision_free is False
    assert cls.min_predicted_clearance_m <= 0.0


def test_margin_violation_without_collision_preserves_distinction() -> None:
    """A trajectory dipping to 0.10 m (< 0.15 m) without hitting <= 0.0 m stays collision_free=True."""
    state = RobotState(x_m=5.0, y_m=7.5, theta_rad=0.0, v_mps=0.0, omega_radps=0.0)
    # Wall at y=7.95 -> clearance at y=7.5 is 7.95 - 7.5 - 0.35 = 0.10 m (constant along x)
    wall = StaticObstacle(x_m=4.0, y_m=7.95, width_m=4.0, height_m=1.0)
    traj = predict_trajectory(state, 0.5, 0.0, 1.0, 0.10, first_step_dt_s=0.05)

    cls = classify_trajectory_safety(
        traj,
        0.10,
        DEFAULT_CONFIG.robot.radius_m,
        DEFAULT_CONFIG.dwa.safety_margin_m,
        DEFAULT_CONFIG.world,
        [wall],
        [],
        v_mps=0.5,
        max_brake_decel_mps2=DEFAULT_CONFIG.dwa.max_brake_decel_mps2,
        reaction_time_s=DEFAULT_CONFIG.dwa.reaction_time_s,
    )
    assert cls.safety_margin_compliant is False
    assert cls.collision_free is True
    assert 0.0 < cls.min_predicted_clearance_m < DEFAULT_CONFIG.dwa.safety_margin_m


def test_reverse_recovery_respects_acceleration_and_braking_constraints() -> None:
    """Reverse recovery velocity must respect max_linear_accel * dt and min_reverse_speed_mps."""
    dwa = DWAPlanner(DEFAULT_CONFIG.world, DEFAULT_CONFIG.robot, DEFAULT_CONFIG.dwa)
    dwa.set_path([(5.0, 7.5), (15.0, 7.5)])
    state = RobotState(x_m=5.0, y_m=7.5, theta_rad=0.0, v_mps=0.0, omega_radps=0.0)
    obs = DynamicObstacle(
        obstacle_id="DYN-ACCEL-REV",
        x_m=5.98,
        y_m=7.5,
        vx_mps=-0.06,
        vy_mps=0.0,
        radius_m=0.45,
        behavior="CONSTANT_VELOCITY",
    )
    dt_s = 0.05
    res = dwa.compute_command(state, dt_s, [], [obs])
    assert res.reverse_active is True
    assert res.v_mps >= -DEFAULT_CONFIG.dwa.max_linear_accel_mps2 * dt_s - 1e-9
    assert res.v_mps >= DEFAULT_CONFIG.dwa.min_reverse_speed_mps - 1e-9
    assert res.min_braking_clearance_m is not None
    assert res.min_braking_clearance_m >= DEFAULT_CONFIG.dwa.safety_margin_m


def test_reverse_recovery_respects_static_and_simobject_geometry() -> None:
    """Reverse recovery must reject reversing into a SimObject behind the robot."""
    dwa = DWAPlanner(DEFAULT_CONFIG.world, DEFAULT_CONFIG.robot, DEFAULT_CONFIG.dwa)
    dwa.set_path([(5.0, 7.5), (15.0, 7.5)])
    state = RobotState(x_m=5.0, y_m=7.5, theta_rad=0.0, v_mps=0.0, omega_radps=0.0)
    front_obs = DynamicObstacle(
        obstacle_id="DYN-FRONT",
        x_m=5.98,
        y_m=7.5,
        vx_mps=-0.06,
        vy_mps=0.0,
        radius_m=0.45,
        behavior="CONSTANT_VELOCITY",
    )
    rear_obj = create_default_object("REAR-CAR", ObjectCategory.PASSENGER_CAR, 2.0, 7.5)
    rear_obj.motion.speed_mps = 0.0
    res = dwa.compute_command(state, 0.05, [], [front_obs], sim_objects=[rear_obj])
    assert not res.reverse_active or (res.predicted_min_clearance_m or 0.0) >= DEFAULT_CONFIG.dwa.safety_margin_m
