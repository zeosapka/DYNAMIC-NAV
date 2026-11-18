"""Phase 6.5.5.1 — STOP_AND_GO Timer IEEE-754 Fix & Regression Tests."""

from __future__ import annotations

import math

import pytest

from simulation.dynamic_obstacles import DynamicObstacle, MovementBehavior


def _make_stop_and_go(
    move_s: float = 2.5,
    stop_s: float = 3.2,
    phase_s: float = 0.0,
    vx: float = 1.2,
) -> DynamicObstacle:
    obs = DynamicObstacle(
        obstacle_id="SG-TIMER",
        x_m=5.0,
        y_m=7.5,
        vx_mps=vx,
        vy_mps=0.0,
        radius_m=0.45,
        behavior=MovementBehavior.STOP_AND_GO,
        move_duration_s=move_s,
        stop_duration_s=stop_s,
        waypoint_a_m=(2.0, 7.5),
        waypoint_b_m=(18.0, 7.5),
        reverse_at_bounds=True,
    )
    obs._phase_time_s = phase_s
    return obs


def test_1_stop_duration_3_2_finite_termination_and_analytical_match() -> None:
    """Previously hung in IEEE-754 loop for stop_duration_s=3.2, move_duration_s=2.5."""
    obs = _make_stop_and_go(move_s=2.5, stop_s=3.2)
    dt = 0.05
    total_moving_dt = 0.0
    for _ in range(400):  # 20.0 s
        total_moving_dt += obs._advance_stop_and_go_timer(dt)
    # 20.0 s = 3 full cycles (3 * 5.7 = 17.1 s -> 7.5 s moving) + 2.9 s (2.5 s moving + 0.4 s stopped)
    assert total_moving_dt == pytest.approx(10.0, abs=1e-9)


def test_2_very_small_remaining_intervals() -> None:
    """Tiny dt steps near machine epsilon must terminate and preserve total moving time."""
    obs = _make_stop_and_go(move_s=2.5, stop_s=3.2, phase_s=2.499999999999999)
    for dt in (1e-16, 1e-15, 1e-13, 1e-10):
        m = obs._advance_stop_and_go_timer(dt)
        assert 0.0 <= m <= dt + 1e-18


def test_3_floating_point_representational_boundaries() -> None:
    """Boundary-adjacent phase values (ulp away from move_duration_s or cycle_s) must not stall."""
    for phase in (2.5 - 1e-15, 2.5, 2.5 + 1e-15, 5.7 - 1e-15, 5.7):
        obs = _make_stop_and_go(move_s=2.5, stop_s=3.2, phase_s=phase)
        m = obs._advance_stop_and_go_timer(0.05)
        assert math.isfinite(m)
        assert 0.0 <= m <= 0.05 + 1e-12


def test_4_multiple_transitions_in_single_step() -> None:
    """A large dt spanning multiple full move/stop cycles integrates exact moving time."""
    obs = _make_stop_and_go(move_s=2.0, stop_s=1.5, phase_s=0.0)
    # 10.5 s = exactly 3 full cycles -> 3 * 2.0 = 6.0 s moving
    m = obs._advance_stop_and_go_timer(10.5)
    assert m == pytest.approx(6.0, abs=1e-9)

    obs2 = _make_stop_and_go(move_s=2.0, stop_s=1.5, phase_s=1.0)
    # 4.5 s = 1 full cycle (2.0 s) + 1.0 s residual from phase=1.0 -> [1.0, 2.0] moving (1.0 s) => 3.0 s
    m2 = obs2._advance_stop_and_go_timer(4.5)
    assert m2 == pytest.approx(3.0, abs=1e-9)


def test_5_exact_transition_boundaries() -> None:
    """Exact move->stop and stop->move boundary transitions set _stopped_now accurately."""
    obs = _make_stop_and_go(move_s=2.5, stop_s=3.2, phase_s=0.0)
    m1 = obs._advance_stop_and_go_timer(2.5)
    assert m1 == pytest.approx(2.5, abs=1e-12)
    assert obs._stopped_now is True

    m2 = obs._advance_stop_and_go_timer(3.2)
    assert m2 == pytest.approx(0.0, abs=1e-12)
    assert obs._stopped_now is False


def test_6_large_simulation_timestamps() -> None:
    """Large _phase_time_s values must remain accurate and never lose progress."""
    for big_t in (1e6, 1e9):
        obs = _make_stop_and_go(move_s=2.5, stop_s=3.2, phase_s=big_t)
        m = obs._advance_stop_and_go_timer(5.7)
        assert m == pytest.approx(2.5, abs=1e-6)


def test_7_zero_and_invalid_dt_handling() -> None:
    """Zero, negative, NaN, and Inf dt_s must return 0.0 without corrupting state."""
    for bad_dt in (0.0, -0.05, math.nan, math.inf, -math.inf):
        obs = _make_stop_and_go(move_s=2.5, stop_s=3.2, phase_s=1.0)
        m = obs._advance_stop_and_go_timer(bad_dt)
        assert m == 0.0
        assert obs._phase_time_s == 1.0


def test_8_deterministic_positions_and_velocities() -> None:
    """Repeated step() runs with stop_duration_s=3.2 produce identical positions and velocities."""
    def _run() -> list[tuple[float, float, float]]:
        obs = _make_stop_and_go(move_s=2.5, stop_s=3.2)
        traj = []
        for _ in range(300):
            obs.step(0.05)
            traj.append((obs.x_m, obs.y_m, obs.vx_mps))
        return traj

    t1 = _run()
    t2 = _run()
    assert t1 == t2
