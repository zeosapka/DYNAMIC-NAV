"""Unit tests for Phase 5 benchmark metrics, determinism, and CSV/JSON export."""

import csv
import json
from pathlib import Path
import pytest

from analysis.benchmark import (
    CONTROLLER_DWA,
    CONTROLLER_FOLLOWER,
    export_results_csv,
    export_results_json,
    run_single_experiment,
)
from analysis.metrics import RunMetricsTracker, compute_swept_clearance_m
from config import DEFAULT_CONFIG
from simulation.dynamic_obstacles import DynamicObstacle, MovementBehavior
from simulation.scenarios import ScenarioManager


def test_collision_vs_safety_margin_strict_separation() -> None:
    tracker = RunMetricsTracker("TEST", CONTROLLER_DWA, dt_s=0.05, safety_margin_m=0.15, is_dwa=True)
    # Step 1: safe (>= 0.15 m)
    tracker.record_step(0.05, (0.0, 0.0), (0.1, 0.0), 1.0, 0.20, 0.20)
    # Step 2 & 3: safety-margin violation (0.0 < c < 0.15 m)
    tracker.record_step(0.10, (0.1, 0.0), (0.2, 0.0), 1.0, 0.10, 0.10)
    tracker.record_step(0.15, (0.2, 0.0), (0.3, 0.0), 1.0, 0.05, 0.05)
    # Step 4: physical collision (c <= 0.0 m) — must NOT increment margin violations
    tracker.record_step(0.20, (0.3, 0.0), (0.4, 0.0), 1.0, -0.02, -0.02)

    m = tracker.finalize(False, "COLLISION_ABORT", 0.20, 1.0)
    assert m.physical_collision_episode_count == 1
    assert m.physical_collision_step_count == 1
    assert m.first_collision_time_s == pytest.approx(0.20)
    assert m.safety_margin_violation_episode_count == 1
    assert m.safety_margin_violation_step_count == 2


def test_swept_clearance_catches_midstep_tunneling() -> None:
    # Robot moves (0, 0) -> (2, 0), obstacle moves (2, 0) -> (0, 0):
    # endpoints are 2.0 m apart, but mid-step they pass directly through (1, 0)!
    obs = DynamicObstacle("TUN", 0.0, 0.0, -1.0, 0.0, 0.40, MovementBehavior.HEAD_ON)
    swept_c = compute_swept_clearance_m(
        (0.0, 0.0),
        (2.0, 0.0),
        0.35,
        DEFAULT_CONFIG.world,
        [],
        [(obs, (2.0, 0.0), (0.0, 0.0))],
    )
    assert swept_c == pytest.approx(-0.75)


def test_stop_count_excludes_initial_stationary_and_goal_stop() -> None:
    tracker = RunMetricsTracker("TEST", CONTROLLER_FOLLOWER, dt_s=0.05, stop_speed_threshold_mps=0.05, is_dwa=False)
    # Initial stationary frames (must NOT count as stops)
    tracker.record_step(0.05, (0.0, 0.0), (0.0, 0.0), 0.0, 1.0, 1.0)
    tracker.record_step(0.10, (0.0, 0.0), (0.0, 0.0), 0.02, 1.0, 1.0)
    # Start moving
    tracker.record_step(0.15, (0.0, 0.0), (0.1, 0.0), 1.0, 1.0, 1.0)
    # Mid-run stop (counts as 1)
    tracker.record_step(0.20, (0.1, 0.0), (0.1, 0.0), 0.0, 1.0, 1.0)
    tracker.record_step(0.25, (0.1, 0.0), (0.1, 0.0), 0.0, 1.0, 1.0)
    # Resume and arrive at goal while stopping (final goal stop excluded from stop_count)
    tracker.record_step(0.30, (0.1, 0.0), (0.2, 0.0), 1.0, 1.0, 1.0)
    tracker.record_step(0.35, (0.2, 0.0), (0.2, 0.0), 0.0, 1.0, 1.0, is_goal_step=True)

    m = tracker.finalize(True, "GOAL_REACHED", 0.35, 0.0)
    assert m.stop_count == 1
    assert m.goal_final_stop is True
    assert m.avg_dwa_time_ms is None
    assert m.max_dwa_time_ms is None
    assert m.p95_dwa_time_ms is None


def test_blocked_and_replan_counters() -> None:
    tracker = RunMetricsTracker("TEST", CONTROLLER_DWA, dt_s=0.05, is_dwa=True)
    tracker.record_dwa_call(2.0, True)
    tracker.record_dwa_call(2.5, True)
    tracker.record_replan(False)
    tracker.record_dwa_call(1.8, False)
    tracker.record_dwa_call(2.1, True)
    tracker.record_replan(True)

    m = tracker.finalize(True, "GOAL_REACHED", 0.20, 0.5)
    assert m.blocked_step_count == 3
    assert m.blocked_episode_count == 2
    assert m.replan_attempts == 2
    assert m.replan_successes == 1
    assert m.no_path_replan_count == 1
    assert m.avg_dwa_time_ms is not None and m.avg_dwa_time_ms > 0.0


def test_all_five_required_benchmark_scenarios_exist() -> None:
    names = set(ScenarioManager.list_scenarios())
    for req in DEFAULT_CONFIG.experiment.benchmark_scenarios:
        assert req in names


def test_deterministic_physical_metrics_and_exports(tmp_path: Path) -> None:
    r1 = run_single_experiment("HEAD-ON", CONTROLLER_DWA)
    r2 = run_single_experiment("HEAD-ON", CONTROLLER_DWA)
    rf = run_single_experiment("HEAD-ON", CONTROLLER_FOLLOWER)

    # Physical metrics must be identical across repeated deterministic runs
    assert r1.goal_reached == r2.goal_reached
    assert r1.termination_reason == r2.termination_reason
    assert r1.completion_time_s == r2.completion_time_s
    assert r1.physical_collision_episode_count == r2.physical_collision_episode_count
    assert r1.min_instantaneous_clearance_m == pytest.approx(r2.min_instantaneous_clearance_m)
    assert r1.min_swept_clearance_m == pytest.approx(r2.min_swept_clearance_m)
    assert r1.path_length_m == pytest.approx(r2.path_length_m)

    assert rf.avg_dwa_time_ms is None
    assert rf.max_dwa_time_ms is None

    json_path = export_results_json([r1, rf], tmp_path / "raw.json")
    csv_path = export_results_csv([r1, rf], tmp_path / "raw.csv")
    data = json.loads(json_path.read_text(encoding="utf-8"))
    assert len(data["runs"]) == 2
    assert data["runs"][1]["avg_dwa_time_ms"] is None

    with csv_path.open("r", encoding="utf-8") as fh:
        rows = list(csv.DictReader(fh))
    assert len(rows) == 2
    assert rows[1]["avg_dwa_time_ms"] == "N/A"
