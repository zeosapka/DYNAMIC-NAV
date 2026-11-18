# Phase 6.5.2 — Causal Observation Pipeline Report

## 1. Architecture & Isolation Summary

Implemented a causal sensor observation, state estimation, and prediction pipeline that strictly isolates A* replanning and DWA from ground-truth simulation internals and hidden scenario schedules:

- **`config.py`**: Added `ObservationConfig` (`observation_latency_s=0.0`, `max_observation_age_s=0.50`, `position_uncertainty_m=0.0`, `velocity_uncertainty_mps=0.0`, `initial_velocity_uncertainty_mps=0.25`, `uncertainty_growth_rate_mps=0.02`, `velocity_ema_alpha=1.0`) and `ProjectConfig.observation`.
- **`algorithms/observation.py`**: Added `ObservedRobotState`, `ObservedObstacle` (with `predict_causal_positions()` and `predict_causal_geometry_at()`), `ObservationFrame`, `ObstacleStateEstimator`, and `ObservationPipeline`.
- **`algorithms/dwa.py`**:
  - Removed all `DynamicObstacle.predict_positions()` calls from `DWAPlanner`, `evaluate_trajectory_with_braking()`, `evaluate_waiting_zone_safety()`, and `plan_lateral_holding_maneuver()`.
  - Updated `DWAPlanner.__init__()`, `DWAPlanner.compute_command()`, `select_safe_holding_pose()`, and `replan_global_path_around_obstacles()` to accept and consume `ObservationFrame` (falling back to causal constant-velocity extrapolation when `observation_frame is None`).
- **`algorithms/__init__.py`**: Exported `ObservationFrame`, `ObservationPipeline`, `ObservedObstacle`, `ObservedRobotState`, and `ObstacleStateEstimator`.
- **Call Sites**: Wired `ObservationPipeline` into `analysis/benchmark.py` (`run_single_experiment`), `analysis/diagnostics.py` (`run_diagnostic_trace`), and `interface/app.py` (`DynamicNavApp`) while keeping ground-truth physics and clearance metrics unmodified.

## 2. Causality & Isolation Verification

All **11/11** tests in `tests/test_phase652_causal_observation.py` passed (`0.20 s`, exported to `analysis/results/phase652_causality_tests.json`):

- `test_1_no_future_schedule_access_during_dwa`: Monkeypatching `DynamicObstacle.predict_positions()` to raise confirms DWA + `ObservationPipeline` never calls it.
- `test_2_identical_observed_history_identical_control`: Two obstacles with identical observed history up to `t` but different hidden future schedules produce identical DWA commands and trajectories.
- `test_3_new_obstacle_appearance_starts_with_zero_velocity`: Newly observed obstacles start with `vx_mps=0.0`, `vy_mps=0.0`, `velocity_valid=False`, and `initial_velocity_uncertainty_mps`.
- `test_4_causal_velocity_estimation_from_two_observations`: Two timestamped observations yield causal finite-difference velocity.
- `test_5_observation_latency_propagation`: Latency buffers deliver delayed observations and propagate predictions from `obs.timestamp_s` to `current_time_s + horizon`.
- `test_6_stale_observation_rejection_and_marking`: Out-of-order, negative, NaN, and future timestamps are rejected; disappeared obstacles exceeding `max_observation_age_s` are marked `is_valid=False` and listed in `stale_obstacle_ids`.
- `test_7_uncertainty_growth_with_age_and_horizon`: Uncertainty pad grows monotonically with observation age and prediction horizon.
- `test_8_stop_and_go_schedule_isolation`: Stopped `STOP_AND_GO` obstacles are predicted stationary until motion is observed.
- `test_9_replan_uses_only_observed_obstacles`: `replan_global_path_around_obstacles(..., observation_frame=frame)` uses only observed obstacles.
- `test_10_source_code_isolation_audit`: Zero occurrences of `predict_positions`, `_phase_time_s`, `move_duration_s`, `stop_duration_s`, `waypoint_a_m`, `waypoint_b_m`, or `reverse_at_bounds` in `algorithms/dwa.py`, `algorithms/astar.py`, and `algorithms/observation.py`.
- `test_11_end_to_end_hidden_reversal_schedule`: End-to-end reversal isolation before bounce and causal velocity tracking after bounce.

## 3. Eight-Scenario Benchmark & Determinism

Exports:
- `analysis/results/benchmark_phase652_raw.json`
- `analysis/results/benchmark_phase652_raw.csv`
- `analysis/results/benchmark_phase652_determinism.json`

| Scenario | Termination | Collisions | Margin Viol Ep | Min Inst Clearance (m) | Min Swept Clearance (m) | Avg DWA (ms) | P95 DWA (ms) | Max DWA (ms) |
| :--- | :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| `HEAD-ON` | `GOAL_REACHED` | 0 | 0 | 0.258924 | 0.257926 | 5.294 | 8.397 | 10.972 |
| `CROSSING` | `GOAL_REACHED` | 0 | 0 | 0.153208 | 0.152699 | 14.118 | 21.193 | 31.809 |
| `CORRIDOR` | `COLLISION_ABORT` | 1 | 1 | -0.036470 | -0.036470 | 9.185 | 20.154 | 28.947 |
| `NARROW-GATE` | `GOAL_REACHED` | 0 | 0 | 0.330847 | 0.330631 | 12.077 | 18.824 | 30.149 |
| `MULTIPLE-DYNAMIC-OBSTACLES` | `GOAL_REACHED` | 0 | 0 | 0.153590 | 0.152792 | 18.767 | 28.919 | 38.576 |
| `STOP-AND-GO` | `GOAL_REACHED` | 0 | 0 | 0.153300 | 0.152024 | 10.242 | 12.547 | 17.124 |
| `SUDDEN-STOP` | `GOAL_REACHED` | 0 | 0 | 0.154380 | 0.153666 | 12.683 | 20.705 | 27.038 |
| `SUDDEN-STOP-FINITE-DECEL` | `GOAL_REACHED` | 0 | 0 | 0.154328 | 0.153625 | 29.556 | 44.515 | 67.935 |

- **3-Run Determinism**: `True` across all 8 scenarios.
- **CORRIDOR Finding**: Previously, `DynamicObstacle.predict_positions()` leaked `waypoint_b_m` and `reverse_at_bounds`, allowing DWA to foresee the obstacle's instantaneous bounce inside the narrow corridor while the robot was following behind it at `1.0 m/s` (`0.6777 m` clearance at `t=17.20 s`). Under strict causal observation, at `t=17.25 s` the obstacle reverses instantaneously from `+1.0 m/s` to `-1.0 m/s`; the robot brakes to `0.0 m/s` by `t=17.55 s`, but the oncoming obstacle contacts the stopped robot at `t=17.70 s` (`-0.036470 m`). No parameters were tuned to mask this causal-observability limitation.

## 4. Per-Component Timing Breakdown

Measured in a dedicated pass across all 8 scenarios (`dt=0.05 s`):

| Scenario | Steps | Observation Avg (ms) | Estimation Avg (ms) | DWA Compute Avg (ms) |
| :--- | :---: | :---: | :---: | :---: |
| `HEAD-ON` | 317 | 0.0125 | 0.0000 | 4.875 |
| `CROSSING` | 276 | 0.0374 | 0.0000 | 11.168 |
| `CORRIDOR` | 354 | 0.0357 | 0.0000 | 14.514 |
| `NARROW-GATE` | 311 | 0.0526 | 0.0000 | 15.931 |
| `MULTIPLE-DYNAMIC-OBSTACLES` | 352 | 0.1542 | 0.0000 | 30.268 |
| `STOP-AND-GO` | 220 | 0.0958 | 0.0000 | 18.800 |
| `SUDDEN-STOP` | 296 | 0.0503 | 0.0000 | 20.006 |
| `SUDDEN-STOP-FINITE-DECEL` | 334 | 0.0885 | 0.0000 | 43.581 |

## 5. Full Test Suite (`pytest -q`)

- **151 passed, 2 failed** (`77.46 s`):
  - `FAILED tests/test_dwa.py::test_dwa_corridor_scenario_deterministic_audit` (CORRIDOR collision under causal observation)
  - `FAILED tests/test_phase62_safe_waiting_reverse.py::test_phase62_integration_scenarios_zero_collisions_and_determinism[CORRIDOR-False]` (CORRIDOR collision under causal observation)
