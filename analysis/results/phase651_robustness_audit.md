# Phase 6.5.1 — Robustness Test Design & Baseline Audit

**Mode**: READ-ONLY INVESTIGATION. No source code, configuration, thresholds, scenario definitions, historical benchmarks, or existing tests were modified.

Deliverables:
- [phase651_baseline_integrity.json](file:///c:/Users/Legolas/Desktop/DYNAMIC-NAV/analysis/results/phase651_baseline_integrity.json)
- [phase651_robustness_test_matrix.json](file:///c:/Users/Legolas/Desktop/DYNAMIC-NAV/analysis/results/phase651_robustness_test_matrix.json)

---

## 1. Baseline Integrity Audit

- **Git status**: `git` CLI is not available on `PATH` in this workspace; source and dataset integrity were verified directly via file inspection.
- **Configuration (`config.py`)**:
  - `WorldConfig`: `20.0 m × 15.0 m`, `grid_resolution_m = 0.2 m`, `5` default static obstacles.
  - `RobotConfig`: `radius_m = 0.35 m`, `max_linear_speed_mps = 3.0 m/s`, `max_angular_speed_radps = 2.8 rad/s`.
  - `DWAConfig`: `min_linear_speed_mps = 0.0`, `min_reverse_speed_mps = -0.6 m/s`, `max_linear_accel_mps2 = 5.0 m/s²`, `max_angular_accel_radps2 = 16.0 rad/s²`, `max_brake_decel_mps2 = 5.0 m/s²`, `reaction_time_s = 0.1 s`, `linear_samples = 7`, `angular_samples = 17`, `predict_time_s = 1.2 s`, `predict_dt_s = 0.1 s`, `safety_margin_m = 0.15 m`, `waiting_horizon_s = 2.5 s`, `waiting_dt_s = 0.1 s`, `waiting_uncertainty_pad_m = 0.05 m`, `waiting_lateral_offset_m = 1.4 m`.
  - `ExperimentConfig`: `dt_s = 0.05 s`, `max_sim_time_s = 60.0 s`, `safety_margin_m = 0.15 m`, `stop_speed_threshold_mps = 0.05 m/s`, `goal_tolerance_m = 0.25 m`.
- **Determinism**: Fixed `dt_s = 0.05 s`, deterministic velocity sampling, and zero RNG in A*, DWA, or scenario execution.

---

## 2. Stage 2.4 Timing Discrepancy Resolution (`8,400` vs `8,340` Calls)

- **`benchmark_phase642_stage24_raw.json`**: Contains `16` runs (8 scenarios × `ASTAR_FOLLOWER` + `ASTAR_DWA`), with `2,780` total `ASTAR_DWA` steps (`318 + 277 + 499 + 223 + 311 + 351 + 467 + 334`).
- **`benchmark_phase642_stage24_determinism.json`**: Contains `24` runs (`3` repeated runs × 8 scenarios for `ASTAR_DWA`), totaling **`3 × 2,780 = 8,340` calls** (`avg = 14.27 ms`, `median = 13.13 ms`, `p95 = 28.49 ms`, `p99 = 36.45 ms`, `max = 72.88 ms`, `>50 ms = 15 (0.18%)`).
- **Why `8,400` appeared in an earlier log**: A transient background-task notification from an earlier execution of `task-1621` reported `8,400` calls (`avg = 4.86 ms`) before the final persisted run wrote **`8,340` calls** to `benchmark_phase642_stage24_determinism.json`. The authoritative persisted dataset is **`8,340` calls**.

---

## 3. Three-Level Perturbation Test Matrix (`246` Variants / `738` Runs at 3× Repeat)

1. **LEVEL 1 — Controlled Perturbation (`120` variants)**:
   - **Initial conditions**: robot start `±0.30 m` (`x, y`), heading `±15°` (`±0.26 rad`), goal `±0.50 m`, dynamic obstacle start `±0.40 m`, initial speed `[0.0, 1.0] m/s`.
   - **Dynamic obstacles**: speed scale `[0.85, 1.15]`, stop/move duration `±0.30 s`.
   - **Static geometry**: gate/corridor width `±0.20 m`, static obstacle shifts `±0.25 m`.
   - **Robot dynamics**: `max_linear_speed_mps ∈ {2.2, 3.0}`, `max_brake_decel_mps2 ∈ {4.0, 5.0}`, `min_reverse_speed_mps ∈ {-0.4, -0.6}`.
2. **LEVEL 2 — Strong Perturbation (`96` variants)**:
   - **Initial conditions**: start `±0.80 m`, heading `±45°` (`±0.79 rad`), initial speed `[1.5, 2.5] m/s`.
   - **Dynamic obstacles**: speed scale `[0.60, 1.50]`, crossing angle `±0.45 rad`, same-direction lead speeds `{0.6, 1.0, 1.4} m/s`, reversal phase shift `±1.0 s`.
   - **Static geometry**: corridor widths `{1.30, 1.45, 1.60} m`, asymmetric wall offsets `±0.40 m`, rotated `OrientedRect`s (`0.35, 0.78, -0.52 rad`).
   - **Robot dynamics**: reduced `max_linear_accel_mps2 ∈ {2.5, 3.5}`, `max_brake_decel_mps2 ∈ {2.5, 3.5}`, `max_angular_speed_radps ∈ {1.8, 2.2}`, `max_angular_accel_radps2 ∈ {8.0, 10.0}`.
3. **LEVEL 3 — Unseen Combinations (`30` variants across `6` new composite scenarios)**:
   - `L3-UC-01`: 3-way converging intersection (`Circle` + `OrientedRect` + `Capsule`) + corner walls.
   - `L3-UC-02`: Asymmetric narrow gate + decelerating-then-reversing lead vehicle.
   - `L3-UC-03`: `Polygon` chicane + lane blockage forcing A* global detour.
   - `L3-UC-04`: Slow same-direction lead vehicle + oncoming vehicle.
   - `L3-UC-05`: High initial speed (`2.2 m/s`) crossing `OrientedRect` + pedestrian `Circle`.
   - `L3-UC-06`: Sequential double corridor bottleneck with out-of-phase opposing patrols.

---

## 4. Metrics & Statistical Methodology

- **Metrics**: Preserve exact `RunMetrics` definitions (`goal_reached`, `physical_collision_episode_count`, `physical_collision_step_count`, `safety_margin_violation_episode_count`, `safety_margin_violation_step_count`, `min_instantaneous_clearance_m`, `min_swept_clearance_m`, `min_braking_clearance_m` on fallback/maneuvers, `completion_time_s`, `stop_count`, `replan_attempts`/`replan_successes`, `blocked_step_count`, and uninstrumented DWA `avg/med/p95/p99/max` + `>50 ms %`).
- **Reproducibility**: Every variant records `baseline_scenario_id`, unique `perturbation_id`, full parameter delta, and `seed` (for future stochastic noise runs). Deterministic variants run `3×` to verify identical physical metrics; future stochastic sensor-noise variants will run `>= 10` seeds per variant and report pass rates, Wilson 95% CIs, and worst-case quantiles (never cherry-picking a single run).

---

## 5. Sensor Uncertainty & Latency — Feasibility Audit

1. **Current Ground-Truth Coupling**:
   - `DWAPlanner.compute_command()` currently receives `world.robot.state`, `world.dynamic_obstacles`, and `world.sim_objects` directly (zero measurement noise, zero latency).
   - **Critical Finding**: `SimObject.predict_geometry_at(t)` extrapolates using **only** current observed `(vx, vy, heading, angular_speed)`. By contrast, `DynamicObstacle.predict_positions()` clones internal scenario-schedule fields (`_phase_time_s`, `move_duration_s`, `stop_duration_s`, `waypoint_a_m`, `waypoint_b_m`, `reverse_at_bounds`), meaning `DynamicObstacle` prediction **currently leaks scenario schedule parameters**!
2. **Proposed Sensor Observation Interface (Future Stage)**:
   - Insert an `ObservationPipeline` between `SimulationWorld` and `DWAPlanner` producing `ObservedRobotState` and `ObservedObstacle(id, timestamp_s, pos_xy_m, vel_xy_mps, geometry, pos_cov_m, vel_cov_mps)`.
   - **Latency / Staleness**: Compare `current_sim_time_s - obs.timestamp_s`; propagate stale observations forward by `age_s = t_now - obs.timestamp_s` and flag/reject observations exceeding a staleness threshold (`max_obs_age_s`).
   - **Uncertainty Propagation**: Expand obstacle effective radius / clearance pad by `r_eff(t) = r_obs + z * sigma_pos + z * sigma_vel * t` (bounded inflation over `predict_time_s` and `waiting_horizon_s`) using observed velocity only (eliminating hidden `STOP_AND_GO` / patrol schedule leakage).

---

## 6. Expected Implementation Stages & Risks
- **Stage 6.5.2**: Implement deterministic Perturbation Runner (`LEVEL 1`–`3`, `246` variants) and export baseline vs. perturbed failure map without altering `DWAPlanner`.
- **Stage 6.5.3**: Address any genuine generalization failures exposed by `LEVEL 1`–`3`.
- **Stage 6.5.4**: Introduce `ObservationPipeline` (velocity-only `DynamicObstacle` estimation, sensor noise, observation latency) and `10`-seed Monte Carlo validation.
- **Key Risks**: Removing `DynamicObstacle` schedule leakage in `predict_positions()` will change how `SUDDEN-STOP` / `STOP-AND-GO` anticipate reversals (requiring uncertainty-padded reactive margins rather than oracle timing), and tighter corridors in `LEVEL 2` (`1.30 m` width vs `0.70 m` robot diameter + `0.30 m` margins = `1.00 m`) may expose narrow-passage deadlocks.
