# Phase 6.5.5 — Level 1 Controlled Robustness Testing Report

**Baseline**: Phase 6.5.4 causal navigation architecture (`ObservationPipeline` + `DWAPlanner` + `CorridorPassingManager`), `dt_s = 0.05 s`, `safety_margin_m = 0.15 m`.
**Modification check**: Zero repository source, configuration, test, or historical benchmark files were modified.

> [!NOTE]
> During Level 1 execution, `SUDDEN-STOP__L1-DO-02__01` (`stop_duration_s = 3.2 s`) exposed a pre-existing IEEE-754 floating-point infinite loop in `simulation/dynamic_obstacles.py:179` (`_advance_stop_and_go_timer`, where `t + chunk == t` when `chunk ~ 1e-16`). To preserve repository integrity without modifying source files, an in-memory boundary-snap wrapper was applied inside the scratch runner only.

## 1. Executive Summary (120 Variants × 3 Repetitions = 360 Runs)

- **Total experiments completed**: `360 / 360` (`120` Level 1 variants × `3` repetitions, zero skipped)
- **Goal completion rate**: **`85.0%`** (`102 / 120` variants)
- **Physical collision rate**: **`15.0%`** (`18 / 120` variants — all `COLLISION_ABORT`)
- **Safety margin violation rate**: **`20.83%`** (`25 / 120` variants — `18` collisions + `7` margin-violation-only `GOAL_REACHED` runs)
- **Three-run physical determinism**: **`100.0%`** (`120 / 120` variants identical across all 3 repetitions)
- **Worst-performing perturbation**: `NARROW-GATE` (`57.1%` collision rate / `8` of `14` variants) and `SUDDEN-STOP__L1-DO-03__01` (minimum swept clearance `-0.062278 m`)
- **Most frequent failure mechanism**: `NARROW_GATE_TIMING_AND_HOLDING_MARGIN` (`8` collision variants) and `SUDDEN_STOP_CAUSAL_REVERSAL_TRAP` (`6` collision variants)
- **Computational performance (all 360 runs, `107,970` DWA calls)**:
  - Mean median DWA time: `12.39 ms`
  - Mean P99 DWA time: `28.81 ms`
  - Calls `> 50 ms`: `1,863 / 107,970` (**`1.73%`**)

## 2. Per-Scenario Baseline Comparison (vs Phase 6.5.4)

| Scenario | Variants | Goal Rate | Collision Rate | Margin Viol Rate | Worst Swept Clear (m) | Phase 6.5.4 Swept Clear (m) | Avg Path Len (m) | Phase 6.5.4 Path Len (m) | Avg Sim Time (s) | Phase 6.5.4 Sim Time (s) |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| `HEAD-ON` | 14 | 100.0% | 0.0% | 0.0% | 0.160307 | 0.257926 | 26.21 | 25.89 | 16.01 | 15.85 |
| `CROSSING` | 14 | 100.0% | 0.0% | 0.0% | 0.151385 | 0.152699 | 19.21 | 19.27 | 13.41 | 13.80 |
| `CORRIDOR` | 14 | 78.6% | 21.4% | 50.0% | -0.045589 | 0.186774 | 17.60 | 16.32 | 19.39 | 18.45 |
| `STOP-AND-GO` | 18 | 100.0% | 0.0% | 11.1% | 0.093101 | 0.152024 | 16.17 | 15.75 | 11.21 | 11.00 |
| `NARROW-GATE` | 14 | 42.9% | 57.1% | 57.1% | -0.034224 | 0.330631 | 14.07 | 24.20 | 11.41 | 15.55 |
| `MULTIPLE-DYNAMIC-OBSTACLES` | 14 | 92.9% | 7.1% | 14.3% | -0.023594 | 0.152792 | 24.25 | 25.47 | 17.01 | 17.60 |
| `SUDDEN-STOP` | 18 | 66.7% | 33.3% | 33.3% | -0.062278 | 0.153666 | 15.83 | 16.43 | 15.78 | 14.80 |
| `SUDDEN-STOP-FINITE-DECEL` | 14 | 100.0% | 0.0% | 0.0% | 0.153510 | 0.153625 | 16.35 | 16.42 | 16.61 | 16.70 |

## 3. Failure Analysis & Root-Cause Grouping (`25` Unsafe/Failed Variants)

Exported in `analysis/results/phase655_level1_failure_analysis.json`:

1. **`NARROW_GATE_TIMING_AND_HOLDING_MARGIN` (`8` collision variants)**:
   - `NARROW-GATE__L1-IC-01__02`, `L1-IC-02__01`, `L1-IC-02__02`, `L1-IC-04__01`, `L1-IC-05__01`, `L1-DO-01__01`, `L1-RD-01__01`, `L1-RD-02__01`.
   - All collide at `t = 6.90–8.00 s` with `SAFETY_MARGIN_VIOLATION` fallback active: small perturbations to robot approach timing, heading, speed, or braking authority cause the robot to enter or hold slightly too close to the narrow-gate entrance (`t ~ 6.9 s`) when the patrolling obstacle exits/reverses, leaving insufficient margin to brake or evade.
2. **`SUDDEN_STOP_CAUSAL_REVERSAL_TRAP` (`6` collision variants)**:
   - `SUDDEN-STOP__L1-IC-02__01`, `L1-IC-02__02`, `L1-DO-01__01`, `L1-DO-02__01`, `L1-DO-03__01`, `L1-DO-03__02`.
   - Collide at `t = 13.65–14.60 s`. Under causal observation, `STOP_AND_GO` timing perturbations (`stop_duration_offset_s`, `move_duration_offset_s`, `obs_speed_scale`, heading offset) alter when the lead obstacle stops and reverses; when reversal occurs while the robot is closely trailing, causal constant-velocity prediction cannot anticipate the unobservable schedule transition.
3. **`CORRIDOR_HEADING_PERTURBATION_PASSING_TIMING` (`2` collision variants)**:
   - `CORRIDOR__L1-IC-02__01`, `CORRIDOR__L1-IC-02__02` (`robot_start_heading_rad = ±0.26 rad`, collision at `t = 11.60 s`).
   - Initial heading offset changes early approach alignment before/during corridor entry, altering lateral passing initiation timing.
4. **`GEOMETRIC_WIDTH_REDUCTION_INFEASIBLE` (`1` collision variant)**:
   - `CORRIDOR__L1-SG-01__01` (`gate_or_corridor_width_delta_m = -0.20 m` -> corridor width `2.80 m`).
   - Reduces side passing gaps around `DYN-CORR-1` (`r = 0.40 m`) to `1.00 m` per side, leaving zero slack above `2 * robot_radius + 2 * safety_margin = 1.00 m`, making lateral passing geometrically borderline/infeasible under discrete grid inflation (`0.20 m`).
5. **`DYNAMICS_OR_SPEED_SCALE_MARGIN_DIP` (`8` variants: `1` collision + `7` margin-violation-only `GOAL_REACHED`)**:
   - `MULTIPLE-DYNAMIC-OBSTACLES__L1-RD-01__01` (`max_linear_speed_mps = 2.2`, collision at `t = 11.35 s`); plus `7` `GOAL_REACHED` runs with brief `< 0.15 m` margin dips (`0.0931–0.1492 m`): `CORRIDOR__L1-IC-01__02`, `CORRIDOR__L1-IC-04__01`, `CORRIDOR__L1-IC-05__01`, `CORRIDOR__L1-RD-02__01`, `STOP-AND-GO__L1-DO-01__01`, `STOP-AND-GO__L1-DO-03__01`, `MULTIPLE-DYNAMIC-OBSTACLES__L1-DO-01__02`.

## 4. Deliverables & Verification

- `analysis/results/phase655_level1_raw.json` & `.csv` (`360` runs)
- `analysis/results/phase655_level1_determinism.json` (`120 / 120` physically deterministic)
- `analysis/results/phase655_level1_summary.json`
- `analysis/results/phase655_level1_failure_analysis.json`
- `analysis/results/phase655_level1_report.md`
- **Full `pytest -q` suite**: **`162 passed in 56.17s`** (`0` failed, `0` skipped; zero source/config/test modifications).
