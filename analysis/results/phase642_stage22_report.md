# Phase 6.4 — Stage 2.2: Targeted Fallback Priority Correction & Collision Classification Integrity Report

## 1. Exact Source Changes

1. [dwa.py](file:///c:/Users/Legolas/Desktop/DYNAMIC-NAV/algorithms/dwa.py):
   - **Collision Classification Integrity**:
     - Added `TrajectorySafetyClassification` dataclass and `classify_trajectory_safety()` helper maintaining separate outcomes for `safety_margin_compliant`, `collision_free`, `min_predicted_clearance_m`, and `braking_valid`.
     - Updated `evaluate_trajectory_with_braking(..., stop_on_margin_violation: bool = False)` so direct/full evaluations never let an early `< 0.15 m` margin violation mask a later `<= 0.0 m` physical collision, and checked `min_brake_c <= 0.0` explicitly. (`compute_command()` passes `stop_on_margin_violation=True` in its fast candidate loop and `stop_on_margin_violation=False` in reverse recovery.)
   - **Conditional Reverse Recovery Priority (`GATE_CLEAR_HOLD_UNSAFE`)**:
     - Extracted `_find_reverse_candidates(require_margin_safe: bool)` inside `DWAPlanner.compute_command()`.
     - When `best_cmd is None` and `waiting_assessment.state == WaitingZoneState.GATE_CLEAR_HOLD_UNSAFE`, evaluates reverse candidates first and selects a reverse recovery (`Priority 3`) **only** when at least one reverse candidate satisfies the complete prediction + braking checks (`>= 0.15 m`) **and** its end pose remains `>= 0.15 m` over `waiting_horizon_s`. Otherwise, existing `Priority 2` fallback is preserved intact.
2. [test_phase642_stage22.py](file:///c:/Users/Legolas/Desktop/DYNAMIC-NAV/tests/test_phase642_stage22.py):
   - Added 6 targeted unit tests covering conditional reverse priority, rear-blocked fallback preservation, early-margin-violation + late-collision classification, margin-violation-without-collision distinction, acceleration/braking constraints, and static/`SimObject` geometry checks.

---

## 2. Collision Classification & Unit Test Results (`tests/test_phase642_stage22.py`)
- **6 passed in 0.19s**:
  - `test_gate_clear_hold_unsafe_prefers_safe_reverse_over_margin_violating_forward`: **PASSED**
  - `test_gate_clear_hold_unsafe_retains_fallback_when_no_safe_reverse_exists`: **PASSED**
  - `test_early_margin_violation_followed_by_collision_is_classified_colliding`: **PASSED**
  - `test_margin_violation_without_collision_preserves_distinction`: **PASSED**
  - `test_reverse_recovery_respects_acceleration_and_braking_constraints`: **PASSED**
  - `test_reverse_recovery_respects_static_and_simobject_geometry`: **PASSED**

---

## 3. Eight-Scenario Benchmark Comparison (`ASTAR_DWA`) & `t = 17.15–17.40 s` Behavior

| Scenario | Stage 2 Outcome | Stage 2.2 Outcome | Collisions (Ep / St) | Margin Viol (Ep / St) | `min_instantaneous_clearance_m` | `min_swept_clearance_m` |
| :--- | :--- | :--- | :---: | :---: | ---: | ---: |
| `HEAD-ON` | `GOAL_REACHED` | `GOAL_REACHED` | 0 / 0 | 0 / 0 | `0.261441810` | `0.260617498` |
| `CROSSING` | `GOAL_REACHED` | `GOAL_REACHED` | 0 / 0 | 0 / 0 | `0.150303320` | `0.150206332` |
| `CORRIDOR` | `GOAL_REACHED` | `GOAL_REACHED` | 0 / 0 | 0 / 0 | `0.150567380` | `0.150000616` |
| `STOP-AND-GO` | `GOAL_REACHED` | `GOAL_REACHED` | 0 / 0 | 0 / 0 | `0.151122714` | `0.150447648` |
| `NARROW-GATE` | `GOAL_REACHED` | `GOAL_REACHED` | 0 / 0 | 0 / 0 | `0.349942855` | `0.349806711` |
| `MULTIPLE-DYNAMIC-OBSTACLES` | `GOAL_REACHED` | `GOAL_REACHED` | 0 / 0 | 0 / 0 | `0.151183292` | `0.150658552` |
| `SUDDEN-STOP-FINITE-DECEL` | `GOAL_REACHED` | `GOAL_REACHED` | 0 / 0 | 0 / 0 | `0.150000833` | `0.150000833` |
| `SUDDEN-STOP` | `COLLISION_ABORT` | `COLLISION_ABORT` | 1 / 1 | 1 / 2 | `-0.029184070` | `-0.029184070` |

### Why `SUDDEN-STOP` Still Collides (`t = 17.15–18.15 s`) — Honest Physical Diagnosis
- **Before vs. After**:
  - Without waiting-horizon validation on `safe_rev`, `SUDDEN-STOP` reversed straight back (`v = -0.60 m/s`) from `t = 17.15 s` to `t = 17.90 s` (`c = 0.91 m`), then was overtaken by `DYN-SUDDEN-1` reversing at `-1.20 m/s` (`t = 18.70 s`, `-0.00447 m`), **and** `HEAD-ON` / `MULTIPLE-DYNAMIC-OBSTACLES` both regressed to `COLLISION_ABORT` because straight-line reverse preempted lateral forward evasion.
  - With waiting-horizon validation (`2.5 s`) on `safe_rev`, all 7 other scenarios (`HEAD-ON`, `CROSSING`, `CORRIDOR`, `STOP-AND-GO`, `NARROW-GATE`, `MULTIPLE-DYNAMIC-OBSTACLES`, `SUDDEN-STOP-FINITE-DECEL`) retain `GOAL_REACHED` with `0` collisions and `0` margin violations, and `SUDDEN-STOP`'s `t = 12.00 s` correction (`0.15155 m`) remains intact.
  - However, in `SUDDEN-STOP` at `t = 17.15 s`, because `DYN-SUDDEN-1` reverses at `-1.20 m/s` for `~5 s` along `y = 7.50 m` while `min_reverse_speed_mps = -0.60 m/s`, **no reverse trajectory stays `>= 0.15 m` over `2.5 s`**. Consequently `safe_rev` is empty, `Priority 2` (`p2_cf`) runs, and `SUDDEN-STOP` still collides at `t = 18.15 s` (`-0.029184070 m`). Resolving `SUDDEN-STOP` without breaking `HEAD-ON` requires earlier lateral lane-change evasion *while `DYN-SUDDEN-1` is stationary* (`t = 16.00–17.10 s`), not straight-line reverse recovery.

---

## 4. Full Pytest & Determinism Results
- **Full pytest suite (`.\.venv\Scripts\pytest.exe -q`)**: **131 passed, 1 failed** in `60.32s`:
  - Only failure: `tests/test_phase62_safe_waiting_reverse.py::test_phase62_integration_scenarios_zero_collisions_and_determinism[SUDDEN-STOP-False]`.
- **3-Run Determinism (`benchmark_phase642_stage22_determinism.json`)**: `physically_identical = True` across all **8/8 scenarios**.

---

## 5. Performance Comparison (3 Runs × 8 Scenarios, `8,013` calls)

| Scenario | Avg (ms) | Median (ms) | p95 (ms) | p99 (ms) | Max (ms) | `> 50 ms` |
| :--- | ---: | ---: | ---: | ---: | ---: | :---: |
| `HEAD-ON` | 4.74 | 4.72 | 5.89 | 6.22 | 11.42 | 0 (0.0%) |
| `CROSSING` | 5.97 | 6.22 | 7.14 | 7.50 | 11.18 | 0 (0.0%) |
| `CORRIDOR` | 3.37 | 4.36 | 5.77 | 6.03 | 6.99 | 0 (0.0%) |
| `STOP-AND-GO` | 4.73 | 4.72 | 5.76 | 6.02 | 7.47 | 0 (0.0%) |
| `NARROW-GATE` | 4.53 | 4.60 | 5.99 | 6.42 | 6.95 | 0 (0.0%) |
| `MULTIPLE-DYNAMIC-OBSTACLES` | 7.34 | 7.52 | 9.05 | 9.61 | 22.71 | 0 (0.0%) |
| `SUDDEN-STOP` | 4.61 | 4.77 | 5.68 | 6.03 | 6.47 | 0 (0.0%) |
| `SUDDEN-STOP-FINITE-DECEL` | 8.97 | 9.43 | 10.59 | 10.90 | 16.26 | 0 (0.0%) |
| **OVERALL (`8,013` calls)** | **5.44** | **5.11** | **9.72** | **10.48** | **22.71** | **0 (0.0%)** |
