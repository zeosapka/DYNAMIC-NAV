# Phase 6.5.5.5 — Open-Space Proactive Overtaking & Safe Following Report

## 1. Source Changes & Physical Rationale

- **[config.py](file:///c:/Users/Legolas/Desktop/DYNAMIC-NAV/config.py)** (`DWAConfig`): Added `proactive_open_space_enabled: bool = True` and `proactive_open_space_hysteresis_steps: int = 8`.
- **[algorithms/corridor_passing.py](file:///c:/Users/Legolas/Desktop/DYNAMIC-NAV/algorithms/corridor_passing.py)** (`CorridorPassingManager`):
  - Extended `assess_corridor()` to consider proactive lateral overtaking in open/post-gate space (`not in_corridor`) when a slower same-direction lead obstacle (`obs_along_v > 0.15 m/s`) continuously obstructs the path ahead for `proactive_open_space_hysteresis_steps = 8` consecutive steps (`0.40 s`) and sufficient lateral passing clearance exists.
  - Requiring `obs_along_v > 0.15 m/s` prevents unnecessary replanning on perpendicular crossing (`obs_along_v ~ 0.0`) or oncoming obstacles.
  - Added anti-oscillation side preference (`_last_chosen_side`) in `generate_and_validate_candidates()` when left/right candidates have comparable DWA/braking clearance (`0.01 m` quantization).
  - Every overtaking candidate continues to pass full DWA, braking, swept-clearance, static, dynamic, and world-boundary validation (`>= 0.15 m`).
- **[tests/test_phase6555_open_space_overtaking.py](file:///c:/Users/Legolas/Desktop/DYNAMIC-NAV/tests/test_phase6555_open_space_overtaking.py)**: Added 7 targeted tests covering `L1-RD-01__01`, `L1-SG-01__01`, open-space trigger hysteresis, insufficient lateral clearance, disabled-overtaking safe following, causal schedule isolation, and braking-margin rejection.

## 2. Targeted Regression Results

Exported to `analysis/results/phase6555_regression_results.json`:
- **`14 passed in 65.38s`** (`0` failed) across `tests/test_phase6555_open_space_overtaking.py` (`7`) and `tests/test_phase6553_narrow_gate_standoff.py` (`7`).

## 3. Baseline (8 Scenarios) & NARROW-GATE Level 1 (14 Variants × 3 Reps = 42 Runs)

Exports:
- `analysis/results/phase6555_open_space_raw.json`
- `analysis/results/phase6555_determinism.json`

### 3.1 Eight Phase 6.5.4 Baseline Scenarios
- **8/8 `GOAL_REACHED`**, **`0` physical collisions**, **`0` safety-margin violations**:
  - `HEAD-ON`: `0.257926 m`
  - `CROSSING`: `0.152699 m`
  - `CORRIDOR`: `0.155123 m`
  - `STOP-AND-GO`: `0.152024 m`
  - `NARROW-GATE`: `0.154363 m`
  - `MULTIPLE-DYNAMIC-OBSTACLES`: `0.152792 m`
  - `SUDDEN-STOP`: `0.298483 m`
  - `SUDDEN-STOP-FINITE-DECEL`: `0.153625 m`

### 3.2 All 14 `NARROW-GATE` Level 1 Variants (3× Deterministic)
- **Goal completion**: **`14 / 14` (`100.0%`)**, up from `42.9%` (Phase 6.5.5) and `85.7%` (Phase 6.5.5.3).
- **Physical collisions**: **`0 / 14` (`0.0%`)** — both `NARROW-GATE__L1-RD-01__01` (`min_swept = 0.1527 m`) and `NARROW-GATE__L1-SG-01__01` (`min_swept = 0.1544 m`) now reach the goal with `0` collisions and `0` margin violations.
- **Safety-margin violations**: `13 / 14` variants have `0` violations (`min_swept >= 0.1527 m`); `NARROW-GATE__L1-DO-01__01` (`obs_speed_scale = 0.85`) has `1` minor margin-violation episode (`min_swept = 0.1498 m`, `GOAL_REACHED`, `0` collisions).
- **3-run physical determinism**: **`14 / 14` (`100.0%`)**.

## 4. Full Pytest Verification
- **Full `pytest -q` suite**: **`184 passed in 152.59s`** (`0` failed, `0` skipped).
