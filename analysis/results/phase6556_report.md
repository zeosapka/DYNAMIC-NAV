# Phase 6.5.5.6 — Safety Margin Precision Audit Report

**Mode**: Strictly READ-ONLY Diagnostic Investigation (zero source, parameter, threshold, test, or historical artifact modifications).

## 1. Exact Reproduction (`NARROW-GATE__L1-DO-01__01`)

Exported to `analysis/results/phase6556_reproduction.json` (identical across all 3 repetitions):
- **Minimum instantaneous clearance**: `0.15009776392091712 m` (at `t = 15.50 s`, `>= 0.150000 m`)
- **Minimum swept clearance**: **`0.14981141189316083 m`** (at `t = 15.55 s`, `< 0.150000 m` by `0.0001886 m` / `0.189 mm`)
- **Geometry pair**: `ROBOT_CIRCLE` vs `STATIC_OBSTACLE_AABB` (dynamic swept clearance at `t = 15.55 s` is `0.251752 m`, well above `0.15 m`)
- **DWA predicted clearance**: `0.15009776 m` (`>= 0.150000 m`)

## 2. Root Cause (Measured vs. Geometric Audit)

Exported to `analysis/results/phase6556_geometry_audit.json` and `analysis/results/phase6556_numerical_analysis.json`:
- **Is the safety margin genuinely violated?**: **Yes** — the true continuous swept clearance along the `[15.50, 15.55] s` robot chord is `0.149811 m < 0.150000 m`.
- **Root cause**: **Candidate-vs-Evaluator Static Swept Geometry Mismatch**.
  - `analysis/metrics.py::compute_swept_clearance_m()` evaluates **continuous segment-to-AABB** clearance along `[prev_pos, curr_pos]` for static obstacles.
  - `algorithms/dwa.py::evaluate_trajectory_with_braking()` evaluates continuous swept-circle clearance for **dynamic** obstacles, but only **pointwise `_point_clearance()`** at trajectory endpoints for **static** obstacles.
  - When the robot grazes past a static gate wall/corner between `t = 15.50 s` (`0.150098 m`) and `t = 15.55 s` (`0.150235 m`), the interior of the chord dips `0.000286 m` closer (`0.149811 m`), which pointwise endpoint checks cannot see.

## 3. Cross-Scenario Integrity & Proposed Minimal Correction

Exported to `analysis/results/phase6556_regression_comparison.json` and `analysis/results/phase6556_improvement_proposals.json`:
- **Baseline & `NARROW-GATE` integrity**: 8/8 baseline scenarios pass with 0 collisions and 0 margin violations; 14/14 `NARROW-GATE` Level 1 variants reach goal with 0 collisions (`13/14` with 0 margin violations).
- **Proposed minimal correction**: Add continuous segment-to-AABB static swept-clearance checking in `algorithms/dwa.py::evaluate_trajectory_with_braking()` so DWA candidate & braking validation matches `compute_swept_clearance_m()` without modifying any safety thresholds.
