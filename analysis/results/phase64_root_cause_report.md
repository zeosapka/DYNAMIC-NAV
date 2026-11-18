# Phase 6.4 — Stage 1: Safety Margin Compliance Root Cause Analysis

**Scope**: Read-only diagnostic investigation of all remaining `< 0.150000 m` safety-margin violations across `CROSSING`, `CORRIDOR`, `SUDDEN-STOP`, and `SUDDEN-STOP-FINITE-DECEL`. No source code, configuration, thresholds, or benchmark datasets were modified.

---

## 1. Executive Summary & Ranked Root Causes

All 4 remaining `< 0.150000 m` safety-margin violations occur with **0 physical collisions** and **100% goal completion**, and trace back to **three distinct mathematical causes**:

1. **Rank 1 (Primary Cause in 4/4 scenarios): Prediction-vs-Simulation Step Mismatch (`predict_dt_s = 0.10 s` vs `dt_s = 0.05 s`) + Curved-vs-Linear Mid-Step Approximation**
   - `DWAPlanner` evaluates candidate trajectories at `predict_dt_s = 0.10 s` intervals, whereas `SimulationWorld.step()` integrates robot and obstacle motion at `dt_s = 0.05 s` (`2×` finer).
   - At `t = 0.05 s` (the half-step *between* DWA's `t = 0.00 s` and `t = 0.10 s` samples):
     - In **`CROSSING`** (`t = 6.05 s`), **`SUDDEN-STOP-FINITE-DECEL`** (`t = 9.90 s`), and **`CORRIDOR`** (`t = 20.15 s`), DWA's `0.10 s` sample was strictly `>= 0.150000 m` (`0.150002 m`, `0.150081 m`, `0.150094 m`), so DWA accepted the candidate as margin-compliant (`prio = None`), while the actual `0.05 s` curved midpoint integration dipped below `0.150000 m` (`0.149955 m` swept, `0.149976 m` inst, `0.149829 m` inst).
     - In **`SUDDEN-STOP`** Episode 2 (`t = 12.00 s`), `DYN-SUDDEN-1` was stationary at `t = 11.95 s` (`vx = 0.0`), resumed motion at `t = 12.00 s` (`pos` still `13.50 m` at `12.00 s`, advancing to `13.56 m` at `12.05 s`), and DWA's `0.10 s` step evaluated `t = 12.05 s` (when the obstacle had already moved `+0.06 m`), commanding `v = 0.496 m/s` and causing a `0.138989 m` dip at the skipped `12.00 s` step.
2. **Rank 2 (Secondary Cause in `CORRIDOR`): Recovery Entry Cascading After Initial `0.171 mm` Sub-Step Dip**
   - Once `CORRIDOR` dipped `0.171 mm` below `0.150000 m` at `t = 20.15 s` (`0.149829 m`), the head-on obstacle (`vx = -1.0 m/s`) kept closing for 3 steps (`t = 20.20–20.30 s`) while `1step_reach_015 = False`, reaching `0.135930 m` at `t = 20.25 s` before Priority 2 evasion cleared the obstacle at `t = 20.35 s`.
3. **Rank 3 (Boundary Precision in `SUDDEN-STOP` Episode 1): IEEE-754 Floating-Point Boundary Graze (`5.58e-14 m`)**
   - At `t = 6.00 s`, `min_inst = min_swept = 0.14999999999994423 m` (`0.150000 m - 5.58e-14 m`) due to floating-point arithmetic at the exact `0.150000 m` boundary (`safety_margin_m - 1e-9` tolerance in DWA vs strict `< 0.15` in `RunMetricsTracker`).

---

## 2. Per-Scenario Diagnostic Breakdown

### 2.1 `CROSSING` (`0.149954986 m` swept clearance)
- **Exact Timestamp**: `t = 6.05 s` (`1` violation episode, `1` step).
- **Robot State (post-step)**: `x = 9.0407 m`, `y = 4.5782 m`, `theta = 0.3200 rad`, `v = 1.0837 m/s`, `omega = 0.2000 rad/s`.
- **Closest Obstacle**: `DynamicObstacle:DYN-CROSS-2` (`Circle r = 0.40 m`, `MovementBehavior.CROSSING`), `pos = (9.5550, 3.8325) m`, `vel = (-0.9000, -1.3500) m/s`.
- **Closest Points**: Robot `(9.2394, 4.2901) m`, Obstacle `(9.3279, 4.1618) m`.
- **Measured Clearances**: `inst = 0.155841447 m` (`>= 0.15`), `swept = 0.149954986 m` (`45 µm` below `0.15`).
- **DWA Command & Prediction**: `cmd = (1.0837 m/s, 0.2000 rad/s)`, `fallback_priority = None`, `predicted_min_clearance_m = 0.150002001 m`, `brake_clearance = 0.477057 m`.
- **Physical Reachability of `0.15 m`**: **Yes** (`reachable_ge_015_in_1step_window = True`, `best_1step_swept = 0.150303 m`).
- **Root Cause**: `predict_dt_s = 0.10 s` skipped the `0.05 s` mid-step swept minimum (`45 µm` dip while `inst` stayed `0.1558 m`). Obstacle motion model did not underestimate risk (constant velocity matched).

### 2.2 `CORRIDOR` (`0.135930414 m` clearance)
- **Exact Timestamps**: `t = 20.15 s` through `t = 20.35 s` (`1` episode, `5` steps; minimum at `t = 20.25 s`).
- **Entry Step (`t = 20.15 s`)**:
  - Robot: `(11.8607, 6.6993) m`, `theta = 1.4683 rad`, `v = 0.5747 m/s`, `omega = -1.1798 rad/s`.
  - Obstacle: `DynamicObstacle:DYN-CORR-1` (`Circle r = 0.40 m`, `pos = (11.4500, 7.5000) m`, `vel = (-1.0000, 0.0000) m/s`).
  - Closest Points: Robot `(11.7009, 7.0108) m`, Obstacle `(11.6326, 7.1441) m`.
  - DWA Predicted vs Actual: `predicted_min_clearance_m = 0.150094367 m` (`prio = None`, `brake_c = 0.790228 m`) vs actual `inst = swept = 0.149829024 m`.
  - Physical Reachability at `t = 20.15 s`: **Yes** (`reachable_ge_015_in_1step_window = True`, `best_1step_swept = 0.153509 m`).
- **Minimum Step (`t = 20.25 s`)**:
  - Robot: `(11.8732, 6.7850) m`, `theta = 1.3804 rad`, `v = 0.9080 m/s`, `omega = -0.9798 rad/s`; Obstacle `(11.3500, 7.5000) m`.
  - Closest Points: Robot `(11.6665, 7.0675) m`, Obstacle `(11.5862, 7.1772) m`.
  - `inst = swept = 0.135930414 m`, `fallback_priority = 2` (`CLEARANCE_INCREASING_RECOVERY`), `1step_reach_015 = False` (because `DYN-CORR-1` is already within `< 0.15 m` while moving at `-1.0 m/s`).
  - Recovers to `inst = 0.158803 m` at `t = 20.35 s`.
- **Root Cause**: Initial `0.171 mm` dip at `t = 20.15 s` from `predict_dt_s = 0.10 s` vs `dt_s = 0.05 s` curved midpoint integration, followed by 3 steps of unavoidable `< 0.15 m` recovery against the `-1.0 m/s` approaching obstacle.

### 2.3 `SUDDEN-STOP` (`0.138989146 m` clearance)
- **Episode 1 (`t = 6.00 s`, `t = 6.05 s`)**:
  - Robot: `(9.5500, 7.5000) m`, `v = 1.0833 m/s`, `omega = 0.0`; Obstacle `DYN-SUDDEN-1` (`r = 0.45 m`, `pos = (10.5000, 7.5000) m`, `vel = (1.2000, 0.0)`).
  - Closest Points: Robot `(9.9000, 7.5000) m`, Obstacle `(10.0500, 7.5000) m`.
  - `inst = swept = 0.14999999999994423 m` (`5.58e-14 m` below `0.150000 m`), `prio = None`, `brake_c = 0.284167 m`, `1step_reach_015 = True` (`best_1step = 0.162509 m`).
  - **Root Cause**: Exact floating-point rounding at `0.150000 m` coupled with DWA's `safety_margin_m - 1e-9` check.
- **Episode 2 (`t = 12.00 s`, `t = 12.05 s`)**:
  - Robot at `t = 12.00 s`: `(12.5610, 7.5000) m`, `v = 0.4960 m/s`, `omega = 0.0`; Obstacle `DYN-SUDDEN-1` at `(13.5000, 7.5000) m` (had `vx = 0.0` at `t = 11.95 s`, transitioned to `vx = 1.2` at `t = 12.00 s` without moving yet).
  - Closest Points: Robot `(12.9110, 7.5000) m`, Obstacle `(13.0500, 7.5000) m`.
  - `inst = swept = 0.138989146 m` while DWA predicted `0.163790497 m` (`prio = None`, `brake_c = 0.948558 m`). At `t = 12.05 s`, Priority 2 (`0.25, -0.80`) recovers to `inst = 0.186690 m`.
  - **Root Cause**: `predict_dt_s = 0.10 s` evaluated `t = 12.05 s` (where `DYN-SUDDEN-1` had resumed moving `+0.06 m`) rather than `t = 12.00 s` (where `DYN-SUDDEN-1` was still at `13.50 m`). Note: `1step_reach_015 = True` (`best_1step = 0.163790 m`) — braking or holding at `v <= 0.25 m/s` would have stayed `>= 0.15 m`.

### 2.4 `SUDDEN-STOP-FINITE-DECEL` (`0.149975839 m` clearance)
- **Exact Timestamps**: `t = 9.90 s` (`inst = swept = 0.149975839 m`) and `t = 9.95 s` (`inst = 0.150810 m`, `swept = 0.149975839 m`).
- **Robot State (`t = 9.90 s`)**: `(8.4011, 7.3010) m`, `theta = -1.3500 rad`, `v = 0.2000 m/s`, `omega = -0.6000 rad/s`.
- **Closest Obstacle**: `SimObject:OBJ-FINITE-DECEL-1` (`CircleGeometry(9.33, 7.5, r=0.45)`, `DECELERATING`, already stationary `vel = (0.0, 0.0)`).
- **Closest Points**: Robot `(8.7433, 7.3743) m`, Obstacle `(8.8900, 7.4057) m`.
- **DWA Command & Prediction**: `cmd = (0.20 m/s, -0.60 rad/s)`, `prio = None`, `predicted_min_clearance_m = 0.150080816 m`, `brake_c = 0.261623 m`.
- **Physical Reachability of `0.15 m`**: **Yes** (`reachable_ge_015_in_1step_window = True`, `best_1step_swept = 0.150276 m`).
- **Root Cause**: `predict_dt_s = 0.10 s` skipped the `0.05 s` intermediate pose where the curved `ω = -0.60 rad/s` trajectory passed `24 µm` closer to stationary `OBJ-FINITE-DECEL-1` than at `t = 0.10 s`.

---

## 3. Physical & Mathematical Analysis Summary

- **Prediction horizon adequacy (`1.0 s` + full braking trajectory)**: Adequate in all 4 scenarios (`brake_c >= 0.19 m` at all entry steps).
- **Prediction sampling interval (`0.10 s` vs `0.05 s`)**: **Insufficient at the first step.** Checking only `t = 0.10 s` instead of `[0.05 s, 0.10 s]` misses curved-path extrema and discrete `STOP_AND_GO` phase boundaries at `dt_s = 0.05 s`.
- **Floating-point tolerance mismatch**: DWA checks `min_clearance < safety_margin_m - 1e-9`, whereas `RunMetricsTracker` counts `< 0.15` strictly, creating a `1e-9 m` gap exploited by `SUDDEN-STOP` (`t = 6.00 s`).
- **Physical reachability of `0.150000 m`**: In **all 4 scenarios**, a command inside the reachable dynamic window at the entry step could have maintained `>= 0.150000 m` (`1step_reach_015 = True`).

---

## 4. Evaluated Solution Alternatives (Cost vs. Benefit)

| Alternative | Addresses | Expected Impact on `20 Hz` Budget | Side Effects / Risks | Recommendation |
| :--- | :--- | :--- | :--- | :--- |
| **A. Sub-step first DWA interval at `dt_s = 0.05 s`** (evaluate `0.05 s` midpoint/step in `evaluate_trajectory_with_braking` + exact `0.05 s` obstacle step) | `CROSSING`, `CORRIDOR`, `SUDDEN-STOP` Ep 2, `SUDDEN-STOP-FINITE-DECEL` | **Low** (`+1` point check per candidate; `~5–8%` cost, keeps `p99 < 50 ms`) | Eliminates `0.10 s` vs `0.05 s` blind spot without slowing the whole `1.0 s` horizon | **Recommended (Primary)** |
| **B. Align DWA safety-margin threshold (`safety_margin_m` without `- 1e-9` slack, or tiny `+ 1e-6 m` planning buffer)** | `SUDDEN-STOP` Ep 1 (`5.6e-14 m` float graze) and sub-millimeter boundary grazes | **Zero** (`O(1)` comparison) | Slight conservatism (`1 µm`) prevents IEEE-754 boundary false violations | **Recommended (Primary)** |
| **C. Halve `predict_dt_s` globally (`0.10 s -> 0.05 s`)** | All `0.05 s` sub-step gaps | **High (`~2×` DWA runtime)** — risks breaking 20 Hz | Unnecessary for `t > 0.10 s` where DWA replans every `0.05 s` anyway | Not recommended |
| **D. Constant-velocity uncertainty pad (`+0.05 m` over whole horizon)** | Unmodeled obstacle stops | **Zero** | Overly conservative in `NARROW-GATE` / `CORRIDOR`; may trigger unnecessary stops/replans | Use only if A+B leave residual dips |
| **E. Adaptive horizon / curved-arc swept analytical solver** | Long-horizon / high-`ω` curvature | **Medium–High** | Complex geometry math; `0.05 s` first-step check already bounds curvature error to `< 1 µm` | Not needed if A is used |

---

## 5. Recommended Phase 6.4 Stage 2 Interventions

1. **Evaluate the first `dt_s = 0.05 s` simulation step explicitly** in `evaluate_trajectory_with_braking()` (and `precomputed_dyn_preds` / `simobj_pred_cache` at `t = 0.05 s`), ensuring DWA's first-step trajectory and obstacle prediction match `SimulationWorld.step(0.05)` identically.
2. **Eliminate the `-1e-9` slack in DWA's safety-margin acceptance** (or require `min_clearance >= safety_margin_m + 1e-6` for normal non-fallback acceptance) so floating-point rounding at `0.150000 m` never passes DWA only to fail `RunMetricsTracker`'s strict `< 0.15` check.
3. **Preserve all Phase 6.3 optimizations** (`simobj_pred_cache`, `static_boxes`, AABB early rejection, `precomputed_dyn_at_steps`) so `20 Hz` performance remains intact.
