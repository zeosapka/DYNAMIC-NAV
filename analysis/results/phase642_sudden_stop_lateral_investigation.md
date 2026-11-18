# Phase 6.4 — Stage 2.3: `SUDDEN-STOP` Predictive Lateral Evasion Investigation

**Scope**: Read-only investigation of physically feasible unicycle lateral evasion maneuvers in `SUDDEN-STOP`, comparing `t = 14.00–18.15 s` failure dynamics against runtime-observable predictions. No source code, configuration, thresholds, or benchmark files were modified.

Full telemetry and maneuver evaluations are saved in [phase642_sudden_stop_lateral_investigation.json](file:///c:/Users/Legolas/Desktop/DYNAMIC-NAV/analysis/results/phase642_sudden_stop_lateral_investigation.json).

---

## 1. Failure Timeline (`t = 14.00 s` → `t = 18.15 s`)

- **`t = 14.00–15.95 s`**: `DYN-SUDDEN-1` decelerates/stops at `(16.500, 7.500)`. Normal DWA (`prio = None`, `safe_fwd = 103..119`) follows the A* centerline (`y ≈ 7.49 m`) at `v = 0.15–0.20 m/s`.
- **`t = 16.00–17.10 s`**: `DYN-SUDDEN-1` is stationary at `(16.500, 7.500)` (`c = 0.644 -> 0.485 m`). Waiting-zone state transitions to `GATE_CLEAR_HOLD_UNSAFE` (because `waiting_horizon_s = 2.5 s` sees `DYN-SUDDEN-1`'s future reversal), while normal DWA still has `safe_fwd = 17..103` candidates on `1.0 s` horizon, creeping forward and stopping at `(15.216, 7.491, v = 0.00 m/s)` at `t = 17.00–17.10 s`.
- **`t = 17.15 s`**: `safe_fwd` drops to `0`, `best_cmd` becomes `None`, and DWA enters `Priority 2` (`p2_cf`, `SAFETY_MARGIN_VIOLATION`) instead of `Priority 4` (`select_safe_holding_pose()`, which is unreachable whenever `collision_free` is non-empty). `p2_cf` accelerates forward (`v = 0.25 -> 0.92 m/s`, `ω = 0.20–0.70 rad/s`) along `y = 7.50 m` until trapped at `t = 17.80 s`, colliding at **`t = 18.15 s`** (`min_inst = min_swept = -0.029184070 m`).

---

## 2. Lateral Evasion Feasibility (Exact Unicycle Physics, `6.0 s` Evaluation)

All maneuvers were simulated using exact `Robot.step(0.05 s)` midpoint integration, `max_linear_accel = 5.0 m/s²`, `max_angular_accel = 16.0 rad/s²`, `max_linear_speed = 2.0 m/s`, `min_reverse_speed = -0.60 m/s`, `max_omega = 2.84 rad/s`, and full `6.0 s` obstacle motion (covering `t = 18.00–22.00 s` reversal at `-1.20 m/s`):

| Start Time | Maneuver | Feasible (`>= 0.15 m`) | Duration (s) | Min Inst (m) | Min Swept (m) | Final Pose `(x, y)` |
| :--- | :--- | :---: | ---: | ---: | ---: | :--- |
| `15.00 s` | `rotate_move_left` / `right` | **YES** | `2.15` | `0.5085` / `0.5313` | `0.5085` / `0.5313` | `(14.87, 8.81)` / `(14.87, 6.17)` |
| `15.00 s` | `curved_fwd_left` / `right` | **YES** | `1.75` | `0.4466` / `0.4671` | `0.4465` / `0.4670` | `(15.07, 8.75)` / `(15.08, 6.23)` |
| `16.00 s` | `rotate_move_left` / `right` | **YES** | `2.15` | `0.5095` / `0.5303` | `0.5095` / `0.5303` | `(15.07, 8.81)` / `(15.07, 6.17)` |
| `16.00 s` | `curved_fwd_left` / `right` | **YES** | `1.75` | `0.3752` / `0.3764` | `0.3752` / `0.3764` | `(15.26, 8.75)` / `(15.26, 6.23)` |
| `16.60 s` | `rotate_move_left` / `right` | **YES** | `2.15` | `0.5100` / `0.5298` | `0.5100` / `0.5298` | `(15.17, 8.81)` / `(15.17, 6.17)` |
| `16.80 s` | `rotate_move_left` / `right` | **YES** | `2.10–2.20` | `0.4001` / `0.3283` | `0.3999` / `0.3283` | `(15.22, 8.81)` / `(15.22, 6.17)` |
| `17.00 s` | `rotate_move_left` / `right` | **YES** | `2.10–2.15` | `0.2238` / `0.1955` | `0.2237` / `0.1946` | `(15.23, 8.81)` / `(15.23, 6.17)` |
| `17.10 s` | `rotate_move_left` | **NO** (`< 0.15`) | `2.10` | `0.1391` | `0.1390` | `(15.23, 8.81)` |
| **`17.10 s`** | **`curved_fwd_left` / `right`** | **YES** | `1.75` | **`0.2484`** / `0.2204` | **`0.2484`** / `0.2204` | `(15.36, 8.76)` / `(15.45, 6.25)` |
| **`17.15 s`** | **`curved_fwd_left`** | **YES** | `1.75` | **`0.2444`** | **`0.2444`** | `(15.30, 8.78)` |
| `17.15 s` | `reverse_turn_right` | **YES** | `2.35` | `0.1837` | `0.1837` | `(14.66, 6.35)` |

**Key Finding**:
- **Earliest feasible evasion**: Any time in `t = 15.00–17.00 s` supports `rotate_move` (`0.20–0.53 m` clearance), and **even at `t = 17.15 s`**, a curved forward lateral evasion (`curved_fwd_left`, `1.75 s`, reaching `(15.30, 8.78) m`) achieves **`0.2444 m >= 0.15 m`** throughout `DYN-SUDDEN-1`'s entire reversal!
- Static obstacle and boundary clearances remained `>= 0.50 m` in all feasible maneuvers (`min_static_clearance_m = min_inst` was bounded only by `DYN-SUDDEN-1`).

---

## 3. Obstacle Future Occupancy & Runtime-Observable Information

- **Ground Truth vs. Runtime Observable**:
  - `DYN-SUDDEN-1` (`r = 0.45 m`) occupies `[x_obs - 0.80 - 0.15, x_obs + 0.80 + 0.15]` along `y = 7.50 ± 0.95 m` (`y ∈ [6.55, 8.45] m`) during its reversal (`t = 18.00–22.00 s`). Any pose with `|y - 7.50| >= 1.20 m` (`y >= 8.70 m` or `y <= 6.30 m`) remains `>= 0.40 m` clear of `DYN-SUDDEN-1` throughout its reversal without needing hidden future schedules.
  - At runtime, `evaluate_waiting_zone_safety()` already computes `waiting_state = GATE_CLEAR_HOLD_UNSAFE` over `waiting_horizon_s = 2.5 s` using `DynamicObstacle.predict_positions()`.

---

## 4. Candidate Strategy Comparison

| Strategy | Feasibility | Safety (`>= 0.15 m`) | Timing Impact | Implementation Complexity & Verdict |
| :--- | :---: | :---: | :--- | :--- |
| **A. Early curved lateral evasion / `Priority 4` (`select_safe_holding_pose`) before `Priority 2` `p2_cf` when `GATE_CLEAR_HOLD_UNSAFE` and `safe_rev == []`** | **High** (`0.2444 m` at `17.15 s`) | High | **Low** (only runs on `best_cmd is None` fallback) | **Low–Medium** — **Recommended (Primary)** |
| **B. Rotate-and-move lateral maneuver** | High up to `17.00 s`, fails at `17.10 s` (`0.139 m`) | Moderate | Low | Medium — slower than curved forward turn |
| **C. Global A* replanning around stationary obstacle (`t = 16.00 s`)** | High if triggered early (`16.00 s`) | High | High (`~15–30 ms` per replan) | Medium — requires triggering replan while `safe_fwd > 0`; risks detouring in `HEAD-ON` / `CORRIDOR` |
| **D. Combined `Priority 4` lateral holding-pose + curved DWA steering** | **Highest** | **Highest** | **Low** (`< 1 ms` on fallback) | **Recommended for Stage 2.4** |

---

## 5. Regression Risk Analysis (Other 7 Scenarios)

- **`HEAD-ON`, `MULTIPLE-DYNAMIC-OBSTACLES`, `CROSSING`, `CORRIDOR`, `STOP-AND-GO`, `NARROW-GATE`, `SUDDEN-STOP-FINITE-DECEL`**:
  - All 7 already pass with `0` collisions and `0` margin violations.
  - Why `Priority 4` (`select_safe_holding_pose`) is safe if **gated properly**:
    - `select_safe_holding_pose()` only returns a command if the lateral holding pose (`±1.4 m`) is **genuinely `>= 0.15 m` safe over the full `2.5 s` waiting horizon** AND the steering command `(v_h, w_h)` is evaluated by `evaluate_trajectory_with_braking()`.
    - If `select_safe_holding_pose()` is also validated with `evaluate_trajectory_with_braking(..., stop_on_margin_violation=False)` (requiring `min_c >= safety_margin_m` or `> start_clear` and `> 0.0`), it will **only** activate when a genuinely safe lateral escape exists (`SUDDEN-STOP` at `t = 17.15 s`), and will **never** trigger false-positive stops or collisions in `CORRIDOR` / `NARROW-GATE` (where side walls block `±1.4 m` lateral poses) or `HEAD-ON` (where `p2_cf` handles active moving obstacles).

---

## 6. Recommended Stage 2.4 Scope
1. Inside `DWAPlanner.compute_command()` fallback (`best_cmd is None`), when `waiting_state == GATE_CLEAR_HOLD_UNSAFE` and `safe_rev` is empty:
   - Check `select_safe_holding_pose()` **before** `Priority 2` (`p2_cf`), validating the holding-pose command through `evaluate_trajectory_with_braking()` so it is strictly collision-free and margin-safe (`>= 0.15 m`) over the prediction + braking horizon, and only activates when `best_cmd is None` (zero impact on `t = 12.00 s` or normal DWA steps).
2. Re-run full `pytest` (all 132 tests) and all 8 benchmark scenarios (3× determinism + 20 Hz timing) to verify `8/8` `GOAL_REACHED` with `0` collisions and `0` margin violations.
