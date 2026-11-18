# Phase 6.4 — Stage 2.1: `SUDDEN-STOP` Predictive Recovery Investigation

**Scope**: Read-only root cause analysis of the `SUDDEN-STOP` collision (`t = 18.15 s`, `min_inst = min_swept = -0.029184070 m`, `COLLISION_ABORT`) introduced when Phase 6.4 Stage 2 resolved the earlier `t = 6.00 s` and `t = 12.00 s` margin violations. No source code, configuration, thresholds, or benchmark files were modified.

Full step-by-step telemetry (`t = 10.00 s` to `t = 18.15 s`) is saved in [phase642_sudden_stop_investigation.json](file:///c:/Users/Legolas/Desktop/DYNAMIC-NAV/analysis/results/phase642_sudden_stop_investigation.json).

---

## 1. Failure Timeline (`t = 10.00 s` → `t = 18.15 s`)

| Time (s) | Robot `(x, y, v, ω)` | Obstacle `(x, vx)` | Inst Clear (m) | Pred Clear (m) | Selected Cmd `(v, ω)` | Fallback / Wait State | Available Safe (`>= 0.15 m`) |
| ---: | :--- | :--- | ---: | ---: | :--- | :--- | :--- |
| `10.00` | `(12.068, 7.487, 0.20, 0.00)` | `(13.500, 0.00)` | `0.63178` | `0.37662` | `(0.20, 0.00)` | `None` / `GATE_CLEAR_HOLD_SAFE` | `fwd=110`, `rev=34` |
| `12.00` | `(12.549, 7.485, 0.25, 0.00)` | `(13.500, 1.20)` | `0.15155` | `0.15155` | `(0.25, 0.00)` | `None` / `GATE_CLEAR_HOLD_UNSAFE` | `fwd=68`, `rev=17` |
| `15.00` | `(14.863, 7.489, 0.20, 0.00)` | `(16.500, 0.00)` | `0.83667` | `0.58088` | `(0.20, 0.00)` | `None` / `GATE_CLEAR_HOLD_SAFE` | `fwd=119`, `rev=34` |
| `16.00` | `(15.056, 7.490, 0.15, 0.00)` | `(16.500, 0.00)` | `0.64403` | `0.45191` | `(0.15, 0.00)` | `None` / `GATE_CLEAR_HOLD_UNSAFE` | `fwd=103`, `rev=17` |
| `16.80` | `(15.208, 7.491, 0.22, 0.70)` | `(16.500, 0.00)` | `0.49229` | `0.15046` | `(0.22, 0.70)` | `None` / `GATE_CLEAR_HOLD_UNSAFE` | `fwd=55`, `rev=17` |
| `17.00` | `(15.216, 7.491, 0.00, 0.00)` | `(16.500, 0.00)` | `0.48453` | `0.30453` | `(0.00, 0.00)` | `None` / `GATE_CLEAR_HOLD_UNSAFE` | `fwd=17`, `rev=51` |
| `17.10` | `(15.216, 7.491, 0.00, 0.00)` | `(16.500, 0.00)` | `0.48453` | `0.18454` | `(0.00, 0.00)` | `None` / `GATE_CLEAR_HOLD_UNSAFE` | `fwd=17`, `rev=42` |
| **`17.15`** | `(15.228, 7.492, 0.25, 0.70)` | `(16.500, 0.00)` | `0.47206` | `0.14399` | `(0.25, 0.70)` | **`P2 SAFETY_MARGIN_VIOLATION`** / `HOLD_UNSAFE` | **`fwd=0`, `rev=33` (`best_rev=0.27554 m`)** |
| `17.40` | `(15.389, 7.516, 0.50, 0.30)` | `(16.500, 0.00)` | `0.31143` | `0.14180` | `(0.50, 0.30)` | `P2 SAFETY_MARGIN_VIOLATION` | `fwd=0`, `rev=0` (`v_min=0.25 > 0`) |
| `17.60` | `(15.479, 7.533, 0.58, 1.10)` | `(16.500, 0.00)` | `0.22196` | `0.14098` | `(0.58, 1.10)` | `P2 SAFETY_MARGIN_VIOLATION` | `fwd=0`, `rev=0` |
| `17.80` | `(15.544, 7.560, 0.08, 2.60)` | `(16.500, 0.00)` | `0.15778` | `0.14997` | `(0.08, 2.60)` | `P2 SAFETY_MARGIN_VIOLATION` | `fwd=0`, `rev=0` |
| `18.00` | `(15.552, 7.568, 0.00, 2.00)` | `(16.500, 1.20)` | `0.15025` | `0.09042` | `(0.00, 2.00)` | `P2 SAFETY_MARGIN_VIOLATION` | `fwd=0`, `rev=0` (`rev_cf=51`) |
| `18.05` | `(15.552, 7.568, 0.00, 1.20)` | `(16.440,-1.20)` | `0.09042` | `0.03060` | `(0.00, 1.20)` | `P2 SAFETY_MARGIN_VIOLATION` | `fwd=0`, `rev=0` (`rev_cf=51`) |
| `18.10` | `(15.552, 7.568, 0.00, 2.00)` | `(16.380,-1.20)` | `0.03060` | `-0.02918` | `(0.00, 2.00)` | `P5 PHYSICAL_COLLISION_RISK` | `fwd=0`, `rev=0` (`rev_cf=0`) |
| **`18.15`** | `(15.552, 7.568, 0.00, 2.80)` | `(16.320,-1.20)` | **`-0.02918`** | `-0.02918` | `(0.00, 2.80)` | `P5 PHYSICAL_COLLISION_RISK` | **COLLISION_ABORT** |

*(Deadlock/progress timers remained below timeout until `t = 18.10 s`, with `deadlock_state = NONE` during `t = 17.15–18.05 s`.)*

---

## 2. Early Avoidance & Why the Robot Remained on `y = 7.50 m`

- **At `t = 12.00 s`**: `DYN-SUDDEN-1` resumed moving at `+1.2 m/s`, and because Stage 2 resolved the `t = 12.00 s` `0.138989 m` margin violation (`c = 0.15155 m >= 0.15 m`), no evasion turn (`ω = -0.80 rad/s`) was triggered, so the robot stayed aligned on the A* centerline (`y ≈ 7.49 m`).
- **Between `t = 15.00 s` and `t = 17.10 s`**: `DYN-SUDDEN-1` was stationary at `(16.500, 7.500)`. Normal DWA (`prio = None`) crept forward along `y = 7.49 m` and braked cleanly to `v = 0.00 m/s` at `t = 16.90–17.10 s` (`c = 0.48453 m`).
- **Critical decision point at `t = 17.15 s` (`0.85 s` before reversal)**:
  - `best_cmd` became `None` (`fwd_safe = 0`), while `waiting_state = GATE_CLEAR_HOLD_UNSAFE`.
  - **At `t = 17.15 s`, `33` margin-safe (`>= 0.15 m`) reverse trajectories existed** (`best_rev = (v = -0.25 m/s, ω = 0.00 rad/s)` with `min_c = 0.27554 m`), whereas the best forward trajectory was margin-violating (`best_fwd = (v = +0.25, ω = +0.70)` with `min_c = 0.14399 m < 0.15 m`).
  - However, `Priority 2` (`p2_cf`, `SAFETY_MARGIN_VIOLATION`) executed **before** `Priority 3` (`REVERSE_RECOVERY`) and selected `v = +0.25 m/s` forward! On the next step (`t = 17.20 s`), because `v = +0.25 m/s` and `max_accel * dt = 0.25 m/s`, the reverse window (`v_min = 0.00 m/s`) immediately lost **all** reverse capability (`best_rev = None`), and DWA accelerated forward up to `v = 0.92 m/s` into the trap.

---

## 3. Obstacle Reversal Analysis (`-1.2 m/s` vs `-0.6 m/s`)

The collision is **NOT** primarily caused by the `-0.6 m/s` reverse speed limit or incorrect obstacle prediction:
1. **Rank 1 Root Cause — Fallback Priority Ordering (`Priority 2` margin-violating forward beats `Priority 3` margin-safe reverse)**:
   - At `t = 17.15 s`, `Priority 2` (`p2_cf`, `min_clear = 0.14399 m < 0.15 m`) pre-empted `Priority 3` (`REVERSE_RECOVERY`, which had `33` candidates with `min_clear >= 0.15 m`, up to `0.27554 m`).
2. **Rank 2 Root Cause — Fallback Early-Return Masking Collisions (`t = 18.00–18.05 s`)**:
   - In `evaluate_trajectory_with_braking()`, `elif horizon_safe and step_idx >= 2: return (False, min_clearance, None, None)` stopped evaluating at `t = 0.10 s` (`min_c = 0.0904 > 0.0`), causing all `119` colliding forward trajectories at `t = 18.00–18.05 s` to be misclassified as `fwd_cf = 119` (`collision_free`) instead of triggering `Priority 3` (`rev_cf = 51`) or `Priority 5`.
3. **Rank 3 Root Cause — Reverse Speed Asymmetry Once Trapped (`t >= 17.40 s`)**:
   - Only *after* `Priority 2` drove forward from `c = 0.4845 m` to `c < 0.31 m` (`t >= 17.40 s`) did `rev_safe` drop to `0` due to `-0.6 m/s < |-1.2 m/s|`.

---

## 4. Evaluated Recovery Policies (No Implementation)

| Policy Alternative | Addresses Rank | Feasibility & Side Effects | Computational Cost | Verdict |
| :--- | :---: | :--- | :--- | :--- |
| **A. Prefer margin-safe (`r_safe == True`, `>= 0.15 m`) Priority 3 reverse over margin-violating (`< 0.15 m`) Priority 2 `p2_cf` when `GATE_CLEAR_HOLD_UNSAFE`** | **Rank 1** | Directly selects `best_rev = (-0.25, 0.0)` (`0.2755 m`) at `t = 17.15 s`. Crucially, if **no** margin-safe reverse exists (`rev_safe == 0`, as in `HEAD-ON` and `MULTIPLE-DYNAMIC-OBSTACLES`), it falls back to `p2_cf` forward evasion — **preserving all 7 other scenarios!** | **Negligible** (only runs on rare `best_cmd is None` fallback steps when `GATE_CLEAR_HOLD_UNSAFE`) | **Recommended (Primary)** |
| **B. Full-horizon collision check (`<= 0.0`) inside fallback re-evaluation only (`Priority 2` / `Priority 5` filter)** | **Rank 2** | Prevents `fwd_cf = 119` false positives when `t = 0.10 s` is `0.09 m` and `t = 0.20 s` is `-0.03 m`. Must NOT run in normal loop or post-horizon straight braking (`task-1444` showed post-horizon straight braking causes false rejections in `HEAD-ON`). | **Low** (only on fallback steps) | **Recommended (Secondary)** |
| **C. Skip `p2_cf` unconditionally for `GATE_CLEAR_HOLD_UNSAFE`** | Rank 1 | Already tested (`task-1434`): **broke `HEAD-ON` and `MULTIPLE-DYNAMIC-OBSTACLES`** by forcing unsafe reverse when `rev_safe == 0`. | Low | **Rejected** |
| **D. Increase `min_reverse_speed_mps` (`-0.6 -> -1.2 m/s`)** | Rank 3 | Would violate robot physical config limits and is unnecessary if `t = 17.15 s` reverses early. | Zero | **Not needed** |
| **E. Adaptive global replanning on `GATE_CLEAR_HOLD_UNSAFE`** | Indirect | `1.5 s` cooldown is too slow for a `0.85 s` reversal window; A* still routes along `y = 7.50`. | High | **Insufficient alone** |

---

## 5. Regression Protection Analysis
- **`t = 12.00 s` (`SUDDEN-STOP`)**: At `t = 12.00 s`, `best_cmd` is NOT `None` (`fwd_safe = 68`, `prio = None`, `c = 0.15155 m >= 0.15 m`). Policy A only executes inside the `best_cmd is None` fallback branch, so `t = 12.00 s` is **100% unaffected**.
- **Other 7 scenarios (`HEAD-ON`, `CROSSING`, `CORRIDOR`, `STOP-AND-GO`, `NARROW-GATE`, `MULTIPLE-DYNAMIC-OBSTACLES`, `SUDDEN-STOP-FINITE-DECEL`)**: All 7 already achieve `0` collisions and `0` margin violations (`mv = 0/0`). Because Policy A only prefers reverse when `r_safe == True` (`>= 0.15 m`) over a `< 0.15 m` forward fallback, `HEAD-ON` and `MULTIPLE-DYNAMIC-OBSTACLES` (where `rev_safe == 0`) retain their exact `p2_cf` forward behavior.

---

## 6. Recommended Intervention
Implement **Policy A** (only inside `best_cmd is None` fallback when `waiting_assessment.state == WaitingZoneState.GATE_CLEAR_HOLD_UNSAFE`: check Priority 3 reverse for a **margin-safe** `r_safe == True` trajectory *before* accepting a margin-violating `p2_cf` `< 0.15 m` forward trajectory; if `rev_safe == 0`, keep `p2_cf`), optionally paired with **Policy B** (horizon-only `<= 0.0` verification on `p2_cf` candidates).
