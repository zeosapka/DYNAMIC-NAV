# Phase 8.0-D — Root Cause Analysis & Ranked Improvement Plan

**Branch:** `hardening/pre-submission-2026-11-19`  
**Base Commit:** `cf348a3`  
**Scope:** Causal algorithmic hardening of DYNAMIC-NAV (`algorithms/dwa.py`, `config.py`) without altering scenario geometry, obstacle sizes, or safety thresholds (`0.15 m`).

---

## 1. Executive Summary: Historical Baseline vs Current `HEAD` (`cf348a3`)

1. **Historical Phase 6.5.5 Baseline (`analysis/results/phase655_level1_failure_analysis.json`)**:
   - `25/120` unique Level 1 variants (`75/360` runs) failed: `18` collisions and `7` margin-only violations.
2. **Current `HEAD` (`cf348a3`) Reproduction with Canonical Closed-Loop Harness (`sim_objects` + `replan_global_path_around_obstacles`)**:
   - `20/25` historical failures now pass (`NARROW-GATE` `8/8`, `SUDDEN-STOP` `1/1`, `SUDDEN-STOP-FINITE-DECEL` `14/14`, plus `CORRIDOR__L1-IC-01__02`, `CORRIDOR__L1-IC-02__01/02`, `CORRIDOR__L1-IC-04__01`, `CORRIDOR__L1-RD-02__01`, `MULTIPLE-DYNAMIC-OBSTACLES__L1-DO-01__02`).
   - Across all `120` unique Level 1 variants (`360` runs = `120 × 3` repetitions):
     - **Determinism:** `0` non-deterministic variants.
     - **Passing:** `114/120` unique variants (`342/360` runs, `95.0%`).
     - **Still Failing (`6/120` unique variants, `18/360` runs):** `4` collisions + `2` margin-only violations.

---

## 2. The 6 Remaining Unsafe Variants on `cf348a3`

| Variant ID | Perturbation | Outcome | `col` | `mviol` | `min_swept_m` | Observed Failure Mechanism |
|---|---|---|---:|---:|---:|---|
| `CORRIDOR__L1-IC-05__01` | `robot_init_v_mps = 1.0` | `COLLISION_ABORT` | 1 | 1 | `-0.021133` | Robot overtakes at `v=1.58..1.67 m/s` (`swept=0.155..0.165 m`); `DYN-CORR-1` reverses at `x=14.40` (`t=17.20`, `vx: +1.0 -> -1.0`); closing speed `2.67 m/s` exceeds braking distance (`0.36 m`). |
| `CORRIDOR__L1-DO-01__02` | `obs_speed_scale = 1.15` | `COLLISION_ABORT` | 1 | 1 | `-0.036039` | Robot steers toward centerline (`w=+0.84..+2.14`) at `v=1.33..1.67 m/s` behind `DYN-CORR-1` (`swept=0.3135 m`); `DYN-CORR-1` reverses at `t=15.00` (`vx: +1.15 -> -1.15`); collision at `t=15.15`. |
| `CORRIDOR__L1-SG-01__01` | `gate_or_corridor_width_delta_m = -0.2` | `COLLISION_ABORT` | 1 | 1 | `-0.038844` | `DYN-CORR-1` reverses at `t=17.20` (`swept=0.6750 m`). At `t=17.30`, `SAFETY_MARGIN_VIOLATION` fallback **accelerates forward** (`v=1.00 -> 1.17 m/s`). At `t=17.55`, robot stops (`v=0.00`, `swept=0.1111 m`), never triggers reverse recovery, and is struck at `t=17.70`. |
| `MULTIPLE-DYNAMIC-OBSTACLES__L1-RD-01__01` | `max_linear_speed_mps = 2.2` | `COLLISION_ABORT` | 1 | 1 | `-0.055516` | `DYN-MULTI-1` reverses at `t=10.95`. At `t=11.10`, `SAFETY_MARGIN_VIOLATION` fallback **accelerates forward** (`v=1.70 -> 1.95 m/s`), exhausting stopping margin before collision at `t=11.45`. |
| `STOP-AND-GO__L1-DO-01__01` | `obs_speed_scale = 0.85` | `GOAL_REACHED` | 0 | 1 | `0.148023` | `DYN-SG-1` crosses `y=7.5` (`vy=1.19 m/s`) and enters stop phase at `y=7.76`. Constant-velocity DWA approaches within `0.298 m` at `v=0.92 m/s`, dipping `2 mm` below `0.15 m` when obstacle stops. |
| `STOP-AND-GO__L1-DO-03__01` | `move_duration_offset_s = -0.3` | `GOAL_REACHED` | 0 | 1 | `0.093101` | `DYN-SG-1` stops at `y=7.76` while robot approaches at `v=0.83..1.33 m/s`; during `SAFETY_MARGIN_VIOLATION` fallback (`t=5.40..5.45`), robot **accelerates forward** (`v=0.05 -> 0.35 m/s`), dropping `swept` to `0.0931 m`. |

---

## 3. Root Causes in `algorithms/dwa.py`

### RC-1: `SAFETY_MARGIN_VIOLATION` Fallback Accelerates Forward Into Dynamic Hazards
In `algorithms/dwa.py` (lines `2326–2336` and `2385–2396`), when no trajectory satisfies `safety_margin_m = 0.15 m`, the `collision_free` fallback ranks candidates by:
```python
max(collision_free, key=lambda c: (round(c[2], 6), -c[0], -abs(c[1])))
```
Because `round(c[2], 6)` (predicted 1.2 s constant-velocity clearance) is the primary key and speed `-c[0]` is only a tie-breaker, any fast-accelerating forward candidate with `1 µm` higher 1.2 s clearance beats braking/decelerating candidates. This directly caused forward acceleration during fallback in `CORRIDOR__L1-SG-01__01` (`1.00 -> 1.17 m/s`), `MULTIPLE-DYNAMIC-OBSTACLES__L1-RD-01__01` (`1.70 -> 1.95 m/s`), and `STOP-AND-GO__L1-DO-03__01` (`0.05 -> 0.35 m/s`).

### RC-2: Reverse Recovery Rejected During Head-On Approach at Standstill
In `algorithms/dwa.py` (`_find_reverse_candidates`, lines `2219–2223`), when `start_clear < safety_margin_m`, reverse candidates are rejected unless `r_min_c >= start_clear - 1e-6 and r_end_c > start_clear + 1e-3`. When an obstacle approaches head-on (`vx = -1.0 m/s`) while the robot is at `v = 0.0 m/s` (`CORRIDOR__L1-SG-01__01`, `t=17.55`), accelerating backward from `0 -> -1.0 m/s` momentarily sees `r_min_c < start_clear` over the first `0.15 s` — even though **standing still collides (`min_clear = 0.0`)** while reversing avoids the collision.

### RC-3: Constant-Velocity-Only Prediction Blindness to Sudden Obstacle Stop / Reversal
`evaluate_trajectory_with_braking` evaluates dynamic obstacles along constant-velocity `p_obs(t) = p_0 + v_0 * t` (plus small linear uncertainty `0.02 m/s * t`). When a moving obstacle suddenly **stops** (`STOP-AND-GO`) or **reverses** (`CORRIDOR`, `MULTIPLE-DYNAMIC-OBSTACLES`), constant-velocity prediction assumes the obstacle vacates its current space, allowing DWA to approach within `0.155–0.31 m` at `1.0–1.67 m/s` — inside the robot's causal stopping distance `d_stop(v) + 0.15 m`.

---

## 4. Ranked Improvement Plan (Phase 8.0-E)

1. **Fix 1 (Priority 1 — Fallback Deceleration Discipline, `algorithms/dwa.py`)**:
   - In `SAFETY_MARGIN_VIOLATION` fallback (`Priority 2` and post-`Priority 4` `collision_free` fallback), strictly filter/prefer non-accelerating candidates (`v <= eff_robot.v_mps + 1e-6`) whenever any decelerating/standstill collision-free candidate exists, and prioritize braking clearance before forward speed.
2. **Fix 2 (Priority 2 — Head-On Emergency Reverse Recovery, `algorithms/dwa.py`)**:
   - When `start_clear < safety_margin_m` and standstill (`v = 0`) is unsafe/colliding within the horizon (`standstill_min_clear <= 0.0` or `standstill_end_clear < start_clear`), allow `_find_reverse_candidates(require_margin_safe=False)` to select collision-free reverse candidates that strictly outperform standstill (`r_min_c > standstill_min_clear` and `r_end_c > standstill_end_clear`).
3. **Fix 3 (Priority 3 — Causal Sudden-Stop & Bounded-Reversal Braking Check, `config.py`, `algorithms/dwa.py`)**:
   - Add a configurable causal worst-case check in `DWAConfig` (`config.py`): along the robot's braking trajectory (`t <= t_brake`) for moving dynamic obstacles, verify clearance against both constant-velocity `p_obs(t)` and a **causal sudden-stop** `p_obs(0)` (and bounded causal reversal/uncertainty envelope within `t_brake`), preventing the robot from tailgating or cutting behind moving obstacles inside its own stopping distance.

---

## 5. Current Hardening Status (`CorridorPassingManager` State Machine & `ABORT_HOLD`)

### 5.1 Implemented Mechanisms (`algorithms/corridor_passing.py`, `algorithms/dwa.py`, `config.py`)
1. **`CorridorPassingManager` State Machine (`PassingState`)**:
   - Separates path acceptance (`IDLE -> PASSING_ARMED`), lateral maneuver engagement (`PASSING_ARMED -> PASSING_ACTIVE`), and blocked-lane holding (`PASSING_ARMED / PASSING_ACTIVE -> ABORT_HOLD`, recovering to `PASSING_ARMED` when a safe lateral path becomes available).
   - Locks the passing axis (`_locked_tangent`, `_locked_origin_m`) at maneuver start so robot rotation does not shift the completion axis.
2. **Multi-Obstacle Tracking (`ActivePassingObstacle`)**:
   - Tracks all threatening dynamic obstacles by ID with last geometry/position/velocity, bounded uncertainty growth on `observation_lost` (`passing_max_lost_uncertainty_m`), and support-function geometric completion (`_geometry_projection_bounds`) across `CircleGeometry`, `OrientedRectGeometry`, `CapsuleGeometry`, and `PolygonGeometry`.
   - Transitions `-> IDLE` only when all tracked obstacles satisfy `geom_passed == True`.
3. **Causal Reversal Cooldown Bypass**:
   - When an active tracked obstacle reverses along the locked axis (`prev_along_v > 0.15 m/s` to `curr_along_v < -0.15 m/s`) while in `PASSING_ARMED` or `PASSING_ACTIVE` and `lateral_passable` is `True`, cooldown is bypassed for replanning/abort evaluation while preserving `PASSING_ACTIVE` and the active obstacle lock.
4. **Isolated `ABORT_HOLD` Hook in `DWAPlanner`**:
   - Skips `score_trajectory` over the stale `self._path_world_m` while `abort_hold_active` is `True`.
   - Selects non-accelerating `is_safe=True` braking candidates (`ABORT_HOLD_BRAKING`) toward `target_brake_v = max(window.v_min_mps, min(0.0, window.v_max_mps))`, skips `Priority 2` forward recovery, falls through to `Priority 3/4` if braking is unavailable, and enforces non-accelerating deceleration toward `target_brake_v` in the post-`Priority 4` `collision_free` fallback.

### 5.2 Recorded Final Test Results (`202 / 202` passed)
- `tests/test_corridor_passing_state_machine.py`: **5 passed**
- `tests/test_phase62_safe_waiting_reverse.py`: **17 passed** (including `CORRIDOR-False`, resolved via hysteresis guard in `PASSING_ARMED`/`PASSING_ACTIVE`)
- `tests/test_phase654_corridor_passing.py`: **9 passed**
- `tests/test_phase6553_narrow_gate_standoff.py`: **7 passed**
- `tests/test_phase80_reproduction.py`: **6 passed** (including `CORRIDOR__L1-DO-01__02`)
- Full suite (`python -m pytest -v`): **202 passed** (`467.93 s`, `0` failed)

### 5.3 `CORRIDOR__L1-DO-01__02` Root Cause & Resolution
- **Root Cause**: During active passing (`PASSING_ACTIVE` on the lower/right lane at `y = 7.09 m`), a replan at `t = 14.40 s` switched sides to `"left"` (above `y = 7.50 m`) solely because `"left"` had higher 1.2 s constant-velocity predicted clearance (`0.524 m` vs `0.334 m`), steering the robot across `y = 7.50 m` right before `d0` reversed at `t = 15.00 s`.
- **Fix (`algorithms/corridor_passing.py`)**: Added `keep_committed_side` priority in `generate_and_validate_candidates` when `self._passing_state in (PASSING_ARMED, PASSING_ACTIVE)` so an already-committed, still-valid (`accepted=True`) passing side is preserved rather than crossing the obstacle's centerline mid-maneuver.
- **Verified `DO-01` Metrics**: `goal_reached=True`, `physical_collision_episode_count=0`, `safety_margin_violation_episode_count=0`, `min_swept_m=0.165947 m`, `min_clearance_m=0.165947 m`, `stop_count=1`, `replan_attempts=0` (`proactive_replans=4`), `completion_time_s=18.5 s`, deterministic (`True`).
