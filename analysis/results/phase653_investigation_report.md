# Phase 6.5.3 — CORRIDOR Causal Failure Investigation & Regression Audit

**Mode:** Strictly READ-ONLY Investigation (no source, config, test, or historical benchmark files modified).

## 1. Reproduced Failure Details

Verified directly from `analysis/results/benchmark_phase652_raw.json` and `analysis/results/phase653_corridor_failure_timeline.json`:

- **Scenario**: `CORRIDOR` (10.0 m corridor: `x in [5.0, 15.0]`, `y in [6.0, 9.0]`, clear width `3.0 m`; `DYN-CORR-1` radius `0.40 m`, patrolling `waypoint_a_m=(5.6, 7.5)` to `waypoint_b_m=(14.4, 7.5)` at `|vx|=1.0 m/s` with `reverse_at_bounds=True`).
- **Controller**: `ASTAR_DWA` with causal `ObservationPipeline`.
- **Termination**: `COLLISION_ABORT` at `sim_time_s = 17.70 s`.
- **Minimum instantaneous & swept clearance**: `-0.036470307565855564 m`.
- **Robot speed before encounter**: `1.00 m/s` along centerline `y = 7.50 m` (`t = 12.00–17.25 s`), maintaining `0.6777 m` surface clearance behind `DYN-CORR-1`.
- **Obstacle reversal**: At `t = 17.20 s` (ground truth), `DYN-CORR-1` reaches `(14.40, 7.50)` and instantaneously reverses from `+1.0 m/s` to `-1.0 m/s`.
- **Robot stop**: Reaches `0.00 m/s` at `t = 17.55 s` (`x = 13.187, y = 7.480`), `0.15 s` before `DYN-CORR-1` contacts it at `t = 17.70 s`.

## 2. Causal Timeline Summary (Key Events)

Full per-step timeline (`t = 14.00–18.70 s`, 94 steps) exported to `analysis/results/phase653_corridor_failure_timeline.json`:

- `t = 2.00–8.40 s`: Robot waits outside corridor (`x = 3.71–3.92 m`) while `DYN-CORR-1` approaches at `-1.0 m/s` and bounces at `x = 5.60 m`.
- `t = 9.00–17.20 s`: Robot enters corridor behind `DYN-CORR-1` on `y = 7.50 m`, matching its `1.00 m/s` speed (`inst_c = 0.6777 m`).
- `t = 17.20 s`: Ground-truth `DYN-CORR-1` hits `waypoint_b_m=(14.40, 7.50)` and reverses to `vx = -1.0 m/s`.
- `t = 17.25 s`: Pre-step observation sees `obs_x = 14.40` (still `+0.05 m` ahead of `14.35`), so `est_vx = +1.0 m/s`. `inst_c` drops to `0.5777 m`.
- `t = 17.30 s`: Pre-step observation sees `obs_x = 14.35`, `est_vx = -1.0 m/s` (detected on the very first backward displacement, 1 step / `0.05 s` after ground-truth reversal). `waiting_state = GATE_CLEAR_HOLD_UNSAFE`, Priority 2 (`SAFETY_MARGIN_VIOLATION`) commands `(1.17, -0.80)`.
- `t = 17.35–17.55 s`: Robot decelerates at max `-5.0 m/s^2` (`1.17 -> 0.92 -> 0.67 -> 0.42 -> 0.17 -> 0.00 m/s`), stopping at `(13.187, 7.480)`.
- `t = 17.60–17.70 s`: `Priority 5 PHYSICAL_COLLISION_RISK`; `DYN-CORR-1` (`-1.0 m/s`) closes the remaining gap (`0.0635 -> 0.0135 -> -0.0365 m`) and collides with the stationary robot at `t = 17.70 s`.

## 3. Root Cause Classification & Sensor/Estimator Analysis (A–J)

- **Primary Root Cause**: **Removal of Non-Causal Schedule Leakage + Centerline Tailgating Trap**. Previously, `DynamicObstacle.predict_positions()` leaked `waypoint_b_m=(14.4, 7.5)` and `reverse_at_bounds=True`, allowing DWA to foresee the bounce before it happened. Under causal observation, DWA cannot predict an instantaneous kinematic bounce at an unobservable invisible waypoint.
- **A. Observation latency & cadence**: `observation_latency_s = 0.0 s`, cadence `0.05 s`. No artificial delay.
- **B–C. Velocity estimation & smoothing**: `velocity_ema_alpha = 1.0`; estimator tracks the reversal in **1 step (`0.05 s`)** with zero smoothing lag (`+1.0 -> -1.0 m/s` at `t = 17.30 s`).
- **D–E. Uncertainty & braking evaluation**: DWA evaluates the complete causal braking trajectory; uncertainty growth (`0.02 m/s`) cannot prevent a collision once a head-on reversal occurs `1.33 m` ahead along the centerline.
- **F–I. Observability & escape**: At `t = 17.30 s` (`0.40 s` before impact), neither centerline braking (`zero_reaction_time_decel_10` still collides at `-0.0002 m`), reverse recovery (`-0.60 m/s` is slower than `-1.00 m/s`), nor reactive lateral holding (`0.40 s` is insufficient to turn and translate `0.95 m` laterally) can avoid collision.
- **J. Physical feasibility**: The scenario **IS** physically feasible without future info, **not** by tailgating along `y = 7.5 m`, but by using the `1.10 m` lateral passing clearance above (`y = 8.45 m`) or below (`y = 6.55 m`) `DYN-CORR-1` inside the `3.0 m`-wide corridor (`robot diameter + 2*margin = 1.00 m < 1.10 m`).

## 4. Counterfactual Comparison

Full results exported to `analysis/results/phase653_counterfactual_analysis.json`:

| Counterfactual | Physically Feasible | Feasible Causally | Hidden Future Info Only | Observed / Verified Outcome |
| :--- | :---: | :---: | :---: | :--- |
| Earlier braking (on `y=7.5`) | No | No | Yes | `zero_reaction_time_decel_10` still collides (`-0.0002 m`) |
| Lower approach speed (`0.4–0.8 m/s`) | No | No | No | All collide (`COLLISION_ABORT`); obstacle returns before exit |
| Increased margin / uncertainty growth | No | No | No | All (`0.25–0.50 m`, `0.10–0.50 m/s`) collide (`COLLISION_ABORT`) |
| Predictive waiting outside corridor | No | No | No | Already waits `t=2.0–8.4 s`; waiting forever prevents crossing |
| Lateral lane offset (`y=6.55` or `8.45 m`) | **Yes** | **Yes** | No | `1.10 m` gap > `1.00 m` required; must be proactive, not at `t=17.3 s` |
| Reverse recovery (`-0.6 m/s`) | No | No | No | Slower than `-1.0 m/s` obstacle; caught inside 10 m corridor |
| Proactive A* replanning in corridor | **Yes** | **Yes** | No | Verified `replan_global_path_around_obstacles` finds `y=6.5–7.5 m` path |

## 5. Regression Test Audit

Exported to `analysis/results/phase653_regression_audit.json`:

- `tests/test_dwa.py::test_dwa_corridor_scenario_deterministic_audit` and `tests/test_phase62_safe_waiting_reverse.py::test_phase62_integration_scenarios_zero_collisions_and_determinism[CORRIDOR-False]` are classified as **obsolete expectations tied to non-causal schedule leakage + a centerline-following design gap**.
- Zero-collision completion in `CORRIDOR` remains a valid engineering goal, but achieving it causally requires proactive lateral passing/lane-offset planning inside wide corridors rather than centerline tailgating.
- All other **7/8 scenarios** (`HEAD-ON`, `CROSSING`, `NARROW-GATE`, `MULTIPLE-DYNAMIC-OBSTACLES`, `STOP-AND-GO`, `SUDDEN-STOP`, `SUDDEN-STOP-FINITE-DECEL`) pass with 0 collisions and 0 margin violations.

## 6. Recommended Next Engineering Phase

**Phase 6.5.4 — Proactive Lateral Corridor Passing / Replanning Trigger**:
1. Enable proactive A* replanning or DWA lateral lane-offset biasing when following a slower/patrolling dynamic obstacle inside a passable corridor (`width >= robot_diameter + obs_diameter + 4*margin`), rather than waiting for `deadlock_timeout_s = 2.5 s` at `v ~ 0`.
2. Additionally, audit `Priority 2` candidate tie-breaking when `GATE_CLEAR_HOLD_UNSAFE` (`p2_cf` selected `v=1.17` instead of `v=0.75` at `t=17.30 s`).
