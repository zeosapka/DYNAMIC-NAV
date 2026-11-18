# Phase 6.5.5.4 — Gate Exit Tracking & Safe Separation Diagnostic Report

**Mode**: Strictly READ-ONLY Diagnosis (zero source, config, test, or historical artifact modifications).

## 1. Observed Facts (Deterministic Reproduction)

Exported to `analysis/results/phase6554_failure_reproduction.json` and `analysis/results/phase6554_trajectory_analysis.json`:

| Variant | Gate Entry (s) | Gate Exit (s) | Collision Time (s) | Collision Pair | Min Inst Clearance (m) | Min Swept Clearance (m) |
| :--- | :---: | :---: | :---: | :--- | :---: | :---: |
| `NARROW-GATE__L1-RD-01__01` | 13.05 | 15.40 | **20.85** | `ROBOT` vs `DYN-GATE-1` | `-0.021721` | `-0.021721` |
| `NARROW-GATE__L1-SG-01__01` | 13.05 | 15.50 | **21.30** | `ROBOT` vs `DYN-GATE-1` | `-0.009610` | `-0.009610` |

## 2. Ground-Truth Evaluator Findings vs. Causal Controller Knowledge

- **Gate Standoff & Transit (`t = 0.00–15.50 s`)**: Both variants **successfully waited outside the gate**, entered at `t = 13.05 s`, and **exited the narrow gate cleanly** at `t = 15.40 s` / `15.50 s` into the `15.0 m`-wide open right-hand area (`x in [10.0, 18.0]`).
- **Post-Gate Tailgating (`t = 15.40–20.60 s`)**:
  - *Controller Knowledge*: Robot observes `DYN-GATE-1` moving at `+0.80 m/s` along `y = 7.50 m` (`obs_age = 0.0 s`, `est_vx = +0.80 m/s`). Because `CorridorPassingManager` only triggers when lateral walls are within `2.0 m` (false in the `15 m` open area), and DWA's `0.83 m/s` straight-ahead trajectory satisfies `>= 0.15 m` clearance (`~0.27 m` swept clearance), the robot tailgates `DYN-GATE-1` at `0.83 m/s` for over `5.0 s`.
  - *Evaluator Ground Truth*: At `t = 20.65 s`, `DYN-GATE-1` hits its hidden `waypoint_b_m` (`15.50 m`) and instantaneously reverses to `-0.80 m/s` directly in front of the `0.83 m/s` robot.
- **Collision (`t = 20.65–21.30 s`)**: Causal observation detects the reversal (`0.0 -> -0.80 m/s` within `0.05–0.10 s`); the robot brakes toward `0.0 m/s`, but `DYN-GATE-1` (`-0.80 m/s`) contacts the robot at `t = 20.85 s` / `21.30 s`.

## 3. Physical Feasibility Conclusions

Exported to `analysis/results/phase6554_causal_feasibility.json`:
- **Post-reversal (`t >= 20.65 s`)**: **Physically infeasible** to avoid causally from `0.27 m` clearance.
- **Pre-reversal (`t = 15.40–20.60 s`)**: **100% causally feasible** — the robot spends `> 5.1 s` in `15 m`-wide open space behind a `0.80 m/s` obstacle and can safely overtake laterally or maintain velocity-dependent standoff without any hidden future info.

## 4. Proposed Corrective Actions & Recommended Next Phase

Exported to `analysis/results/phase6554_improvement_proposals.json`:
1. **`P6554-01` (Primary)**: Generalize `CorridorPassingManager` (with hysteresis and full DWA+braking validation) to trigger proactive lateral overtaking of slower lead obstacles in wide/open post-gate areas (`min_wall_dist > 2.0 m`).
2. **`P6554-02` (Secondary)**: Add causal velocity-dependent following standoff preference in DWA scoring/clearance so the robot does not tailgate at `0.83 m/s` with `0.17 m` buffer.
- **Recommended Next Implementation Phase**: Implement `P6554-01` (Open-Space Proactive Lead-Obstacle Overtaking) and verify on all 14 `NARROW-GATE` variants + the 8 baseline scenarios before proceeding to Level 2 robustness testing.
