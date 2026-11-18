# Phase 6.5.5.2 — Failure-Driven Algorithm Improvement: Diagnostic Design Report

**Mode**: Strictly READ-ONLY Investigation (zero source, config, test, or historical benchmark modifications).

## 1. Failure Family Diagnostics

Exact step-by-step traces for all 5 representative families are exported in `analysis/results/phase6552_failure_diagnostics.json`.

### 1.1 `NARROW-GATE` (`8` collisions — `57.1%` collision rate)
- **Representative trace (`NARROW-GATE__L1-IC-01__02`)**: At `t = 6.75–6.90 s`, the robot is stationary (`[7.646, 7.260, v=0.0]`) at the gate entrance while `DYN-GATE-1` approaches at `-0.80 m/s` (`8.64 -> 8.36 m`). `waiting_state = GATE_CLEAR_HOLD_UNSAFE`, and **`Priority 2 SAFETY_MARGIN_VIOLATION`** selects `v = 0.0` standstill (`pred_c = 0.1198 m`), blocking `Priority 3/4` evasion until `DYN-GATE-1` collides at `t = 7.10 s` (`swept_c = -0.0342 m`).
- **Origin**: Local planning & fallback priority ordering.
- **Causally feasible?**: **Yes**.

### 1.2 `SUDDEN-STOP` (`6` collisions — `33.3%` collision rate)
- **Representative trace (`SUDDEN-STOP__L1-IC-02__01`)**: At `t = 17.80–18.00 s`, the robot is parked `0.1743 m` (`pred_c = 0.1503 m`) behind stationary `DYN-SUDDEN-1` (`16.50, 7.50, vx=0.0`). At `t = 18.05 s`, `DYN-SUDDEN-1` reverses at `-1.20 m/s`; causal observation detects `-1.20 m/s` at `t = 18.10 s` (`swept_c = 0.0559 m`), and collision occurs at `t = 18.15 s` (`-0.0031 m`).
- **Origin**: Local/global planning (creeping up to `0.15 m` behind a stopped obstacle in a `7.0 m`-wide channel `y in [4, 11]` instead of passing around it).
- **Causally feasible?**: **Partially** — predicting the hidden reversal from `0.17 m` behind is impossible causally, but passing laterally around the stopped obstacle in the `7.0 m` channel *before* parking at `0.15 m` IS causally feasible.

### 1.3 `MULTIPLE-DYNAMIC-OBSTACLES` (`1` collision + `7` margin-only dips across Level 1)
- **Representative trace (`MULTIPLE-DYNAMIC-OBSTACLES__L1-RD-01__01`, `max_linear_speed_mps = 2.2`)**: At `t = 11.15–11.50 s`, the robot slows to `0.0 m/s` at `[5.356, 7.329]` for crossing `DYN-MULTI-2` (`7.50, 7.57 -> 7.11, vy=-1.30`), while `DYN-MULTI-1` (`4.21 -> 4.59, 7.50, vx=+1.10`) approaches from behind and collides at `t = 11.50 s` (`-0.0236 m`).
- **Origin**: Local planning (no rear-approach lateral evasion when stopping for a crossing obstacle).
- **Causally feasible?**: **Yes**.

### 1.4 `CORRIDOR` Heading Perturbations (`2` collisions)
- **Representative trace (`CORRIDOR__L1-IC-02__01`, `robot_start_heading_rad = -0.26`)**: At `t = 16.95–17.25 s`, the robot is cruising diagonally at `[13.582, 7.239, v=1.55]` (`swept_c = 0.154 m` behind `DYN-CORR-1`), not yet in a full lateral passing lane (`y = 6.5` or `8.5`). When `DYN-CORR-1` bounces at `t = 17.25 s`, collision occurs in one step at `t = 17.30 s` (`-0.0206 m`).
- **Origin**: Corridor passing entry alignment / insufficient lateral separation before passing.
- **Causally feasible?**: **Yes**.

### 1.5 `CORRIDOR__L1-SG-01__01` (`1` collision — `-0.20 m` width reduction)
- **Trace**: Corridor width `2.80 m` leaves `1.00 m` side gaps around `DYN-CORR-1` (`r = 0.40 m`), which equals `2 * robot_radius + 2 * safety_margin = 1.00 m` with `0.00 m` slack — infeasible under `0.20 m` discrete A* grid inflation.
- **Origin**: Scenario / geometric grid-resolution boundary.
- **Causally feasible?**: **No** (physically/geometrically borderline-infeasible under `0.20 m` grid).

## 2. Prioritized Implementation Plan & Acceptance Criteria

Exported in `analysis/results/phase6552_improvement_proposals.json`:

1. **Priority 1 — `NARROW-GATE` Fallback & Standstill Guard**: When `GATE_CLEAR_HOLD_UNSAFE` and an obstacle is closing, prevent `v ~ 0` `Priority 2` standstill from blocking `Priority 3/4` evasion; hold outside gate entrance (`x < 7.5 m`) while obstacle is inside/exiting.
   - *Acceptance*: `NARROW-GATE` Level 1 goal rate `42.9% -> >= 90%`, `0.15 m` margin preserved, `0` baseline regressions.
2. **Priority 2 — Wide-Channel Stationary Obstacle Passing (`SUDDEN-STOP`)**: Trigger proactive A* lateral passing around stationary obstacles in wide channels (`y in [4, 11]`) before creeping to `0.15 m` behind them.
   - *Acceptance*: `SUDDEN-STOP` Level 1 goal rate `66.7% -> >= 90%` without hidden schedules.
3. **Priority 3 — `CORRIDOR` Full Lateral Separation**: Require full lateral lane clearance in `CorridorPassingManager` and prevent `1.55 m/s` diagonal tailgating at `< 0.25 m` clearance.
   - *Acceptance*: `CORRIDOR__L1-IC-02__01/02` reach goal with `>= 0.15 m` clearance.
4. **Priority 4 — Rear-Approach Lateral Evasion (`MULTIPLE-DYNAMIC-OBSTACLES`)**: Bias lateral evasion in `Priority 2/4` when an observed obstacle behind the robot is closing along the same axis.
   - *Acceptance*: `MULTIPLE-DYNAMIC-OBSTACLES__L1-RD-01__01` reaches goal collision-free.
5. **Priority 5 — Geometric Infeasibility Classification**: Keep `0.15 m` safety margin untouched; classify `CORRIDOR__L1-SG-01__01` as a known `0.20 m` grid-resolution geometric boundary.
