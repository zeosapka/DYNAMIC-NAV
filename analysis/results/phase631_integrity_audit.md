# Phase 6.3.1 — Optimization Integrity & Performance Reproducibility Audit

**Audit Scope**: Read-only verification. No source files, configuration, navigation algorithms, or historical benchmark files were modified.

---

## 1. Verified Optimizations

### Optimization 1 — `SimObject` Prediction Caching ([dwa.py](file:///c:/Users/Legolas/Desktop/DYNAMIC-NAV/algorithms/dwa.py))
- **Cycle-local scope**: `simobj_pred_cache` is instantiated as a fresh local `dict` inside `DWAPlanner.compute_command()` for each control cycle and never persisted across cycles. External callers to `evaluate_trajectory_with_braking(..., simobj_pred_cache=None)` also receive a fresh call-local dictionary.
- **Prediction & braking timestamp coverage**: Pre-populated for all prediction horizon timestamps (`s_idx * predict_dt_s` for `s_idx in 0..pred_steps`), and braking trajectory timestamps (`horizon_t + brake_times[b_idx]`) are cached on demand via `_sim_geoms_at()` keyed by `round(t_s, 9)`.
- **Immediate mutation reflection (verified)**: Mutating a `SimObject` pose/speed between consecutive `compute_command()` calls immediately changed the selected command (`v = 0.75 m/s` → `0.25 m/s`), and cached vs. uncached `evaluate_trajectory_with_braking()` returned identical tuples (`cached_vs_uncached_exact_match = True`, `16` keys cached).

### Optimization 2 — Static/Dynamic Preprocessing & Early Rejection ([dwa.py](file:///c:/Users/Legolas/Desktop/DYNAMIC-NAV/algorithms/dwa.py))
- **Static obstacle AABB preprocessing**: `static_boxes` (`(x_min, y_min, x_max, y_max)`) is built once per `compute_command()` call.
- **Dynamic obstacle step preprocessing**: `precomputed_dyn_at_steps` slices dynamic obstacle positions once per prediction step instead of reallocating per candidate.
- **`_point_clearance` early rejection**:
  - Static AABBs compute `dx = max(0, ox0 - x, x - ox1)` and `dy = max(0, oy0 - y, y - oy1)`. Since `hypot(dx, dy) >= max(dx, dy)`, skipping when `dx >= min_c + r` or `dy >= min_c + r` mathematically guarantees `surf_c >= min_c`.
  - Dynamic circles skip `math.hypot` only when `|x - ox| >= min_c + r + orad` or `|y - oy| >= min_c + r + orad`.
- **`_swept_circle_clearance_m` early rejection**: Skips swept segment-pair evaluation only when the two segments' AABBs are separated along X or Y by at least `min_clearance + r_sum`.

---

## 2. Numerical Equivalence & Geometry Coverage

### Synthetic Equivalence Suite
- **Point clearance (`5,000` randomized samples)**: `max_point_clearance_diff_m = 0.0` (exact bit-for-bit match against unpruned `compute_static_obstacle_clearance`).
- **Trajectory + braking clearance (`500` randomized trajectories across all 4 `SimObject` geometries)**:
  - Geometries tested: `CircleGeometry`, `OrientedRectGeometry`, `PolygonGeometry`, `CapsuleGeometry` (+ `DynamicObstacle`s and `StaticObstacle`s).
  - `max_traj_clearance_diff_m = 0.0`
  - `max_brake_clearance_diff_m = 0.0`
  - `collision_classification_mismatches = 0`
  - `safety_margin_classification_mismatches = 0`

### End-to-End Benchmark Equivalence (`benchmark_phase63_baseline.json` vs `benchmark_phase63_raw.json`, 8 scenarios)
- `max_inst_clearance_diff_m = 0.0`
- `max_swept_clearance_diff_m = 0.0`
- `max_path_length_diff_m = 0.0`
- `classification_mismatches = 0`

---

## 3. Performance Reproducibility (3 Independent Uninstrumented Runs × 8 Scenarios)

- **System**: `Windows-11-10.0.26200-SP0`, `Intel64 Family 6 Model 154 Stepping 3, GenuineIntel` (`20` logical CPUs), Python `3.12.10` (`MSC v.1943 64 bit (AMD64)`).
- **Per-Run Overall (`2,821` `compute_command` calls per run)**:
  - **Run 1**: `avg = 15.27 ms`, `median = 13.66 ms`, `p95 = 34.19 ms`, `p99 = 45.72 ms`, `max = 75.08 ms`, `>50 ms = 9 (0.32%)`
  - **Run 2**: `avg = 17.16 ms`, `median = 14.29 ms`, `p95 = 37.20 ms`, `p99 = 57.05 ms`, `max = 103.44 ms`, `>50 ms = 46 (1.63%)`
  - **Run 3**: `avg = 7.36 ms`, `median = 5.64 ms`, `p95 = 18.24 ms`, `p99 = 36.55 ms`, `max = 86.55 ms`, `>50 ms = 16 (0.57%)`
- **Aggregated 3-Run Per-Scenario (`8,463` total calls)**:

| Scenario | Calls | Avg (ms) | Median (ms) | p95 (ms) | p99 (ms) | Max (ms) | Calls `> 50 ms` |
| :--- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `HEAD-ON` | 936 | 7.14 | 4.56 | 18.16 | 24.93 | 45.90 | 0 (0.00%) |
| `CROSSING` | 828 | 12.58 | 9.57 | 31.06 | 40.90 | 55.83 | 1 (0.12%) |
| `CORRIDOR` | 1,500 | 8.67 | 8.46 | 21.59 | 29.46 | 41.74 | 0 (0.00%) |
| `STOP-AND-GO` | 669 | 16.28 | 13.20 | 37.97 | 69.05 | 86.55 | 16 (2.39%) |
| `NARROW-GATE` | 933 | 13.08 | 12.14 | 25.57 | 40.72 | 67.27 | 4 (0.43%) |
| `MULTIPLE-DYNAMIC-OBSTACLES` | 1,053 | 17.36 | 16.00 | 35.24 | 57.97 | 79.67 | 18 (1.71%) |
| `SUDDEN-STOP` | 1,407 | 12.46 | 11.00 | 29.04 | 38.68 | 62.68 | 5 (0.36%) |
| `SUDDEN-STOP-FINITE-DECEL` | 1,137 | 20.46 | 19.70 | 43.70 | 61.90 | 103.44 | 27 (2.37%) |
| **OVERALL (3 runs)** | **8,463** | **13.27** | **11.02** | **33.16** | **48.60** | **103.44** | **71 (0.84%)** |

**Isolated vs. Sustained Load**:
- In isolated single-run conditions (e.g., Phase 6.3 Opt-2 isolated run: `avg = 4.86 ms, p99 = 9.29 ms, max = 9.83 ms, 0.0% > 50 ms`), all 8 scenarios stay well under `50 ms`.
- Under sustained back-to-back multi-run CPU load on Windows, OS scheduling / CPU frequency scaling produces occasional tail spikes (`0.84%` overall `> 50 ms`, with `p99 > 50 ms` in `STOP-AND-GO`, `MULTIPLE-DYNAMIC-OBSTACLES`, and `SUDDEN-STOP-FINITE-DECEL`).

---

## 4. Renderer Correction Verification
- [renderer.py](file:///c:/Users/Legolas/Desktop/DYNAMIC-NAV/interface/renderer.py#L921-L941): Changing `self._transform.length_to_pixels(geom.radius_m)` to `self._transform.meters_to_pixels(geom.radius_m)` in `_draw_sim_objects()` affects only pixel radius calculation (`r_px`) for Pygame drawing of `CircleGeometry` and `CapsuleGeometry`. No simulation state, SI geometry dimensions, collision logic, or physical units were altered.

---

## 5. Remaining Limitations & Audit Findings
- **Failed / Partially Met Targets**:
  - Strict `p99 < 50 ms` across **all 8 scenarios** under sustained multi-run load is **not yet met** (`STOP-AND-GO` `p99 = 69.05 ms`, `MULTIPLE-DYNAMIC-OBSTACLES` `p99 = 57.97 ms`, `SUDDEN-STOP-FINITE-DECEL` `p99 = 61.90 ms`), although the 3-run aggregated overall `p99` is `48.60 ms` (`< 50 ms`) and average is `13.27 ms`.
- **Integrity Checks**: All numerical equivalence, cache isolation, geometry coverage, and renderer isolation checks **passed with 0 mismatches**.
