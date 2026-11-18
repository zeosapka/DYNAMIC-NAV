"""Geometric safety validation tests for square-cell obstacle inflation."""

import math
import numpy as np
import pytest

from algorithms.astar import AStarPlanner, inflate_occupancy_grid


def _min_distance_to_occupied_squares_m(
    grid: np.ndarray,
    cand_row: int,
    cand_col: int,
    resolution_m: float,
) -> float:
    """Compute exact minimum Euclidean distance from candidate cell center to any occupied square."""
    half = 0.5 * resolution_m
    cx = (cand_col + 0.5) * resolution_m
    cy = (cand_row + 0.5) * resolution_m

    min_dist = math.inf
    occ_rows, occ_cols = np.where(grid != 0)
    for r, c in zip(occ_rows, occ_cols):
        ox_min = c * resolution_m
        ox_max = ox_min + resolution_m
        oy_min = r * resolution_m
        oy_max = oy_min + resolution_m

        dx = max(ox_min - cx, 0.0, cx - ox_max)
        dy = max(oy_min - cy, 0.0, cy - oy_max)
        d = math.hypot(dx, dy)
        if d < min_dist:
            min_dist = d
    return min_dist


def test_minimum_clearance_around_square_obstacle() -> None:
    res_m = 0.20
    radius_m = 0.35
    margin_m = 0.15
    required_clearance_m = radius_m + margin_m  # 0.50 m

    grid = np.zeros((15, 15), dtype=np.uint8)
    grid[7, 7] = 1

    inflated = inflate_occupancy_grid(grid, res_m, radius_m, margin_m)

    free_rows, free_cols = np.where(inflated == 0)
    for r, c in zip(free_rows, free_cols):
        d = _min_distance_to_occupied_squares_m(grid, int(r), int(c), res_m)
        assert d >= required_clearance_m - 1e-9


def test_diagonal_clearance_blocks_unsafe_corner_cells() -> None:
    res_m = 0.20
    radius_m = 0.35
    margin_m = 0.15  # required = 0.50 m

    grid = np.zeros((11, 11), dtype=np.uint8)
    grid[5, 5] = 1
    inflated = inflate_occupancy_grid(grid, res_m, radius_m, margin_m)

    # Diagonal offset (+2, +2) has center distance 0.5657 m > 0.50 m,
    # but exact distance to square corner is sqrt(0.3^2 + 0.3^2) = 0.4243 m < 0.50 m.
    for dr in (-2, 2):
        for dc in (-2, 2):
            assert inflated[5 + dr, 5 + dc] == 1

    # Cardinal offset (+3, 0) has exact distance 0.60 - 0.10 = 0.50 m >= 0.50 m -> safe.
    assert inflated[5 + 3, 5] == 0
    assert inflated[5 - 3, 5] == 0
    assert inflated[5, 5 + 3] == 0
    assert inflated[5, 5 - 3] == 0


def test_narrow_corridor_accessibility_exact_threshold() -> None:
    res_m = 0.20
    radius_m = 0.35
    margin_m = 0.15  # required = 0.50 m on each side -> 1.00 m total corridor width

    # Feasible: walls at row 2 and row 8 -> 5 free cells (rows 3..7, 1.00 m clear width).
    # Row 5 center is 0.50 m from both wall edges -> feasible!
    feasible = np.zeros((11, 15), dtype=np.uint8)
    feasible[2, :] = 1
    feasible[8, :] = 1
    inf_feasible = inflate_occupancy_grid(feasible, res_m, radius_m, margin_m)
    assert inf_feasible[5, 7] == 0
    res_ok = AStarPlanner(res_m, allow_diagonal=True).plan(inf_feasible, (5, 1), (5, 13))
    assert res_ok.success is True

    # Infeasible: walls at row 2 and row 7 -> 4 free cells (0.80 m clear width < 1.00 m).
    infeasible = np.zeros((11, 15), dtype=np.uint8)
    infeasible[2, :] = 1
    infeasible[7, :] = 1
    inf_blocked = inflate_occupancy_grid(infeasible, res_m, radius_m, margin_m)
    assert np.all(inf_blocked[2:8, :] == 1)
    res_fail = AStarPlanner(res_m, allow_diagonal=True).plan(inf_blocked, (4, 1), (4, 13))
    assert res_fail.success is False


def test_multiple_adjacent_obstacle_cells() -> None:
    res_m = 0.20
    radius_m = 0.35
    margin_m = 0.15
    required_clearance_m = radius_m + margin_m

    grid = np.zeros((15, 15), dtype=np.uint8)
    # L-shaped obstacle cluster
    grid[6:9, 6] = 1
    grid[8, 6:9] = 1

    inflated = inflate_occupancy_grid(grid, res_m, radius_m, margin_m)
    free_rows, free_cols = np.where(inflated == 0)
    for r, c in zip(free_rows, free_cols):
        d = _min_distance_to_occupied_squares_m(grid, int(r), int(c), res_m)
        assert d >= required_clearance_m - 1e-9


def test_grid_boundary_behavior() -> None:
    res_m = 0.20
    grid = np.zeros((10, 10), dtype=np.uint8)
    grid[0, 0] = 1
    grid[9, 9] = 1

    inflated = inflate_occupancy_grid(grid, res_m, robot_radius_m=0.35, safety_margin_m=0.15)
    assert inflated.shape == (10, 10)
    assert inflated[0, 0] == 1
    assert inflated[9, 9] == 1
    assert inflated[2, 2] == 1
    assert inflated[7, 7] == 1


def test_start_and_goal_safety_rejection() -> None:
    res_m = 0.20
    grid = np.zeros((15, 15), dtype=np.uint8)
    grid[7, 7] = 1
    inflated = inflate_occupancy_grid(grid, res_m, robot_radius_m=0.35, safety_margin_m=0.15)

    planner = AStarPlanner(res_m, allow_diagonal=True)
    # Start at unsafe diagonal corner (9, 9)
    res_bad_start = planner.plan(inflated, (9, 9), (1, 1))
    assert res_bad_start.success is False
    assert res_bad_start.failure_reason is not None and "Start" in res_bad_start.failure_reason

    # Goal at unsafe diagonal corner (5, 5)
    res_bad_goal = planner.plan(inflated, (1, 1), (5, 5))
    assert res_bad_goal.success is False
    assert res_bad_goal.failure_reason is not None and "Goal" in res_bad_goal.failure_reason


def test_original_occupancy_grid_immutability() -> None:
    grid = np.zeros((10, 10), dtype=np.uint8)
    grid[4, 4] = 1
    snapshot = grid.copy()

    inflated = inflate_occupancy_grid(grid, 0.20, robot_radius_m=0.35, safety_margin_m=0.15)
    assert inflated is not grid
    assert np.array_equal(grid, snapshot)
