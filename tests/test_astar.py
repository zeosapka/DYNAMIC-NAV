"""Comprehensive tests for A* global path planning and obstacle inflation."""

import math
import numpy as np
import pytest

from algorithms.astar import (
    AStarPlanner,
    AStarResult,
    inflate_occupancy_grid,
    manhattan_heuristic_m,
    octile_heuristic_m,
    world_to_grid,
)
from config import DEFAULT_CONFIG
from simulation.world import SimulationWorld


def _assert_valid_path(
    result: AStarResult,
    grid: np.ndarray,
    resolution_m: float,
    allow_diagonal: bool,
) -> None:
    """Verify path connectivity, obstacle-freedom, and exact SI path cost."""
    assert result.success is True
    assert len(result.path_grid) >= 1
    assert len(result.path_grid) == len(result.path_world_m)

    total_cost_m = 0.0
    for idx, (r, c) in enumerate(result.path_grid):
        assert 0 <= r < grid.shape[0]
        assert 0 <= c < grid.shape[1]
        assert grid[r, c] == 0, f"Path cell {(r, c)} crosses an occupied/inflated cell!"

        if idx > 0:
            pr, pc = result.path_grid[idx - 1]
            dr = abs(r - pr)
            dc = abs(c - pc)
            assert max(dr, dc) == 1
            if not allow_diagonal:
                assert dr + dc == 1
                total_cost_m += resolution_m
            else:
                if dr == 1 and dc == 1:
                    # Ensure no diagonal corner cutting occurred.
                    assert grid[r, pc] == 0 and grid[pr, c] == 0
                    total_cost_m += math.sqrt(2.0) * resolution_m
                else:
                    total_cost_m += resolution_m

    assert result.path_cost_m == pytest.approx(total_cost_m, rel=1e-6, abs=1e-6)


def test_default_world_grid_dimensions_100x75() -> None:
    world = SimulationWorld(
        DEFAULT_CONFIG.world, DEFAULT_CONFIG.robot, DEFAULT_CONFIG.simulation
    )
    # 20.0m / 0.2m = 100 cols, 15.0m / 0.2m = 75 rows -> shape (75, 100)
    assert world.occupancy_grid.shape == (75, 100)

    inflated = inflate_occupancy_grid(
        world.occupancy_grid,
        resolution_m=DEFAULT_CONFIG.world.grid_resolution_m,
        robot_radius_m=DEFAULT_CONFIG.robot.radius_m,
        safety_margin_m=DEFAULT_CONFIG.planner.safety_margin_m,
    )
    start = world_to_grid(
        DEFAULT_CONFIG.robot.initial_x_m,
        DEFAULT_CONFIG.robot.initial_y_m,
        DEFAULT_CONFIG.world.grid_resolution_m,
        inflated.shape,
    )
    goal = world_to_grid(
        DEFAULT_CONFIG.planner.default_goal_x_m,
        DEFAULT_CONFIG.planner.default_goal_y_m,
        DEFAULT_CONFIG.world.grid_resolution_m,
        inflated.shape,
    )
    planner = AStarPlanner(DEFAULT_CONFIG.world.grid_resolution_m, allow_diagonal=True)
    res = planner.plan(inflated, start, goal)
    _assert_valid_path(res, inflated, DEFAULT_CONFIG.world.grid_resolution_m, True)


def test_empty_grid_4_and_8_dir_cost_comparison() -> None:
    res_m = 0.2
    grid = np.zeros((20, 20), dtype=np.uint8)
    start, goal = (1, 1), (15, 10)

    p4 = AStarPlanner(res_m, allow_diagonal=False)
    p8 = AStarPlanner(res_m, allow_diagonal=True)

    r4 = p4.plan(grid, start, goal)
    r8 = p8.plan(grid, start, goal)

    _assert_valid_path(r4, grid, res_m, allow_diagonal=False)
    _assert_valid_path(r8, grid, res_m, allow_diagonal=True)

    assert r4.path_cost_m == pytest.approx(manhattan_heuristic_m(start, goal, res_m))
    assert r8.path_cost_m == pytest.approx(octile_heuristic_m(start, goal, res_m))
    assert r8.path_cost_m < r4.path_cost_m


def test_single_obstacle_detour() -> None:
    res_m = 0.2
    grid = np.zeros((25, 25), dtype=np.uint8)
    grid[8:17, 12] = 1
    inflated = inflate_occupancy_grid(grid, res_m, robot_radius_m=0.3, safety_margin_m=0.1)

    planner = AStarPlanner(res_m, allow_diagonal=True)
    res = planner.plan(inflated, (12, 4), (12, 20))
    _assert_valid_path(res, inflated, res_m, allow_diagonal=True)
    assert res.path_cost_m > octile_heuristic_m((12, 4), (12, 20), res_m)


def test_maze_navigation() -> None:
    res_m = 0.2
    grid = np.zeros((15, 15), dtype=np.uint8)
    # Construct a winding maze with 1-cell walls.
    grid[0:12, 3] = 1
    grid[3:15, 7] = 1
    grid[0:12, 11] = 1

    for allow_diag in (False, True):
        planner = AStarPlanner(res_m, allow_diagonal=allow_diag)
        res = planner.plan(grid, (1, 1), (13, 13))
        _assert_valid_path(res, grid, res_m, allow_diagonal=allow_diag)


def test_narrow_corridor_inflation_and_immutability() -> None:
    res_m = 0.2
    # Corridor between row 4 and row 12: free rows are 5..11 (7 cells = 1.4m wide).
    grid = np.zeros((17, 20), dtype=np.uint8)
    grid[4, :] = 1
    grid[12, :] = 1
    original_snapshot = grid.copy()

    # Robot radius 0.35m + margin 0.15m = 0.50m (2 cells each side -> rows 5,6 and 10,11 blocked;
    # rows 7,8,9 remain open!).
    inflated_passable = inflate_occupancy_grid(
        grid, resolution_m=res_m, robot_radius_m=0.35, safety_margin_m=0.15
    )
    assert np.array_equal(grid, original_snapshot), "Original grid must not be modified!"

    planner = AStarPlanner(res_m, allow_diagonal=True)
    res_pass = planner.plan(inflated_passable, (8, 1), (8, 18))
    _assert_valid_path(res_pass, inflated_passable, res_m, allow_diagonal=True)

    # Larger safety margin (0.35 + 0.50 = 0.85m -> 4 cells each side) closes the 1.4m corridor.
    inflated_blocked = inflate_occupancy_grid(
        grid, resolution_m=res_m, robot_radius_m=0.35, safety_margin_m=0.50
    )
    res_fail = planner.plan(inflated_blocked, (8, 1), (8, 18))
    assert res_fail.success is False
    assert res_fail.failure_reason is not None


def test_unreachable_goal_returns_clear_failure() -> None:
    res_m = 0.2
    grid = np.zeros((15, 15), dtype=np.uint8)
    grid[:, 7] = 1  # Complete vertical barrier.

    planner = AStarPlanner(res_m, allow_diagonal=True)
    res = planner.plan(grid, (7, 2), (7, 12))
    assert res.success is False
    assert res.path_grid == []
    assert math.isinf(res.path_cost_m)
    assert res.failure_reason is not None
    assert "No valid" in res.failure_reason


def test_start_equals_goal() -> None:
    res_m = 0.2
    grid = np.zeros((10, 10), dtype=np.uint8)
    planner = AStarPlanner(res_m, allow_diagonal=True)

    res = planner.plan(grid, (4, 5), (4, 5))
    assert res.success is True
    assert res.path_grid == [(4, 5)]
    assert res.path_cost_m == 0.0


def test_diagonal_corner_cutting_prevention() -> None:
    res_m = 0.2
    grid = np.zeros((5, 5), dtype=np.uint8)
    # Two diagonal obstacles touching at corner (2, 2):
    # Path from (2, 1) to (1, 2) would cut between (1, 1) and (2, 2).
    grid[1, 1] = 1
    grid[2, 2] = 1

    planner = AStarPlanner(res_m, allow_diagonal=True)
    res = planner.plan(grid, (2, 1), (1, 2))
    _assert_valid_path(res, grid, res_m, allow_diagonal=True)
    # Must detour around (3,1)->(3,2)->(3,3)->(2,3)->(1,3)->(1,2) or via (2,0)...
    # Cannot take single diagonal step from (2,1) to (1,2).
    assert len(res.path_grid) > 2
