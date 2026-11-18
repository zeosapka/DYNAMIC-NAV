"""Continuous path-segment and world-boundary safety validation tests."""

import math
import numpy as np
import pytest

from algorithms.astar import (
    AStarPlanner,
    inflate_occupancy_grid,
    is_segment_collision_free,
    segment_to_square_distance_m,
    world_to_grid,
)
from config import DEFAULT_CONFIG
from simulation.world import SimulationWorld


def test_diagonal_segment_near_obstacle_corner() -> None:
    # Occupied square [1.0, 1.2] x [1.0, 1.2]
    # Diagonal segment from (1.5, 1.1) to (1.1, 1.5) passes closest at (1.3, 1.3),
    # distance to corner (1.2, 1.2) is sqrt(0.1^2 + 0.1^2) = 0.1414 m.
    d = segment_to_square_distance_m(
        (1.5, 1.1), (1.1, 1.5), 1.0, 1.0, 1.2, 1.2
    )
    assert d == pytest.approx(math.sqrt(0.02), rel=1e-6)


def test_endpoints_safe_but_interior_unsafe() -> None:
    # Occupied square at grid (5, 5) -> [1.0, 1.2] x [1.0, 1.2] with res=0.20 m.
    # Endpoints (1.60, 0.90) and (0.90, 1.60):
    # - Distance from (1.60, 0.90) to square [1.0, 1.2]x[1.0, 1.2]: dx=0.40, dy=0.10 -> 0.4123 m (or shifted to >= 0.50 m!)
    # Use (1.70, 0.90) and (0.90, 1.70):
    #   dx = 1.70 - 1.20 = 0.50 m, dy = 1.00 - 0.90 = 0.10 m -> endpoint dist = sqrt(0.26) = 0.5099 m >= 0.50 m!
    #   Midpoint (1.30, 1.30): dx = 0.10, dy = 0.10 -> midpoint dist = sqrt(0.02) = 0.1414 m < 0.50 m!
    res_m = 0.20
    raw_grid = np.zeros((15, 15), dtype=np.uint8)
    raw_grid[5, 5] = 1
    p0 = (1.70, 0.90)
    p1 = (0.90, 1.70)

    d_p0 = segment_to_square_distance_m(p0, p0, 1.0, 1.0, 1.2, 1.2)
    d_p1 = segment_to_square_distance_m(p1, p1, 1.0, 1.0, 1.2, 1.2)
    assert d_p0 >= 0.50
    assert d_p1 >= 0.50

    assert (
        is_segment_collision_free(p0, p1, raw_grid, res_m, required_clearance_m=0.50)
        is False
    )


def test_segment_at_exact_050m_threshold() -> None:
    res_m = 0.20
    raw_grid = np.zeros((15, 15), dtype=np.uint8)
    raw_grid[5, 5] = 1  # [1.0, 1.2] x [1.0, 1.2]

    # Horizontal segment at y = 1.70 m -> distance to y_max = 1.20 m is exactly 0.50 m.
    p0 = (0.50, 1.70)
    p1 = (2.10, 1.70)
    d = segment_to_square_distance_m(p0, p1, 1.0, 1.0, 1.2, 1.2)
    assert d == pytest.approx(0.50, abs=1e-9)
    assert (
        is_segment_collision_free(p0, p1, raw_grid, res_m, required_clearance_m=0.50)
        is True
    )


def test_valid_diagonal_segment_sufficient_clearance() -> None:
    res_m = 0.20
    raw_grid = np.zeros((15, 15), dtype=np.uint8)
    raw_grid[5, 5] = 1  # [1.0, 1.2] x [1.0, 1.2]

    # Diagonal from cell (8, 7) center (1.5, 1.7) to cell (7, 8) center (1.7, 1.5).
    # Closest point is midpoint (1.6, 1.6); distance to (1.2, 1.2) is sqrt(0.4^2 + 0.4^2) = 0.5657 m >= 0.50 m.
    p0 = (1.5, 1.7)
    p1 = (1.7, 1.5)
    assert (
        is_segment_collision_free(p0, p1, raw_grid, res_m, required_clearance_m=0.50)
        is True
    )


def test_horizontal_and_vertical_segment_clearance() -> None:
    res_m = 0.20
    raw_grid = np.zeros((15, 15), dtype=np.uint8)
    raw_grid[5, 5] = 1  # [1.0, 1.2] x [1.0, 1.2]

    # Horizontal segment too close (y = 1.50 m -> clearance 0.30 m < 0.50 m)
    assert (
        is_segment_collision_free(
            (0.50, 1.50), (2.10, 1.50), raw_grid, res_m, required_clearance_m=0.50
        )
        is False
    )
    # Vertical segment too close (x = 1.50 m -> clearance 0.30 m < 0.50 m)
    assert (
        is_segment_collision_free(
            (1.50, 0.50), (1.50, 2.10), raw_grid, res_m, required_clearance_m=0.50
        )
        is False
    )


def test_multiple_adjacent_obstacle_cells_segment_clearance() -> None:
    res_m = 0.20
    raw_grid = np.zeros((15, 15), dtype=np.uint8)
    raw_grid[5:8, 5:8] = 1  # 0.6m x 0.6m block: [1.0, 1.6] x [1.0, 1.6]

    # Segment at x = 2.10 m has exact 0.50 m clearance from x_max = 1.60 m -> safe.
    assert (
        is_segment_collision_free(
            (2.10, 0.50), (2.10, 2.50), raw_grid, res_m, required_clearance_m=0.50
        )
        is True
    )
    # Segment at x = 1.90 m has 0.30 m clearance -> unsafe.
    assert (
        is_segment_collision_free(
            (1.90, 0.50), (1.90, 2.50), raw_grid, res_m, required_clearance_m=0.50
        )
        is False
    )


def test_world_boundary_clearance_all_four_edges() -> None:
    res_m = 0.20
    req_m = 0.50
    raw_grid = np.zeros((15, 20), dtype=np.uint8)  # 4.0 m x 3.0 m
    snapshot = raw_grid.copy()

    inflated = inflate_occupancy_grid(
        raw_grid,
        resolution_m=res_m,
        robot_radius_m=0.35,
        safety_margin_m=0.15,
        enforce_boundary_clearance=True,
    )
    assert np.array_equal(raw_grid, snapshot)

    # First two and last two rows/cols have centers at 0.10, 0.30 (< 0.50 m) -> blocked.
    assert np.all(inflated[0:2, :] == 1)
    assert np.all(inflated[-2:, :] == 1)
    assert np.all(inflated[:, 0:2] == 1)
    assert np.all(inflated[:, -2:] == 1)

    # Row/col index 2 has center at 0.50 m == req_m -> safe!
    assert inflated[2, 2] == 0
    assert inflated[12, 17] == 0

    planner = AStarPlanner(res_m, allow_diagonal=True)
    # Start near left boundary (col 1, x=0.30 m) must be rejected.
    res_bad = planner.plan(
        inflated,
        (5, 1),
        (5, 10),
        raw_grid=raw_grid,
        required_clearance_m=req_m,
        enforce_boundary_clearance=True,
    )
    assert res_bad.success is False

    # Start and goal at exact 0.50 m boundary threshold succeed.
    res_ok = planner.plan(
        inflated,
        (2, 2),
        (12, 17),
        raw_grid=raw_grid,
        required_clearance_m=req_m,
        enforce_boundary_clearance=True,
    )
    assert res_ok.success is True


def test_default_world_continuous_path_clearance() -> None:
    world = SimulationWorld(
        DEFAULT_CONFIG.world, DEFAULT_CONFIG.robot, DEFAULT_CONFIG.simulation
    )
    res_m = DEFAULT_CONFIG.world.grid_resolution_m
    req_m = DEFAULT_CONFIG.robot.radius_m + DEFAULT_CONFIG.planner.safety_margin_m
    raw_snapshot = world.occupancy_grid.copy()

    inflated = inflate_occupancy_grid(
        world.occupancy_grid,
        resolution_m=res_m,
        robot_radius_m=DEFAULT_CONFIG.robot.radius_m,
        safety_margin_m=DEFAULT_CONFIG.planner.safety_margin_m,
        enforce_boundary_clearance=True,
    )
    start = world_to_grid(
        DEFAULT_CONFIG.robot.initial_x_m,
        DEFAULT_CONFIG.robot.initial_y_m,
        res_m,
        inflated.shape,
    )
    goal = world_to_grid(
        DEFAULT_CONFIG.planner.default_goal_x_m,
        DEFAULT_CONFIG.planner.default_goal_y_m,
        res_m,
        inflated.shape,
    )

    for allow_diag in (False, True):
        res = AStarPlanner(res_m, allow_diagonal=allow_diag).plan(
            inflated,
            start,
            goal,
            raw_grid=world.occupancy_grid,
            required_clearance_m=req_m,
            enforce_boundary_clearance=True,
        )
        assert res.success is True
        for i in range(len(res.path_world_m) - 1):
            assert is_segment_collision_free(
                res.path_world_m[i],
                res.path_world_m[i + 1],
                world.occupancy_grid,
                res_m,
                required_clearance_m=req_m,
                enforce_boundary_clearance=True,
            )

    assert np.array_equal(world.occupancy_grid, raw_snapshot)
