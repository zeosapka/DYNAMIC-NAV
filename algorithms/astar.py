"""A* global path planning and metric obstacle inflation on 2D occupancy grids.

All distances, heuristics, and path costs are expressed in SI meters.
This module is strictly independent of Pygame and any external graph libraries.
"""

from dataclasses import dataclass, field
import heapq
import math
from typing import Sequence
import numpy as np

from simulation.geometry import Geometry, geometry_bounding_box_m, geometry_overlaps_cell


_CARDINAL_STEPS: tuple[tuple[int, int, float], ...] = (
    (-1, 0, 1.0),
    (1, 0, 1.0),
    (0, -1, 1.0),
    (0, 1, 1.0),
)

_DIAGONAL_STEPS: tuple[tuple[int, int, float], ...] = (
    (-1, -1, math.sqrt(2.0)),
    (-1, 1, math.sqrt(2.0)),
    (1, -1, math.sqrt(2.0)),
    (1, 1, math.sqrt(2.0)),
)


@dataclass(frozen=True)
class AStarResult:
    """Result container returned by AStarPlanner.plan()."""

    success: bool
    path_grid: list[tuple[int, int]] = field(default_factory=list)
    path_world_m: list[tuple[float, float]] = field(default_factory=list)
    path_cost_m: float = math.inf
    explored_nodes: list[tuple[int, int]] = field(default_factory=list)
    g_scores: dict[tuple[int, int], float] = field(default_factory=dict)
    f_scores: dict[tuple[int, int], float] = field(default_factory=dict)
    parents: dict[tuple[int, int], tuple[int, int]] = field(default_factory=dict)
    failure_reason: str | None = None


def inflate_occupancy_grid(
    occupancy_grid: np.ndarray,
    resolution_m: float,
    robot_radius_m: float,
    safety_margin_m: float = 0.0,
    enforce_boundary_clearance: bool = False,
) -> np.ndarray:
    """Return a new occupancy grid with obstacles inflated by robot_radius + safety_margin.

    The input occupancy_grid is never modified in place.
    Uses exact minimum Euclidean distance from each candidate cell center to the
    occupied square cell geometry:
        d_min = sqrt(max(0, |dr|*h - h/2)^2 + max(0, |dc|*h - h/2)^2)
    A cell is marked occupied iff d_min < robot_radius_m + safety_margin_m,
    guaranteeing physical clearance >= robot_radius_m + safety_margin_m at all
    free cell centers (including diagonal corners) without over-inflating corridors.
    When enforce_boundary_clearance is True, cells whose centers are closer than
    robot_radius_m + safety_margin_m to any of the four world boundaries are also
    marked occupied.
    """
    if resolution_m <= 0.0:
        raise ValueError("Grid resolution_m must be strictly positive.")
    if robot_radius_m < 0.0 or safety_margin_m < 0.0:
        raise ValueError("robot_radius_m and safety_margin_m must be non-negative.")

    inflated = (occupancy_grid != 0).astype(np.uint8, copy=True)
    inflation_radius_m = robot_radius_m + safety_margin_m
    if inflation_radius_m <= 0.0:
        return inflated

    rows, cols = inflated.shape
    half_cell_m = 0.5 * resolution_m
    cell_radius = math.ceil(inflation_radius_m / resolution_m + 0.5)

    result = inflated.copy()
    if np.any(inflated):
        # Precompute exact square-to-point distance structuring element offsets.
        offsets: list[tuple[int, int]] = []
        radius_sq_m = (inflation_radius_m - 1e-9) ** 2
        for dr in range(-cell_radius, cell_radius + 1):
            dy_m = max(0.0, abs(dr) * resolution_m - half_cell_m)
            for dc in range(-cell_radius, cell_radius + 1):
                dx_m = max(0.0, abs(dc) * resolution_m - half_cell_m)
                if (dx_m * dx_m + dy_m * dy_m) < radius_sq_m:
                    offsets.append((dr, dc))

        occupied_mask = occupancy_grid != 0
        for dr, dc in offsets:
            if dr == 0 and dc == 0:
                continue
            src_r_start = max(0, -dr)
            src_r_end = min(rows, rows - dr)
            src_c_start = max(0, -dc)
            src_c_end = min(cols, cols - dc)

            dst_r_start = max(0, dr)
            dst_r_end = min(rows, rows + dr)
            dst_c_start = max(0, dc)
            dst_c_end = min(cols, cols + dc)

            if src_r_start < src_r_end and src_c_start < src_c_end:
                result[dst_r_start:dst_r_end, dst_c_start:dst_c_end] |= occupied_mask[
                    src_r_start:src_r_end, src_c_start:src_c_end
                ]

    if enforce_boundary_clearance:
        world_w_m = cols * resolution_m
        world_h_m = rows * resolution_m
        limit_m = inflation_radius_m - 1e-9
        for r in range(rows):
            cy_m = (r + 0.5) * resolution_m
            if cy_m < limit_m or (world_h_m - cy_m) < limit_m:
                result[r, :] = 1
        for c in range(cols):
            cx_m = (c + 0.5) * resolution_m
            if cx_m < limit_m or (world_w_m - cx_m) < limit_m:
                result[:, c] = 1

    return result


def segment_to_square_distance_m(
    p0_m: tuple[float, float],
    p1_m: tuple[float, float],
    sq_min_x_m: float,
    sq_min_y_m: float,
    sq_max_x_m: float,
    sq_max_y_m: float,
) -> float:
    """Compute exact minimum Euclidean distance between a 2D line segment and an axis-aligned square.

    Uses piecewise-quadratic minimization over the parameter t in [0, 1] split at
    the axis-aligned clamping breakpoints x = sq_min_x_m, sq_max_x_m and
    y = sq_min_y_m, sq_max_y_m.
    """
    x0, y0 = p0_m
    x1, y1 = p1_m
    dx = x1 - x0
    dy = y1 - y0

    breakpoints: list[float] = [0.0, 1.0]
    if abs(dx) > 1e-12:
        for bx in (sq_min_x_m, sq_max_x_m):
            t = (bx - x0) / dx
            if 0.0 < t < 1.0:
                breakpoints.append(t)
    if abs(dy) > 1e-12:
        for by in (sq_min_y_m, sq_max_y_m):
            t = (by - y0) / dy
            if 0.0 < t < 1.0:
                breakpoints.append(t)

    breakpoints.sort()
    min_dist_sq = math.inf

    for i in range(len(breakpoints) - 1):
        ta = breakpoints[i]
        tb = breakpoints[i + 1]
        if tb - ta < 1e-15:
            continue
        t_mid = 0.5 * (ta + tb)
        xm = x0 + t_mid * dx
        ym = y0 + t_mid * dy

        if xm < sq_min_x_m:
            ax, bx_off = dx, x0 - sq_min_x_m
        elif xm > sq_max_x_m:
            ax, bx_off = dx, x0 - sq_max_x_m
        else:
            ax, bx_off = 0.0, 0.0

        if ym < sq_min_y_m:
            ay, by_off = dy, y0 - sq_min_y_m
        elif ym > sq_max_y_m:
            ay, by_off = dy, y0 - sq_max_y_m
        else:
            ay, by_off = 0.0, 0.0

        denom = ax * ax + ay * ay
        if denom > 1e-15:
            t_star = -(ax * bx_off + ay * by_off) / denom
            t_star = max(ta, min(tb, t_star))
        else:
            t_star = ta

        for t_eval in (ta, t_star, tb):
            ex = ax * t_eval + bx_off
            ey = ay * t_eval + by_off
            d_sq = ex * ex + ey * ey
            if d_sq < min_dist_sq:
                min_dist_sq = d_sq

    return math.sqrt(max(0.0, min_dist_sq))


def is_segment_collision_free(
    p0_m: tuple[float, float],
    p1_m: tuple[float, float],
    raw_grid: np.ndarray,
    resolution_m: float,
    required_clearance_m: float,
    enforce_boundary_clearance: bool = False,
) -> bool:
    """Return True iff every point on segment p0_m -> p1_m maintains >= required_clearance_m.

    Checks exact segment-to-square distance against all occupied cells in raw_grid
    within the segment's expanded bounding box, and optionally against the four
    world boundaries.
    """
    rows, cols = raw_grid.shape
    if enforce_boundary_clearance and required_clearance_m > 0.0:
        world_w_m = cols * resolution_m
        world_h_m = rows * resolution_m
        limit_m = required_clearance_m - 1e-9
        for px, py in (p0_m, p1_m):
            if (
                px < limit_m
                or (world_w_m - px) < limit_m
                or py < limit_m
                or (world_h_m - py) < limit_m
            ):
                return False

    if required_clearance_m <= 0.0:
        return True

    min_x = min(p0_m[0], p1_m[0]) - required_clearance_m
    max_x = max(p0_m[0], p1_m[0]) + required_clearance_m
    min_y = min(p0_m[1], p1_m[1]) - required_clearance_m
    max_y = max(p0_m[1], p1_m[1]) + required_clearance_m

    c_start = max(0, math.floor(min_x / resolution_m))
    c_end = min(cols, math.ceil(max_x / resolution_m))
    r_start = max(0, math.floor(min_y / resolution_m))
    r_end = min(rows, math.ceil(max_y / resolution_m))

    threshold_m = required_clearance_m - 1e-9
    for r in range(r_start, r_end):
        sq_min_y = r * resolution_m
        sq_max_y = sq_min_y + resolution_m
        for c in range(c_start, c_end):
            if raw_grid[r, c] != 0:
                sq_min_x = c * resolution_m
                sq_max_x = sq_min_x + resolution_m
                if (
                    segment_to_square_distance_m(
                        p0_m, p1_m, sq_min_x, sq_min_y, sq_max_x, sq_max_y
                    )
                    < threshold_m
                ):
                    return False

    return True


def world_to_grid(
    x_m: float,
    y_m: float,
    resolution_m: float,
    grid_shape: tuple[int, int],
) -> tuple[int, int]:
    """Convert SI world coordinates (x_m, y_m) to clamped grid indices (row, col)."""
    rows, cols = grid_shape
    col = max(0, min(cols - 1, math.floor(x_m / resolution_m)))
    row = max(0, min(rows - 1, math.floor(y_m / resolution_m)))
    return (row, col)


def grid_to_world(row: int, col: int, resolution_m: float) -> tuple[float, float]:
    """Convert grid cell (row, col) to its center in SI world meters (x_m, y_m)."""
    x_m = (col + 0.5) * resolution_m
    y_m = (row + 0.5) * resolution_m
    return (x_m, y_m)


def manhattan_heuristic_m(
    a: tuple[int, int], b: tuple[int, int], resolution_m: float
) -> float:
    """Admissible Manhattan distance heuristic in meters for 4-directional motion."""
    return (abs(a[0] - b[0]) + abs(a[1] - b[1])) * resolution_m


def octile_heuristic_m(
    a: tuple[int, int], b: tuple[int, int], resolution_m: float
) -> float:
    """Admissible octile distance heuristic in meters for 8-directional motion."""
    dr = abs(a[0] - b[0])
    dc = abs(a[1] - b[1])
    return (max(dr, dc) + (math.sqrt(2.0) - 1.0) * min(dr, dc)) * resolution_m


class AStarPlanner:
    """Grid-based A* global path planner with 4/8-connectivity and metric costs."""

    def __init__(self, resolution_m: float, allow_diagonal: bool = True) -> None:
        if resolution_m <= 0.0:
            raise ValueError("resolution_m must be strictly positive.")
        self._resolution_m = resolution_m
        self._allow_diagonal = allow_diagonal

    @property
    def resolution_m(self) -> float:
        """Return grid cell size in meters."""
        return self._resolution_m

    @property
    def allow_diagonal(self) -> bool:
        """Return whether 8-directional diagonal transitions are enabled."""
        return self._allow_diagonal

    def heuristic_m(self, a: tuple[int, int], b: tuple[int, int]) -> float:
        """Compute admissible heuristic distance in meters between grid cells."""
        if self._allow_diagonal:
            return octile_heuristic_m(a, b, self._resolution_m)
        return manhattan_heuristic_m(a, b, self._resolution_m)

    def plan(
        self,
        grid: np.ndarray,
        start: tuple[int, int],
        goal: tuple[int, int],
        raw_grid: np.ndarray | None = None,
        required_clearance_m: float = 0.0,
        enforce_boundary_clearance: bool = False,
    ) -> AStarResult:
        """Find a minimum-cost collision-free path from start to goal on grid.

        Args:
            grid: 2D array where 0 is free and non-zero is occupied.
            start: (row, col) start cell.
            goal: (row, col) goal cell.
            raw_grid: Optional uninflated occupancy grid for exact continuous
                segment-to-square clearance validation on every transition.
            required_clearance_m: Minimum required clearance in meters when
                validating continuous segments against raw_grid.
            enforce_boundary_clearance: When True, also enforce
                required_clearance_m against all four world boundaries.
        """
        rows, cols = grid.shape
        if not self._in_bounds(start, rows, cols):
            return AStarResult(
                success=False,
                failure_reason=f"Start cell {start} is out of grid bounds {grid.shape}.",
            )
        if not self._in_bounds(goal, rows, cols):
            return AStarResult(
                success=False,
                failure_reason=f"Goal cell {goal} is out of grid bounds {grid.shape}.",
            )
        if grid[start[0], start[1]] != 0:
            return AStarResult(
                success=False,
                failure_reason=f"Start cell {start} is inside an occupied or inflated obstacle.",
            )
        if grid[goal[0], goal[1]] != 0:
            return AStarResult(
                success=False,
                failure_reason=f"Goal cell {goal} is inside an occupied or inflated obstacle.",
            )

        if start == goal:
            wx, wy = grid_to_world(start[0], start[1], self._resolution_m)
            return AStarResult(
                success=True,
                path_grid=[start],
                path_world_m=[(wx, wy)],
                path_cost_m=0.0,
                explored_nodes=[start],
                g_scores={start: 0.0},
                f_scores={start: 0.0},
                parents={},
            )

        g_scores: dict[tuple[int, int], float] = {start: 0.0}
        f_scores: dict[tuple[int, int], float] = {start: self.heuristic_m(start, goal)}
        parents: dict[tuple[int, int], tuple[int, int]] = {}
        explored_nodes: list[tuple[int, int]] = []
        closed_set: set[tuple[int, int]] = set()

        counter = 0
        open_heap: list[tuple[float, int, tuple[int, int]]] = [
            (f_scores[start], counter, start)
        ]

        moves = (
            _CARDINAL_STEPS + _DIAGONAL_STEPS
            if self._allow_diagonal
            else _CARDINAL_STEPS
        )

        while open_heap:
            _, _, current = heapq.heappop(open_heap)
            if current in closed_set:
                continue

            closed_set.add(current)
            explored_nodes.append(current)

            if current == goal:
                path_grid = self._reconstruct_path(parents, start, goal)
                path_world = [
                    grid_to_world(r, c, self._resolution_m) for r, c in path_grid
                ]
                return AStarResult(
                    success=True,
                    path_grid=path_grid,
                    path_world_m=path_world,
                    path_cost_m=g_scores[goal],
                    explored_nodes=explored_nodes,
                    g_scores=g_scores,
                    f_scores=f_scores,
                    parents=parents,
                )

            cur_r, cur_c = current
            cur_g = g_scores[current]

            for dr, dc, step_scale in moves:
                nr, nc = cur_r + dr, cur_c + dc
                if not self._in_bounds((nr, nc), rows, cols):
                    continue
                if grid[nr, nc] != 0:
                    continue

                # Prevent diagonal corner cutting: both adjacent cardinal cells must be free.
                if dr != 0 and dc != 0:
                    if grid[cur_r + dr, cur_c] != 0 or grid[cur_r, cur_c + dc] != 0:
                        continue

                neighbor = (nr, nc)
                if neighbor in closed_set:
                    continue

                if raw_grid is not None and required_clearance_m > 0.0:
                    p0_m = grid_to_world(cur_r, cur_c, self._resolution_m)
                    p1_m = grid_to_world(nr, nc, self._resolution_m)
                    if not is_segment_collision_free(
                        p0_m,
                        p1_m,
                        raw_grid,
                        self._resolution_m,
                        required_clearance_m,
                        enforce_boundary_clearance,
                    ):
                        continue

                tentative_g = cur_g + step_scale * self._resolution_m
                if tentative_g < g_scores.get(neighbor, math.inf):
                    parents[neighbor] = current
                    g_scores[neighbor] = tentative_g
                    f_val = tentative_g + self.heuristic_m(neighbor, goal)
                    f_scores[neighbor] = f_val
                    counter += 1
                    heapq.heappush(open_heap, (f_val, counter, neighbor))

        return AStarResult(
            success=False,
            explored_nodes=explored_nodes,
            g_scores=g_scores,
            f_scores=f_scores,
            parents=parents,
            failure_reason="No valid collision-free path exists between start and goal.",
        )

    @staticmethod
    def _in_bounds(cell: tuple[int, int], rows: int, cols: int) -> bool:
        r, c = cell
        return 0 <= r < rows and 0 <= c < cols

    @staticmethod
    def _reconstruct_path(
        parents: dict[tuple[int, int], tuple[int, int]],
        start: tuple[int, int],
        goal: tuple[int, int],
    ) -> list[tuple[int, int]]:
        path: list[tuple[int, int]] = [goal]
        cur = goal
        while cur != start:
            cur = parents[cur]
            path.append(cur)
        path.reverse()
        return path


def rasterize_geometry_onto_grid(
    grid: np.ndarray,
    geom: Geometry,
    resolution_m: float,
) -> None:
    """Mark every cell in `grid` whose square area overlaps `geom` (in-place)."""
    if resolution_m <= 0.0:
        raise ValueError("Grid resolution_m must be strictly positive.")
    rows, cols = grid.shape
    min_x, min_y, max_x, max_y = geometry_bounding_box_m(geom)
    c0 = max(0, math.floor(min_x / resolution_m))
    c1 = min(cols - 1, math.ceil(max_x / resolution_m) - 1)
    r0 = max(0, math.floor(min_y / resolution_m))
    r1 = min(rows - 1, math.ceil(max_y / resolution_m) - 1)
    for r in range(r0, r1 + 1):
        cy0 = r * resolution_m
        cy1 = cy0 + resolution_m
        for c in range(c0, c1 + 1):
            cx0 = c * resolution_m
            if geometry_overlaps_cell(geom, cx0, cy0, cx0 + resolution_m, cy1):
                grid[r, c] = 1


def rasterize_geometries_onto_grid(
    grid: np.ndarray,
    geometries: Sequence[Geometry],
    resolution_m: float,
) -> None:
    """Mark cells overlapping any geometry in `geometries` onto `grid` (in-place)."""
    for geom in geometries:
        rasterize_geometry_onto_grid(grid, geom, resolution_m)
