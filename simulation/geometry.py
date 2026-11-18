"""Universal 2D SI-unit geometry and exact distance/collision framework.

This module is strictly independent of Pygame and rendering logic.
All coordinates, lengths, and radii are in meters (m), and angles in radians (rad).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum
from typing import Union


class GeometryType(str, Enum):
    """Supported 2D geometric footprint primitives."""

    CIRCLE = "CIRCLE"
    ORIENTED_RECT = "ORIENTED_RECT"
    POLYGON = "POLYGON"
    CAPSULE = "CAPSULE"


@dataclass(frozen=True)
class CircleGeometry:
    """Circular geometry defined by center (m) and physical radius (m)."""

    center_x_m: float
    center_y_m: float
    radius_m: float

    def __post_init__(self) -> None:
        if not (
            math.isfinite(self.center_x_m)
            and math.isfinite(self.center_y_m)
            and math.isfinite(self.radius_m)
            and self.radius_m > 0.0
        ):
            raise ValueError("CircleGeometry requires finite center and radius_m > 0.")

    @property
    def center_m(self) -> tuple[float, float]:
        return (self.center_x_m, self.center_y_m)


@dataclass(frozen=True)
class OrientedRectGeometry:
    """Oriented rectangle (vehicle/obstacle footprint) in world SI coordinates.

    `length_m` is measured along local heading `heading_rad` (+X_local),
    and `width_m` is measured perpendicular to heading (+Y_local).
    """

    center_x_m: float
    center_y_m: float
    width_m: float
    length_m: float
    heading_rad: float = 0.0

    def __post_init__(self) -> None:
        if not (
            math.isfinite(self.center_x_m)
            and math.isfinite(self.center_y_m)
            and math.isfinite(self.width_m)
            and math.isfinite(self.length_m)
            and math.isfinite(self.heading_rad)
            and self.width_m > 0.0
            and self.length_m > 0.0
        ):
            raise ValueError(
                "OrientedRectGeometry requires finite center/heading and positive width_m/length_m."
            )

    @property
    def center_m(self) -> tuple[float, float]:
        return (self.center_x_m, self.center_y_m)

    def corners_m(self) -> tuple[tuple[float, float], ...]:
        """Return the 4 world-space corners in counter-clockwise order."""
        hl = 0.5 * self.length_m
        hw = 0.5 * self.width_m
        cos_h = math.cos(self.heading_rad)
        sin_h = math.sin(self.heading_rad)
        local = ((hl, -hw), (hl, hw), (-hl, hw), (-hl, -hw))
        return tuple(
            (
                self.center_x_m + lx * cos_h - ly * sin_h,
                self.center_y_m + lx * sin_h + ly * cos_h,
            )
            for lx, ly in local
        )

    def as_polygon(self) -> PolygonGeometry:
        """Convert this oriented rectangle into a validated PolygonGeometry."""
        return PolygonGeometry(self.corners_m())


@dataclass(frozen=True)
class CapsuleGeometry:
    """Capsule (swept line segment) geometry in world SI coordinates."""

    center_x_m: float
    center_y_m: float
    length_m: float
    radius_m: float
    heading_rad: float = 0.0

    def __post_init__(self) -> None:
        if not (
            math.isfinite(self.center_x_m)
            and math.isfinite(self.center_y_m)
            and math.isfinite(self.length_m)
            and math.isfinite(self.radius_m)
            and math.isfinite(self.heading_rad)
            and self.length_m > 0.0
            and self.radius_m > 0.0
        ):
            raise ValueError(
                "CapsuleGeometry requires finite center/heading and positive length_m/radius_m."
            )

    @property
    def center_m(self) -> tuple[float, float]:
        return (self.center_x_m, self.center_y_m)

    def spine_segment_m(self) -> tuple[tuple[float, float], tuple[float, float]]:
        """Return the start and end points of the capsule's medial line segment."""
        half_spine = max(0.0, 0.5 * self.length_m - self.radius_m)
        dx = half_spine * math.cos(self.heading_rad)
        dy = half_spine * math.sin(self.heading_rad)
        return (
            (self.center_x_m - dx, self.center_y_m - dy),
            (self.center_x_m + dx, self.center_y_m + dy),
        )


def _orientation(
    a: tuple[float, float], b: tuple[float, float], c: tuple[float, float]
) -> float:
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def _on_segment(
    p: tuple[float, float], q: tuple[float, float], r: tuple[float, float]
) -> bool:
    return (
        min(p[0], r[0]) - 1e-12 <= q[0] <= max(p[0], r[0]) + 1e-12
        and min(p[1], r[1]) - 1e-12 <= q[1] <= max(p[1], r[1]) + 1e-12
    )


def segment_intersection(
    a0: tuple[float, float],
    a1: tuple[float, float],
    b0: tuple[float, float],
    b1: tuple[float, float],
) -> tuple[bool, tuple[float, float] | None]:
    """Return (intersects, intersection_point_or_None) for segments a0-a1 and b0-b1."""
    d1 = _orientation(b0, b1, a0)
    d2 = _orientation(b0, b1, a1)
    d3 = _orientation(a0, a1, b0)
    d4 = _orientation(a0, a1, b1)

    if ((d1 > 1e-12 and d2 < -1e-12) or (d1 < -1e-12 and d2 > 1e-12)) and (
        (d3 > 1e-12 and d4 < -1e-12) or (d3 < -1e-12 and d4 > 1e-12)
    ):
        denom = (a0[0] - a1[0]) * (b0[1] - b1[1]) - (a0[1] - a1[1]) * (b0[0] - b1[0])
        if abs(denom) > 1e-12:
            t = ((a0[0] - b0[0]) * (b0[1] - b1[1]) - (a0[1] - b0[1]) * (b0[0] - b1[0])) / denom
            return (True, (a0[0] + t * (a1[0] - a0[0]), a0[1] + t * (a1[1] - a0[1])))
        return (True, a0)

    if abs(d1) <= 1e-12 and _on_segment(b0, a0, b1):
        return (True, a0)
    if abs(d2) <= 1e-12 and _on_segment(b0, a1, b1):
        return (True, a1)
    if abs(d3) <= 1e-12 and _on_segment(a0, b0, a1):
        return (True, b0)
    if abs(d4) <= 1e-12 and _on_segment(a0, b1, a1):
        return (True, b1)
    return (False, None)


def validate_polygon_vertices(vertices_m: tuple[tuple[float, float], ...] | list[tuple[float, float]]) -> tuple[tuple[float, float], ...]:
    """Validate simple non-degenerate polygon vertices and return immutable tuple."""
    verts = tuple((float(x), float(y)) for x, y in vertices_m)
    if len(verts) < 3:
        raise ValueError("PolygonGeometry requires at least 3 vertices.")
    for x, y in verts:
        if not (math.isfinite(x) and math.isfinite(y)):
            raise ValueError("PolygonGeometry vertices must be finite coordinates.")

    # Non-zero signed area check (shoelace formula)
    area2 = 0.0
    n = len(verts)
    for i in range(n):
        x0, y0 = verts[i]
        x1, y1 = verts[(i + 1) % n]
        area2 += x0 * y1 - x1 * y0
    if abs(area2) <= 1e-9:
        raise ValueError("PolygonGeometry has zero area (collinear/degenerate vertices).")

    # Simple polygon check (no non-adjacent edge self-intersections)
    for i in range(n):
        a0, a1 = verts[i], verts[(i + 1) % n]
        if math.hypot(a1[0] - a0[0], a1[1] - a0[1]) <= 1e-9:
            raise ValueError("PolygonGeometry contains duplicate consecutive vertices.")
        for j in range(i + 1, n):
            if j == i or (j + 1) % n == i or (i + 1) % n == j:
                continue
            b0, b1 = verts[j], verts[(j + 1) % n]
            hit, _ = segment_intersection(a0, a1, b0, b1)
            if hit:
                raise ValueError("PolygonGeometry must be simple (non-self-intersecting).")
    return verts


@dataclass(frozen=True)
class PolygonGeometry:
    """Validated simple 2D polygon in world SI coordinates (meters)."""

    vertices_m: tuple[tuple[float, float], ...]

    def __post_init__(self) -> None:
        validated = validate_polygon_vertices(self.vertices_m)
        object.__setattr__(self, "vertices_m", validated)

    @property
    def center_m(self) -> tuple[float, float]:
        """Return polygon centroid (average of vertices if area centroid is tiny)."""
        n = len(self.vertices_m)
        area2 = 0.0
        cx = 0.0
        cy = 0.0
        for i in range(n):
            x0, y0 = self.vertices_m[i]
            x1, y1 = self.vertices_m[(i + 1) % n]
            cross = x0 * y1 - x1 * y0
            area2 += cross
            cx += (x0 + x1) * cross
            cy += (y0 + y1) * cross
        if abs(area2) > 1e-9:
            return (cx / (3.0 * area2), cy / (3.0 * area2))
        return (
            sum(v[0] for v in self.vertices_m) / n,
            sum(v[1] for v in self.vertices_m) / n,
        )

    def edges_m(self) -> tuple[tuple[tuple[float, float], tuple[float, float]], ...]:
        n = len(self.vertices_m)
        return tuple(
            (self.vertices_m[i], self.vertices_m[(i + 1) % n]) for i in range(n)
        )


Geometry = Union[CircleGeometry, OrientedRectGeometry, PolygonGeometry, CapsuleGeometry]


@dataclass(frozen=True)
class GeometryDistanceResult:
    """Result of an exact geometric distance and collision query (in SI meters)."""

    center_distance_m: float
    surface_clearance_m: float
    is_overlapping: bool
    closest_point_a_m: tuple[float, float]
    closest_point_b_m: tuple[float, float]


def point_to_segment_closest(
    px: float,
    py: float,
    ax: float,
    ay: float,
    bx: float,
    by: float,
) -> tuple[float, tuple[float, float]]:
    """Return (distance_m, closest_point_on_segment) from point (px, py) to segment AB."""
    dx = bx - ax
    dy = by - ay
    len_sq = dx * dx + dy * dy
    if len_sq <= 1e-15:
        return (math.hypot(px - ax, py - ay), (ax, ay))
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / len_sq))
    qx = ax + t * dx
    qy = ay + t * dy
    return (math.hypot(px - qx, py - qy), (qx, qy))


def closest_points_segments(
    a0: tuple[float, float],
    a1: tuple[float, float],
    b0: tuple[float, float],
    b1: tuple[float, float],
) -> tuple[float, tuple[float, float], tuple[float, float]]:
    """Return (distance_m, closest_on_A, closest_on_B) between two segments."""
    hit, ipt = segment_intersection(a0, a1, b0, b1)
    if hit and ipt is not None:
        return (0.0, ipt, ipt)

    best_d = math.inf
    best_pa = a0
    best_pb = b0

    for px, py in (a0, a1):
        d, q = point_to_segment_closest(px, py, b0[0], b0[1], b1[0], b1[1])
        if d < best_d:
            best_d = d
            best_pa = (px, py)
            best_pb = q

    for px, py in (b0, b1):
        d, q = point_to_segment_closest(px, py, a0[0], a0[1], a1[0], a1[1])
        if d < best_d:
            best_d = d
            best_pa = q
            best_pb = (px, py)

    return (best_d, best_pa, best_pb)


def point_in_polygon(
    px: float, py: float, vertices_m: tuple[tuple[float, float], ...]
) -> bool:
    """Ray-casting + boundary-inclusive point-in-polygon test."""
    n = len(vertices_m)
    for i in range(n):
        ax, ay = vertices_m[i]
        bx, by = vertices_m[(i + 1) % n]
        d, _ = point_to_segment_closest(px, py, ax, ay, bx, by)
        if d <= 1e-10:
            return True

    inside = False
    j = n - 1
    for i in range(n):
        xi, yi = vertices_m[i]
        xj, yj = vertices_m[j]
        if ((yi > py) != (yj > py)) and (
            px < (xj - xi) * (py - yi) / (yj - yi + 1e-18) + xi
        ):
            inside = not inside
        j = i
    return inside


def polygon_intersection(
    verts_a: tuple[tuple[float, float], ...],
    verts_b: tuple[tuple[float, float], ...],
) -> bool:
    """Return True iff two simple polygons overlap or touch."""
    na = len(verts_a)
    nb = len(verts_b)
    for i in range(na):
        a0, a1 = verts_a[i], verts_a[(i + 1) % na]
        for j in range(nb):
            b0, b1 = verts_b[j], verts_b[(j + 1) % nb]
            hit, _ = segment_intersection(a0, a1, b0, b1)
            if hit:
                return True
    if point_in_polygon(verts_a[0][0], verts_a[0][1], verts_b):
        return True
    if point_in_polygon(verts_b[0][0], verts_b[0][1], verts_a):
        return True
    return False


def closest_points_point_polygon(
    px: float, py: float, poly: PolygonGeometry
) -> tuple[float, tuple[float, float], bool]:
    """Return (unsigned_boundary_distance_m, closest_boundary_pt, is_inside)."""
    inside = point_in_polygon(px, py, poly.vertices_m)
    best_d = math.inf
    best_pt = poly.vertices_m[0]
    for a, b in poly.edges_m():
        d, q = point_to_segment_closest(px, py, a[0], a[1], b[0], b[1])
        if d < best_d:
            best_d = d
            best_pt = q
    return (best_d, best_pt, inside)


def closest_points_polygons(
    poly_a: PolygonGeometry, poly_b: PolygonGeometry
) -> tuple[float, tuple[float, float], tuple[float, float], bool]:
    """Return (signed_clearance_m, closest_on_A, closest_on_B, is_overlapping)."""
    overlapping = polygon_intersection(poly_a.vertices_m, poly_b.vertices_m)
    best_d = math.inf
    best_pa = poly_a.vertices_m[0]
    best_pb = poly_b.vertices_m[0]

    for a0, a1 in poly_a.edges_m():
        for b0, b1 in poly_b.edges_m():
            d, pa, pb = closest_points_segments(a0, a1, b0, b1)
            if d < best_d:
                best_d = d
                best_pa = pa
                best_pb = pb

    if overlapping:
        # Estimate signed penetration depth (negative) from minimum vertex-to-other-boundary
        min_pen = math.inf
        pen_pa = best_pa
        pen_pb = best_pb
        for vx, vy in poly_a.vertices_m:
            if point_in_polygon(vx, vy, poly_b.vertices_m):
                d, q, _ = closest_points_point_polygon(vx, vy, poly_b)
                if d < min_pen:
                    min_pen = d
                    pen_pa = (vx, vy)
                    pen_pb = q
        for vx, vy in poly_b.vertices_m:
            if point_in_polygon(vx, vy, poly_a.vertices_m):
                d, q, _ = closest_points_point_polygon(vx, vy, poly_a)
                if d < min_pen:
                    min_pen = d
                    pen_pa = q
                    pen_pb = (vx, vy)
        signed = -min_pen if math.isfinite(min_pen) else -best_d
        return (signed, pen_pa, pen_pb, True)

    return (best_d, best_pa, best_pb, False)


def _to_polygon_if_rect(geom: Geometry) -> Geometry:
    if isinstance(geom, OrientedRectGeometry):
        return geom.as_polygon()
    return geom


def compute_geometry_distance(geom_a: Geometry, geom_b: Geometry) -> GeometryDistanceResult:
    """Compute exact center distance, signed surface clearance, overlap, and closest
    boundary points for any pair of supported 2D geometries.
    """
    ca = geom_a.center_m
    cb = geom_b.center_m
    center_dist = math.hypot(cb[0] - ca[0], cb[1] - ca[1])

    # Handle CapsuleGeometry via its medial segment minus radius_m
    if isinstance(geom_a, CapsuleGeometry) and isinstance(geom_b, CapsuleGeometry):
        a0, a1 = geom_a.spine_segment_m()
        b0, b1 = geom_b.spine_segment_m()
        seg_d, pa, pb = closest_points_segments(a0, a1, b0, b1)
        clr = seg_d - geom_a.radius_m - geom_b.radius_m
        ux, uy = _unit_vec(pa, pb, geom_a.heading_rad)
        return GeometryDistanceResult(
            center_distance_m=center_dist,
            surface_clearance_m=clr,
            is_overlapping=clr <= 0.0,
            closest_point_a_m=(pa[0] + ux * geom_a.radius_m, pa[1] + uy * geom_a.radius_m),
            closest_point_b_m=(pb[0] - ux * geom_b.radius_m, pb[1] - uy * geom_b.radius_m),
        )

    if isinstance(geom_a, CapsuleGeometry):
        a0, a1 = geom_a.spine_segment_m()
        if isinstance(geom_b, CircleGeometry):
            d, q = point_to_segment_closest(
                geom_b.center_x_m, geom_b.center_y_m, a0[0], a0[1], a1[0], a1[1]
            )
            clr = d - geom_a.radius_m - geom_b.radius_m
            ux, uy = _unit_vec(q, cb, geom_a.heading_rad)
            return GeometryDistanceResult(
                center_distance_m=center_dist,
                surface_clearance_m=clr,
                is_overlapping=clr <= 0.0,
                closest_point_a_m=(q[0] + ux * geom_a.radius_m, q[1] + uy * geom_a.radius_m),
                closest_point_b_m=(cb[0] - ux * geom_b.radius_m, cb[1] - uy * geom_b.radius_m),
            )
        poly_b = _to_polygon_if_rect(geom_b)
        assert isinstance(poly_b, PolygonGeometry)
        seg_poly = PolygonGeometry((a0, a1, (a1[0] + 1e-6, a1[1] + 1e-6))) if False else None
        # Exact segment-to-polygon distance
        d, pa, pb, inside = _segment_to_polygon_closest(a0, a1, poly_b)
        clr = (-d - geom_a.radius_m) if inside else (d - geom_a.radius_m)
        ux, uy = _unit_vec(pa, pb, geom_a.heading_rad)
        return GeometryDistanceResult(
            center_distance_m=center_dist,
            surface_clearance_m=clr,
            is_overlapping=clr <= 0.0,
            closest_point_a_m=(pa[0] + ux * geom_a.radius_m, pa[1] + uy * geom_a.radius_m),
            closest_point_b_m=pb,
        )

    if isinstance(geom_b, CapsuleGeometry):
        rev = compute_geometry_distance(geom_b, geom_a)
        return GeometryDistanceResult(
            center_distance_m=rev.center_distance_m,
            surface_clearance_m=rev.surface_clearance_m,
            is_overlapping=rev.is_overlapping,
            closest_point_a_m=rev.closest_point_b_m,
            closest_point_b_m=rev.closest_point_a_m,
        )

    # Circle vs Circle
    if isinstance(geom_a, CircleGeometry) and isinstance(geom_b, CircleGeometry):
        clr = center_dist - geom_a.radius_m - geom_b.radius_m
        ux, uy = _unit_vec(ca, cb, 0.0)
        return GeometryDistanceResult(
            center_distance_m=center_dist,
            surface_clearance_m=clr,
            is_overlapping=clr <= 0.0,
            closest_point_a_m=(ca[0] + ux * geom_a.radius_m, ca[1] + uy * geom_a.radius_m),
            closest_point_b_m=(cb[0] - ux * geom_b.radius_m, cb[1] - uy * geom_b.radius_m),
        )

    # Circle vs OrientedRect / Polygon
    if isinstance(geom_a, CircleGeometry):
        poly_b = _to_polygon_if_rect(geom_b)
        assert isinstance(poly_b, PolygonGeometry)
        d, pb, inside = closest_points_point_polygon(ca[0], ca[1], poly_b)
        clr = (-d - geom_a.radius_m) if inside else (d - geom_a.radius_m)
        ux, uy = _unit_vec(ca, pb, 0.0)
        pa = (ca[0] - ux * geom_a.radius_m, ca[1] - uy * geom_a.radius_m) if inside else (
            ca[0] + ux * geom_a.radius_m,
            ca[1] + uy * geom_a.radius_m,
        )
        return GeometryDistanceResult(
            center_distance_m=center_dist,
            surface_clearance_m=clr,
            is_overlapping=clr <= 0.0,
            closest_point_a_m=pa,
            closest_point_b_m=pb,
        )

    if isinstance(geom_b, CircleGeometry):
        rev = compute_geometry_distance(geom_b, geom_a)
        return GeometryDistanceResult(
            center_distance_m=rev.center_distance_m,
            surface_clearance_m=rev.surface_clearance_m,
            is_overlapping=rev.is_overlapping,
            closest_point_a_m=rev.closest_point_b_m,
            closest_point_b_m=rev.closest_point_a_m,
        )

    # OrientedRect / Polygon vs OrientedRect / Polygon
    poly_a = _to_polygon_if_rect(geom_a)
    poly_b = _to_polygon_if_rect(geom_b)
    assert isinstance(poly_a, PolygonGeometry) and isinstance(poly_b, PolygonGeometry)
    clr, pa, pb, overlapping = closest_points_polygons(poly_a, poly_b)
    return GeometryDistanceResult(
        center_distance_m=center_dist,
        surface_clearance_m=clr,
        is_overlapping=overlapping or clr <= 0.0,
        closest_point_a_m=pa,
        closest_point_b_m=pb,
    )


def _unit_vec(
    p0: tuple[float, float], p1: tuple[float, float], fallback_angle: float
) -> tuple[float, float]:
    dx = p1[0] - p0[0]
    dy = p1[1] - p0[1]
    d = math.hypot(dx, dy)
    if d <= 1e-12:
        return (math.cos(fallback_angle), math.sin(fallback_angle))
    return (dx / d, dy / d)


def _segment_to_polygon_closest(
    a0: tuple[float, float], a1: tuple[float, float], poly: PolygonGeometry
) -> tuple[float, tuple[float, float], tuple[float, float], bool]:
    for e0, e1 in poly.edges_m():
        hit, ipt = segment_intersection(a0, a1, e0, e1)
        if hit and ipt is not None:
            return (0.0, ipt, ipt, True)
    inside = point_in_polygon(a0[0], a0[1], poly.vertices_m)
    best_d = math.inf
    best_pa = a0
    best_pb = poly.vertices_m[0]
    for e0, e1 in poly.edges_m():
        d, pa, pb = closest_points_segments(a0, a1, e0, e1)
        if d < best_d:
            best_d = d
            best_pa = pa
            best_pb = pb
    return (best_d, best_pa, best_pb, inside)


def geometry_bounding_box_m(geom: Geometry) -> tuple[float, float, float, float]:
    """Return tight axis-aligned bounding box (min_x, min_y, max_x, max_y) in meters."""
    if isinstance(geom, CircleGeometry):
        return (
            geom.center_x_m - geom.radius_m,
            geom.center_y_m - geom.radius_m,
            geom.center_x_m + geom.radius_m,
            geom.center_y_m + geom.radius_m,
        )
    if isinstance(geom, OrientedRectGeometry):
        pts = geom.corners_m()
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        return (min(xs), min(ys), max(xs), max(ys))
    if isinstance(geom, PolygonGeometry):
        xs = [p[0] for p in geom.vertices_m]
        ys = [p[1] for p in geom.vertices_m]
        return (min(xs), min(ys), max(xs), max(ys))
    # CapsuleGeometry
    a0, a1 = geom.spine_segment_m()
    return (
        min(a0[0], a1[0]) - geom.radius_m,
        min(a0[1], a1[1]) - geom.radius_m,
        max(a0[0], a1[0]) + geom.radius_m,
        max(a0[1], a1[1]) + geom.radius_m,
    )


def geometry_overlaps_cell(
    geom: Geometry,
    cell_min_x_m: float,
    cell_min_y_m: float,
    cell_max_x_m: float,
    cell_max_y_m: float,
) -> bool:
    """Return True iff `geom` physically overlaps or touches the axis-aligned cell."""
    if isinstance(geom, CircleGeometry):
        cx = max(cell_min_x_m, min(geom.center_x_m, cell_max_x_m))
        cy = max(cell_min_y_m, min(geom.center_y_m, cell_max_y_m))
        dx = geom.center_x_m - cx
        dy = geom.center_y_m - cy
        return (dx * dx + dy * dy) <= (geom.radius_m * geom.radius_m + 1e-12)

    cell_w = cell_max_x_m - cell_min_x_m
    cell_h = cell_max_y_m - cell_min_y_m
    cell_rect = OrientedRectGeometry(
        0.5 * (cell_min_x_m + cell_max_x_m),
        0.5 * (cell_min_y_m + cell_max_y_m),
        width_m=cell_h,
        length_m=cell_w,
        heading_rad=0.0,
    )
    return compute_geometry_distance(geom, cell_rect).is_overlapping
