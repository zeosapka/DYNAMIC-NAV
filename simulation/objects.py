"""Universal object model, motion behaviors, interactive editor, and measurement tool.

Strictly independent of Pygame. All quantities are in SI units (m, s, rad, m/s, rad/s, m/s^2).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum
from typing import Sequence

from config import DEFAULT_CONFIG, EditorConfig, ObjectDefaultsConfig, ObjectDefaultSpec, ThemeConfig
from simulation.entities import Robot, wrap_angle_rad
from simulation.geometry import (
    CapsuleGeometry,
    CircleGeometry,
    Geometry,
    OrientedRectGeometry,
    PolygonGeometry,
    compute_geometry_distance,
    validate_polygon_vertices,
)


class ObjectCategory(str, Enum):
    """Editable object categories supported in the simulator."""

    STATIC_OBSTACLE = "STATIC_OBSTACLE"
    DYNAMIC_OBSTACLE = "DYNAMIC_OBSTACLE"
    PASSENGER_CAR = "PASSENGER_CAR"
    TRUCK = "TRUCK"
    BUS = "BUS"
    FORKLIFT = "FORKLIFT"
    PEDESTRIAN = "PEDESTRIAN"
    CYCLIST = "CYCLIST"
    CUSTOM = "CUSTOM"

    @property
    def label_tr(self) -> str:
        return _CATEGORY_LABELS_TR[self]


_CATEGORY_LABELS_TR: dict[ObjectCategory, str] = {
    ObjectCategory.STATIC_OBSTACLE: "Statik Engel",
    ObjectCategory.DYNAMIC_OBSTACLE: "Dinamik Engel",
    ObjectCategory.PASSENGER_CAR: "Binek Araç",
    ObjectCategory.TRUCK: "Kamyon",
    ObjectCategory.BUS: "Otobüs",
    ObjectCategory.FORKLIFT: "Forklift",
    ObjectCategory.PEDESTRIAN: "Yaya",
    ObjectCategory.CYCLIST: "Bisikletli",
    ObjectCategory.CUSTOM: "Özel Nesne",
}


class MotionBehaviorType(str, Enum):
    """Supported deterministic SI-unit motion behaviors."""

    STATIONARY = "STATIONARY"
    CONSTANT_STRAIGHT = "CONSTANT_STRAIGHT"
    HEAD_ON = "HEAD_ON"
    CROSSING = "CROSSING"
    SAME_DIRECTION = "SAME_DIRECTION"
    LANE_CHANGE = "LANE_CHANGE"
    ACCELERATING = "ACCELERATING"
    DECELERATING = "DECELERATING"
    STOP_AND_GO = "STOP_AND_GO"
    USER_CONTROLLED = "USER_CONTROLLED"

    @property
    def label_tr(self) -> str:
        return _BEHAVIOR_LABELS_TR[self]


_BEHAVIOR_LABELS_TR: dict[MotionBehaviorType, str] = {
    MotionBehaviorType.STATIONARY: "Sabit",
    MotionBehaviorType.CONSTANT_STRAIGHT: "Sabit Doğrusal",
    MotionBehaviorType.HEAD_ON: "Karşıdan Gelen",
    MotionBehaviorType.CROSSING: "Çapraz Geçiş",
    MotionBehaviorType.SAME_DIRECTION: "Aynı Yönde",
    MotionBehaviorType.LANE_CHANGE: "Şerit Değiştirme",
    MotionBehaviorType.ACCELERATING: "Hızlanan",
    MotionBehaviorType.DECELERATING: "Yavaşlayan",
    MotionBehaviorType.STOP_AND_GO: "Dur-Kalk",
    MotionBehaviorType.USER_CONTROLLED: "Kullanıcı Kontrollü",
}


@dataclass
class PhysicalProperties:
    """Physical footprint parameters in SI meters."""

    geometry_type: str  # GeometryType value
    width_m: float
    length_m: float
    radius_m: float
    local_vertices_m: tuple[tuple[float, float], ...] = ()


@dataclass
class MotionState:
    """Kinematic and behavior state of a SimObject in SI units."""

    x_m: float
    y_m: float
    heading_rad: float = 0.0
    linear_speed_mps: float = 0.0
    angular_speed_radps: float = 0.0
    acceleration_mps2: float = 0.0
    max_speed_mps: float = 8.0
    min_speed_mps: float = 0.0
    behavior: MotionBehaviorType = MotionBehaviorType.STATIONARY
    waypoints_m: tuple[tuple[float, float], ...] = ()
    patrol_min_x_m: float = 0.5
    patrol_max_x_m: float = 19.5
    patrol_min_y_m: float = 0.5
    patrol_max_y_m: float = 14.5
    move_duration_s: float = 2.0
    stop_duration_s: float = 1.5
    lane_amplitude_m: float = 1.5
    lane_frequency_hz: float = 0.25
    elapsed_s: float = 0.0
    phase_timer_s: float = 0.0
    stopped_phase: bool = False
    active: bool = True

    @property
    def vx_mps(self) -> float:
        if not self.active or (
            self.behavior == MotionBehaviorType.STOP_AND_GO and self.stopped_phase
        ):
            return 0.0
        return self.linear_speed_mps * math.cos(self.heading_rad)

    @property
    def vy_mps(self) -> float:
        if not self.active or (
            self.behavior == MotionBehaviorType.STOP_AND_GO and self.stopped_phase
        ):
            return 0.0
        return self.linear_speed_mps * math.sin(self.heading_rad)


@dataclass
class SafetyParameters:
    """Per-object safety thresholds in SI units."""

    safety_margin_m: float = 0.25
    ttc_warning_s: float = 3.0


@dataclass(frozen=True)
class RenderStyle:
    """Pure RGB visual metadata decoupled from Pygame."""

    fill_rgb: tuple[int, int, int]
    outline_rgb: tuple[int, int, int]


def _default_spec_for_category(
    category: ObjectCategory, defaults: ObjectDefaultsConfig
) -> ObjectDefaultSpec:
    mapping = {
        ObjectCategory.STATIC_OBSTACLE: defaults.static_obstacle,
        ObjectCategory.DYNAMIC_OBSTACLE: defaults.dynamic_obstacle,
        ObjectCategory.PASSENGER_CAR: defaults.passenger_car,
        ObjectCategory.TRUCK: defaults.truck,
        ObjectCategory.BUS: defaults.bus,
        ObjectCategory.FORKLIFT: defaults.forklift,
        ObjectCategory.PEDESTRIAN: defaults.pedestrian,
        ObjectCategory.CYCLIST: defaults.cyclist,
        ObjectCategory.CUSTOM: defaults.custom,
    }
    return mapping[category]


def _default_style_for_category(
    category: ObjectCategory, theme: ThemeConfig
) -> RenderStyle:
    mapping = {
        ObjectCategory.STATIC_OBSTACLE: RenderStyle(theme.obstacle_fill, theme.obstacle_border),
        ObjectCategory.DYNAMIC_OBSTACLE: RenderStyle(theme.dyn_obs_fill, theme.dyn_obs_outline),
        ObjectCategory.PASSENGER_CAR: RenderStyle(theme.adas_vehicle_car, (191, 219, 254)),
        ObjectCategory.TRUCK: RenderStyle(theme.adas_vehicle_truck, (221, 214, 254)),
        ObjectCategory.BUS: RenderStyle(theme.adas_vehicle_bus, (254, 240, 138)),
        ObjectCategory.FORKLIFT: RenderStyle(theme.adas_vehicle_forklift, (254, 215, 170)),
        ObjectCategory.PEDESTRIAN: RenderStyle(theme.adas_pedestrian, (167, 243, 208)),
        ObjectCategory.CYCLIST: RenderStyle(theme.adas_cyclist, (165, 243, 252)),
        ObjectCategory.CUSTOM: RenderStyle(theme.adas_custom, (251, 207, 232)),
    }
    return mapping[category]


class SimObject:
    """Universal simulation entity separating geometry, physics, motion, safety, and style."""

    def __init__(
        self,
        object_id: str,
        name: str,
        category: ObjectCategory,
        physics: PhysicalProperties,
        motion: MotionState,
        safety: SafetyParameters,
        style: RenderStyle,
    ) -> None:
        self.object_id = object_id
        self.name = name
        self.category = category
        self.physics = physics
        self.motion = motion
        self.safety = safety
        self.style = style
        self._initial_motion = MotionState(**vars(motion))
        self._base_y_m = motion.y_m
        self._waypoint_idx = 0

    def geometry(self) -> Geometry:
        """Build exact world-space SI geometry from current motion & physical state."""
        return self._geometry_at_pose(
            self.motion.x_m, self.motion.y_m, self.motion.heading_rad
        )

    def _geometry_at_pose(self, x_m: float, y_m: float, heading_rad: float) -> Geometry:
        gt = self.physics.geometry_type
        if gt == "CIRCLE":
            return CircleGeometry(x_m, y_m, self.physics.radius_m)
        if gt == "CAPSULE":
            return CapsuleGeometry(
                x_m, y_m, self.physics.length_m, self.physics.radius_m, heading_rad
            )
        if gt == "POLYGON" and len(self.physics.local_vertices_m) >= 3:
            cos_h = math.cos(heading_rad)
            sin_h = math.sin(heading_rad)
            world_verts = tuple(
                (
                    x_m + lx * cos_h - ly * sin_h,
                    y_m + lx * sin_h + ly * cos_h,
                )
                for lx, ly in self.physics.local_vertices_m
            )
            return PolygonGeometry(world_verts)
        return OrientedRectGeometry(
            x_m, y_m, self.physics.width_m, self.physics.length_m, heading_rad
        )

    def predict_geometry_at(self, t_offset_s: float) -> Geometry:
        """Predict future geometry using ONLY current observed velocity & heading
        (no hidden future waypoint schedules or future acceleration changes).
        """
        if not self.motion.active or self.motion.behavior in (
            MotionBehaviorType.STATIONARY,
            MotionBehaviorType.USER_CONTROLLED,
        ):
            return self.geometry()
        dt = max(0.0, t_offset_s)
        vx = self.motion.vx_mps
        vy = self.motion.vy_mps
        px = self.motion.x_m + vx * dt
        py = self.motion.y_m + vy * dt
        p_heading = wrap_angle_rad(
            self.motion.heading_rad + self.motion.angular_speed_radps * dt
        )
        return self._geometry_at_pose(px, py, p_heading)

    def step(self, dt_s: float) -> None:
        """Advance object state by delta-time `dt_s` (seconds)."""
        if dt_s <= 0.0 or not self.motion.active:
            return

        m = self.motion
        m.elapsed_s += dt_s
        b = m.behavior

        if b == MotionBehaviorType.STATIONARY or self.category == ObjectCategory.STATIC_OBSTACLE:
            return

        if b == MotionBehaviorType.STOP_AND_GO:
            m.phase_timer_s += dt_s
            if m.stopped_phase:
                if m.phase_timer_s >= m.stop_duration_s:
                    m.stopped_phase = False
                    m.phase_timer_s = 0.0
                return
            if m.phase_timer_s >= m.move_duration_s:
                m.stopped_phase = True
                m.phase_timer_s = 0.0
                return

        if b == MotionBehaviorType.ACCELERATING:
            accel = abs(m.acceleration_mps2) if abs(m.acceleration_mps2) > 1e-6 else 1.0
            m.linear_speed_mps = min(m.max_speed_mps, m.linear_speed_mps + accel * dt_s)
        elif b == MotionBehaviorType.DECELERATING:
            decel = abs(m.acceleration_mps2) if abs(m.acceleration_mps2) > 1e-6 else 1.5
            m.linear_speed_mps = max(m.min_speed_mps, m.linear_speed_mps - decel * dt_s)

        if b == MotionBehaviorType.LANE_CHANGE:
            omega_lane = 2.0 * math.pi * m.lane_frequency_hz
            vy_lane = m.lane_amplitude_m * omega_lane * math.cos(omega_lane * m.elapsed_s)
            vx_fwd = m.linear_speed_mps * math.cos(m.heading_rad)
            m.x_m += vx_fwd * dt_s
            m.y_m = self._base_y_m + m.lane_amplitude_m * math.sin(omega_lane * m.elapsed_s)
            m.heading_rad = math.atan2(vy_lane, max(1e-4, abs(vx_fwd)))
        else:
            if abs(m.angular_speed_radps) > 1e-9:
                mid_h = m.heading_rad + 0.5 * m.angular_speed_radps * dt_s
                m.x_m += m.linear_speed_mps * math.cos(mid_h) * dt_s
                m.y_m += m.linear_speed_mps * math.sin(mid_h) * dt_s
                m.heading_rad = wrap_angle_rad(m.heading_rad + m.angular_speed_radps * dt_s)
            else:
                m.x_m += m.linear_speed_mps * math.cos(m.heading_rad) * dt_s
                m.y_m += m.linear_speed_mps * math.sin(m.heading_rad) * dt_s

        # Patrol / world boundary bounce for autonomous moving entities
        if b in (
            MotionBehaviorType.HEAD_ON,
            MotionBehaviorType.CROSSING,
            MotionBehaviorType.CONSTANT_STRAIGHT,
            MotionBehaviorType.SAME_DIRECTION,
            MotionBehaviorType.STOP_AND_GO,
        ):
            if m.x_m <= m.patrol_min_x_m:
                m.x_m = m.patrol_min_x_m
                m.heading_rad = wrap_angle_rad(math.pi - m.heading_rad)
            elif m.x_m >= m.patrol_max_x_m:
                m.x_m = m.patrol_max_x_m
                m.heading_rad = wrap_angle_rad(math.pi - m.heading_rad)
            if m.y_m <= m.patrol_min_y_m:
                m.y_m = m.patrol_min_y_m
                m.heading_rad = wrap_angle_rad(-m.heading_rad)
            elif m.y_m >= m.patrol_max_y_m:
                m.y_m = m.patrol_max_y_m
                m.heading_rad = wrap_angle_rad(-m.heading_rad)

    def reset(self) -> None:
        """Restore initial motion state."""
        self.motion = MotionState(**vars(self._initial_motion))
        self._base_y_m = self.motion.y_m
        self._waypoint_idx = 0


def create_default_object(
    object_id: str,
    category: ObjectCategory,
    x_m: float,
    y_m: float,
    defaults: ObjectDefaultsConfig = DEFAULT_CONFIG.object_defaults,
    theme: ThemeConfig = DEFAULT_CONFIG.theme,
) -> SimObject:
    """Create a SimObject with sensible SI defaults for the given category."""
    spec = _default_spec_for_category(category, defaults)
    style = _default_style_for_category(category, theme)

    if category in (ObjectCategory.DYNAMIC_OBSTACLE, ObjectCategory.PEDESTRIAN):
        geom_type = "CIRCLE"
    elif category == ObjectCategory.CUSTOM:
        geom_type = "POLYGON"
    else:
        geom_type = "ORIENTED_RECT"

    local_verts: tuple[tuple[float, float], ...] = ()
    if geom_type == "POLYGON":
        hl = 0.5 * spec.length_m
        hw = 0.5 * spec.width_m
        local_verts = ((hl, 0.0), (0.0, hw), (-hl, hw * 0.8), (-hl, -hw * 0.8), (0.0, -hw))

    behavior = (
        MotionBehaviorType.STATIONARY
        if category == ObjectCategory.STATIC_OBSTACLE
        else MotionBehaviorType.CONSTANT_STRAIGHT
    )
    return SimObject(
        object_id=object_id,
        name=f"{category.label_tr} #{object_id.split('-')[-1]}",
        category=category,
        physics=PhysicalProperties(
            geometry_type=geom_type,
            width_m=spec.width_m,
            length_m=spec.length_m,
            radius_m=spec.radius_m,
            local_vertices_m=local_verts,
        ),
        motion=MotionState(
            x_m=x_m,
            y_m=y_m,
            heading_rad=0.0,
            linear_speed_mps=spec.default_speed_mps,
            max_speed_mps=spec.max_speed_mps,
            acceleration_mps2=spec.max_accel_mps2,
            behavior=behavior,
        ),
        safety=SafetyParameters(safety_margin_m=spec.safety_margin_m),
        style=style,
    )


class ObjectEditorManager:
    """Interactive CRUD and property validation manager for SimObjects."""

    def __init__(
        self,
        editor_cfg: EditorConfig = DEFAULT_CONFIG.editor,
        defaults_cfg: ObjectDefaultsConfig = DEFAULT_CONFIG.object_defaults,
        theme_cfg: ThemeConfig = DEFAULT_CONFIG.theme,
    ) -> None:
        self._cfg = editor_cfg
        self._defaults = defaults_cfg
        self._theme = theme_cfg
        self._objects: list[SimObject] = []
        self._selected_id: str | None = None
        self._next_seq: int = 1

    @property
    def objects(self) -> list[SimObject]:
        return self._objects

    @property
    def selected_id(self) -> str | None:
        return self._selected_id

    @property
    def selected_object(self) -> SimObject | None:
        if self._selected_id is None:
            return None
        for obj in self._objects:
            if obj.object_id == self._selected_id:
                return obj
        return None

    def clear(self) -> None:
        self._objects.clear()
        self._selected_id = None

    def get(self, object_id: str) -> SimObject | None:
        for obj in self._objects:
            if obj.object_id == object_id:
                return obj
        return None

    def add_object(
        self,
        category: ObjectCategory,
        x_m: float,
        y_m: float,
        heading_rad: float = 0.0,
    ) -> SimObject:
        obj_id = f"OBJ-{self._next_seq}"
        self._next_seq += 1
        obj = create_default_object(
            obj_id, category, x_m, y_m, self._defaults, self._theme
        )
        obj.motion.heading_rad = wrap_angle_rad(heading_rad)
        obj._initial_motion = MotionState(**vars(obj.motion))
        self._objects.append(obj)
        self._selected_id = obj.object_id
        return obj

    def select_by_id(self, object_id: str | None) -> SimObject | None:
        if object_id is None:
            self._selected_id = None
            return None
        obj = self.get(object_id)
        self._selected_id = obj.object_id if obj is not None else None
        return obj

    def select_at(self, x_m: float, y_m: float) -> SimObject | None:
        probe = CircleGeometry(x_m, y_m, 1e-3)
        best_obj: SimObject | None = None
        best_clr = self._cfg.selection_tolerance_m
        for obj in reversed(self._objects):
            res = compute_geometry_distance(probe, obj.geometry())
            if res.surface_clearance_m <= best_clr:
                best_clr = res.surface_clearance_m
                best_obj = obj
        self._selected_id = best_obj.object_id if best_obj is not None else None
        return best_obj

    def move_object(self, object_id: str, x_m: float, y_m: float) -> bool:
        obj = self.get(object_id)
        if obj is None or not (math.isfinite(x_m) and math.isfinite(y_m)):
            return False
        obj.motion.x_m = x_m
        obj.motion.y_m = y_m
        obj._base_y_m = y_m
        obj._initial_motion.x_m = x_m
        obj._initial_motion.y_m = y_m
        return True

    def rotate_object(self, object_id: str, heading_rad: float) -> bool:
        obj = self.get(object_id)
        if obj is None or not math.isfinite(heading_rad):
            return False
        wrapped = wrap_angle_rad(heading_rad)
        obj.motion.heading_rad = wrapped
        obj._initial_motion.heading_rad = wrapped
        return True

    def resize_object(
        self,
        object_id: str,
        width_m: float | None = None,
        length_m: float | None = None,
        radius_m: float | None = None,
    ) -> bool:
        obj = self.get(object_id)
        if obj is None:
            return False
        if width_m is not None:
            if not (math.isfinite(width_m) and self._cfg.min_dimension_m <= width_m <= self._cfg.max_dimension_m):
                return False
            obj.physics.width_m = width_m
        if length_m is not None:
            if not (math.isfinite(length_m) and self._cfg.min_dimension_m <= length_m <= self._cfg.max_dimension_m):
                return False
            obj.physics.length_m = length_m
        if radius_m is not None:
            if not (math.isfinite(radius_m) and self._cfg.min_radius_m <= radius_m <= self._cfg.max_radius_m):
                return False
            obj.physics.radius_m = radius_m
        return True

    def duplicate_object(self, object_id: str, offset_m: float = 1.0) -> SimObject | None:
        src = self.get(object_id)
        if src is None:
            return None
        new_id = f"OBJ-{self._next_seq}"
        self._next_seq += 1
        phys = PhysicalProperties(**vars(src.physics))
        mot = MotionState(**vars(src.motion))
        mot.x_m += offset_m
        mot.y_m += offset_m
        saf = SafetyParameters(**vars(src.safety))
        dup = SimObject(
            object_id=new_id,
            name=f"{src.name} (Kopya)",
            category=src.category,
            physics=phys,
            motion=mot,
            safety=saf,
            style=src.style,
        )
        self._objects.append(dup)
        self._selected_id = dup.object_id
        return dup

    def delete_object(self, object_id: str) -> bool:
        for idx, obj in enumerate(self._objects):
            if obj.object_id == object_id:
                self._objects.pop(idx)
                if self._selected_id == object_id:
                    self._selected_id = None
                return True
        return False

    def update_properties(
        self,
        object_id: str,
        *,
        name: str | None = None,
        category: ObjectCategory | None = None,
        geometry_type: str | None = None,
        x_m: float | None = None,
        y_m: float | None = None,
        heading_rad: float | None = None,
        width_m: float | None = None,
        length_m: float | None = None,
        radius_m: float | None = None,
        linear_speed_mps: float | None = None,
        angular_speed_radps: float | None = None,
        acceleration_mps2: float | None = None,
        behavior: MotionBehaviorType | None = None,
        safety_margin_m: float | None = None,
        local_vertices_m: Sequence[tuple[float, float]] | None = None,
    ) -> bool:
        """Validate and apply exact numeric/categorical property updates."""
        obj = self.get(object_id)
        if obj is None:
            return False

        if local_vertices_m is not None:
            validated = validate_polygon_vertices(list(local_vertices_m))
            obj.physics.local_vertices_m = validated

        if geometry_type is not None:
            if geometry_type not in ("CIRCLE", "ORIENTED_RECT", "POLYGON", "CAPSULE"):
                return False
            if geometry_type == "POLYGON" and len(obj.physics.local_vertices_m) < 3:
                hl = 0.5 * obj.physics.length_m
                hw = 0.5 * obj.physics.width_m
                obj.physics.local_vertices_m = (
                    (hl, 0.0),
                    (0.0, hw),
                    (-hl, hw * 0.8),
                    (-hl, -hw * 0.8),
                    (0.0, -hw),
                )
            obj.physics.geometry_type = geometry_type

        if not self.resize_object(object_id, width_m=width_m, length_m=length_m, radius_m=radius_m):
            return False

        if x_m is not None or y_m is not None:
            nx = obj.motion.x_m if x_m is None else x_m
            ny = obj.motion.y_m if y_m is None else y_m
            if not self.move_object(object_id, nx, ny):
                return False

        if heading_rad is not None and not self.rotate_object(object_id, heading_rad):
            return False

        if linear_speed_mps is not None:
            if not math.isfinite(linear_speed_mps):
                return False
            obj.motion.linear_speed_mps = linear_speed_mps
            obj._initial_motion.linear_speed_mps = linear_speed_mps

        if angular_speed_radps is not None:
            if not math.isfinite(angular_speed_radps):
                return False
            obj.motion.angular_speed_radps = angular_speed_radps
            obj._initial_motion.angular_speed_radps = angular_speed_radps

        if acceleration_mps2 is not None:
            if not math.isfinite(acceleration_mps2):
                return False
            obj.motion.acceleration_mps2 = acceleration_mps2
            obj._initial_motion.acceleration_mps2 = acceleration_mps2

        if safety_margin_m is not None:
            if not (math.isfinite(safety_margin_m) and safety_margin_m >= 0.0):
                return False
            obj.safety.safety_margin_m = safety_margin_m

        if name is not None and name.strip():
            obj.name = name.strip()
        if category is not None:
            obj.category = category
            obj.style = _default_style_for_category(category, self._theme)
        if behavior is not None:
            obj.motion.behavior = behavior
            obj._initial_motion.behavior = behavior

        return True

    def step_all(self, dt_s: float) -> None:
        for obj in self._objects:
            obj.step(dt_s)

    def reset_all(self) -> None:
        for obj in self._objects:
            obj.reset()


@dataclass(frozen=True)
class MeasurementResult:
    """Live vehicle-to-vehicle / robot-to-object geometric & kinematic measurement."""

    entity_a_id: str
    entity_b_id: str
    center_distance_m: float
    surface_clearance_m: float
    required_safety_distance_m: float
    safety_violated: bool
    is_overlapping: bool
    relative_speed_mps: float
    closing_speed_mps: float
    ttc_s: float | None
    closest_point_a_m: tuple[float, float]
    closest_point_b_m: tuple[float, float]


class MeasurementTool:
    """Interactive measurement tool supporting Robot and any SimObject pair."""

    def __init__(self, default_safety_margin_m: float = 0.15) -> None:
        self.entity_a_id: str | None = None
        self.entity_b_id: str | None = None
        self.active: bool = False
        self._default_margin_m = default_safety_margin_m

    def select_entity(self, entity_id: str) -> None:
        if self.entity_a_id is None or (self.entity_a_id is not None and self.entity_b_id is not None):
            self.entity_a_id = entity_id
            self.entity_b_id = None
        elif entity_id != self.entity_a_id:
            self.entity_b_id = entity_id

    def clear(self) -> None:
        self.entity_a_id = None
        self.entity_b_id = None

    def compute(
        self, robot: Robot, objects: list[SimObject]
    ) -> MeasurementResult | None:
        if self.entity_a_id is None or self.entity_b_id is None:
            return None

        ga, va, ma = self._resolve_entity(self.entity_a_id, robot, objects)
        gb, vb, mb = self._resolve_entity(self.entity_b_id, robot, objects)
        if ga is None or gb is None:
            return None

        dist = compute_geometry_distance(ga, gb)
        req_safety = max(ma, mb)

        rel_vx = vb[0] - va[0]
        rel_vy = vb[1] - va[1]
        rel_speed = math.hypot(rel_vx, rel_vy)

        # Closing rate along unit vector between closest boundary points (or centers if overlapping)
        dx = dist.closest_point_b_m[0] - dist.closest_point_a_m[0]
        dy = dist.closest_point_b_m[1] - dist.closest_point_a_m[1]
        d_pts = math.hypot(dx, dy)
        if d_pts <= 1e-9:
            ca, cb = ga.center_m, gb.center_m
            dx = cb[0] - ca[0]
            dy = cb[1] - ca[1]
            d_pts = math.hypot(dx, dy)

        if d_pts > 1e-9:
            ux, uy = dx / d_pts, dy / d_pts
            closing_speed = -(rel_vx * ux + rel_vy * uy)
        else:
            closing_speed = 0.0

        if dist.is_overlapping:
            ttc_s: float | None = 0.0
        elif closing_speed > 1e-4:
            ttc_s = dist.surface_clearance_m / closing_speed
        else:
            ttc_s = None

        return MeasurementResult(
            entity_a_id=self.entity_a_id,
            entity_b_id=self.entity_b_id,
            center_distance_m=dist.center_distance_m,
            surface_clearance_m=dist.surface_clearance_m,
            required_safety_distance_m=req_safety,
            safety_violated=dist.surface_clearance_m < req_safety,
            is_overlapping=dist.is_overlapping,
            relative_speed_mps=rel_speed,
            closing_speed_mps=closing_speed,
            ttc_s=ttc_s,
            closest_point_a_m=dist.closest_point_a_m,
            closest_point_b_m=dist.closest_point_b_m,
        )

    def _resolve_entity(
        self, entity_id: str, robot: Robot, objects: list[SimObject]
    ) -> tuple[Geometry | None, tuple[float, float], float]:
        if entity_id == "ROBOT":
            rs = robot.state
            return (
                CircleGeometry(rs.x_m, rs.y_m, robot.radius_m),
                (rs.v_mps * math.cos(rs.theta_rad), rs.v_mps * math.sin(rs.theta_rad)),
                self._default_margin_m,
            )
        for obj in objects:
            if obj.object_id == entity_id:
                return (
                    obj.geometry(),
                    (obj.motion.vx_mps, obj.motion.vy_mps),
                    obj.safety.safety_margin_m,
                )
        return (None, (0.0, 0.0), self._default_margin_m)
