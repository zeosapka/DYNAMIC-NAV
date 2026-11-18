"""Automated tests for Phase 4.1 universal geometry, object editor, behaviors, and measurement tool."""

import math
import pytest

from algorithms.dwa import evaluate_trajectory_with_braking, predict_trajectory
from config import DEFAULT_CONFIG
from simulation.coordinates import CoordinateTransform, ScreenRect
from simulation.entities import Robot, RobotState
from simulation.geometry import (
    CapsuleGeometry,
    CircleGeometry,
    OrientedRectGeometry,
    PolygonGeometry,
    compute_geometry_distance,
    point_to_segment_closest,
    polygon_intersection,
    segment_intersection,
)
from simulation.objects import (
    MeasurementTool,
    MotionBehaviorType,
    ObjectCategory,
    ObjectEditorManager,
    create_default_object,
)


def test_geometry_pairings_exact_distances() -> None:
    c1 = CircleGeometry(0.0, 0.0, 1.0)
    c2 = CircleGeometry(3.0, 0.0, 0.5)
    r_cc = compute_geometry_distance(c1, c2)
    assert r_cc.center_distance_m == pytest.approx(3.0)
    assert r_cc.surface_clearance_m == pytest.approx(1.5)
    assert r_cc.is_overlapping is False

    rect = OrientedRectGeometry(4.0, 0.0, width_m=2.0, length_m=2.0, heading_rad=0.0)
    r_cr = compute_geometry_distance(c1, rect)
    assert r_cr.surface_clearance_m == pytest.approx(2.0)
    assert r_cr.closest_point_a_m[0] == pytest.approx(1.0)
    assert r_cr.closest_point_b_m[0] == pytest.approx(3.0)

    tri = PolygonGeometry(((4.0, 0.0), (6.0, 1.0), (6.0, -1.0)))
    r_cp = compute_geometry_distance(c1, tri)
    assert r_cp.surface_clearance_m == pytest.approx(3.0)

    rect2 = OrientedRectGeometry(6.5, 0.0, width_m=2.0, length_m=1.0, heading_rad=0.0)
    r_rr = compute_geometry_distance(rect, rect2)
    assert r_rr.surface_clearance_m == pytest.approx(1.0)

    r_rp = compute_geometry_distance(rect, tri)
    assert r_rp.is_overlapping is True
    assert r_rp.surface_clearance_m <= 0.0

    tri2 = PolygonGeometry(((7.0, 0.0), (8.0, 1.0), (8.0, -1.0)))
    r_pp = compute_geometry_distance(tri, tri2)
    assert r_pp.surface_clearance_m == pytest.approx(1.0)

    cap = CapsuleGeometry(0.0, 0.0, length_m=4.0, radius_m=0.5, heading_rad=0.0)
    r_cap_c = compute_geometry_distance(cap, CircleGeometry(3.0, 0.0, 0.5))
    assert r_cap_c.surface_clearance_m == pytest.approx(0.5)


def test_oriented_rectangle_rotation_and_corners() -> None:
    rect = OrientedRectGeometry(10.0, 5.0, width_m=2.0, length_m=4.0, heading_rad=math.pi / 2.0)
    corners = rect.corners_m()
    assert len(corners) == 4
    xs = [c[0] for c in corners]
    ys = [c[1] for c in corners]
    assert min(xs) == pytest.approx(9.0)
    assert max(xs) == pytest.approx(11.0)
    assert min(ys) == pytest.approx(3.0)
    assert max(ys) == pytest.approx(7.0)


def test_polygon_validation_rejects_invalid_polygons() -> None:
    with pytest.raises(ValueError):
        PolygonGeometry(((0.0, 0.0), (1.0, 0.0)))
    with pytest.raises(ValueError):
        PolygonGeometry(((0.0, 0.0), (1.0, 0.0), (2.0, 0.0)))
    with pytest.raises(ValueError):
        PolygonGeometry(((0.0, 0.0), (2.0, 2.0), (2.0, 0.0), (0.0, 2.0)))
    with pytest.raises(ValueError):
        PolygonGeometry(((0.0, 0.0), (math.nan, 1.0), (1.0, 0.0)))


def test_segment_and_closest_boundary_calculations() -> None:
    d, q = point_to_segment_closest(1.0, 2.0, 0.0, 0.0, 2.0, 0.0)
    assert d == pytest.approx(2.0)
    assert q == pytest.approx((1.0, 0.0))
    hit, ipt = segment_intersection((0.0, 0.0), (2.0, 2.0), (0.0, 2.0), (2.0, 0.0))
    assert hit is True
    assert ipt == pytest.approx((1.0, 1.0))
    assert polygon_intersection(
        ((0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0)),
        ((1.0, 1.0), (3.0, 1.0), (3.0, 3.0), (1.0, 3.0)),
    ) is True


def test_head_on_vehicle_surface_clearance_vs_center_distance() -> None:
    car_a = OrientedRectGeometry(5.0, 7.5, width_m=1.85, length_m=4.40, heading_rad=0.0)
    car_b = OrientedRectGeometry(10.4, 7.5, width_m=1.85, length_m=4.40, heading_rad=math.pi)
    res = compute_geometry_distance(car_a, car_b)
    assert res.center_distance_m == pytest.approx(5.40)
    assert res.surface_clearance_m == pytest.approx(1.00)
    assert res.closest_point_a_m[0] == pytest.approx(7.20)
    assert res.closest_point_b_m[0] == pytest.approx(8.20)


def test_object_editor_crud_and_validation() -> None:
    editor = ObjectEditorManager()
    for cat in ObjectCategory:
        obj = editor.add_object(cat, 5.0, 5.0)
        assert obj.category == cat
    assert len(editor.objects) == len(ObjectCategory)

    first = editor.objects[0]
    assert editor.select_at(5.0, 5.0) is not None
    assert editor.move_object(first.object_id, 8.0, 6.0) is True
    assert editor.rotate_object(first.object_id, math.pi / 4.0) is True
    assert editor.resize_object(first.object_id, width_m=2.0, length_m=5.0) is True
    assert editor.resize_object(first.object_id, width_m=-1.0) is False

    dup = editor.duplicate_object(first.object_id)
    assert dup is not None and dup.object_id != first.object_id
    assert editor.delete_object(dup.object_id) is True
    assert editor.update_properties(first.object_id, linear_speed_mps=3.5, safety_margin_m=0.45) is True


def test_all_motion_behaviors_delta_time_integration() -> None:
    for behavior in MotionBehaviorType:
        obj = create_default_object("T-1", ObjectCategory.PASSENGER_CAR, 5.0, 7.5)
        obj.motion.behavior = behavior
        obj.motion.linear_speed_mps = 2.0
        obj.motion.acceleration_mps2 = 2.0
        obj.motion.angular_speed_radps = 0.0
        x0, y0 = obj.motion.x_m, obj.motion.y_m
        for _ in range(10):
            obj.step(0.05)
        if behavior == MotionBehaviorType.STATIONARY:
            assert (obj.motion.x_m, obj.motion.y_m) == pytest.approx((x0, y0))
        elif behavior == MotionBehaviorType.ACCELERATING:
            assert obj.motion.linear_speed_mps > 2.0
        elif behavior == MotionBehaviorType.DECELERATING:
            assert obj.motion.linear_speed_mps < 2.0
        elif behavior == MotionBehaviorType.LANE_CHANGE:
            assert obj.motion.y_m != pytest.approx(y0)
        else:
            assert math.hypot(obj.motion.x_m - x0, obj.motion.y_m - y0) > 0.5


def test_measurement_tool_live_consistency_and_ttc() -> None:
    robot = Robot(DEFAULT_CONFIG.robot)
    robot.set_initial_pose(2.0, 7.5, 0.0)
    robot.set_velocity_command(1.0, 0.0)
    robot.step(0.0, 20.0, 15.0, [])
    robot._state.v_mps = 1.0

    car = create_default_object("CAR-1", ObjectCategory.PASSENGER_CAR, 10.0, 7.5)
    car.motion.heading_rad = math.pi
    car.motion.linear_speed_mps = 1.0
    car.safety.safety_margin_m = 0.40

    tool = MeasurementTool(default_safety_margin_m=0.15)
    tool.select_entity("ROBOT")
    tool.select_entity("CAR-1")
    m1 = tool.compute(robot, [car])
    assert m1 is not None
    # Robot edge at 2.0 + 0.35 = 2.35; Car front bumper at 10.0 - 2.20 = 7.80 -> 5.45 m
    assert m1.center_distance_m == pytest.approx(8.0)
    assert m1.surface_clearance_m == pytest.approx(5.45)
    assert m1.required_safety_distance_m == pytest.approx(0.40)
    assert m1.closing_speed_mps == pytest.approx(2.0)
    assert m1.ttc_s == pytest.approx(5.45 / 2.0)


def test_dwa_and_rendering_geometry_consistency() -> None:
    car = create_default_object("CAR-DWA", ObjectCategory.PASSENGER_CAR, 6.0, 7.5)
    car.motion.linear_speed_mps = 0.0
    car.motion.behavior = MotionBehaviorType.STATIONARY
    # Car extends from x = 6.0 - 2.2 = 3.80 to 8.20
    traj = predict_trajectory(RobotState(2.0, 7.5, 0.0), v_mps=2.0, omega_radps=0.0, predict_time_s=1.0, predict_dt_s=0.1)
    is_safe, min_c, _, _ = evaluate_trajectory_with_braking(
        traj,
        0.1,
        0.35,
        0.15,
        DEFAULT_CONFIG.world,
        [],
        [],
        v_mps=2.0,
        max_brake_decel_mps2=5.0,
        reaction_time_s=0.1,
        sim_objects=[car],
    )
    assert is_safe is False
    assert min_c < 0.15

    # Rendering transform preserves exact OrientedRectGeometry corners
    tf = CoordinateTransform(20.0, 15.0, ScreenRect(232, 56, 864, 648))
    geom = car.geometry()
    assert isinstance(geom, OrientedRectGeometry)
    screen_pts = [tf.world_to_screen(x, y) for x, y in geom.corners_m()]
    round_trip = [tf.screen_to_world(px, py) for px, py in screen_pts]
    for (ox, oy), (rx, ry) in zip(geom.corners_m(), round_trip):
        assert math.hypot(ox - rx, oy - ry) < 0.05
