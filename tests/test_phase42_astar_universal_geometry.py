"""Automated tests for Phase 4.2 universal geometry integration into A* planning & replanning."""

import math
import numpy as np
import pytest

from algorithms.astar import (
    inflate_occupancy_grid,
    is_segment_collision_free,
    rasterize_geometry_onto_grid,
)
from algorithms.dwa import (
    evaluate_trajectory_with_braking,
    predict_trajectory,
    replan_global_path_around_obstacles,
)
from config import DEFAULT_CONFIG
from simulation.entities import RobotState
from simulation.geometry import (
    CapsuleGeometry,
    CircleGeometry,
    OrientedRectGeometry,
    PolygonGeometry,
    compute_geometry_distance,
)
from simulation.objects import (
    MotionBehaviorType,
    ObjectCategory,
    create_default_object,
)
from simulation.world import SimulationWorld


def test_rasterize_all_four_geometry_types() -> None:
    res_m = 0.2
    rows, cols = 75, 100

    # 1. Circle
    g_circle = np.zeros((rows, cols), dtype=np.uint8)
    circle = CircleGeometry(5.0, 5.0, 0.8)
    rasterize_geometry_onto_grid(g_circle, circle, res_m)
    assert g_circle[25, 25] == 1  # (5.1, 5.1) inside circle
    assert g_circle[0, 0] == 0

    # 2. Oriented rectangle (45 deg)
    g_rect = np.zeros((rows, cols), dtype=np.uint8)
    rect = OrientedRectGeometry(10.0, 7.5, width_m=1.2, length_m=4.0, heading_rad=math.pi / 4.0)
    rasterize_geometry_onto_grid(g_rect, rect, res_m)
    assert g_rect[37, 50] == 1  # center (10.1, 7.5)
    assert np.sum(g_rect) >= 20

    # 3. Polygon (triangle)
    g_poly = np.zeros((rows, cols), dtype=np.uint8)
    tri = PolygonGeometry(((8.0, 6.0), (11.0, 6.0), (9.5, 9.0)))
    rasterize_geometry_onto_grid(g_poly, tri, res_m)
    assert g_poly[35, 47] == 1  # (9.5, 7.1) inside triangle
    assert g_poly[0, 0] == 0

    # 4. Capsule
    g_cap = np.zeros((rows, cols), dtype=np.uint8)
    cap = CapsuleGeometry(12.0, 7.5, length_m=4.0, radius_m=0.5, heading_rad=0.0)
    rasterize_geometry_onto_grid(g_cap, cap, res_m)
    assert g_cap[37, 60] == 1  # (12.1, 7.5)
    assert g_cap[37, 52] == 1  # (10.5, 7.5)


def test_rotated_vehicle_footprint_and_050m_astar_clearance() -> None:
    res_m = 0.2
    grid = np.zeros((75, 100), dtype=np.uint8)
    truck = create_default_object("TRUCK-1", ObjectCategory.TRUCK, 10.0, 7.5)
    truck.motion.heading_rad = math.radians(35.0)
    truck.motion.behavior = MotionBehaviorType.STATIONARY

    ok, path = replan_global_path_around_obstacles(
        base_occupancy_grid=grid,
        resolution_m=res_m,
        robot_x_m=2.0,
        robot_y_m=7.5,
        goal_x_m=18.0,
        goal_y_m=7.5,
        robot_radius_m=0.35,
        safety_margin_m=0.15,
        dynamic_obstacles=[],
        allow_diagonal=True,
        enforce_boundary_clearance=True,
        sim_objects=[truck],
    )
    assert ok is True
    assert len(path) >= 2
    truck_geom = truck.geometry()
    for px, py in path:
        clr = compute_geometry_distance(CircleGeometry(px, py, 0.35), truck_geom).surface_clearance_m
        assert clr >= 0.15 - 1e-6


def test_vehicle_to_vehicle_and_vehicle_to_polygon_interactions() -> None:
    car = create_default_object("CAR-1", ObjectCategory.PASSENGER_CAR, 10.0, 7.5)
    car.motion.heading_rad = math.pi / 2.0
    poly_obj = create_default_object("POLY-1", ObjectCategory.CUSTOM, 10.0, 11.0)
    res = compute_geometry_distance(car.geometry(), poly_obj.geometry())
    px, py = poly_obj.geometry().center_m
    assert res.center_distance_m == pytest.approx(math.hypot(px - 10.0, py - 7.5))
    assert res.surface_clearance_m < 3.5

    # Replanning around both rotated vehicle and custom polygon
    grid = np.zeros((75, 100), dtype=np.uint8)
    ok, path = replan_global_path_around_obstacles(
        base_occupancy_grid=grid,
        resolution_m=0.2,
        robot_x_m=2.0,
        robot_y_m=7.5,
        goal_x_m=18.0,
        goal_y_m=7.5,
        robot_radius_m=0.35,
        safety_margin_m=0.15,
        dynamic_obstacles=[],
        sim_objects=[car, poly_obj],
    )
    assert ok is True
    for px, py in path:
        rc = CircleGeometry(px, py, 0.35)
        assert compute_geometry_distance(rc, car.geometry()).surface_clearance_m >= 0.15 - 1e-6
        assert compute_geometry_distance(rc, poly_obj.geometry()).surface_clearance_m >= 0.15 - 1e-6


def test_dynamic_non_circular_obstacle_detour_and_sealed_corridor() -> None:
    world = SimulationWorld(
        DEFAULT_CONFIG.world, DEFAULT_CONFIG.robot, DEFAULT_CONFIG.simulation
    )
    # Build two corridors via static SimObjects (upper y=6.5..9.0, lower detour y=0..4.5)
    wall_mid = world.object_editor.add_object(ObjectCategory.STATIC_OBSTACLE, 10.0, 5.5)
    world.object_editor.update_properties(wall_mid.object_id, width_m=2.0, length_m=4.0)
    wall_top = world.object_editor.add_object(ObjectCategory.STATIC_OBSTACLE, 10.0, 12.0)
    world.object_editor.update_properties(wall_top.object_id, width_m=6.0, length_m=4.0)
    world.rebuild_occupancy_grid()

    # Block upper corridor with a rotated bus (non-circular OrientedRectGeometry)
    bus = world.object_editor.add_object(ObjectCategory.BUS, 10.0, 7.75, heading_rad=math.pi / 2.0)
    bus.motion.behavior = MotionBehaviorType.CONSTANT_STRAIGHT
    bus.motion.linear_speed_mps = 0.5

    ok, path = replan_global_path_around_obstacles(
        base_occupancy_grid=world.occupancy_grid,
        resolution_m=0.2,
        robot_x_m=2.0,
        robot_y_m=7.5,
        goal_x_m=18.0,
        goal_y_m=7.5,
        robot_radius_m=0.35,
        safety_margin_m=0.15,
        dynamic_obstacles=[],
        sim_objects=world.sim_objects,
    )
    assert ok is True
    # Detour must route below wall_mid (y < 4.5)
    assert any(py < 4.5 for _, py in path)

    # Now seal the lower corridor too with a truck -> no valid route exists
    truck = world.object_editor.add_object(ObjectCategory.TRUCK, 10.0, 2.25, heading_rad=math.pi / 2.0)
    assert truck is not None
    ok_sealed, path_sealed = replan_global_path_around_obstacles(
        base_occupancy_grid=world.occupancy_grid,
        resolution_m=0.2,
        robot_x_m=2.0,
        robot_y_m=7.5,
        goal_x_m=18.0,
        goal_y_m=7.5,
        robot_radius_m=0.35,
        safety_margin_m=0.15,
        dynamic_obstacles=[],
        sim_objects=world.sim_objects,
    )
    assert ok_sealed is False
    assert path_sealed == []


def test_astar_occupancy_and_dwa_collision_consistency() -> None:
    car = create_default_object("CAR-CONS", ObjectCategory.PASSENGER_CAR, 8.0, 7.5)
    car.motion.behavior = MotionBehaviorType.STATIONARY
    car.motion.linear_speed_mps = 0.0
    car.motion.heading_rad = math.radians(20.0)

    # Every A* replanned segment passes DWA's exact geometry evaluation at v=0
    grid = np.zeros((75, 100), dtype=np.uint8)
    ok, path = replan_global_path_around_obstacles(
        base_occupancy_grid=grid,
        resolution_m=0.2,
        robot_x_m=2.0,
        robot_y_m=7.5,
        goal_x_m=16.0,
        goal_y_m=7.5,
        robot_radius_m=0.35,
        safety_margin_m=0.15,
        dynamic_obstacles=[],
        sim_objects=[car],
    )
    assert ok is True
    for px, py in path:
        traj = [(px, py, 0.0)]
        safe, min_c, _, _ = evaluate_trajectory_with_braking(
            traj,
            0.1,
            0.35,
            0.15,
            DEFAULT_CONFIG.world,
            [],
            [],
            v_mps=0.0,
            sim_objects=[car],
        )
        assert safe is True
        assert min_c >= 0.15 - 1e-6


def test_nearby_free_start_cell_when_robot_close_to_dynamic_object() -> None:
    grid = np.zeros((75, 100), dtype=np.uint8)
    # Place a stationary car so robot at (7.6, 7.5) is 0.25 m from front bumper
    # (collision-free since 0.25 >= 0.0, but inside 0.50 m A* inflation buffer).
    # Car length 4.40 m at x=10.4 -> front bumper at x=8.20.
    # Robot at (7.60, 7.5) has surface clearance 8.20 - 7.60 - 0.35 = 0.25 m.
    car = create_default_object("CAR-CLOSE", ObjectCategory.PASSENGER_CAR, 10.4, 7.5)
    car.motion.behavior = MotionBehaviorType.STATIONARY
    car.motion.linear_speed_mps = 0.0

    ok, path = replan_global_path_around_obstacles(
        base_occupancy_grid=grid,
        resolution_m=0.2,
        robot_x_m=7.6,
        robot_y_m=7.5,
        goal_x_m=16.0,
        goal_y_m=7.5,
        robot_radius_m=0.35,
        safety_margin_m=0.15,
        dynamic_obstacles=[],
        sim_objects=[car],
    )
    assert ok is True
    assert len(path) >= 2
    assert is_segment_collision_free(
        (7.6, 7.5),
        path[0],
        grid,
        0.2,
        0.35,
    )
