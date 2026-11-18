"""Automated tests for dynamic obstacle behaviors, scenarios, and swept collision."""

import math
import numpy as np
import pytest

from config import DEFAULT_CONFIG
from simulation.dynamic_obstacles import (
    DynamicObstacle,
    MovementBehavior,
    evaluate_dynamic_collisions,
    swept_circle_min_distance_m,
)
from simulation.scenarios import ScenarioManager
from simulation.world import SimulationWorld


def test_delta_time_movement_frame_rate_independence() -> None:
    obs_coarse = DynamicObstacle(
        obstacle_id="D1",
        x_m=10.0,
        y_m=5.0,
        vx_mps=-1.5,
        vy_mps=0.5,
        radius_m=0.4,
        behavior=MovementBehavior.CROSSING,
    )
    obs_fine = DynamicObstacle(
        obstacle_id="D2",
        x_m=10.0,
        y_m=5.0,
        vx_mps=-1.5,
        vy_mps=0.5,
        radius_m=0.4,
        behavior=MovementBehavior.CROSSING,
    )

    for _ in range(10):
        obs_coarse.step(0.1)
    for _ in range(100):
        obs_fine.step(0.01)

    assert obs_coarse.x_m == pytest.approx(obs_fine.x_m, abs=1e-9)
    assert obs_coarse.y_m == pytest.approx(obs_fine.y_m, abs=1e-9)
    assert obs_coarse.x_m == pytest.approx(8.5, abs=1e-9)
    assert obs_coarse.y_m == pytest.approx(5.5, abs=1e-9)


def test_velocity_and_position_consistency() -> None:
    obs = DynamicObstacle(
        obstacle_id="V1",
        x_m=2.0,
        y_m=3.0,
        vx_mps=1.2,
        vy_mps=-0.4,
        radius_m=0.3,
        behavior=MovementBehavior.CROSSING,
    )
    dt_s = 0.25
    px, py, nx, ny = obs.step(dt_s)
    assert (nx - px) / dt_s == pytest.approx(1.2, abs=1e-9)
    assert (ny - py) / dt_s == pytest.approx(-0.4, abs=1e-9)
    assert obs.speed_mps == pytest.approx(math.hypot(1.2, -0.4), abs=1e-9)


def test_head_on_behavior_stop_and_reverse() -> None:
    # Stop at bound mode
    obs_stop = DynamicObstacle(
        obstacle_id="H-STOP",
        x_m=5.0,
        y_m=7.5,
        vx_mps=-2.0,
        vy_mps=0.0,
        radius_m=0.4,
        behavior=MovementBehavior.HEAD_ON,
        waypoint_a_m=(3.0, 7.5),
        waypoint_b_m=(6.0, 7.5),
        reverse_at_bounds=False,
    )
    obs_stop.step(2.0)  # Would reach 1.0, clamped to 3.0
    assert obs_stop.x_m == pytest.approx(3.0)
    assert obs_stop.speed_mps == pytest.approx(0.0)

    # Reverse at bound mode
    obs_rev = DynamicObstacle(
        obstacle_id="H-REV",
        x_m=4.0,
        y_m=7.5,
        vx_mps=-2.0,
        vy_mps=0.0,
        radius_m=0.4,
        behavior=MovementBehavior.HEAD_ON,
        waypoint_a_m=(3.0, 7.5),
        waypoint_b_m=(6.0, 7.5),
        reverse_at_bounds=True,
    )
    obs_rev.step(1.0)  # 4.0 -> 3.0 (0.5s), reverses -> 4.0 (0.5s)
    assert obs_rev.x_m == pytest.approx(4.0, abs=1e-9)
    assert obs_rev.vx_mps == pytest.approx(2.0, abs=1e-9)


def test_crossing_behavior_diagonal_trajectory() -> None:
    scen = ScenarioManager.get_scenario("CROSSING")
    obs = scen.dynamic_obstacles[0].instantiate()
    initial_x, initial_y = obs.x_m, obs.y_m
    obs.step(1.0)
    assert obs.x_m < initial_x
    assert obs.y_m > initial_y


def test_corridor_boundary_behavior() -> None:
    scen = ScenarioManager.get_scenario("CORRIDOR")
    obs = scen.dynamic_obstacles[0].instantiate()
    ax = obs.waypoint_a_m[0]  # 5.6
    bx = obs.waypoint_b_m[0]  # 14.4

    for _ in range(400):
        obs.step(0.05)
        assert ax - 1e-9 <= obs.x_m <= bx + 1e-9
        assert obs.y_m == pytest.approx(7.5, abs=1e-9)


def test_stop_and_go_timing() -> None:
    obs = DynamicObstacle(
        obstacle_id="SG",
        x_m=10.0,
        y_m=3.0,
        vx_mps=0.0,
        vy_mps=1.5,
        radius_m=0.4,
        behavior=MovementBehavior.STOP_AND_GO,
        move_duration_s=2.0,
        stop_duration_s=1.5,
    )

    # Move for 2.0 s -> y = 3.0 + 1.5 * 2.0 = 6.0 m
    for _ in range(40):
        obs.step(0.05)
    assert obs.y_m == pytest.approx(6.0, abs=1e-6)
    assert obs.is_stopped is True
    assert obs.speed_mps == pytest.approx(0.0)

    # Stop for 1.5 s -> position remains 6.0 m
    for _ in range(30):
        obs.step(0.05)
    assert obs.y_m == pytest.approx(6.0, abs=1e-6)
    assert obs.is_stopped is False

    # Resume moving for 1.0 s -> y = 6.0 + 1.5 * 1.0 = 7.5 m
    for _ in range(20):
        obs.step(0.05)
    assert obs.y_m == pytest.approx(7.5, abs=1e-6)


def test_deterministic_scenario_initialization_and_static_grid_unchanged() -> None:
    w1 = SimulationWorld(DEFAULT_CONFIG.world, DEFAULT_CONFIG.robot, DEFAULT_CONFIG.simulation)
    w2 = SimulationWorld(DEFAULT_CONFIG.world, DEFAULT_CONFIG.robot, DEFAULT_CONFIG.simulation)

    for name in ("HEAD-ON", "CROSSING", "CORRIDOR", "STOP-AND-GO"):
        scen = ScenarioManager.get_scenario(name)
        w1.load_scenario(scen)
        w2.load_scenario(scen)
        grid_before = w1.occupancy_grid.copy()

        assert w1.robot.state == w2.robot.state
        assert len(w1.dynamic_obstacles) == len(w2.dynamic_obstacles)
        for o1, o2 in zip(w1.dynamic_obstacles, w2.dynamic_obstacles):
            assert (o1.x_m, o1.y_m, o1.vx_mps, o1.vy_mps) == (
                o2.x_m, o2.y_m, o2.vx_mps, o2.vy_mps
            )

        w1.start()
        for _ in range(10):
            w1.step(0.05)
        # Dynamic obstacles must never mutate the static A* occupancy grid.
        assert np.array_equal(w1.occupancy_grid, grid_before)


def test_physical_collision_vs_safety_warning() -> None:
    robot_r = 0.35
    margin = 0.15
    obs = DynamicObstacle(
        obstacle_id="D-COL",
        x_m=5.0,
        y_m=5.0,
        vx_mps=0.0,
        vy_mps=0.0,
        radius_m=0.45,
        behavior=MovementBehavior.HEAD_ON,
    )
    # Physical limit = 0.35 + 0.45 = 0.80 m; safety limit = 0.80 + 0.15 = 0.95 m.

    # Case 1: distance = 0.90 m -> safety warning ONLY, no physical overlap!
    rep_warn = evaluate_dynamic_collisions(
        (4.10, 5.0), (4.10, 5.0), robot_r, margin, [(obs, (5.0, 5.0), (5.0, 5.0))]
    )[0]
    assert rep_warn.safety_warning is True
    assert rep_warn.physical_overlap is False

    # Case 2: distance = 0.70 m -> physical overlap AND safety warning!
    rep_col = evaluate_dynamic_collisions(
        (4.30, 5.0), (4.30, 5.0), robot_r, margin, [(obs, (5.0, 5.0), (5.0, 5.0))]
    )[0]
    assert rep_col.safety_warning is True
    assert rep_col.physical_overlap is True


def test_swept_collision_prevents_tunneling() -> None:
    # Robot stationary at (5.0, 5.0) with radius 0.35 m.
    # Obstacle with radius 0.40 m jumps from (3.0, 5.0) to (7.0, 5.0) in one step.
    # Both start (2.0 m) and end (2.0 m) are well outside 0.75 m,
    # yet the obstacle passed directly through the robot at t = 0.5!
    min_d, t_star = swept_circle_min_distance_m(
        (5.0, 5.0), (5.0, 5.0), (3.0, 5.0), (7.0, 5.0)
    )
    assert min_d == pytest.approx(0.0, abs=1e-9)
    assert t_star == pytest.approx(0.5, abs=1e-9)

    obs = DynamicObstacle(
        obstacle_id="TUNNEL",
        x_m=7.0,
        y_m=5.0,
        vx_mps=4.0,
        vy_mps=0.0,
        radius_m=0.40,
        behavior=MovementBehavior.HEAD_ON,
    )
    rep = evaluate_dynamic_collisions(
        (5.0, 5.0), (5.0, 5.0), 0.35, 0.15, [(obs, (3.0, 5.0), (7.0, 5.0))]
    )[0]
    assert rep.physical_overlap is True
    assert rep.safety_warning is True


def test_multiple_dynamic_obstacles_and_reset_behavior() -> None:
    world = SimulationWorld(
        DEFAULT_CONFIG.world,
        DEFAULT_CONFIG.robot,
        DEFAULT_CONFIG.simulation,
        safety_margin_m=DEFAULT_CONFIG.planner.safety_margin_m,
    )
    world.load_scenario(ScenarioManager.get_scenario("CROSSING"))
    assert len(world.dynamic_obstacles) == 2

    init_positions = [(o.x_m, o.y_m) for o in world.dynamic_obstacles]
    world.start()
    for _ in range(20):
        world.step(0.05)

    moved_positions = [(o.x_m, o.y_m) for o in world.dynamic_obstacles]
    assert moved_positions != init_positions
    assert world.sim_time_s > 0.0

    world.reset()
    reset_positions = [(o.x_m, o.y_m) for o in world.dynamic_obstacles]
    assert reset_positions == init_positions
    assert world.sim_time_s == 0.0
    assert world.collision_reports == []
