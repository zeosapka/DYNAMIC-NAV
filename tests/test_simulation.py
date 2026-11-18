"""Unit tests for SimulationWorld, Robot kinematics, and static obstacles."""

import math
import pytest

from config import RobotConfig, SimulationConfig, StaticObstacleConfig, WorldConfig
from simulation.world import SimulationState, SimulationWorld


def _make_world(
    obstacles: tuple[StaticObstacleConfig, ...] = (),
) -> SimulationWorld:
    world_cfg = WorldConfig(
        width_m=10.0,
        height_m=10.0,
        grid_resolution_m=1.0,
        major_grid_interval_m=5.0,
        static_obstacles=obstacles,
    )
    robot_cfg = RobotConfig(
        initial_x_m=2.0,
        initial_y_m=2.0,
        initial_theta_rad=0.0,
        radius_m=0.5,
        max_linear_speed_mps=2.0,
        max_angular_speed_radps=2.0,
    )
    sim_cfg = SimulationConfig(max_dt_s=0.1, auto_start=False)
    return SimulationWorld(world_cfg, robot_cfg, sim_cfg)


def test_lifecycle_start_pause_reset() -> None:
    world = _make_world()
    assert world.state == SimulationState.READY

    # Should not advance while READY.
    world.robot.set_velocity_command(1.0, 0.0)
    world.step(0.1)
    assert world.sim_time_s == 0.0
    assert world.robot.state.x_m == pytest.approx(2.0)

    # Start and advance.
    world.start()
    assert world.state == SimulationState.RUNNING
    world.step(0.1)
    assert world.sim_time_s == pytest.approx(0.1)
    assert world.robot.state.x_m == pytest.approx(2.1)

    # Pause freezes simulation.
    world.pause()
    assert world.state == SimulationState.PAUSED
    world.step(0.1)
    assert world.sim_time_s == pytest.approx(0.1)
    assert world.robot.state.x_m == pytest.approx(2.1)

    # Reset restores initial state.
    world.reset()
    assert world.state == SimulationState.READY
    assert world.sim_time_s == 0.0
    assert world.robot.state.x_m == pytest.approx(2.0)
    assert world.robot.state.y_m == pytest.approx(2.0)


def test_delta_time_rotation_and_translation() -> None:
    world = _make_world()
    world.start()
    world.robot.set_velocity_command(0.0, math.pi / 2.0)
    world.step(0.1)
    # 10 steps of 0.1s -> 1.0s total -> rotated pi/2 rad (90 deg).
    for _ in range(9):
        world.step(0.1)

    assert world.robot.state.theta_rad == pytest.approx(math.pi / 2.0, abs=1e-5)

    # Now drive forward along +Y.
    world.robot.set_velocity_command(2.0, 0.0)
    world.step(0.1)
    assert world.robot.state.y_m == pytest.approx(2.2, abs=1e-5)


def test_world_boundary_and_static_obstacle_collision() -> None:
    obs = (StaticObstacleConfig(x_m=3.0, y_m=1.0, width_m=1.0, height_m=2.0),)
    world = _make_world(obstacles=obs)
    world.start()

    # Drive straight into obstacle at x=3.0 (robot starts at x=2.0, radius=0.5).
    world.robot.set_velocity_command(2.0, 0.0)
    for _ in range(20):
        world.step(0.05)

    # Robot center must remain strictly outside obstacle left edge (3.0 - 0.5 = 2.5).
    assert world.robot.state.x_m <= 2.5 + 1e-6

    # Verify occupancy grid marks obstacle cells.
    assert world.occupancy_grid.shape == (10, 10)
    assert world.occupancy_grid[1, 3] == 1
    assert world.occupancy_grid[0, 0] == 0
