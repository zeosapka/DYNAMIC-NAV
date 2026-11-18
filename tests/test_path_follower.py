"""Automated tests for autonomous A* waypoint following, safety stops, and Turkish UI."""

import math
import pygame
import pytest

from algorithms.path_follower import WaypointFollower
from config import DEFAULT_CONFIG
from interface.app import DynamicNavApp
from simulation.dynamic_obstacles import DynamicObstacle, MovementBehavior
from simulation.entities import Robot, StaticObstacle
from simulation.world import SimulationState


@pytest.fixture
def headless_app(monkeypatch: pytest.MonkeyPatch) -> DynamicNavApp:
    """Create a DynamicNavApp using Pygame's dummy video driver."""
    monkeypatch.setenv("SDL_VIDEODRIVER", "dummy")
    app = DynamicNavApp(DEFAULT_CONFIG)
    yield app
    pygame.quit()


def _click_button(app: DynamicNavApp, label: str) -> None:
    btn = next(b for b in app.buttons if b.label == label)
    pygame.event.post(
        pygame.event.Event(
            pygame.MOUSEBUTTONDOWN, {"button": 1, "pos": btn.rect.center}
        )
    )


def test_start_with_valid_path_moves_robot(headless_app: DynamicNavApp) -> None:
    app = headless_app
    init_x, init_y = app.world.robot.state.x_m, app.world.robot.state.y_m
    assert app.astar_result is not None and app.astar_result.success

    _click_button(app, "BAŞLAT")
    for _ in range(15):
        app.step_frame(0.05)

    assert app.world.state == SimulationState.RUNNING
    dist_moved = math.hypot(
        app.world.robot.state.x_m - init_x, app.world.robot.state.y_m - init_y
    )
    assert dist_moved > 0.2


def test_start_without_valid_path_shows_turkish_warning(
    headless_app: DynamicNavApp,
) -> None:
    app = headless_app
    app.clear_path()
    assert app.astar_result is None

    _click_button(app, "BAŞLAT")
    app.step_frame(0.05)

    assert app.world.state == SimulationState.READY
    assert app.status_message is not None
    assert "rota" in app.status_message.lower()


def test_autonomous_waypoint_progression(headless_app: DynamicNavApp) -> None:
    app = headless_app
    app.set_dwa_enabled(False)
    start_idx = app.follower.current_index

    _click_button(app, "BAŞLAT")
    for _ in range(40):
        app.step_frame(0.05)

    assert app.follower.current_index > start_idx


def test_robot_reaches_goal_and_displays_target_reached(
    headless_app: DynamicNavApp,
) -> None:
    app = headless_app
    app.set_dwa_enabled(False)
    # Place goal in free space relatively close for fast deterministic completion.
    app._goal_m = (4.0, 2.5)
    app.plan_path()
    assert app.astar_result is not None and app.astar_result.success

    _click_button(app, "BAŞLAT")
    for _ in range(200):
        app.step_frame(0.05)
        if app.follower.target_reached:
            break

    assert app.follower.target_reached is True
    assert app.status_message == "HEDEFE ULAŞILDI"
    dist_to_goal = math.hypot(
        app.world.robot.state.x_m - app.goal_m[0],
        app.world.robot.state.y_m - app.goal_m[1],
    )
    assert dist_to_goal <= DEFAULT_CONFIG.follower.goal_tolerance_m + 0.25


def test_pause_preserves_robot_position(headless_app: DynamicNavApp) -> None:
    app = headless_app
    _click_button(app, "BAŞLAT")
    for _ in range(15):
        app.step_frame(0.05)

    _click_button(app, "DURAKLAT")
    app.step_frame(0.05)
    assert app.world.state == SimulationState.PAUSED
    paused_pose = (app.world.robot.state.x_m, app.world.robot.state.y_m)

    for _ in range(10):
        app.step_frame(0.05)
    assert (app.world.robot.state.x_m, app.world.robot.state.y_m) == pytest.approx(
        paused_pose, abs=1e-9
    )


def test_resume_after_pause_continues_to_goal(headless_app: DynamicNavApp) -> None:
    app = headless_app
    app.set_dwa_enabled(False)
    app._goal_m = (5.0, 2.5)
    app.plan_path()

    _click_button(app, "BAŞLAT")
    for _ in range(10):
        app.step_frame(0.05)

    _click_button(app, "DURAKLAT")
    app.step_frame(0.05)
    paused_x = app.world.robot.state.x_m

    _click_button(app, "BAŞLAT")
    for _ in range(120):
        app.step_frame(0.05)
        if app.follower.target_reached:
            break

    assert app.world.robot.state.x_m > paused_x
    assert app.follower.target_reached is True


def test_reset_restores_initial_conditions_and_clears_warnings(
    headless_app: DynamicNavApp,
) -> None:
    app = headless_app
    app.load_scenario("HEAD-ON")
    init_robot = (app.world.robot.state.x_m, app.world.robot.state.y_m)
    init_dyn = [(o.x_m, o.y_m) for o in app.world.dynamic_obstacles]

    _click_button(app, "BAŞLAT")
    for _ in range(20):
        app.step_frame(0.05)

    assert app.world.sim_time_s > 0.0
    _click_button(app, "SIFIRLA")
    app.step_frame(0.05)

    assert app.world.state == SimulationState.READY
    assert app.world.sim_time_s == 0.0
    assert (app.world.robot.state.x_m, app.world.robot.state.y_m) == pytest.approx(
        init_robot
    )
    assert [(o.x_m, o.y_m) for o in app.world.dynamic_obstacles] == init_dyn
    assert app.status_message is None


def test_dynamic_obstacle_safety_stop(headless_app: DynamicNavApp) -> None:
    app = headless_app
    app.set_dwa_enabled(False)
    app.load_scenario("HEAD-ON")
    # Place dynamic obstacle directly ahead on path inside safety threshold.
    app.world.set_dynamic_obstacles(
        [
            DynamicObstacle(
                obstacle_id="DYN-TEST",
                x_m=3.2,
                y_m=7.5,
                vx_mps=-1.0,
                vy_mps=0.0,
                radius_m=0.45,
                behavior=MovementBehavior.HEAD_ON,
            )
        ]
    )

    _click_button(app, "BAŞLAT")
    for _ in range(20):
        app.step_frame(0.05)
        if app.world.state == SimulationState.PAUSED:
            break

    assert app.world.state == SimulationState.PAUSED
    assert app.status_message is not None
    assert "GÜVENLİK" in app.status_message or "ÇARPIŞMA" in app.status_message


def test_static_obstacle_collision_prevention_during_following() -> None:
    robot = Robot(DEFAULT_CONFIG.robot)
    obs = [StaticObstacle(x_m=2.5, y_m=1.5, width_m=1.0, height_m=1.0)]
    follower = WaypointFollower(DEFAULT_CONFIG.follower)
    # Force a naive straight-line path through the static obstacle to verify Robot.step blocks it.
    follower.set_path([(2.0, 2.0), (5.0, 2.0)])

    for _ in range(40):
        v, w = follower.compute_command(
            robot.state,
            DEFAULT_CONFIG.robot.max_linear_speed_mps,
            DEFAULT_CONFIG.robot.max_angular_speed_radps,
            0.05,
        )
        robot.set_velocity_command(v, w)
        robot.step(0.05, 20.0, 15.0, obs)

    assert robot.state.x_m <= 2.5 - DEFAULT_CONFIG.robot.radius_m + 1e-6


def test_waypoint_follower_delta_time_independence() -> None:
    r1 = Robot(DEFAULT_CONFIG.robot)
    r2 = Robot(DEFAULT_CONFIG.robot)
    f1 = WaypointFollower(DEFAULT_CONFIG.follower)
    f2 = WaypointFollower(DEFAULT_CONFIG.follower)
    path = [(2.0, 2.0), (4.0, 2.0), (5.0, 3.5)]
    f1.set_path(path)
    f2.set_path(path)

    for _ in range(20):
        v, w = f1.compute_command(r1.state, 3.0, 2.8, 0.05)
        r1.set_velocity_command(v, w)
        r1.step(0.05, 20.0, 15.0, [])

    for _ in range(100):
        v, w = f2.compute_command(r2.state, 3.0, 2.8, 0.01)
        r2.set_velocity_command(v, w)
        r2.step(0.01, 20.0, 15.0, [])

    assert r1.state.x_m == pytest.approx(r2.state.x_m, abs=0.15)
    assert r1.state.y_m == pytest.approx(r2.state.y_m, abs=0.15)
