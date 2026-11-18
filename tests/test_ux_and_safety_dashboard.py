"""Tests for multi-view navigation, user guide, and real-time safety analytics."""

import math
import pygame
import pytest

from config import DEFAULT_CONFIG, SafetyThresholdConfig
from interface.app import DynamicNavApp
from interface.views import AppView, GuideTab
from simulation.dynamic_obstacles import DynamicObstacle, MovementBehavior
from simulation.entities import RobotState, StaticObstacle
from simulation.safety import (
    SafetyLevel,
    SafetyTracker,
    classify_safety_level,
    compute_dynamic_obstacle_clearance,
    compute_static_obstacle_clearance,
    compute_ttc_s,
)


@pytest.fixture
def home_app(monkeypatch: pytest.MonkeyPatch) -> DynamicNavApp:
    monkeypatch.setenv("SDL_VIDEODRIVER", "dummy")
    app = DynamicNavApp(DEFAULT_CONFIG, initial_view=AppView.HOME)
    yield app
    pygame.quit()


def _click(app: DynamicNavApp, label: str) -> None:
    btn = next(b for b in app.buttons if b.label == label)
    pygame.event.post(
        pygame.event.Event(
            pygame.MOUSEBUTTONDOWN, {"button": 1, "pos": btn.rect.center}
        )
    )
    app.step_frame(0.02)


def test_home_screen_navigation_and_scenario_launch(home_app: DynamicNavApp) -> None:
    app = home_app
    assert app.current_view == AppView.HOME
    _click(app, "KARŞIDAN GELEN")
    assert app.current_view == AppView.SIMULATION
    assert app.world.active_scenario_name == "HEAD-ON"


def test_returning_from_simulation_to_home(home_app: DynamicNavApp) -> None:
    app = home_app
    _click(app, "YENİ SİMÜLASYON BAŞLAT")
    assert app.current_view == AppView.SIMULATION
    _click(app, "ANA MENÜ")
    assert app.current_view == AppView.HOME


def test_opening_and_closing_user_guide(home_app: DynamicNavApp) -> None:
    app = home_app
    _click(app, "ALGORİTMA BİLGİSİ")
    assert app.current_view == AppView.GUIDE
    assert app.guide_tab == GuideTab.ALGORITHMS

    _click(app, "GÜVENLİK GÖSTERGELERİ")
    assert app.guide_tab == GuideTab.SAFETY

    _click(app, "GERİ DÖN")
    assert app.current_view == AppView.HOME


def test_preserving_simulation_state_while_visiting_guide(
    home_app: DynamicNavApp,
) -> None:
    app = home_app
    _click(app, "YENİ SİMÜLASYON BAŞLAT")
    _click(app, "BAŞLAT")
    for _ in range(15):
        app.step_frame(0.05)

    saved_pose = (app.world.robot.state.x_m, app.world.robot.state.y_m)
    saved_time = app.world.sim_time_s
    assert saved_time > 0.0

    # Visit Guide and return
    _click(app, "KILAVUZ")
    assert app.current_view == AppView.GUIDE
    _click(app, "GERİ DÖN")
    assert app.current_view == AppView.SIMULATION

    assert (app.world.robot.state.x_m, app.world.robot.state.y_m) == pytest.approx(
        saved_pose
    )
    assert app.world.sim_time_s == pytest.approx(saved_time)


def test_static_obstacle_clearance_geometry() -> None:
    obs = StaticObstacle(x_m=5.0, y_m=3.0, width_m=2.0, height_m=4.0)
    # Robot at (4.0, 5.0) with radius 0.35 m -> closest edge is x=5.0 (dist 1.0 m)
    # Surface clearance = 1.0 - 0.35 = 0.65 m
    surf_c, center_d, pt = compute_static_obstacle_clearance(4.0, 5.0, 0.35, obs)
    assert surf_c == pytest.approx(0.65, abs=1e-9)
    assert pt == pytest.approx((5.0, 5.0))
    assert center_d == pytest.approx(2.0, abs=1e-9)  # obs center is (6.0, 5.0)


def test_dynamic_obstacle_clearance_geometry() -> None:
    obs = DynamicObstacle(
        obstacle_id="D1",
        x_m=6.0,
        y_m=2.0,
        vx_mps=0.0,
        vy_mps=0.0,
        radius_m=0.45,
        behavior=MovementBehavior.HEAD_ON,
    )
    # Robot at (4.0, 2.0), radius 0.35 -> center_dist=2.0, surface=2.0 - 0.35 - 0.45 = 1.20 m
    surf_c, center_d, pt = compute_dynamic_obstacle_clearance(4.0, 2.0, 0.35, obs)
    assert center_d == pytest.approx(2.0, abs=1e-9)
    assert surf_c == pytest.approx(1.20, abs=1e-9)
    assert pt == pytest.approx((5.55, 2.0))


def test_safety_threshold_classification() -> None:
    cfg = SafetyThresholdConfig()
    assert classify_safety_level(0.31, cfg) == SafetyLevel.SAFE
    assert classify_safety_level(0.30, cfg) == SafetyLevel.CAUTION
    assert classify_safety_level(0.16, cfg) == SafetyLevel.CAUTION
    assert classify_safety_level(0.15, cfg) == SafetyLevel.CRITICAL
    assert classify_safety_level(0.01, cfg) == SafetyLevel.CRITICAL
    assert classify_safety_level(0.00, cfg) == SafetyLevel.COLLISION
    assert classify_safety_level(-0.05, cfg) == SafetyLevel.COLLISION


def test_collision_vs_safety_warning_distinction() -> None:
    tracker = SafetyTracker(SafetyThresholdConfig())
    obs = StaticObstacle(x_m=5.0, y_m=0.0, width_m=1.0, height_m=10.0)

    # 1) Safety margin violated (0.10 m <= 0.15 m), NOT a collision!
    # Robot at x = 5.0 - 0.35 - 0.10 = 4.55 m
    snap1 = tracker.evaluate(
        1.0, RobotState(4.55, 5.0, 0.0), 0.35, [obs], [], 0.15, record_history=True
    )
    assert snap1.safety_level == SafetyLevel.CRITICAL
    assert tracker.near_miss_count == 1
    assert tracker.warning_count == 1
    assert tracker.collision_count == 0

    # 2) Physical overlap (clearance <= 0.0 m)
    snap2 = tracker.evaluate(
        2.0, RobotState(4.70, 5.0, 0.0), 0.35, [obs], [], 0.15, record_history=True
    )
    assert snap2.safety_level == SafetyLevel.COLLISION
    assert tracker.collision_count == 1


def test_ttc_calculation_and_invalid_cases() -> None:
    # Robot moving at +2.0 m/s toward obstacle at x=5.0, surface clearance 1.0 m -> TTC = 0.5 s
    ttc = compute_ttc_s(
        RobotState(3.65, 5.0, 0.0, v_mps=2.0),
        (5.0, 5.0),
        surface_clearance_m=1.0,
    )
    assert ttc == pytest.approx(0.5, abs=1e-6)

    # Moving away -> TTC is None
    assert (
        compute_ttc_s(
            RobotState(3.65, 5.0, math.pi, v_mps=2.0),
            (5.0, 5.0),
            surface_clearance_m=1.0,
        )
        is None
    )

    # Stationary -> TTC is None
    assert (
        compute_ttc_s(
            RobotState(3.65, 5.0, 0.0, v_mps=0.0),
            (5.0, 5.0),
            surface_clearance_m=1.0,
        )
        is None
    )

    # Already colliding (clearance <= 0) -> TTC is None
    assert (
        compute_ttc_s(
            RobotState(4.80, 5.0, 0.0, v_mps=2.0),
            (5.0, 5.0),
            surface_clearance_m=-0.15,
        )
        is None
    )


def test_safety_event_logging_min_clearance_and_reset() -> None:
    tracker = SafetyTracker(SafetyThresholdConfig())
    obs = StaticObstacle(x_m=5.0, y_m=0.0, width_m=1.0, height_m=10.0)

    tracker.evaluate(0.5, RobotState(4.40, 5.0, 0.0), 0.35, [obs], [], 0.15)  # 0.25 m (CAUTION)
    tracker.evaluate(1.0, RobotState(4.55, 5.0, 0.0), 0.35, [obs], [], 0.15)  # 0.10 m (CRITICAL)

    assert tracker.min_clearance_m == pytest.approx(0.10, abs=1e-9)
    assert len(tracker.event_log) == 2
    assert "ENGEL YAKLAŞIYOR" in tracker.event_log[0].format_line()
    assert "GÜVENLİK PAYI İHLAL EDİLDİ" in tracker.event_log[1].format_line()

    tracker.reset()
    assert tracker.min_clearance_m is None
    assert tracker.warning_count == 0
    assert tracker.collision_count == 0
    assert tracker.near_miss_count == 0
    assert tracker.clearance_history == []
    assert tracker.event_log == []
