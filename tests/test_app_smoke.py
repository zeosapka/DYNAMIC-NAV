"""Integration smoke tests for the Pygame window, buttons, and keyboard controls."""

import os
import pygame
import pytest

from config import DEFAULT_CONFIG
from interface.app import DynamicNavApp
from simulation.world import SimulationState


@pytest.fixture
def headless_app(monkeypatch: pytest.MonkeyPatch) -> DynamicNavApp:
    """Create a DynamicNavApp using Pygame's dummy video driver for automated testing."""
    monkeypatch.setenv("SDL_VIDEODRIVER", "dummy")
    app = DynamicNavApp(DEFAULT_CONFIG)
    yield app
    pygame.quit()


def test_window_dimensions_and_fixed_behavior(headless_app: DynamicNavApp) -> None:
    width, height = headless_app.screen.get_size()
    assert (width, height) == (1440, 900)
    assert (headless_app.screen.get_flags() & pygame.RESIZABLE) == 0


def test_sidebar_buttons_start_pause_reset(headless_app: DynamicNavApp) -> None:
    app = headless_app
    btn_map = {b.label: b for b in app.buttons}
    assert {"BAŞLAT", "DURAKLAT", "SIFIRLA"}.issubset(btn_map.keys())
    assert app.world.state == SimulationState.READY

    # Click BAŞLAT
    pygame.event.post(
        pygame.event.Event(
            pygame.MOUSEBUTTONDOWN,
            {"button": 1, "pos": btn_map["BAŞLAT"].rect.center},
        )
    )
    app.step_frame(0.02)
    assert app.world.state == SimulationState.RUNNING
    assert app.world.sim_time_s > 0.0

    # Click DURAKLAT
    pygame.event.post(
        pygame.event.Event(
            pygame.MOUSEBUTTONDOWN,
            {"button": 1, "pos": btn_map["DURAKLAT"].rect.center},
        )
    )
    app.step_frame(0.02)
    assert app.world.state == SimulationState.PAUSED
    paused_time = app.world.sim_time_s
    app.step_frame(0.02)
    assert app.world.sim_time_s == pytest.approx(paused_time)

    # Click SIFIRLA
    pygame.event.post(
        pygame.event.Event(
            pygame.MOUSEBUTTONDOWN,
            {"button": 1, "pos": btn_map["SIFIRLA"].rect.center},
        )
    )
    app.step_frame(0.02)
    assert app.world.state == SimulationState.READY
    assert app.world.sim_time_s == 0.0


def test_keyboard_controls_move_robot(
    headless_app: DynamicNavApp, monkeypatch: pytest.MonkeyPatch
) -> None:
    app = headless_app
    app.world.start()
    initial_x = app.world.robot.state.x_m

    class FakePressed:
        def __init__(self, active_keys: set[int]) -> None:
            self._active = active_keys

        def __getitem__(self, key: int) -> bool:
            return key in self._active

    monkeypatch.setattr(pygame.key, "get_pressed", lambda: FakePressed({pygame.K_w}))
    app.step_frame(0.05)
    assert app.world.robot.state.x_m > initial_x
