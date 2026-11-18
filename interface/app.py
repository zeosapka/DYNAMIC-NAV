"""Main Pygame application loop managing views, simulation, A*, follower, and safety."""

import math
import os
import sys
import numpy as np
import pygame

from algorithms.astar import (
    AStarPlanner,
    AStarResult,
    inflate_occupancy_grid,
    world_to_grid,
)
from algorithms.dwa import DWAPlanner, replan_global_path_around_obstacles
from algorithms.dwa_interface import DWATelemetryPlaceholder
from algorithms.observation import ObservationFrame, ObservationPipeline
from algorithms.path_follower import WaypointFollower
from config import DEFAULT_CONFIG, ProjectConfig
from interface.controls import Button, KeyboardController, NumericField
from interface.renderer import (
    LEFT_PANEL_WIDTH,
    RIGHT_PANEL_WIDTH,
    InteractionMode,
    SimulationRenderer,
)
from interface.views import AppView, GuideTab, GuideViewRenderer, HomeViewRenderer
from simulation.entities import StaticObstacle
from simulation.objects import (
    MeasurementTool,
    MotionBehaviorType,
    ObjectCategory,
    SimObject,
)
from simulation.safety import SafetyLevel, SafetyTracker
from simulation.scenarios import ScenarioManager
from simulation.world import SimulationState, SimulationWorld


class DynamicNavApp:
    """Top-level interactive Pygame application for DYNAMIC NAV."""

    def __init__(
        self,
        config: ProjectConfig = DEFAULT_CONFIG,
        initial_view: AppView = AppView.SIMULATION,
    ) -> None:
        self._config = config
        os.environ.setdefault("SDL_VIDEO_CENTERED", "1")
        pygame.init()
        if not pygame.display.get_init():
            raise RuntimeError("Pygame görüntü alt sistemi başlatılamadı.")

        flags = pygame.RESIZABLE if config.window.resizable else 0
        self._screen = pygame.display.set_mode(
            (config.window.width, config.window.height),
            flags,
        )
        pygame.display.set_caption(config.window.title)
        pygame.event.pump()
        self._ensure_windows_visibility()
        self._clock = pygame.time.Clock()

        self._world = SimulationWorld(
            world_config=config.world,
            robot_config=config.robot,
            sim_config=config.simulation,
            safety_margin_m=config.planner.safety_margin_m,
        )
        self._renderer = SimulationRenderer(config)
        self._home_renderer = HomeViewRenderer(config)
        self._guide_renderer = GuideViewRenderer(config)
        self._keyboard = KeyboardController(config.robot)
        self._follower = WaypointFollower(config.follower)
        self._dwa_planner = DWAPlanner(
            config.world, config.robot, config.dwa, observation_cfg=config.observation
        )
        self._obs_pipeline = ObservationPipeline(config.observation)
        self._last_observation_frame: ObservationFrame | None = None
        self._use_dwa: bool = True
        self._safety_tracker = SafetyTracker(config.safety)
        self._dwa_telemetry = DWATelemetryPlaceholder()

        self._view: AppView = initial_view
        self._previous_view: AppView = AppView.HOME
        self._guide_tab: GuideTab = GuideTab.TUTORIAL
        self._has_active_sim: bool = False
        self._safety_panel_open: bool = True
        self._esc_confirm_pending: bool = False

        self._start_m: tuple[float, float] = (
            config.robot.initial_x_m,
            config.robot.initial_y_m,
        )
        self._goal_m: tuple[float, float] = (
            config.planner.default_goal_x_m,
            config.planner.default_goal_y_m,
        )
        self._allow_diagonal: bool = config.planner.allow_diagonal
        self._interaction_mode: InteractionMode = InteractionMode.DRIVE
        self._inflated_grid: np.ndarray | None = None
        self._astar_result: AStarResult | None = None
        self._status_message: str | None = None
        self._hovered_tooltip: str | None = None
        self._replan_cooldown_remaining_s: float = 0.0
        self._replan_attempts: int = 0
        self._successful_replans: int = 0
        self._selected_category: ObjectCategory = ObjectCategory.PASSENGER_CAR
        self._measurement_tool = MeasurementTool(config.editor.default_safety_margin_m)
        self._inspector_fields: list[NumericField] = []
        self._dragging_obj_id: str | None = None
        self._clearance_history: list[float] = []
        self._speed_history: list[float] = []
        self._sim_speed_idx: int = 1
        self._demo_mode: bool = True
        self._demo_event_text: str | None = None
        self._demo_event_timer_s: float = 0.0
        self._stop_count: int = 0
        self._was_stopped: bool = False
        self._min_safety_margin_observed_m: float | None = None
        self._collision_occurred: bool = False
        self._dwa_started_announced: bool = False
        self._dyn_obs_announced: bool = False
        self._goal_announced: bool = False

        self._sim_buttons = self._create_sim_buttons()
        self._home_buttons = self._create_home_buttons()
        self._guide_buttons = self._create_guide_buttons()
        self._running = False

        self._rebuild_inflated_grid()
        self.plan_path()
        self._evaluate_safety(record_history=False)
        self._render_current_frame()

    @staticmethod
    def _ensure_windows_visibility() -> None:
        """Ensure the Win32 window is shown normally when using the Windows driver."""
        if sys.platform != "win32" or pygame.display.get_driver() != "windows":
            return
        try:
            import ctypes

            hwnd = pygame.display.get_wm_info().get("window")
            if hwnd:
                ctypes.windll.user32.ShowWindow(hwnd, 1)
        except Exception:
            pass

    @property
    def screen(self) -> pygame.Surface:
        """Return the Pygame display surface."""
        return self._screen

    @property
    def world(self) -> SimulationWorld:
        """Return the underlying simulation world."""
        return self._world

    @property
    def current_view(self) -> AppView:
        """Return the active application view."""
        return self._view

    @property
    def guide_tab(self) -> GuideTab:
        """Return the active tab in the User Guide."""
        return self._guide_tab

    @property
    def buttons(self) -> list[Button]:
        """Return the active view's buttons (simulation buttons when in SIMULATION)."""
        if self._view == AppView.HOME:
            return self._home_buttons
        if self._view == AppView.GUIDE:
            return self._guide_buttons
        return self._sim_buttons

    @property
    def astar_result(self) -> AStarResult | None:
        """Return the latest A* planning result."""
        return self._astar_result

    @property
    def replan_attempts(self) -> int:
        """Return total automatic A* global replanning attempts."""
        return self._replan_attempts

    @property
    def successful_replans(self) -> int:
        """Return count of successful automatic A* global replans."""
        return self._successful_replans

    def attempt_global_replan(self) -> bool:
        """Attempt controlled A* global replanning from current robot pose to goal
        treating active dynamic obstacles as temporary occupied regions.
        """
        self._replan_attempts += 1
        self._replan_cooldown_remaining_s = self._config.dwa.replan_cooldown_s
        st = self._world.robot.state
        obs_frame = getattr(self, "_last_observation_frame", None)
        ok, new_path = replan_global_path_around_obstacles(
            base_occupancy_grid=self._world.occupancy_grid,
            resolution_m=self._config.world.grid_resolution_m,
            robot_x_m=obs_frame.robot.x_m if obs_frame is not None else st.x_m,
            robot_y_m=obs_frame.robot.y_m if obs_frame is not None else st.y_m,
            goal_x_m=self._goal_m[0],
            goal_y_m=self._goal_m[1],
            robot_radius_m=self._config.robot.radius_m,
            safety_margin_m=self._config.planner.safety_margin_m,
            dynamic_obstacles=self._world.dynamic_obstacles,
            allow_diagonal=self._allow_diagonal,
            enforce_boundary_clearance=self._config.planner.enforce_boundary_clearance,
            sim_objects=self._world.sim_objects,
            observation_frame=obs_frame,
        )
        if ok and new_path:
            self._successful_replans += 1
            self._dwa_planner.set_path(new_path)
            self._follower.set_path(new_path)
            self._status_message = "ROTA YENİLENDİ: Dinamik engel aşıldı."
            self._trigger_demo_event("YENİDEN PLANLAMA")
            self._safety_tracker.log_event(
                self._world.sim_time_s, "OTOMATİK ROTA YENİLENDİ", SafetyLevel.CAUTION
            )
            return True

        self._status_message = "ROTA ENGELLENDİ: Güvenli alternatif yok."
        self._safety_tracker.log_event(
            self._world.sim_time_s, "ROTA ENGELLENDİ", SafetyLevel.CRITICAL
        )
        return False

    @property
    def follower(self) -> WaypointFollower:
        """Return the autonomous waypoint follower."""
        return self._follower

    @property
    def dwa_planner(self) -> DWAPlanner:
        """Return the local DWA obstacle avoidance planner."""
        return self._dwa_planner

    @property
    def use_dwa(self) -> bool:
        """Return True when DWA local avoidance is active (False = WaypointFollower)."""
        return self._use_dwa

    def set_dwa_enabled(self, enabled: bool) -> None:
        """Enable or disable DWA local avoidance (fallback to WaypointFollower)."""
        self._use_dwa = enabled
        if not enabled:
            self._dwa_telemetry = DWATelemetryPlaceholder()
        for btn in self._sim_buttons:
            if btn.label in ("MOD: DWA", "MOD: TAKİPÇİ"):
                btn.label = "MOD: DWA" if self._use_dwa else "MOD: TAKİPÇİ"

    @property
    def safety_tracker(self) -> SafetyTracker:
        """Return the real-time safety analytics tracker."""
        return self._safety_tracker

    @property
    def status_message(self) -> str | None:
        """Return the active Turkish status/warning message."""
        return self._status_message

    @property
    def start_m(self) -> tuple[float, float]:
        """Return current start position in meters."""
        return self._start_m

    @property
    def goal_m(self) -> tuple[float, float]:
        """Return current goal position in meters."""
        return self._goal_m

    def navigate_to(self, view: AppView, guide_tab: GuideTab | None = None) -> None:
        """Switch application view while preserving simulation state."""
        if view == AppView.GUIDE:
            if self._view != AppView.GUIDE:
                self._previous_view = self._view
            if self._world.state == SimulationState.RUNNING:
                self._on_pause()
            if guide_tab is not None:
                self._guide_tab = guide_tab
        elif view == AppView.SIMULATION:
            self._has_active_sim = True
        elif view == AppView.HOME and self._world.state == SimulationState.RUNNING:
            self._on_pause()
        self._esc_confirm_pending = False
        self._view = view

    def _create_home_buttons(self) -> list[Button]:
        return [
            Button(
                pygame.Rect(148, 272, 564, 46),
                "YENİ SİMÜLASYON BAŞLAT",
                lambda: self.navigate_to(AppView.SIMULATION),
            ),
            Button(
                pygame.Rect(148, 384, 274, 40),
                "KARŞIDAN GELEN",
                lambda: self._start_scenario_from_home("HEAD-ON"),
            ),
            Button(
                pygame.Rect(438, 384, 274, 40),
                "ÇAPRAZ GEÇİŞ",
                lambda: self._start_scenario_from_home("CROSSING"),
            ),
            Button(
                pygame.Rect(148, 432, 274, 40),
                "KORİDOR",
                lambda: self._start_scenario_from_home("CORRIDOR"),
            ),
            Button(
                pygame.Rect(438, 432, 274, 40),
                "DUR-KALK",
                lambda: self._start_scenario_from_home("STOP-AND-GO"),
            ),
            Button(
                pygame.Rect(148, 480, 274, 40),
                "DAR GEÇİT",
                lambda: self._start_scenario_from_home("NARROW-GATE"),
            ),
            Button(
                pygame.Rect(438, 480, 274, 40),
                "ÇOKLU ENGEL",
                lambda: self._start_scenario_from_home("MULTIPLE-DYNAMIC-OBSTACLES"),
            ),
            Button(
                pygame.Rect(148, 528, 274, 40),
                "ANİ DURUŞ",
                lambda: self._start_scenario_from_home("SUDDEN-STOP"),
            ),
            Button(
                pygame.Rect(438, 528, 274, 40),
                "SONLU FREN",
                lambda: self._start_scenario_from_home("SUDDEN-STOP-FINITE-DECEL"),
            ),
            Button(
                pygame.Rect(792, 258, 500, 44),
                "KULLANIM KILAVUZU",
                lambda: self.navigate_to(AppView.GUIDE, GuideTab.TUTORIAL),
            ),
            Button(
                pygame.Rect(792, 314, 500, 44),
                "ALGORİTMA BİLGİSİ",
                lambda: self.navigate_to(AppView.GUIDE, GuideTab.ALGORITHMS),
            ),
            Button(
                pygame.Rect(1160, 650, 160, 40),
                "ÇIKIŞ",
                self._quit_app,
            ),
        ]

    def _create_guide_buttons(self) -> list[Button]:
        return [
            Button(
                pygame.Rect(60, 108, 220, 40),
                "ADIM ADIM REHBER",
                lambda: self._set_guide_tab(GuideTab.TUTORIAL),
            ),
            Button(
                pygame.Rect(292, 108, 220, 40),
                "ALGORİTMALAR",
                lambda: self._set_guide_tab(GuideTab.ALGORITHMS),
            ),
            Button(
                pygame.Rect(524, 108, 240, 40),
                "GÜVENLİK GÖSTERGELERİ",
                lambda: self._set_guide_tab(GuideTab.SAFETY),
            ),
            Button(
                pygame.Rect(1040, 40, 150, 40),
                "GERİ DÖN",
                lambda: self.navigate_to(self._previous_view),
            ),
            Button(
                pygame.Rect(1204, 40, 150, 40),
                "ANA MENÜ",
                lambda: self.navigate_to(AppView.HOME),
            ),
        ]

    def _create_sim_buttons(self) -> list[Button]:
        win = self._config.window
        top_y = 8
        top_h = 28
        lx = 10
        lw = win.left_rail_width - 20
        lw2 = (lw - 6) // 2
        ly = win.top_bar_height + 44
        bh = 24
        rx = win.width - win.right_inspector_width + 14
        rw2 = (win.right_inspector_width - 34) // 2
        speed_val = self._config.window.demo_speed_presets[self._sim_speed_idx]

        return [
            # Top bar controls
            Button(pygame.Rect(156, top_y, 70, top_h), "BAŞLAT", self._on_start, tooltip="Otonom navigasyonu başlat."),
            Button(pygame.Rect(232, top_y, 78, top_h), "DURAKLAT", self._on_pause, tooltip="Simülasyonu duraklat."),
            Button(pygame.Rect(316, top_y, 68, top_h), "SIFIRLA", self._on_reset, tooltip="Simülasyonu ve istatistikleri sıfırla."),
            Button(pygame.Rect(390, top_y, 82, top_h), "A* PLANLA", self.plan_path, tooltip="A* küresel rotasını yeniden hesapla."),
            Button(pygame.Rect(478, top_y, 92, top_h), "MOD: DWA" if self._use_dwa else "MOD: TAKİPÇİ", self._toggle_dwa_mode, tooltip="DWA ve takipçi arası geçiş."),
            Button(pygame.Rect(576, top_y, 78, top_h), f"HIZ: {speed_val:.1f}x", self._cycle_sim_speed, tooltip="Simülasyon oynatma hızını değiştir."),
            Button(pygame.Rect(660, top_y, 96, top_h), "DEMO: AÇIK" if self._demo_mode else "DEMO: KAPALI", self._toggle_demo_mode, tooltip="Demo olay bildirimleri ve özetini aç/kapat."),
            Button(pygame.Rect(762, top_y, 72, top_h), "KILAVUZ", lambda: self.navigate_to(AppView.GUIDE, GuideTab.TUTORIAL), tooltip="Kullanım kılavuzunu aç."),
            Button(pygame.Rect(840, top_y, 82, top_h), "ANA MENÜ", lambda: self.navigate_to(AppView.HOME), tooltip="Ana menüye dön."),
            Button(pygame.Rect(928, top_y, 104, top_h), "PANELİ GİZLE", self._toggle_safety_panel, tooltip="Sağ denetçi panelini gizle/göster."),

            # Left tool rail: Object Editor & Modes
            Button(pygame.Rect(lx, ly, lw2, bh), "SEÇ / TAŞI", lambda: self._set_mode(InteractionMode.SELECT_EDIT), tooltip="Nesne seç ve sürükle."),
            Button(pygame.Rect(lx + lw2 + 6, ly, lw2, bh), "ÖLÇÜM ARACI", lambda: self._set_mode(InteractionMode.MEASURE), tooltip="İki nesne/robot arası canlı ölçüm."),
            Button(pygame.Rect(lx, ly + 28, lw2, bh), "NESNE EKLE", self._on_add_object, tooltip="Seçili kategoride nesne ekle."),
            Button(pygame.Rect(lx + lw2 + 6, ly + 28, lw2, bh), "KATEGORİ", self._on_cycle_category, tooltip="Nesne kategorisini değiştir."),
            Button(pygame.Rect(lx, ly + 56, lw2, bh), "KOPYALA", self._on_duplicate_object, tooltip="Seçili nesneyi kopyala."),
            Button(pygame.Rect(lx + lw2 + 6, ly + 56, lw2, bh), "SİL", self._on_delete_object, tooltip="Seçili nesneyi sil."),
            Button(pygame.Rect(lx, ly + 84, lw2, bh), "DÖNDÜR +15°", lambda: self._on_rotate_object(self._config.editor.angle_nudge_rad), tooltip="Seçili nesneyi +15° döndür."),
            Button(pygame.Rect(lx + lw2 + 6, ly + 84, lw2, bh), "DÖNDÜR -15°", lambda: self._on_rotate_object(-self._config.editor.angle_nudge_rad), tooltip="Seçili nesneyi -15° döndür."),

            # Map & A* modes
            Button(pygame.Rect(lx, ly + 120, lw2, bh), "SÜRÜŞ", lambda: self._set_mode(InteractionMode.DRIVE), tooltip="Normal sürüş / otonom takip modu."),
            Button(pygame.Rect(lx + lw2 + 6, ly + 120, lw2, bh), "BAŞLANGIÇ", lambda: self._set_mode(InteractionMode.SET_START), tooltip="Robot başlangıcını seç."),
            Button(pygame.Rect(lx, ly + 148, lw2, bh), "HEDEF", lambda: self._set_mode(InteractionMode.SET_GOAL), tooltip="Hedef noktasını seç."),
            Button(pygame.Rect(lx + lw2 + 6, ly + 148, lw2, bh), "STATİK ENGEL", lambda: self._set_mode(InteractionMode.PLACE_OBSTACLE), tooltip="Sol tıkla statik engel ekle/sağ tıkla sil."),
            Button(pygame.Rect(lx, ly + 176, lw2, bh), "8-YÖN" if self._allow_diagonal else "4-YÖN", self._toggle_diagonal, tooltip="8-yön / 4-yön geçişi."),
            Button(pygame.Rect(lx + lw2 + 6, ly + 176, lw2, bh), "TEMİZLE", self.clear_path, tooltip="Mevcut A* rotasını temizle."),

            # All 8 Scenarios
            Button(pygame.Rect(lx, ly + 214, lw2, bh), "KARŞIDAN GELEN", lambda: self.load_scenario("HEAD-ON"), tooltip="HEAD-ON: Karşıdan yaklaşan engel."),
            Button(pygame.Rect(lx + lw2 + 6, ly + 214, lw2, bh), "ÇAPRAZ GEÇİŞ", lambda: self.load_scenario("CROSSING"), tooltip="CROSSING: Çapraz geçen engel."),
            Button(pygame.Rect(lx, ly + 242, lw2, bh), "KORİDOR", lambda: self.load_scenario("CORRIDOR"), tooltip="CORRIDOR: Dar koridor geçişi."),
            Button(pygame.Rect(lx + lw2 + 6, ly + 242, lw2, bh), "DUR-KALK", lambda: self.load_scenario("STOP-AND-GO"), tooltip="STOP-AND-GO: Dur-kalk engel."),
            Button(pygame.Rect(lx, ly + 270, lw2, bh), "DAR GEÇİT", lambda: self.load_scenario("NARROW-GATE"), tooltip="NARROW-GATE: Dar geçit testi."),
            Button(pygame.Rect(lx + lw2 + 6, ly + 270, lw2, bh), "ÇOKLU ENGEL", lambda: self.load_scenario("MULTIPLE-DYNAMIC-OBSTACLES"), tooltip="MULTIPLE-DYNAMIC-OBSTACLES: Çoklu dinamik engel."),
            Button(pygame.Rect(lx, ly + 298, lw2, bh), "ANİ DURUŞ", lambda: self.load_scenario("SUDDEN-STOP"), tooltip="SUDDEN-STOP: Ani duran ön engel."),
            Button(pygame.Rect(lx + lw2 + 6, ly + 298, lw2, bh), "SONLU FREN", lambda: self.load_scenario("SUDDEN-STOP-FINITE-DECEL"), tooltip="SUDDEN-STOP-FINITE-DECEL: Sonlu ivmeli duruş."),

            # Right inspector quick actions
            Button(pygame.Rect(rx, win.top_bar_height + 332, rw2, 24), "DAVRANIŞ", self._on_cycle_behavior, tooltip="Seçili nesne hareket davranışını değiştir."),
            Button(pygame.Rect(rx + rw2 + 6, win.top_bar_height + 332, rw2, 24), "GEOMETRİ", self._on_cycle_geometry, tooltip="Seçili nesne geometrisini değiştir."),
        ]

    def _cycle_sim_speed(self) -> None:
        presets = self._config.window.demo_speed_presets
        self._sim_speed_idx = (self._sim_speed_idx + 1) % len(presets)
        val = presets[self._sim_speed_idx]
        for btn in self._sim_buttons:
            if btn.label.startswith("HIZ:"):
                btn.label = f"HIZ: {val:.1f}x"

    def _toggle_demo_mode(self) -> None:
        self._demo_mode = not self._demo_mode
        for btn in self._sim_buttons:
            if btn.label.startswith("DEMO:"):
                btn.label = "DEMO: AÇIK" if self._demo_mode else "DEMO: KAPALI"

    def _trigger_demo_event(self, text: str) -> None:
        if self._demo_mode:
            self._demo_event_text = text
            self._demo_event_timer_s = self._config.window.demo_event_duration_s

    def _toggle_dwa_mode(self) -> None:
        self.set_dwa_enabled(not self._use_dwa)

    def _toggle_safety_panel(self) -> None:
        self._safety_panel_open = not self._safety_panel_open
        for btn in self._sim_buttons:
            if btn.label in ("PANELİ GİZLE", "GÜVENLİK PANELİ"):
                btn.label = (
                    "PANELİ GİZLE" if self._safety_panel_open else "GÜVENLİK PANELİ"
                )

    def _set_guide_tab(self, tab: GuideTab) -> None:
        self._guide_tab = tab

    def _start_scenario_from_home(self, name: str) -> None:
        self.load_scenario(name)
        self.navigate_to(AppView.SIMULATION)

    def _quit_app(self) -> None:
        self._running = False

    def _reset_run_metrics(self) -> None:
        self._obs_pipeline = ObservationPipeline(self._config.observation)
        self._last_observation_frame = None
        self._replan_cooldown_remaining_s = 0.0
        self._replan_attempts = 0
        self._successful_replans = 0
        self._stop_count = 0
        self._was_stopped = False
        self._min_safety_margin_observed_m = None
        self._collision_occurred = False
        self._dwa_started_announced = False
        self._dyn_obs_announced = False
        self._goal_announced = False
        self._clearance_history.clear()
        self._speed_history.clear()

    def load_scenario(self, name: str) -> None:
        """Load a deterministic benchmark scenario and re-plan static A* path."""
        scenario = ScenarioManager.get_scenario(name)
        self._world.load_scenario(scenario)
        self._start_m = scenario.robot_start_m
        self._goal_m = scenario.goal_m
        self._status_message = None
        self._follower.reset()
        self._dwa_planner = DWAPlanner(
            self._config.world,
            self._config.robot,
            self._config.dwa,
            observation_cfg=self._config.observation,
        )
        self._dwa_telemetry = DWATelemetryPlaceholder()
        self._safety_tracker.reset()
        self._reset_run_metrics()
        self.plan_path()
        self._evaluate_safety(record_history=False)

    def _set_mode(self, mode: InteractionMode) -> None:
        self._interaction_mode = mode

    def _toggle_diagonal(self) -> None:
        self._allow_diagonal = not self._allow_diagonal
        for btn in self._sim_buttons:
            if btn.label in ("8-YÖN", "4-YÖN"):
                btn.label = "8-YÖN" if self._allow_diagonal else "4-YÖN"
        self.plan_path()

    def _on_start(self) -> None:
        if self._astar_result is None or not self._astar_result.success:
            self._status_message = "Geçerli A* rotası yok! Önce rota planlayın."
            return
        if self._follower.target_reached or self._dwa_planner.target_reached:
            self._status_message = "HEDEFE ULAŞILDI"
            return
        self._status_message = None
        self._esc_confirm_pending = False
        self._has_active_sim = True
        self._world.start()

    def _on_pause(self) -> None:
        self._world.pause()
        self._world.robot.set_velocity_command(0.0, 0.0)
        self._follower.reset_velocity_ramp()

    def _on_reset(self) -> None:
        if self._world.active_scenario_name is not None:
            scenario = ScenarioManager.get_scenario(self._world.active_scenario_name)
            self._world.load_scenario(scenario)
            self._start_m = scenario.robot_start_m
            self._goal_m = scenario.goal_m
        else:
            self._world.reset()
        self._status_message = None
        self._esc_confirm_pending = False
        self._follower.reset()
        self._dwa_planner = DWAPlanner(
            self._config.world,
            self._config.robot,
            self._config.dwa,
            observation_cfg=self._config.observation,
        )
        self._dwa_telemetry = DWATelemetryPlaceholder()
        self._safety_tracker.reset()
        self._reset_run_metrics()
        self.plan_path()
        self._evaluate_safety(record_history=False)

    def _on_toggle_pause(self) -> None:
        if self._world.state == SimulationState.RUNNING:
            self._on_pause()
        else:
            self._on_start()

    def clear_path(self) -> None:
        """Clear the currently displayed A* result and follower waypoints."""
        self._astar_result = None
        self._follower.clear()
        self._dwa_planner.clear()
        self._dwa_telemetry = DWATelemetryPlaceholder()
        self._status_message = None

    def _rebuild_inflated_grid(self) -> None:
        self._inflated_grid = inflate_occupancy_grid(
            occupancy_grid=self._world.occupancy_grid,
            resolution_m=self._world.world_config.grid_resolution_m,
            robot_radius_m=self._config.robot.radius_m,
            safety_margin_m=self._config.planner.safety_margin_m,
            enforce_boundary_clearance=self._config.planner.enforce_boundary_clearance,
        )

    def plan_path(self) -> AStarResult:
        """Run A* on the static inflated occupancy grid from start_m to goal_m."""
        self._rebuild_inflated_grid()
        assert self._inflated_grid is not None
        res_m = self._world.world_config.grid_resolution_m
        start_cell = world_to_grid(
            self._start_m[0], self._start_m[1], res_m, self._inflated_grid.shape
        )
        goal_cell = world_to_grid(
            self._goal_m[0], self._goal_m[1], res_m, self._inflated_grid.shape
        )
        planner = AStarPlanner(
            resolution_m=res_m, allow_diagonal=self._allow_diagonal
        )
        required_clearance_m = (
            self._config.robot.radius_m + self._config.planner.safety_margin_m
        )
        self._astar_result = planner.plan(
            self._inflated_grid,
            start_cell,
            goal_cell,
            raw_grid=self._world.occupancy_grid,
            required_clearance_m=required_clearance_m,
            enforce_boundary_clearance=self._config.planner.enforce_boundary_clearance,
        )
        if self._astar_result.success:
            self._follower.set_path(self._astar_result.path_world_m)
            self._dwa_planner.set_path(self._astar_result.path_world_m)
            self._status_message = None
            self._trigger_demo_event("A* GLOBAL YOL OLUŞTURULDU")
        else:
            self._follower.clear()
            self._dwa_planner.clear()
            self._dwa_telemetry = DWATelemetryPlaceholder()
            self._status_message = "ROTA BULUNAMADI: Başlangıç/hedef engelli."
        return self._astar_result

    @property
    def replan_attempts(self) -> int:
        """Return total automatic A* global replanning attempts."""
        return self._replan_attempts

    @property
    def successful_replans(self) -> int:
        """Return count of successful automatic A* global replans."""
        return self._successful_replans

    def attempt_global_replan(self) -> bool:
        """Attempt controlled A* global replanning from current robot pose to goal
        treating active dynamic obstacles as temporary occupied regions.
        """
        self._replan_attempts += 1
        self._replan_cooldown_remaining_s = self._config.dwa.replan_cooldown_s
        st = self._world.robot.state
        obs_frame = getattr(self, "_last_observation_frame", None)
        ok, new_path = replan_global_path_around_obstacles(
            base_occupancy_grid=self._world.occupancy_grid,
            resolution_m=self._config.world.grid_resolution_m,
            robot_x_m=obs_frame.robot.x_m if obs_frame is not None else st.x_m,
            robot_y_m=obs_frame.robot.y_m if obs_frame is not None else st.y_m,
            goal_x_m=self._goal_m[0],
            goal_y_m=self._goal_m[1],
            robot_radius_m=self._config.robot.radius_m,
            safety_margin_m=self._config.planner.safety_margin_m,
            dynamic_obstacles=self._world.dynamic_obstacles,
            allow_diagonal=self._allow_diagonal,
            enforce_boundary_clearance=self._config.planner.enforce_boundary_clearance,
            sim_objects=self._world.sim_objects,
            observation_frame=obs_frame,
        )
        if ok and new_path:
            self._successful_replans += 1
            self._dwa_planner.set_path(new_path)
            self._follower.set_path(new_path)
            self._status_message = "ROTA YENİLENDİ: Dinamik engel aşıldı."
            self._trigger_demo_event("YENİDEN PLANLAMA")
            self._safety_tracker.log_event(
                self._world.sim_time_s, "OTOMATİK ROTA YENİLENDİ", SafetyLevel.CAUTION
            )
            return True

        self._status_message = "ROTA ENGELLENDİ: Güvenli alternatif yok."
        self._safety_tracker.log_event(
            self._world.sim_time_s, "ROTA ENGELLENDİ", SafetyLevel.CRITICAL
        )
        return False

    def _evaluate_safety(self, record_history: bool) -> None:
        self._safety_tracker.evaluate(
            sim_time_s=self._world.sim_time_s,
            robot_state=self._world.robot.state,
            robot_radius_m=self._world.robot.radius_m,
            static_obstacles=self._world.obstacles,
            dynamic_obstacles=self._world.dynamic_obstacles,
            required_safety_margin_m=self._config.planner.safety_margin_m,
            record_history=record_history,
        )

    def _on_add_object(self) -> None:
        w_cfg = self._world.world_config
        cx = 0.5 * w_cfg.width_m
        cy = 0.5 * w_cfg.height_m
        obj = self._world.object_editor.add_object(self._selected_category, cx, cy)
        if obj.category == ObjectCategory.STATIC_OBSTACLE:
            self._world.rebuild_occupancy_grid()
            self._rebuild_inflated_grid()
        self._interaction_mode = InteractionMode.SELECT_EDIT
        self._sync_inspector_fields()
        self._status_message = f"{obj.category.label_tr} eklendi ({obj.object_id})."

    def _on_cycle_category(self) -> None:
        cats = list(ObjectCategory)
        idx = (cats.index(self._selected_category) + 1) % len(cats)
        self._selected_category = cats[idx]
        self._status_message = f"Kategori: {self._selected_category.label_tr}"

    def _on_duplicate_object(self) -> None:
        sel = self._world.object_editor.selected_object
        if sel is None:
            return
        dup = self._world.object_editor.duplicate_object(sel.object_id)
        if dup is not None and dup.category == ObjectCategory.STATIC_OBSTACLE:
            self._world.rebuild_occupancy_grid()
            self._rebuild_inflated_grid()
        self._sync_inspector_fields()

    def _on_delete_object(self) -> None:
        sel = self._world.object_editor.selected_object
        if sel is None:
            return
        was_static = sel.category == ObjectCategory.STATIC_OBSTACLE
        self._world.object_editor.delete_object(sel.object_id)
        if was_static:
            self._world.rebuild_occupancy_grid()
            self._rebuild_inflated_grid()
        self._sync_inspector_fields()

    def _on_rotate_object(self, delta_rad: float) -> None:
        sel = self._world.object_editor.selected_object
        if sel is None:
            return
        self._world.object_editor.rotate_object(
            sel.object_id, sel.motion.heading_rad + delta_rad
        )
        if sel.category == ObjectCategory.STATIC_OBSTACLE:
            self._world.rebuild_occupancy_grid()
            self._rebuild_inflated_grid()
        self._sync_inspector_fields()

    def _on_cycle_behavior(self) -> None:
        sel = self._world.object_editor.selected_object
        if sel is None:
            return
        behaviors = list(MotionBehaviorType)
        idx = (behaviors.index(sel.motion.behavior) + 1) % len(behaviors)
        self._world.object_editor.update_properties(
            sel.object_id, behavior=behaviors[idx]
        )
        self._sync_inspector_fields()

    def _on_cycle_geometry(self) -> None:
        sel = self._world.object_editor.selected_object
        if sel is None:
            return
        geoms = ["CIRCLE", "ORIENTED_RECT", "POLYGON", "CAPSULE"]
        cur = sel.physics.geometry_type
        nxt = geoms[(geoms.index(cur) + 1) % len(geoms)] if cur in geoms else "ORIENTED_RECT"
        self._world.object_editor.update_properties(sel.object_id, geometry_type=nxt)
        if sel.category == ObjectCategory.STATIC_OBSTACLE:
            self._world.rebuild_occupancy_grid()
            self._rebuild_inflated_grid()
        self._sync_inspector_fields()

    def _sync_inspector_fields(self) -> None:
        sel = self._world.object_editor.selected_object
        if sel is None:
            self._inspector_fields = []
            return
        win = self._config.window
        rx = win.width - win.right_inspector_width + 14
        fw = (win.right_inspector_width - 34) // 2
        y0 = win.top_bar_height + 88
        specs = [
            ("x_m", "X (m)", f"{sel.motion.x_m:.2f}"),
            ("y_m", "Y (m)", f"{sel.motion.y_m:.2f}"),
            ("heading_deg", "Açı (°)", f"{math.degrees(sel.motion.heading_rad):.1f}"),
            ("width_m", "En (m)", f"{sel.physics.width_m:.2f}"),
            ("length_m", "Boy (m)", f"{sel.physics.length_m:.2f}"),
            ("radius_m", "Yarıçap", f"{sel.physics.radius_m:.2f}"),
            ("linear_speed_mps", "Hız (m/s)", f"{sel.motion.linear_speed_mps:.2f}"),
            ("angular_speed_radps", "ω (rad/s)", f"{sel.motion.angular_speed_radps:.2f}"),
            ("acceleration_mps2", "İvme", f"{sel.motion.acceleration_mps2:.2f}"),
            ("safety_margin_m", "Emniyet", f"{sel.safety.safety_margin_m:.2f}"),
        ]
        fields: list[NumericField] = []
        for idx, (key, lbl, val_str) in enumerate(specs):
            col = idx % 2
            row = idx // 2
            rect = pygame.Rect(rx + col * (fw + 6), y0 + row * 28, fw, 24)
            fields.append(NumericField(key, lbl, rect, val_str))
        self._inspector_fields = fields

    def _commit_inspector_field(self, key: str, value: float) -> None:
        sel = self._world.object_editor.selected_object
        if sel is None:
            return
        kwargs: dict[str, object] = {}
        if key == "heading_deg":
            kwargs["heading_rad"] = math.radians(value)
        else:
            kwargs[key] = value
        ok = self._world.object_editor.update_properties(sel.object_id, **kwargs)  # type: ignore[arg-type]
        if ok and sel.category == ObjectCategory.STATIC_OBSTACLE:
            self._world.rebuild_occupancy_grid()
            self._rebuild_inflated_grid()
        self._sync_inspector_fields()

    def _check_dynamic_safety_violation(self) -> tuple[bool, bool]:
        if self._world.has_physical_collision:
            return (True, True)
        if self._world.has_safety_warning:
            return (False, True)

        rs = self._world.robot.state
        r_rob = self._world.robot.radius_m
        margin = self._config.planner.safety_margin_m
        for obs in self._world.dynamic_obstacles:
            if not obs.active:
                continue
            dist = math.hypot(rs.x_m - obs.x_m, rs.y_m - obs.y_m)
            if dist <= r_rob + obs.radius_m + 1e-9:
                return (True, True)
            if dist <= r_rob + obs.radius_m + margin + 1e-9:
                return (False, True)
        return (False, False)

    def _handle_viewport_click(self, pos: tuple[int, int], button: int) -> None:
        left, top, width, height = self._renderer.transform.world_screen_bounds
        world_rect = pygame.Rect(left, top, width, height)
        if not world_rect.collidepoint(pos):
            return

        x_m, y_m = self._renderer.transform.screen_to_world(pos[0], pos[1])
        w_cfg = self._world.world_config
        if not (0.0 <= x_m <= w_cfg.width_m and 0.0 <= y_m <= w_cfg.height_m):
            return

        if button == 3:
            if self._world.remove_static_obstacle_at(x_m, y_m):
                self.plan_path()
                self._evaluate_safety(record_history=False)
            return

        if button != 1:
            return

        if self._interaction_mode == InteractionMode.SELECT_EDIT:
            sel = self._world.object_editor.select_at(x_m, y_m)
            self._dragging_obj_id = sel.object_id if sel is not None else None
            self._sync_inspector_fields()
        elif self._interaction_mode == InteractionMode.MEASURE:
            rs = self._world.robot.state
            if math.hypot(x_m - rs.x_m, y_m - rs.y_m) <= self._world.robot.radius_m + 0.35:
                self._measurement_tool.select_entity("ROBOT")
            else:
                sel = self._world.object_editor.select_at(x_m, y_m)
                if sel is not None:
                    self._measurement_tool.select_entity(sel.object_id)
        elif self._interaction_mode == InteractionMode.SET_START:
            self._start_m = (x_m, y_m)
            self._world.robot.set_initial_position(x_m, y_m)
            self.plan_path()
            self._evaluate_safety(record_history=False)
        elif self._interaction_mode == InteractionMode.SET_GOAL:
            self._goal_m = (x_m, y_m)
            self.plan_path()
        elif self._interaction_mode == InteractionMode.PLACE_OBSTACLE:
            size_m = self._config.planner.interactive_obstacle_size_m
            obs_x = max(0.0, min(w_cfg.width_m - size_m, x_m - 0.5 * size_m))
            obs_y = max(0.0, min(w_cfg.height_m - size_m, y_m - 0.5 * size_m))
            self._world.add_static_obstacle(
                StaticObstacle(x_m=obs_x, y_m=obs_y, width_m=size_m, height_m=size_m)
            )
            self.plan_path()
            self._evaluate_safety(record_history=False)

    def _sync_button_states(self) -> None:
        state = self._world.state
        for btn in self._sim_buttons:
            if btn.label == "BAŞLAT":
                btn.enabled = state != SimulationState.RUNNING
            elif btn.label == "DURAKLAT":
                btn.enabled = state == SimulationState.RUNNING
            else:
                btn.enabled = True

    def _render_current_frame(self) -> None:
        if self._view == AppView.HOME:
            self._home_renderer.render(
                self._screen, self._home_buttons, self._has_active_sim
            )
        elif self._view == AppView.GUIDE:
            self._guide_renderer.render(
                self._screen, self._guide_buttons, self._guide_tab
            )
        else:
            self._sync_button_states()
            meas_res = self._measurement_tool.compute(
                self._world.robot, self._world.sim_objects
            )
            self._renderer.render(
                self._screen,
                self._world,
                self._sim_buttons,
                self._start_m,
                self._goal_m,
                self._inflated_grid,
                self._astar_result,
                self._interaction_mode,
                self._allow_diagonal,
                safety_tracker=self._safety_tracker,
                dwa_telemetry=self._dwa_telemetry,
                target_reached=(self._follower.target_reached or self._dwa_planner.target_reached),
                status_message=self._status_message,
                hovered_tooltip=self._hovered_tooltip,
                safety_panel_open=self._safety_panel_open,
                selected_category=self._selected_category,
                measurement_result=meas_res,
                inspector_fields=self._inspector_fields,
                clearance_history=self._clearance_history,
                speed_history=self._speed_history,
                replan_counts=(self._successful_replans, self._replan_attempts),
                stop_count=self._stop_count,
                min_safety_margin_m=self._min_safety_margin_observed_m,
                collision_occurred=self._collision_occurred,
                sim_speed_multiplier=self._config.window.demo_speed_presets[self._sim_speed_idx],
                demo_mode=self._demo_mode,
                demo_event_text=self._demo_event_text if self._demo_event_timer_s > 0.0 else None,
            )
        pygame.display.flip()

    def _handle_escape(self) -> None:
        if self._view == AppView.GUIDE:
            self.navigate_to(self._previous_view)
        elif self._view == AppView.SIMULATION:
            if self._world.state == SimulationState.RUNNING and not self._esc_confirm_pending:
                self._on_pause()
                self._esc_confirm_pending = True
                self._status_message = "Duraklatıldı. Ana Menü için tekrar ESC'ye basın."
            else:
                self.navigate_to(AppView.HOME)
        else:
            self._running = False

    def step_frame(self, dt_s: float) -> None:
        """Process events, update active simulation if in SIMULATION view, and render."""
        active_buttons = self.buttons
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self._running = False
            elif event.type == pygame.KEYDOWN:
                consumed_by_field = False
                if self._view == AppView.SIMULATION:
                    for fld in self._inspector_fields:
                        hit, committed = fld.handle_key(event)
                        if hit:
                            consumed_by_field = True
                            if committed is not None:
                                self._commit_inspector_field(fld.key, committed)
                            break
                if consumed_by_field:
                    continue
                if event.key == pygame.K_ESCAPE:
                    self._handle_escape()
                elif self._view == AppView.SIMULATION:
                    if event.key == pygame.K_SPACE:
                        self._on_toggle_pause()
                    elif event.key == pygame.K_r:
                        self._on_reset()
            elif event.type == pygame.MOUSEMOTION:
                self._hovered_tooltip = None
                for btn in active_buttons:
                    btn.update_hover(event.pos)
                    if btn.hovered and btn.tooltip:
                        self._hovered_tooltip = btn.tooltip
                if self._view == AppView.SIMULATION and self._dragging_obj_id is not None:
                    left, top, width, height = self._renderer.transform.world_screen_bounds
                    if pygame.Rect(left, top, width, height).collidepoint(event.pos):
                        xm, ym = self._renderer.transform.screen_to_world(event.pos[0], event.pos[1])
                        self._world.object_editor.move_object(self._dragging_obj_id, xm, ym)
                        self._sync_inspector_fields()
            elif event.type == pygame.MOUSEBUTTONUP:
                if event.button == 1 and self._dragging_obj_id is not None:
                    obj = self._world.object_editor.get(self._dragging_obj_id)
                    if obj is not None and obj.category == ObjectCategory.STATIC_OBSTACLE:
                        self._world.rebuild_occupancy_grid()
                        self._rebuild_inflated_grid()
                    self._dragging_obj_id = None
            elif event.type == pygame.MOUSEBUTTONDOWN:
                clicked_button = False
                if event.button == 1 and self._view == AppView.SIMULATION:
                    for fld in self._inspector_fields:
                        if fld.handle_click(event.pos):
                            clicked_button = True
                if event.button == 1 and not clicked_button:
                    for btn in active_buttons:
                        if btn.handle_click(event.pos):
                            clicked_button = True
                            break
                if not clicked_button and self._view == AppView.SIMULATION:
                    self._handle_viewport_click(event.pos, event.button)

        if self._demo_event_timer_s > 0.0:
            self._demo_event_timer_s = max(0.0, self._demo_event_timer_s - dt_s)

        if self._view == AppView.SIMULATION:
            sim_dt_s = dt_s * self._config.window.demo_speed_presets[self._sim_speed_idx]
            pressed = pygame.key.get_pressed()
            v_key, omega_key = self._keyboard.compute_velocity_command(pressed)
            manual_override = (abs(v_key) > 1e-6) or (abs(omega_key) > 1e-6)

            if manual_override:
                self._world.robot.set_velocity_command(v_key, omega_key)
            elif self._world.state == SimulationState.RUNNING:
                if self._use_dwa and self._dwa_planner.has_path and not self._dwa_planner.target_reached:
                    if not self._dwa_started_announced:
                        self._dwa_started_announced = True
                        self._trigger_demo_event("DWA YÖRÜNGE DEĞERLENDİRMESİ")
                    self._replan_cooldown_remaining_s = max(
                        0.0, self._replan_cooldown_remaining_s - sim_dt_s
                    )
                    self._last_observation_frame = self._obs_pipeline.observe(
                        self._world.sim_time_s,
                        self._world.robot.state,
                        self._world.dynamic_obstacles,
                        sim_objects=self._world.sim_objects,
                    )
                    dwa_res = self._dwa_planner.compute_command(
                        self._world.robot.state,
                        sim_dt_s,
                        self._world.obstacles,
                        self._world.dynamic_obstacles,
                        sim_objects=self._world.sim_objects,
                        observation_frame=self._last_observation_frame,
                    )
                    self._world.robot.set_velocity_command(dwa_res.v_mps, dwa_res.omega_radps)
                    if dwa_res.proactive_replan_triggered:
                        self._replan_attempts += 1
                        self._successful_replans += 1
                        self._follower.set_path(self._dwa_planner._path_world_m)
                        self._trigger_demo_event("YENİDEN PLANLAMA")
                    if dwa_res.blocked and self._replan_cooldown_remaining_s <= 0.0:
                        self.attempt_global_replan()
                    if dwa_res.fallback_active:
                        self._trigger_demo_event("GÜVENLİ MANEVRA SEÇİLDİ")
                        self._safety_tracker.log_event(
                            self._world.sim_time_s,
                            "DWA ACİL FRENLEME"
                            if not dwa_res.collision_free_exists
                            else "DWA GÜVENLİK MARJI UYARISI",
                            SafetyLevel.CRITICAL
                            if not dwa_res.collision_free_exists
                            else SafetyLevel.CAUTION,
                        )
                    self._dwa_telemetry = DWATelemetryPlaceholder(
                        active=True,
                        candidate_trajectories=dwa_res.candidate_trajectories,
                        selected_trajectory=dwa_res.selected_trajectory,
                        rejected_trajectories=dwa_res.rejected_trajectories,
                        selected_v_mps=dwa_res.v_mps,
                        selected_omega_radps=dwa_res.omega_radps,
                        evaluation_score=dwa_res.score,
                        predicted_min_clearance_m=dwa_res.predicted_min_clearance_m,
                        min_braking_clearance_m=dwa_res.min_braking_clearance_m,
                        blocked=dwa_res.blocked,
                        waiting_state=dwa_res.waiting_state,
                        waiting_min_clearance_m=dwa_res.waiting_min_clearance_m,
                        holding_pose_m=dwa_res.holding_pose_m,
                        reverse_active=dwa_res.reverse_active,
                        fallback_priority=dwa_res.fallback_priority,
                        fallback_duration_s=dwa_res.fallback_duration_s,
                        deadlock_state=dwa_res.deadlock_state,
                    )
                elif not self._use_dwa:
                    phys_col, safety_warn = self._check_dynamic_safety_violation()
                    if phys_col or safety_warn:
                        self._world.robot.set_velocity_command(0.0, 0.0)
                        self._follower.reset_velocity_ramp()
                        self._world.pause()
                        self._safety_tracker.log_event(
                            self._world.sim_time_s,
                            "ROBOT DURDURULDU",
                            SafetyLevel.COLLISION if phys_col else SafetyLevel.CRITICAL,
                        )
                        self._status_message = (
                            "ÇARPIŞMA: Robot durduruldu!"
                            if phys_col
                            else "GÜVENLİK DURDURMASI: Engel yaklaştı!"
                        )
                    elif self._follower.has_path and not self._follower.target_reached:
                        v_cmd, omega_cmd = self._follower.compute_command(
                            self._world.robot.state,
                            self._config.robot.max_linear_speed_mps,
                            self._config.robot.max_angular_speed_radps,
                            sim_dt_s,
                        )
                        self._world.robot.set_velocity_command(v_cmd, omega_cmd)
                    else:
                        self._world.robot.set_velocity_command(0.0, 0.0)
                else:
                    self._world.robot.set_velocity_command(0.0, 0.0)

            was_running = self._world.state == SimulationState.RUNNING
            self._world.step(sim_dt_s)
            if was_running:
                self._evaluate_safety(record_history=True)
                snap = self._safety_tracker.latest_snapshot
                if snap is not None and snap.surface_clearance_m is not None:
                    self._clearance_history.append(snap.surface_clearance_m)
                    if len(self._clearance_history) > 240:
                        self._clearance_history.pop(0)
                    margin_val = snap.surface_clearance_m - self._config.planner.safety_margin_m
                    if (
                        self._min_safety_margin_observed_m is None
                        or margin_val < self._min_safety_margin_observed_m
                    ):
                        self._min_safety_margin_observed_m = margin_val
                    if snap.surface_clearance_m < 4.0 and not self._dyn_obs_announced:
                        self._dyn_obs_announced = True
                        self._trigger_demo_event("DİNAMİK ENGEL ALGILANDI")
                    if snap.safety_level == SafetyLevel.COLLISION:
                        self._collision_occurred = True
                is_stopped_now = abs(self._world.robot.state.v_mps) < 0.05
                if is_stopped_now and not self._was_stopped and self._world.sim_time_s > 0.2:
                    self._stop_count += 1
                self._was_stopped = is_stopped_now
                self._speed_history.append(abs(self._world.robot.state.v_mps))
                if len(self._speed_history) > 240:
                    self._speed_history.pop(0)

            if was_running and not manual_override:
                reached = (
                    self._dwa_planner.target_reached
                    if self._use_dwa
                    else self._follower.target_reached
                )
                if reached:
                    self._world.robot.set_velocity_command(0.0, 0.0)
                    self._world.pause()
                    self._safety_tracker.log_event(
                        self._world.sim_time_s, "HEDEFE ULAŞILDI", SafetyLevel.SAFE
                    )
                    self._status_message = "HEDEFE ULAŞILDI"
                    if not self._goal_announced:
                        self._goal_announced = True
                        self._trigger_demo_event("HEDEFE ULAŞILDI")
                elif not self._use_dwa:
                    phys_col, safety_warn = self._check_dynamic_safety_violation()
                    if phys_col or safety_warn:
                        self._world.robot.set_velocity_command(0.0, 0.0)
                        self._follower.reset_velocity_ramp()
                        self._world.pause()
                        self._safety_tracker.log_event(
                            self._world.sim_time_s,
                            "ROBOT DURDURULDU",
                            SafetyLevel.COLLISION if phys_col else SafetyLevel.CRITICAL,
                        )
                        self._status_message = (
                            "ÇARPIŞMA: Robot durduruldu!"
                            if phys_col
                            else "GÜVENLİK DURDURMASI: Engel yaklaştı!"
                        )

        self._render_current_frame()

    def run(self, max_frames: int | None = None) -> None:
        """Run the main application loop until closed or max_frames is reached."""
        self._running = True
        frame_count = 0
        try:
            while self._running:
                dt_s = self._clock.tick(self._config.window.fps) / 1000.0
                self.step_frame(dt_s)
                frame_count += 1
                if max_frames is not None and frame_count >= max_frames:
                    break
        finally:
            pygame.quit()
