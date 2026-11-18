"""Pygame renderer for the 3-column Turkish simulation dashboard and safety analytics."""

from enum import Enum, auto
import math
import numpy as np
import pygame

from algorithms.astar import AStarResult, grid_to_world
from algorithms.dwa_interface import DWATelemetryPlaceholder
from config import ProjectConfig
from interface.controls import Button, NumericField
from simulation.coordinates import CoordinateTransform, ScreenRect
from simulation.geometry import (
    CapsuleGeometry,
    CircleGeometry,
    OrientedRectGeometry,
    PolygonGeometry,
)
from simulation.objects import MeasurementResult, ObjectCategory, SimObject
from simulation.safety import SafetyLevel, SafetySnapshot, SafetyTracker
from simulation.scenarios import ScenarioManager
from simulation.world import SimulationState, SimulationWorld


class InteractionMode(Enum):
    """Interactive mouse modes for the simulation viewport."""

    DRIVE = auto()
    SET_START = auto()
    SET_GOAL = auto()
    PLACE_OBSTACLE = auto()
    SELECT_EDIT = auto()
    MEASURE = auto()


LEFT_PANEL_WIDTH = 270
RIGHT_PANEL_WIDTH = 340


class SimulationRenderer:
    """Renders the central metric map, left control panel, and right safety dashboard."""

    def __init__(self, config: ProjectConfig) -> None:
        self._config = config
        self._theme = config.theme
        self._safety_panel_open = True

        self._font_title = pygame.font.SysFont("segoeui", 19, bold=True)
        self._font_section = pygame.font.SysFont("segoeui", 11, bold=True)
        self._font_body = pygame.font.SysFont("segoeui", 11)
        self._font_small = pygame.font.SysFont("segoeui", 10)

        self._transform = self._build_transform(panel_open=True)

    def _build_transform(self, panel_open: bool) -> CoordinateTransform:
        win = self._config.window
        left_w = win.left_rail_width
        right_w = win.right_inspector_width if panel_open else 0
        top_h = win.top_bar_height
        bottom_h = win.bottom_dock_height
        pad = win.viewport_padding
        viewport_w = win.width - left_w - right_w - 2 * pad
        viewport_h = win.height - top_h - bottom_h - 2 * pad
        viewport_rect = ScreenRect(
            left=left_w + pad,
            top=top_h + pad,
            width=max(200, viewport_w),
            height=max(200, viewport_h),
        )
        return CoordinateTransform(
            world_width_m=self._config.world.width_m,
            world_height_m=self._config.world.height_m,
            viewport=viewport_rect,
        )

    def set_safety_panel_open(self, is_open: bool) -> None:
        """Update collapsible right safety panel state and recompute map transform."""
        if is_open != self._safety_panel_open:
            self._safety_panel_open = is_open
            self._transform = self._build_transform(is_open)

    @property
    def transform(self) -> CoordinateTransform:
        """Return the active world-to-screen coordinate transform."""
        return self._transform

    def _safety_color(self, level: SafetyLevel) -> tuple[int, int, int]:
        return {
            SafetyLevel.SAFE: self._theme.safety_safe,
            SafetyLevel.CAUTION: self._theme.safety_caution,
            SafetyLevel.CRITICAL: self._theme.safety_critical,
            SafetyLevel.COLLISION: self._theme.safety_collision,
        }[level]

    @staticmethod
    def _safety_label_tr(level: SafetyLevel) -> str:
        return {
            SafetyLevel.SAFE: "GÜVENLİ",
            SafetyLevel.CAUTION: "DİKKAT",
            SafetyLevel.CRITICAL: "KRİTİK",
            SafetyLevel.COLLISION: "ÇARPIŞMA",
        }[level]

    def render(
        self,
        surface: pygame.Surface,
        world: SimulationWorld,
        buttons: list[Button],
        start_m: tuple[float, float],
        goal_m: tuple[float, float],
        inflated_grid: np.ndarray | None,
        astar_result: AStarResult | None,
        interaction_mode: InteractionMode,
        allow_diagonal: bool,
        safety_tracker: SafetyTracker,
        dwa_telemetry: DWATelemetryPlaceholder,
        target_reached: bool = False,
        status_message: str | None = None,
        hovered_tooltip: str | None = None,
        safety_panel_open: bool = True,
        selected_category: ObjectCategory = ObjectCategory.PASSENGER_CAR,
        measurement_result: MeasurementResult | None = None,
        inspector_fields: list[NumericField] | None = None,
        clearance_history: list[float] | None = None,
        speed_history: list[float] | None = None,
        replan_counts: tuple[int, int] = (0, 0),
        stop_count: int = 0,
        min_safety_margin_m: float | None = None,
        collision_occurred: bool = False,
        sim_speed_multiplier: float = 1.0,
        demo_mode: bool = True,
        demo_event_text: str | None = None,
    ) -> None:
        """Draw the complete 5-zone ADAS simulation view."""
        self.set_safety_panel_open(safety_panel_open)
        surface.fill(self._theme.background)

        snapshot = safety_tracker.latest_snapshot
        exp_status, exp_color = self._compute_experiment_status(
            world, astar_result, target_reached, collision_occurred
        )
        self._draw_world_viewport(
            surface, world, start_m, goal_m, inflated_grid, astar_result,
            snapshot, dwa_telemetry, measurement_result,
            demo_mode=demo_mode, demo_event_text=demo_event_text,
            exp_status=exp_status, exp_color=exp_color,
            min_safety_margin_m=min_safety_margin_m,
            stop_count=stop_count, replan_counts=replan_counts,
        )
        self._draw_adas_top_bar(
            surface, world, buttons, goal_m, exp_status, exp_color, status_message
        )
        self._draw_adas_left_rail(
            surface, world, buttons, interaction_mode, allow_diagonal, selected_category
        )
        if safety_panel_open:
            self._draw_adas_right_inspector(
                surface, world, buttons, safety_tracker, dwa_telemetry,
                measurement_result, inspector_fields or [],
            )
        self._draw_adas_bottom_dock(
            surface, world, goal_m, safety_tracker, dwa_telemetry,
            clearance_history or [], speed_history or [], replan_counts, hovered_tooltip,
            stop_count=stop_count, min_safety_margin_m=min_safety_margin_m,
            sim_speed_multiplier=sim_speed_multiplier, exp_status=exp_status, exp_color=exp_color,
        )

    def _compute_experiment_status(
        self,
        world: SimulationWorld,
        astar_result: AStarResult | None,
        target_reached: bool,
        collision_occurred: bool,
    ) -> tuple[str, tuple[int, int, int]]:
        if collision_occurred:
            return ("COLLISION", self._theme.safety_collision)
        if target_reached:
            return ("GOAL_REACHED", self._theme.safety_safe)
        if astar_result is not None and not astar_result.success:
            return ("NO_PATH", self._theme.safety_collision)
        if world.state == SimulationState.RUNNING:
            return ("RUNNING", self._theme.status_running)
        if world.state == SimulationState.PAUSED:
            return ("PAUSED", self._theme.status_paused)
        return ("READY", self._theme.status_ready)

    def _draw_world_viewport(
        self,
        surface: pygame.Surface,
        world: SimulationWorld,
        start_m: tuple[float, float],
        goal_m: tuple[float, float],
        inflated_grid: np.ndarray | None,
        astar_result: AStarResult | None,
        snapshot: SafetySnapshot | None,
        dwa_telemetry: DWATelemetryPlaceholder,
        measurement_result: MeasurementResult | None = None,
        demo_mode: bool = True,
        demo_event_text: str | None = None,
        exp_status: str = "READY",
        exp_color: tuple[int, int, int] = (56, 189, 248),
        min_safety_margin_m: float | None = None,
        stop_count: int = 0,
        replan_counts: tuple[int, int] = (0, 0),
    ) -> None:
        left, top, width, height = self._transform.world_screen_bounds
        world_rect = pygame.Rect(left, top, width, height)
        pygame.draw.rect(surface, self._theme.viewport_bg, world_rect)

        if inflated_grid is not None:
            self._draw_inflated_cells(surface, world, inflated_grid)
        if astar_result is not None and astar_result.explored_nodes:
            self._draw_explored_nodes(surface, world, astar_result.explored_nodes)

        self._draw_metric_grid(surface, world)
        self._draw_obstacles(surface, world)

        if astar_result is not None and astar_result.success:
            self._draw_path(surface, astar_result)

        if dwa_telemetry.active:
            self._draw_dwa_trajectories(surface, dwa_telemetry)

        self._draw_start_and_goal(surface, start_m, goal_m)
        self._draw_dynamic_obstacles(surface, world)
        self._draw_sim_objects(surface, world)
        self._draw_nearest_obstacle_line(surface, world, snapshot)
        if measurement_result is not None:
            self._draw_measurement_overlay(surface, measurement_result)
        self._draw_robot(surface, world, snapshot)

        if demo_mode:
            if demo_event_text:
                ev_rect = pygame.Rect(left + 12, top + 10, 320, 28)
                pygame.draw.rect(surface, self._theme.adas_card_bg, ev_rect, border_radius=5)
                pygame.draw.rect(surface, self._theme.accent, ev_rect, width=1, border_radius=5)
                ev_surf = self._font_small.render(f"OLAY: {demo_event_text}", True, self._theme.accent)
                surface.blit(ev_surf, (ev_rect.x + 10, ev_rect.y + 6))
            elif exp_status == "READY" and world.sim_time_s < 1e-6:
                hint_rect = pygame.Rect(left + 12, top + 10, 380, 26)
                pygame.draw.rect(surface, self._theme.adas_card_bg, hint_rect, border_radius=5)
                pygame.draw.rect(surface, self._theme.border, hint_rect, width=1, border_radius=5)
                hint_surf = self._font_small.render(
                    "DEMO MODU: Deneyi başlatmak için BAŞLAT (veya SPACE) tuşuna basın.",
                    True,
                    self._theme.text_primary,
                )
                surface.blit(hint_surf, (hint_rect.x + 10, hint_rect.y + 5))

            if exp_status in ("GOAL_REACHED", "COLLISION"):
                sum_rect = pygame.Rect(left + width - 310, top + 10, 298, 92)
                pygame.draw.rect(surface, self._theme.adas_card_bg, sum_rect, border_radius=6)
                pygame.draw.rect(surface, exp_color, sum_rect, width=2, border_radius=6)
                title_lbl = "DENEY ÖZETİ: HEDEFE ULAŞILDI" if exp_status == "GOAL_REACHED" else "DENEY ÖZETİ: ÇARPIŞMA"
                surface.blit(self._font_section.render(title_lbl, True, exp_color), (sum_rect.x + 10, sum_rect.y + 8))
                margin_s = f"{min_safety_margin_m:.3f} m" if min_safety_margin_m is not None else "—"
                sum_lines = [
                    f"Süre: {world.sim_time_s:.2f} s | Durma: {stop_count}",
                    f"Min. Güvenlik Payı: {margin_s}",
                    f"Yeniden Planlama: {replan_counts[0]}/{replan_counts[1]}",
                ]
                for idx, sl in enumerate(sum_lines):
                    surface.blit(self._font_small.render(sl, True, self._theme.text_primary), (sum_rect.x + 10, sum_rect.y + 28 + idx * 19))

        pygame.draw.rect(surface, self._theme.border, world_rect, width=2)

    def _draw_dwa_trajectories(
        self, surface: pygame.Surface, dwa: DWATelemetryPlaceholder
    ) -> None:
        """Render real DWA rejected (orange/red) and selected (green) trajectories."""
        for traj in dwa.rejected_trajectories:
            if len(traj) >= 2:
                pts = [self._transform.world_to_screen(x, y) for x, y in traj]
                pygame.draw.lines(surface, self._theme.safety_critical, False, pts, width=1)
        if dwa.selected_trajectory and len(dwa.selected_trajectory) >= 2:
            pts = [
                self._transform.world_to_screen(x, y)
                for x, y in dwa.selected_trajectory
            ]
            pygame.draw.lines(surface, self._theme.safety_safe, False, pts, width=3)

    def _draw_nearest_obstacle_line(
        self,
        surface: pygame.Surface,
        world: SimulationWorld,
        snapshot: SafetySnapshot | None,
    ) -> None:
        if (
            snapshot is None
            or snapshot.nearest_point_on_obstacle_m is None
            or snapshot.surface_clearance_m is None
        ):
            return

        rs = world.robot.state
        p_rob = self._transform.world_to_screen(rs.x_m, rs.y_m)
        p_obs = self._transform.world_to_screen(
            snapshot.nearest_point_on_obstacle_m[0],
            snapshot.nearest_point_on_obstacle_m[1],
        )
        color = self._safety_color(snapshot.safety_level)
        pygame.draw.line(surface, color, p_rob, p_obs, width=2)
        pygame.draw.circle(surface, color, p_obs, 4)

        mid_x = (p_rob[0] + p_obs[0]) // 2
        mid_y = (p_rob[1] + p_obs[1]) // 2
        lbl = self._font_small.render(
            f"{snapshot.surface_clearance_m:.2f} m", True, color
        )
        bg = pygame.Rect(
            mid_x - lbl.get_width() // 2 - 4,
            mid_y - lbl.get_height() // 2 - 2,
            lbl.get_width() + 8,
            lbl.get_height() + 4,
        )
        pygame.draw.rect(surface, self._theme.viewport_bg, bg, border_radius=3)
        pygame.draw.rect(surface, color, bg, width=1, border_radius=3)
        surface.blit(lbl, (bg.x + 4, bg.y + 2))

    def _draw_cell_rect(
        self,
        surface: pygame.Surface,
        row: int,
        col: int,
        res_m: float,
        color: tuple[int, int, int],
    ) -> None:
        cx_m, cy_m = grid_to_world(row, col, res_m)
        half = 0.5 * res_m
        tl_x, tl_y = self._transform.world_to_screen(cx_m - half, cy_m + half)
        br_x, br_y = self._transform.world_to_screen(cx_m + half, cy_m - half)
        rect = pygame.Rect(tl_x, tl_y, max(1, br_x - tl_x), max(1, br_y - tl_y))
        pygame.draw.rect(surface, color, rect)

    def _draw_inflated_cells(
        self,
        surface: pygame.Surface,
        world: SimulationWorld,
        inflated_grid: np.ndarray,
    ) -> None:
        res_m = world.world_config.grid_resolution_m
        raw_grid = world.occupancy_grid
        margin_mask = (inflated_grid != 0) & (raw_grid == 0)
        rows, cols = np.where(margin_mask)
        for r, c in zip(rows, cols):
            self._draw_cell_rect(surface, int(r), int(c), res_m, self._theme.inflated_cell)

    def _draw_explored_nodes(
        self,
        surface: pygame.Surface,
        world: SimulationWorld,
        explored_nodes: list[tuple[int, int]],
    ) -> None:
        res_m = world.world_config.grid_resolution_m
        for r, c in explored_nodes:
            self._draw_cell_rect(surface, r, c, res_m, self._theme.explored_node)

    def _draw_metric_grid(
        self, surface: pygame.Surface, world: SimulationWorld
    ) -> None:
        w_cfg = world.world_config
        minor = w_cfg.minor_grid_interval_m
        major = w_cfg.major_grid_interval_m

        x_m = 0.0
        while x_m <= w_cfg.width_m + 1e-6:
            is_major = math.isclose(x_m % major, 0.0, abs_tol=1e-6) or math.isclose(
                x_m % major, major, abs_tol=1e-6
            )
            color = self._theme.grid_major if is_major else self._theme.grid_minor
            p_start = self._transform.world_to_screen(x_m, 0.0)
            p_end = self._transform.world_to_screen(x_m, w_cfg.height_m)
            pygame.draw.line(surface, color, p_start, p_end, width=1)
            if is_major:
                label = self._font_small.render(f"{x_m:.0f}m", True, self._theme.axis_text)
                surface.blit(label, (p_start[0] - 8, p_start[1] + 4))
            x_m += minor

        y_m = 0.0
        while y_m <= w_cfg.height_m + 1e-6:
            is_major = math.isclose(y_m % major, 0.0, abs_tol=1e-6) or math.isclose(
                y_m % major, major, abs_tol=1e-6
            )
            color = self._theme.grid_major if is_major else self._theme.grid_minor
            p_start = self._transform.world_to_screen(0.0, y_m)
            p_end = self._transform.world_to_screen(w_cfg.width_m, y_m)
            pygame.draw.line(surface, color, p_start, p_end, width=1)
            if is_major and y_m > 0.0:
                label = self._font_small.render(f"{y_m:.0f}m", True, self._theme.axis_text)
                surface.blit(label, (p_start[0] - 22, p_start[1] - 7))
            y_m += minor

    def _draw_obstacles(
        self, surface: pygame.Surface, world: SimulationWorld
    ) -> None:
        for obs in world.obstacles:
            tl_x, tl_y = self._transform.world_to_screen(
                obs.x_m, obs.y_m + obs.height_m
            )
            br_x, br_y = self._transform.world_to_screen(
                obs.x_m + obs.width_m, obs.y_m
            )
            rect = pygame.Rect(tl_x, tl_y, br_x - tl_x, br_y - tl_y)
            pygame.draw.rect(surface, self._theme.obstacle_fill, rect)
            pygame.draw.rect(surface, self._theme.obstacle_border, rect, width=2)

    def _draw_dynamic_obstacles(
        self, surface: pygame.Surface, world: SimulationWorld
    ) -> None:
        report_map = {r.obstacle_id: r for r in world.collision_reports}
        margin_m = self._config.planner.safety_margin_m

        for obs in world.dynamic_obstacles:
            if not obs.active:
                continue
            center_px = self._transform.world_to_screen(obs.x_m, obs.y_m)
            radius_px = max(5, round(self._transform.meters_to_pixels(obs.radius_m)))
            safety_px = max(
                radius_px + 2,
                round(self._transform.meters_to_pixels(obs.radius_m + margin_m)),
            )

            report = report_map.get(obs.obstacle_id)
            if report and report.physical_overlap:
                fill_color = self._theme.safety_collision
                ring_color = self._theme.safety_collision
            elif report and report.safety_warning:
                fill_color = self._theme.dyn_obs_fill
                ring_color = self._theme.safety_caution
            else:
                fill_color = self._theme.dyn_obs_fill
                ring_color = self._theme.dyn_obs_outline

            pygame.draw.circle(surface, ring_color, center_px, safety_px, width=1)
            pygame.draw.circle(surface, fill_color, center_px, radius_px)
            pygame.draw.circle(
                surface, self._theme.dyn_obs_outline, center_px, radius_px, width=2
            )

            speed = obs.speed_mps
            if speed > 1e-3:
                tip_x_m = obs.x_m + obs.vx_mps * 0.8
                tip_y_m = obs.y_m + obs.vy_mps * 0.8
                tip_px = self._transform.world_to_screen(tip_x_m, tip_y_m)
                pygame.draw.line(
                    surface, self._theme.dyn_obs_vector, center_px, tip_px, width=2
                )
                pygame.draw.circle(surface, self._theme.dyn_obs_vector, tip_px, 3)

            speed_txt = "DURDU" if obs.is_stopped else f"{speed:.1f} m/s"
            lbl = self._font_small.render(
                f"{obs.obstacle_id} ({speed_txt})", True, self._theme.text_primary
            )
            surface.blit(
                lbl,
                (center_px[0] - lbl.get_width() // 2, center_px[1] - safety_px - 14),
            )

    def _draw_path(self, surface: pygame.Surface, astar_result: AStarResult) -> None:
        if len(astar_result.path_world_m) >= 2:
            pts = [
                self._transform.world_to_screen(x_m, y_m)
                for x_m, y_m in astar_result.path_world_m
            ]
            pygame.draw.lines(surface, self._theme.path_line, False, pts, width=3)
            for pt in pts:
                pygame.draw.circle(surface, self._theme.path_node, pt, 2)

    def _draw_start_and_goal(
        self,
        surface: pygame.Surface,
        start_m: tuple[float, float],
        goal_m: tuple[float, float],
    ) -> None:
        sx, sy = self._transform.world_to_screen(start_m[0], start_m[1])
        pygame.draw.circle(surface, self._theme.start_marker, (sx, sy), 10, width=2)
        s_lbl = self._font_small.render("B", True, self._theme.start_marker)
        surface.blit(s_lbl, (sx - 4, sy - 22))

        gx, gy = self._transform.world_to_screen(goal_m[0], goal_m[1])
        diamond = [(gx, gy - 10), (gx + 10, gy), (gx, gy + 10), (gx - 10, gy)]
        pygame.draw.polygon(surface, self._theme.goal_marker, diamond, width=2)
        pygame.draw.circle(surface, self._theme.goal_marker, (gx, gy), 3)
        g_lbl = self._font_small.render("H", True, self._theme.goal_marker)
        surface.blit(g_lbl, (gx - 4, gy - 23))

    def _draw_robot(
        self,
        surface: pygame.Surface,
        world: SimulationWorld,
        snapshot: SafetySnapshot | None,
    ) -> None:
        state = world.robot.state
        radius_m = world.robot.radius_m
        margin_m = self._config.planner.safety_margin_m
        center_px = self._transform.world_to_screen(state.x_m, state.y_m)
        radius_px = max(4, round(self._transform.meters_to_pixels(radius_m)))
        safety_px = max(
            radius_px + 3,
            round(self._transform.meters_to_pixels(radius_m + margin_m)),
        )

        # Configurable safety boundary around robot colored by live safety level.
        ring_color = (
            self._safety_color(snapshot.safety_level)
            if snapshot is not None
            else self._theme.safety_safe
        )
        pygame.draw.circle(surface, ring_color, center_px, safety_px, width=1)

        pygame.draw.circle(surface, self._theme.nav_blue, center_px, radius_px)
        pygame.draw.circle(
            surface, self._theme.robot_outline, center_px, radius_px, width=2
        )

        tip_x_m = state.x_m + radius_m * 1.15 * math.cos(state.theta_rad)
        tip_y_m = state.y_m + radius_m * 1.15 * math.sin(state.theta_rad)
        tip_px = self._transform.world_to_screen(tip_x_m, tip_y_m)
        pygame.draw.line(surface, self._theme.robot_heading, center_px, tip_px, width=3)
        pygame.draw.circle(surface, self._theme.robot_heading, center_px, 4)

    def _draw_left_panel(
        self,
        surface: pygame.Surface,
        world: SimulationWorld,
        buttons: list[Button],
        start_m: tuple[float, float],
        goal_m: tuple[float, float],
        astar_result: AStarResult | None,
        interaction_mode: InteractionMode,
        allow_diagonal: bool,
        target_reached: bool,
        status_message: str | None,
        hovered_tooltip: str | None,
    ) -> None:
        win = self._config.window
        panel_rect = pygame.Rect(0, 0, LEFT_PANEL_WIDTH, win.height)
        pygame.draw.rect(surface, self._theme.sidebar_bg, panel_rect)
        pygame.draw.line(
            surface,
            self._theme.border,
            (LEFT_PANEL_WIDTH, 0),
            (LEFT_PANEL_WIDTH, win.height),
            width=2,
        )

        pad = 12
        x = pad
        panel_w = LEFT_PANEL_WIDTH - 2 * pad

        surface.blit(
            self._font_title.render("DYNAMIC NAV", True, self._theme.nav_blue),
            (x, 12),
        )

        # Status badge
        if target_reached:
            status_text, status_color = "HEDEFE ULAŞILDI", self._theme.safety_safe
        else:
            status_map = {
                SimulationState.READY: ("HAZIR", self._theme.nav_blue),
                SimulationState.RUNNING: ("ÇALIŞIYOR", self._theme.safety_safe),
                SimulationState.PAUSED: ("DURAKLATILDI", self._theme.safety_caution),
            }
            status_text, status_color = status_map[world.state]

        status_rect = pygame.Rect(x, 76, panel_w, 24)
        pygame.draw.rect(surface, self._theme.panel_bg, status_rect, border_radius=5)
        pygame.draw.rect(surface, status_color, status_rect, width=1, border_radius=5)
        pygame.draw.circle(surface, status_color, (x + 12, 88), 4)
        surface.blit(
            self._font_section.render(f"DURUM: {status_text}", True, self._theme.text_primary),
            (x + 22, 81),
        )

        surface.blit(
            self._font_section.render("SİMÜLASYON", True, self._theme.text_secondary),
            (x, 108),
        )
        surface.blit(
            self._font_section.render("SENARYOLAR", True, self._theme.text_secondary),
            (x, 162),
        )
        surface.blit(
            self._font_section.render(
                "ETKİLEŞİM VE A* PLANLAYICI", True, self._theme.text_secondary
            ),
            (x, 248),
        )

        mode_label_map = {
            InteractionMode.DRIVE: "SÜRÜŞ",
            InteractionMode.SET_START: "BAŞLANGIÇ NOKTASI",
            InteractionMode.SET_GOAL: "HEDEF NOKTASI",
            InteractionMode.PLACE_OBSTACLE: "ENGEL EKLE",
        }
        active_mode_label = mode_label_map[interaction_mode]
        scen_label_map = {
            "HEAD-ON": "KARŞIDAN GELEN",
            "CROSSING": "ÇAPRAZ GEÇİŞ",
            "CORRIDOR": "KORİDOR",
            "STOP-AND-GO": "DUR-KALK",
        }
        active_scen_label = (
            scen_label_map.get(world.active_scenario_name)
            if world.active_scenario_name
            else None
        )

        for btn in buttons:
            if btn.label in ("PANELİ GİZLE", "GÜVENLİK PANELİ"):
                continue
            is_active = (
                (btn.label == "BAŞLAT" and world.state == SimulationState.RUNNING)
                or (btn.label == "DURAKLAT" and world.state == SimulationState.PAUSED)
                or (btn.label == active_mode_label)
                or (btn.label == "8-YÖN" and allow_diagonal)
                or (active_scen_label is not None and btn.label == active_scen_label)
            )
            btn.draw(surface, self._font_small, self._theme, active=is_active)

        # Compact Navigation Telemetry
        y = 376
        surface.blit(
            self._font_section.render("NAVİGASYON TELEMETRİSİ", True, self._theme.text_secondary),
            (x, y),
        )
        y += 16

        s = world.robot.state
        rows, cols = world.occupancy_grid.shape
        res_m = world.world_config.grid_resolution_m
        if astar_result is None:
            plan_txt = "BEKLEMEDE"
        elif astar_result.success:
            plan_txt = f"BULUNDU ({astar_result.path_cost_m:.1f}m)"
        else:
            plan_txt = "ROTA BULUNAMADI"

        nav_rows = [
            ("SİMÜLASYON ZAMANI", f"{world.sim_time_s:6.2f} s"),
            ("ROBOT KONUMU", f"({s.x_m:4.1f}, {s.y_m:4.1f}) m"),
            ("YÖNELİM", f"{math.degrees(s.theta_rad):5.1f} deg"),
            ("HIZ", f"{s.v_mps:.1f} m/s, {s.omega_radps:.1f} r/s"),
            ("IZGARA", f"{cols}x{rows} ({res_m:.2f}m)"),
            ("BAŞLANGIÇ / HEDEF", f"({start_m[0]:.0f},{start_m[1]:.0f})->({goal_m[0]:.0f},{goal_m[1]:.0f})"),
            ("A* ROTASI", plan_txt),
        ]
        nav_h = len(nav_rows) * 19 + 8
        nav_rect = pygame.Rect(x, y, panel_w, nav_h)
        pygame.draw.rect(surface, self._theme.panel_bg, nav_rect, border_radius=6)
        pygame.draw.rect(surface, self._theme.border, nav_rect, width=1, border_radius=6)

        ry = y + 5
        for lbl, val in nav_rows:
            surface.blit(self._font_small.render(lbl, True, self._theme.text_secondary), (x + 8, ry))
            v_surf = self._font_small.render(val, True, self._theme.text_primary)
            surface.blit(v_surf, (x + panel_w - v_surf.get_width() - 8, ry))
            ry += 19

        y += nav_h + 10

        # Controls summary
        surface.blit(
            self._font_section.render("KONTROLLER", True, self._theme.text_secondary),
            (x, y),
        )
        y += 16
        ctrl_rows = [
            ("W/S veya Yukarı/Aşağı", "İleri / Geri"),
            ("A/D veya Sol/Sağ", "Döndür"),
            ("Boşluk / R", "Başlat / Sıfırla"),
            ("Sol / Sağ Tık", "Uygula / Sil"),
            ("Esc", "Ana Menü / Çıkış"),
        ]
        ctrl_h = len(ctrl_rows) * 18 + 8
        ctrl_rect = pygame.Rect(x, y, panel_w, ctrl_h)
        pygame.draw.rect(surface, self._theme.panel_bg, ctrl_rect, border_radius=6)
        pygame.draw.rect(surface, self._theme.border, ctrl_rect, width=1, border_radius=6)
        ry = y + 5
        for k, a in ctrl_rows:
            surface.blit(self._font_small.render(k, True, self._theme.nav_blue), (x + 8, ry))
            a_surf = self._font_small.render(a, True, self._theme.text_primary)
            surface.blit(a_surf, (x + panel_w - a_surf.get_width() - 8, ry))
            ry += 18

        y += ctrl_h + 10

        # Status notification or contextual tooltip
        banner_text = hovered_tooltip or status_message
        if banner_text:
            b_color = (
                self._theme.nav_blue
                if hovered_tooltip
                else (self._theme.safety_safe if target_reached else self._theme.safety_caution)
            )
            b_rect = pygame.Rect(x, y, panel_w, 42)
            pygame.draw.rect(surface, self._theme.panel_bg, b_rect, border_radius=6)
            pygame.draw.rect(surface, b_color, b_rect, width=1, border_radius=6)
            self._blit_wrapped(surface, banner_text, x + 8, y + 6, panel_w - 16, b_color)

    def _draw_right_safety_panel(
        self,
        surface: pygame.Surface,
        world: SimulationWorld,
        buttons: list[Button],
        safety_tracker: SafetyTracker,
        dwa_telemetry: DWATelemetryPlaceholder,
    ) -> None:
        win = self._config.window
        panel_left = win.width - RIGHT_PANEL_WIDTH
        panel_rect = pygame.Rect(panel_left, 0, RIGHT_PANEL_WIDTH, win.height)
        pygame.draw.rect(surface, self._theme.sidebar_bg, panel_rect)
        pygame.draw.line(
            surface,
            self._theme.border,
            (panel_left, 0),
            (panel_left, win.height),
            width=2,
        )

        pad = 12
        x = panel_left + pad
        panel_w = RIGHT_PANEL_WIDTH - 2 * pad

        surface.blit(
            self._font_title.render("GÜVENLİK PANELİ", True, self._theme.accent),
            (x, 12),
        )
        for btn in buttons:
            if btn.label == "PANELİ GİZLE":
                btn.draw(surface, self._font_small, self._theme, active=False)

        snapshot = safety_tracker.latest_snapshot
        level = snapshot.safety_level if snapshot else SafetyLevel.SAFE
        lvl_color = self._safety_color(level)
        lvl_text = self._safety_label_tr(level)

        # Safety Light & Distance Bar
        y = 46
        bar_box = pygame.Rect(x, y, panel_w, 52)
        pygame.draw.rect(surface, self._theme.panel_bg, bar_box, border_radius=6)
        pygame.draw.rect(surface, lvl_color, bar_box, width=1, border_radius=6)
        pygame.draw.circle(surface, lvl_color, (x + 18, y + 18), 8)
        surface.blit(
            self._font_section.render(f"GÜVENLİK IŞIĞI: {lvl_text}", True, lvl_color),
            (x + 34, y + 10),
        )

        # Distance progress bar (0.0m to 1.0m scale)
        bar_x, bar_y, bar_w, bar_h = x + 12, y + 34, panel_w - 24, 10
        pygame.draw.rect(surface, self._theme.viewport_bg, (bar_x, bar_y, bar_w, bar_h), border_radius=4)
        if snapshot and snapshot.surface_clearance_m is not None:
            ratio = max(0.0, min(1.0, snapshot.surface_clearance_m / 1.0))
            fill_w = round(bar_w * ratio)
            if fill_w > 0:
                pygame.draw.rect(surface, lvl_color, (bar_x, bar_y, fill_w, bar_h), border_radius=4)
        # 0.15 m safety margin tick mark
        tick_x = bar_x + round(bar_w * 0.15)
        pygame.draw.line(surface, self._theme.text_primary, (tick_x, bar_y - 2), (tick_x, bar_y + bar_h + 2), 1)

        # Safety Telemetry Table
        y = 108
        surface.blit(
            self._font_section.render("GERÇEK ZAMANLI GÜVENLİK ÖLÇÜMLERİ", True, self._theme.text_secondary),
            (x, y),
        )
        y += 16

        if snapshot is None:
            obs_str = "--"
            center_str = "--"
            clear_str = "--"
            margin_str = f"{self._config.planner.safety_margin_m:.2f} m"
            ttc_str = "Yok"
        else:
            obs_str = (
                f"{snapshot.nearest_obstacle_id} ({snapshot.nearest_obstacle_type})"
                if snapshot.nearest_obstacle_id
                else "Yok"
            )
            center_str = (
                f"{snapshot.center_to_center_distance_m:.3f} m"
                if snapshot.center_to_center_distance_m is not None
                else "--"
            )
            clear_str = (
                f"{snapshot.surface_clearance_m:.3f} m"
                if snapshot.surface_clearance_m is not None
                else "--"
            )
            margin_str = f"{snapshot.required_safety_margin_m:.2f} m"
            ttc_str = f"{snapshot.ttc_s:.2f} s" if snapshot.ttc_s is not None else "Yok"

        min_c = (
            f"{safety_tracker.min_clearance_m:.3f} m"
            if safety_tracker.min_clearance_m is not None
            else "--"
        )

        safety_rows = [
            ("EN YAKIN ENGEL", obs_str),
            ("MERKEZLER ARASI MESAFE", center_str),
            ("GERÇEK AÇIKLIK", clear_str),
            ("GEREKLİ GÜVENLİK PAYI", margin_str),
            ("MİNİMUM AÇIKLIK", min_c),
            ("GÜVENLİK DURUMU", lvl_text),
            ("GÜVENLİK İHLALİ", str(safety_tracker.warning_count)),
            ("ÇARPIŞMA SAYISI", str(safety_tracker.collision_count)),
            ("RAMAK KALA OLAYI", str(safety_tracker.near_miss_count)),
            ("TAHMİNİ ÇARPIŞMA SÜRESİ", ttc_str),
        ]

        row_h = 20
        tbl_h = len(safety_rows) * row_h + 10
        tbl_rect = pygame.Rect(x, y, panel_w, tbl_h)
        pygame.draw.rect(surface, self._theme.panel_bg, tbl_rect, border_radius=6)
        pygame.draw.rect(surface, self._theme.border, tbl_rect, width=1, border_radius=6)

        ry = y + 5
        for lbl, val in safety_rows:
            surface.blit(self._font_small.render(lbl, True, self._theme.text_secondary), (x + 8, ry))
            v_col = lvl_color if lbl in ("GÜVENLİK DURUMU", "GERÇEK AÇIKLIK") else self._theme.text_primary
            v_surf = self._font_small.render(val, True, v_col)
            surface.blit(v_surf, (x + panel_w - v_surf.get_width() - 8, ry))
            ry += row_h

        y += tbl_h + 10

        # Live Clearance vs Time Chart
        surface.blit(
            self._font_section.render("AÇIKLIK - ZAMAN GRAFİĞİ (m)", True, self._theme.text_secondary),
            (x, y),
        )
        y += 16
        chart_rect = pygame.Rect(x, y, panel_w, 112)
        self._draw_clearance_chart(surface, chart_rect, safety_tracker)
        y += 122

        # Safety Event Log
        surface.blit(
            self._font_section.render("GÜVENLİK OLAY GÜNLÜĞÜ", True, self._theme.text_secondary),
            (x, y),
        )
        y += 16
        log_rect = pygame.Rect(x, y, panel_w, 148)
        pygame.draw.rect(surface, self._theme.panel_bg, log_rect, border_radius=6)
        pygame.draw.rect(surface, self._theme.border, log_rect, width=1, border_radius=6)

        events = safety_tracker.event_log
        if not events:
            surface.blit(
                self._font_small.render("Henüz güvenlik olayı kaydedilmedi.", True, self._theme.text_secondary),
                (x + 8, y + 8),
            )
        else:
            ey = y + 6
            for ev in events[-7:]:
                ev_col = self._safety_color(ev.level)
                surface.blit(
                    self._font_small.render(ev.format_line(), True, ev_col),
                    (x + 8, ey),
                )
                ey += 19

        y += 158

        # DWA Trajectory Analysis (Phase 4)
        surface.blit(
            self._font_section.render("DWA YÖRÜNGE ANALİZİ (SI)", True, self._theme.text_secondary),
            (x, y),
        )
        y += 16
        dwa_rect = pygame.Rect(x, y, panel_w, 66)
        pygame.draw.rect(surface, self._theme.panel_bg, dwa_rect, border_radius=6)
        pygame.draw.rect(surface, self._theme.border, dwa_rect, width=1, border_radius=6)
        if not dwa_telemetry.active:
            surface.blit(
                self._font_small.render("Durum: TAKİPÇİ modu aktif (DWA beklemede).", True, self._theme.text_secondary),
                (x + 8, y + 10),
            )
            surface.blit(
                self._font_small.render("Aday yörüngeler yalnızca DWA modunda üretilir.", True, self._theme.text_secondary),
                (x + 8, y + 34),
            )
        else:
            v_str = f"{dwa_telemetry.selected_v_mps:.2f} m/s" if dwa_telemetry.selected_v_mps is not None else "--"
            w_str = f"{dwa_telemetry.selected_omega_radps:.2f} rad/s" if dwa_telemetry.selected_omega_radps is not None else "--"
            sc_str = f"{dwa_telemetry.evaluation_score:.2f}" if dwa_telemetry.evaluation_score is not None else "--"
            cl_str = (
                f"{dwa_telemetry.predicted_min_clearance_m:.2f} m"
                if dwa_telemetry.predicted_min_clearance_m is not None
                else "--"
            )
            cand_n = len(dwa_telemetry.candidate_trajectories)
            rej_n = len(dwa_telemetry.rejected_trajectories)
            surface.blit(
                self._font_small.render(f"Seçilen: v={v_str}, ω={w_str}", True, self._theme.nav_blue),
                (x + 8, y + 8),
            )
            surface.blit(
                self._font_small.render(f"Skor: {sc_str}  |  Tahmini Açıklık: {cl_str}", True, self._theme.text_primary),
                (x + 8, y + 26),
            )
            surface.blit(
                self._font_small.render(f"Yörüngeler: {cand_n} aday / {rej_n} reddedildi", True, self._theme.text_secondary),
                (x + 8, y + 44),
            )

    def _draw_clearance_chart(
        self,
        surface: pygame.Surface,
        rect: pygame.Rect,
        safety_tracker: SafetyTracker,
    ) -> None:
        pygame.draw.rect(surface, self._theme.panel_bg, rect, border_radius=6)
        pygame.draw.rect(surface, self._theme.border, rect, width=1, border_radius=6)

        pad = 8
        gx = rect.x + pad
        gy = rect.y + pad
        gw = rect.width - 2 * pad
        gh = rect.height - 2 * pad
        max_y_m = 2.0

        # Horizontal safety margin reference line (0.15 m)
        margin_m = self._config.planner.safety_margin_m
        margin_y = gy + gh - round((min(max_y_m, margin_m) / max_y_m) * gh)
        for sx in range(gx, gx + gw, 8):
            pygame.draw.line(
                surface, self._theme.safety_critical, (sx, margin_y), (min(gx + gw, sx + 4), margin_y), 1
            )
        surface.blit(
            self._font_small.render(f"{margin_m:.2f}m", True, self._theme.safety_critical),
            (gx + 2, margin_y - 13),
        )

        # Minimum observed clearance marker line
        if safety_tracker.min_clearance_m is not None:
            mc = max(0.0, min(max_y_m, safety_tracker.min_clearance_m))
            mc_y = gy + gh - round((mc / max_y_m) * gh)
            pygame.draw.line(surface, self._theme.safety_caution, (gx, mc_y), (gx + gw, mc_y), 1)

        history = safety_tracker.clearance_history
        if len(history) >= 2:
            n = len(history)
            pts: list[tuple[int, int]] = []
            for idx, (_, val_m) in enumerate(history):
                px = gx + round((idx / max(1, n - 1)) * gw)
                clamped = max(0.0, min(max_y_m, val_m))
                py = gy + gh - round((clamped / max_y_m) * gh)
                pts.append((px, py))
            pygame.draw.lines(surface, self._theme.nav_blue, False, pts, width=2)

    def _blit_wrapped(
        self,
        surface: pygame.Surface,
        text: str,
        x: int,
        y: int,
        max_w: int,
        color: tuple[int, int, int],
    ) -> None:
        words = text.split()
        lines: list[str] = []
        cur = ""
        for w in words:
            test = f"{cur} {w}".strip()
            if self._font_small.size(test)[0] <= max_w:
                cur = test
            else:
                if cur:
                    lines.append(cur)
                cur = w
        if cur:
            lines.append(cur)
        for i, line in enumerate(lines[:2]):
            surface.blit(self._font_small.render(line, True, color), (x, y + i * 15))

    def _draw_sim_objects(
        self, surface: pygame.Surface, world: SimulationWorld
    ) -> None:
        """Render universal SimObjects using their exact world SI geometry."""
        selected_id = world.object_editor.selected_id
        for obj in world.sim_objects:
            if not obj.motion.active:
                continue
            geom = obj.geometry()
            is_sel = obj.object_id == selected_id
            fill = obj.style.fill_rgb
            outline = self._theme.adas_selected if is_sel else obj.style.outline_rgb
            border_w = 3 if is_sel else 2

            if isinstance(geom, CircleGeometry):
                cx, cy = self._transform.world_to_screen(geom.center_x_m, geom.center_y_m)
                r_px = max(3, self._transform.meters_to_pixels(geom.radius_m))
                pygame.draw.circle(surface, fill, (cx, cy), r_px)
                pygame.draw.circle(surface, outline, (cx, cy), r_px, width=border_w)
            elif isinstance(geom, OrientedRectGeometry):
                pts = [self._transform.world_to_screen(vx, vy) for vx, vy in geom.corners_m()]
                pygame.draw.polygon(surface, fill, pts)
                pygame.draw.polygon(surface, outline, pts, width=border_w)
                cx, cy = self._transform.world_to_screen(geom.center_x_m, geom.center_y_m)
                hx_m = geom.center_x_m + 0.42 * geom.length_m * math.cos(geom.heading_rad)
                hy_m = geom.center_y_m + 0.42 * geom.length_m * math.sin(geom.heading_rad)
                hx, hy = self._transform.world_to_screen(hx_m, hy_m)
                pygame.draw.line(surface, self._theme.robot_heading, (cx, cy), (hx, hy), width=2)
            elif isinstance(geom, PolygonGeometry):
                pts = [self._transform.world_to_screen(vx, vy) for vx, vy in geom.vertices_m]
                pygame.draw.polygon(surface, fill, pts)
                pygame.draw.polygon(surface, outline, pts, width=border_w)
            elif isinstance(geom, CapsuleGeometry):
                a0, a1 = geom.spine_segment_m()
                p0 = self._transform.world_to_screen(a0[0], a0[1])
                p1 = self._transform.world_to_screen(a1[0], a1[1])
                r_px = max(3, self._transform.meters_to_pixels(geom.radius_m))
                pygame.draw.line(surface, fill, p0, p1, width=2 * r_px)
                pygame.draw.circle(surface, fill, p0, r_px)
                pygame.draw.circle(surface, fill, p1, r_px)
                pygame.draw.circle(surface, outline, p0, r_px, width=border_w)
                pygame.draw.circle(surface, outline, p1, r_px, width=border_w)

            # Label & velocity vector
            lx, ly = self._transform.world_to_screen(obj.motion.x_m, obj.motion.y_m)
            lbl = self._font_small.render(obj.object_id, True, self._theme.text_primary)
            surface.blit(lbl, (lx - lbl.get_width() // 2, ly - 18))
            spd = math.hypot(obj.motion.vx_mps, obj.motion.vy_mps)
            if spd > 0.05:
                vx_m = obj.motion.x_m + obj.motion.vx_mps * 0.6
                vy_m = obj.motion.y_m + obj.motion.vy_mps * 0.6
                vp = self._transform.world_to_screen(vx_m, vy_m)
                pygame.draw.line(surface, self._theme.dyn_obs_vector, (lx, ly), vp, width=2)

    def _draw_measurement_overlay(
        self, surface: pygame.Surface, m: MeasurementResult
    ) -> None:
        """Render exact closest-boundary measurement line and live readout badge."""
        pa = self._transform.world_to_screen(m.closest_point_a_m[0], m.closest_point_a_m[1])
        pb = self._transform.world_to_screen(m.closest_point_b_m[0], m.closest_point_b_m[1])
        color = (
            self._theme.safety_collision
            if m.is_overlapping
            else (self._theme.safety_critical if m.safety_violated else self._theme.adas_measure_line)
        )
        pygame.draw.line(surface, color, pa, pb, width=2)
        pygame.draw.circle(surface, color, pa, 5)
        pygame.draw.circle(surface, color, pb, 5)
        mx = (pa[0] + pb[0]) // 2
        my = (pa[1] + pb[1]) // 2
        txt = f"{m.surface_clearance_m:.2f}m"
        badge = self._font_small.render(txt, True, self._theme.text_primary)
        bg_rect = pygame.Rect(mx - badge.get_width() // 2 - 4, my - 18, badge.get_width() + 8, 16)
        pygame.draw.rect(surface, self._theme.adas_card_bg, bg_rect, border_radius=4)
        pygame.draw.rect(surface, color, bg_rect, width=1, border_radius=4)
        surface.blit(badge, (bg_rect.x + 4, bg_rect.y + 1))

    def _draw_adas_top_bar(
        self,
        surface: pygame.Surface,
        world: SimulationWorld,
        buttons: list[Button],
        goal_m: tuple[float, float],
        exp_status: str,
        exp_color: tuple[int, int, int],
        status_message: str | None,
    ) -> None:
        win = self._config.window
        top_rect = pygame.Rect(0, 0, win.width, win.top_bar_height)
        pygame.draw.rect(surface, self._theme.adas_topbar_bg, top_rect)
        pygame.draw.line(surface, self._theme.adas_card_border, (0, win.top_bar_height), (win.width, win.top_bar_height), width=1)

        title = self._font_title.render("DYNAMIC NAV", True, self._theme.accent)
        surface.blit(title, (14, 8))

        for btn in buttons:
            if btn.rect.y < 40:
                is_active = btn.label == "DEMO: AÇIK" or btn.label == "MOD: DWA"
                btn.draw(surface, self._font_small, self._theme, active=is_active)

        pill_rect = pygame.Rect(win.width - 148, 8, 134, 28)
        pygame.draw.rect(surface, self._theme.adas_card_bg, pill_rect, border_radius=6)
        pygame.draw.rect(surface, exp_color, pill_rect, width=1, border_radius=6)
        pill_txt = self._font_small.render(exp_status, True, exp_color)
        surface.blit(pill_txt, pill_txt.get_rect(center=pill_rect.center))

        if status_message:
            msg_surf = self._font_small.render(status_message[:36], True, self._theme.safety_caution)
            surface.blit(msg_surf, (win.width - 395, 14))

        # Experiment Identity Panel (Deney Kimliği Paneli)
        id_rect = pygame.Rect(14, 42, win.width - 28, 56)
        pygame.draw.rect(surface, self._theme.adas_card_bg, id_rect, border_radius=6)
        pygame.draw.rect(surface, self._theme.adas_card_border, id_rect, width=1, border_radius=6)

        sc_code = world.active_scenario_name or "CUSTOM"
        sc_def = None
        if world.active_scenario_name is not None:
            try:
                sc_def = ScenarioManager.get_scenario(world.active_scenario_name)
            except KeyError:
                sc_def = None
        if sc_def is not None:
            sc_desc = sc_def.description
            beh_txt = sc_def.behavior.name
        else:
            sc_desc = "Kullanıcı / test tanımlı özel simülasyon senaryosu."
            beh_txt = "SERBEST / ÖZEL"

        line1 = (
            f"DENEY: {sc_code}  |  Durum: {exp_status}  |  "
            f"Hedef: ({goal_m[0]:.1f}, {goal_m[1]:.1f}) m  |  "
            f"Engel Davranışı: {beh_txt}"
        )
        line2 = f"Amaç: {sc_desc}"
        surface.blit(self._font_section.render(line1, True, self._theme.accent), (id_rect.x + 12, id_rect.y + 7))
        surface.blit(self._font_small.render(line2, True, self._theme.text_primary), (id_rect.x + 12, id_rect.y + 29))

    def _draw_adas_left_rail(
        self,
        surface: pygame.Surface,
        world: SimulationWorld,
        buttons: list[Button],
        interaction_mode: InteractionMode,
        allow_diagonal: bool,
        selected_category: ObjectCategory,
    ) -> None:
        win = self._config.window
        rail_h = win.height - win.top_bar_height - win.bottom_dock_height
        rail_rect = pygame.Rect(0, win.top_bar_height, win.left_rail_width, rail_h)
        pygame.draw.rect(surface, self._theme.adas_rail_bg, rail_rect)
        pygame.draw.line(surface, self._theme.adas_card_border, (win.left_rail_width, win.top_bar_height), (win.left_rail_width, win.height - win.bottom_dock_height), width=1)

        surface.blit(self._font_section.render("EDİTÖR & ARAÇLAR", True, self._theme.accent), (14, win.top_bar_height + 10))
        cat_txt = self._font_small.render(f"Kategori: {selected_category.label_tr}", True, self._theme.text_primary)
        surface.blit(cat_txt, (14, win.top_bar_height + 30))

        mode_map = {
            "SEÇ / TAŞI": InteractionMode.SELECT_EDIT,
            "ÖLÇÜM ARACI": InteractionMode.MEASURE,
            "SÜRÜŞ": InteractionMode.DRIVE,
            "BAŞLANGIÇ": InteractionMode.SET_START,
            "HEDEF": InteractionMode.SET_GOAL,
            "STATİK ENGEL": InteractionMode.PLACE_OBSTACLE,
        }
        for btn in buttons:
            if btn.rect.y < 40 or btn.rect.x >= win.left_rail_width:
                continue
            is_active = False
            if btn.label in mode_map:
                is_active = interaction_mode == mode_map[btn.label]
            elif btn.label.startswith("8-YÖN"):
                is_active = allow_diagonal
            btn.draw(surface, self._font_small, self._theme, active=is_active)

    def _draw_adas_right_inspector(
        self,
        surface: pygame.Surface,
        world: SimulationWorld,
        buttons: list[Button],
        safety_tracker: SafetyTracker,
        dwa_telemetry: DWATelemetryPlaceholder,
        measurement_result: MeasurementResult | None,
        inspector_fields: list[NumericField],
    ) -> None:
        win = self._config.window
        rx = win.width - win.right_inspector_width
        rh = win.height - win.top_bar_height - win.bottom_dock_height
        panel_rect = pygame.Rect(rx, win.top_bar_height, win.right_inspector_width, rh)
        pygame.draw.rect(surface, self._theme.adas_rail_bg, panel_rect)
        pygame.draw.line(surface, self._theme.adas_card_border, (rx, win.top_bar_height), (rx, win.height - win.bottom_dock_height), width=1)

        y = win.top_bar_height + 10
        surface.blit(self._font_section.render("NESNE DENETÇİSİ", True, self._theme.accent), (rx + 14, y))
        y += 22

        sel = world.object_editor.selected_object
        if sel is not None:
            info_lines = [
                f"ID: {sel.object_id} | {sel.category.label_tr}",
                f"Geometri: {sel.physics.geometry_type}",
                f"Davranış: {sel.motion.behavior.label_tr}",
            ]
            for line in info_lines:
                surface.blit(self._font_small.render(line, True, self._theme.text_primary), (rx + 14, y))
                y += 17
            for fld in inspector_fields:
                fld.draw(surface, self._font_small, self._theme)
            for btn in buttons:
                if btn.rect.x >= rx:
                    btn.draw(surface, self._font_small, self._theme, active=False)
        else:
            surface.blit(self._font_small.render("Seçili nesne yok (Sol araçlardan ekleyin).", True, self._theme.text_secondary), (rx + 14, y))
            y += 24

        # Measurement card
        my = win.top_bar_height + 370
        m_rect = pygame.Rect(rx + 12, my, win.right_inspector_width - 24, 130)
        pygame.draw.rect(surface, self._theme.adas_card_bg, m_rect, border_radius=6)
        pygame.draw.rect(surface, self._theme.adas_card_border, m_rect, width=1, border_radius=6)
        surface.blit(self._font_section.render("CANLI ÖLÇÜM ARACI", True, self._theme.adas_measure_line), (m_rect.x + 10, m_rect.y + 8))
        if measurement_result is not None:
            ttc_txt = f"{measurement_result.ttc_s:.2f} s" if measurement_result.ttc_s is not None else "—"
            m_lines = [
                f"Çift: {measurement_result.entity_a_id} <-> {measurement_result.entity_b_id}",
                f"Merkez Uzaklığı: {measurement_result.center_distance_m:.3f} m",
                f"Yüzey Açıklığı:  {measurement_result.surface_clearance_m:.3f} m",
                f"Gerekli Emniyet: {measurement_result.required_safety_distance_m:.2f} m",
                f"Bağıl Hız: {measurement_result.relative_speed_mps:.2f} m/s | TTC: {ttc_txt}",
            ]
            for idx, ml in enumerate(m_lines):
                col = self._theme.safety_collision if (idx == 2 and measurement_result.safety_violated) else self._theme.text_primary
                surface.blit(self._font_small.render(ml, True, col), (m_rect.x + 10, m_rect.y + 28 + idx * 18))
        else:
            surface.blit(self._font_small.render("Ölçüm Aracı ile iki nesne/robot seçin.", True, self._theme.text_secondary), (m_rect.x + 10, m_rect.y + 34))

        # Live Safety Summary card
        sy = my + 142
        s_rect = pygame.Rect(rx + 12, sy, win.right_inspector_width - 24, 118)
        pygame.draw.rect(surface, self._theme.adas_card_bg, s_rect, border_radius=6)
        pygame.draw.rect(surface, self._theme.adas_card_border, s_rect, width=1, border_radius=6)
        snap = safety_tracker.latest_snapshot
        surface.blit(self._font_section.render("CANLI GÜVENLİK ÖZETİ", True, self._theme.accent), (s_rect.x + 10, s_rect.y + 8))
        if snap is not None:
            lvl_col = self._safety_color(snap.safety_level)
            clr_txt = f"{snap.surface_clearance_m:.3f} m" if snap.surface_clearance_m is not None else "—"
            min_txt = f"{safety_tracker.min_clearance_m:.3f} m" if safety_tracker.min_clearance_m is not None else "—"
            s_lines = [
                f"Durum: {self._safety_label_tr(snap.safety_level)}",
                f"En Yakın Mesafe: {clr_txt}",
                f"Min Kayıt: {min_txt}",
                f"Çarpışma: {safety_tracker.collision_count} | İhlal: {safety_tracker.warning_count}",
            ]
            for idx, sl in enumerate(s_lines):
                surface.blit(self._font_small.render(sl, True, lvl_col if idx == 0 else self._theme.text_primary), (s_rect.x + 10, s_rect.y + 28 + idx * 19))

    def _draw_adas_bottom_dock(
        self,
        surface: pygame.Surface,
        world: SimulationWorld,
        goal_m: tuple[float, float],
        safety_tracker: SafetyTracker,
        dwa_telemetry: DWATelemetryPlaceholder,
        clearance_history: list[float],
        speed_history: list[float],
        replan_counts: tuple[int, int],
        hovered_tooltip: str | None,
        stop_count: int = 0,
        min_safety_margin_m: float | None = None,
        sim_speed_multiplier: float = 1.0,
        exp_status: str = "READY",
        exp_color: tuple[int, int, int] = (56, 189, 248),
    ) -> None:
        win = self._config.window
        dy = win.height - win.bottom_dock_height
        dock_rect = pygame.Rect(0, dy, win.width, win.bottom_dock_height)
        pygame.draw.rect(surface, self._theme.adas_topbar_bg, dock_rect)
        pygame.draw.line(surface, self._theme.adas_card_border, (0, dy), (win.width, dy), width=1)

        # Left column: Live Telemetry (distinct geometric distance vs safety margin)
        rs = world.robot.state
        rem_goal_m = math.hypot(goal_m[0] - rs.x_m, goal_m[1] - rs.y_m)
        snap = safety_tracker.latest_snapshot
        geom_dist_txt = (
            f"{snap.surface_clearance_m:.3f} m"
            if snap is not None and snap.surface_clearance_m is not None
            else "—"
        )
        margin_txt = f"{min_safety_margin_m:.3f} m" if min_safety_margin_m is not None else "—"

        t_rect = pygame.Rect(14, dy + 10, 420, win.bottom_dock_height - 20)
        pygame.draw.rect(surface, self._theme.adas_card_bg, t_rect, border_radius=6)
        pygame.draw.rect(surface, self._theme.adas_card_border, t_rect, width=1, border_radius=6)
        surface.blit(self._font_section.render("CANLI TELEMETRİ", True, self._theme.accent), (t_rect.x + 10, t_rect.y + 6))
        t_lines = [
            f"Hız: {rs.v_mps:.2f} m/s (ω={rs.omega_radps:.2f} rad/s) | Hedefe Kalan: {rem_goal_m:.2f} m",
            f"En Yakın Engel (Geometrik): {geom_dist_txt} | Min. Güvenlik Payı: {margin_txt}",
            f"Replan: {replan_counts[0]}/{replan_counts[1]} | Durma: {stop_count} | Süre: {world.sim_time_s:.2f} s ({sim_speed_multiplier:.1f}x)",
            hovered_tooltip or f"Durum: {exp_status} | Mod: {'DWA' if dwa_telemetry.active else 'A*/Takipçi'}",
        ]
        for idx, tl in enumerate(t_lines):
            col = exp_color if idx == 3 and not hovered_tooltip else self._theme.text_primary
            surface.blit(self._font_small.render(tl, True, col), (t_rect.x + 10, t_rect.y + 26 + idx * 21))

        # Middle column: Live clearance & speed chart
        g_rect = pygame.Rect(450, dy + 10, 450, win.bottom_dock_height - 20)
        pygame.draw.rect(surface, self._theme.adas_card_bg, g_rect, border_radius=6)
        pygame.draw.rect(surface, self._theme.adas_card_border, g_rect, width=1, border_radius=6)
        surface.blit(self._font_section.render("MESAFE (m) & HIZ (m/s) GRAFİĞİ", True, self._theme.accent), (g_rect.x + 10, g_rect.y + 8))
        plot_rect = pygame.Rect(g_rect.x + 12, g_rect.y + 30, g_rect.width - 24, g_rect.height - 42)
        pygame.draw.rect(surface, self._theme.viewport_bg, plot_rect, border_radius=4)
        ch = clearance_history[-120:]
        if len(ch) >= 2:
            pts_c = [
                (
                    plot_rect.x + int(i / max(1, len(ch) - 1) * plot_rect.width),
                    plot_rect.bottom - int(max(0.0, min(3.0, val)) / 3.0 * plot_rect.height),
                )
                for i, val in enumerate(ch)
            ]
            pygame.draw.lines(surface, self._theme.adas_chart_line, False, pts_c, width=2)
        sh = speed_history[-120:]
        if len(sh) >= 2:
            pts_v = [
                (
                    plot_rect.x + int(i / max(1, len(sh) - 1) * plot_rect.width),
                    plot_rect.bottom - int(max(0.0, min(3.0, val)) / 3.0 * plot_rect.height),
                )
                for i, val in enumerate(sh)
            ]
            pygame.draw.lines(surface, self._theme.robot_heading, False, pts_v, width=2)

        # Right column: Event timeline
        e_rect = pygame.Rect(906, dy + 12, win.width - 920, win.bottom_dock_height - 24)
        pygame.draw.rect(surface, self._theme.adas_card_bg, e_rect, border_radius=6)
        pygame.draw.rect(surface, self._theme.adas_card_border, e_rect, width=1, border_radius=6)
        surface.blit(self._font_section.render("OLAY ZAMAN ÇİZELGESİ", True, self._theme.accent), (e_rect.x + 10, e_rect.y + 8))
        events = list(safety_tracker.event_log)[-4:]
        if not events:
            surface.blit(self._font_small.render("Kayıtlı güvenlik olayı yok.", True, self._theme.text_secondary), (e_rect.x + 10, e_rect.y + 34))
        else:
            for idx, ev in enumerate(reversed(events)):
                ev_str = ev.format_line()
                surface.blit(self._font_small.render(ev_str, True, self._theme.safety_caution), (e_rect.x + 10, e_rect.y + 30 + idx * 22))
