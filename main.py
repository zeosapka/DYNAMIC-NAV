"""Entry point for the DYNAMIC NAV autonomous mobile robot simulator."""

import argparse
import os
import sys
import traceback

import pygame

from config import DEFAULT_CONFIG
from interface.app import DynamicNavApp
from interface.views import AppView
from simulation.scenarios import load_scenario_from_json


def main(argv: list[str] | None = None) -> None:
    """Initialize and run the DYNAMIC NAV application with startup diagnostics."""
    parser = argparse.ArgumentParser(description="DYNAMIC-NAV Autonomous Mobile Robot Simulator")
    parser.add_argument(
        "--scenario-json",
        type=str,
        default=None,
        help="Optional path to a custom JSON scenario definition file (SI units).",
    )
    args = parser.parse_args(argv)
    try:
        print("[DYNAMIC NAV] Initializing simulator...", flush=True)
        sdl_override = os.environ.get("SDL_VIDEODRIVER")
        if sdl_override:
            print(f"[DYNAMIC NAV] SDL_VIDEODRIVER={sdl_override!r}", flush=True)

        initial_view = AppView.SIMULATION if args.scenario_json else AppView.HOME
        app = DynamicNavApp(DEFAULT_CONFIG, initial_view=initial_view)
        if args.scenario_json:
            custom_scen = load_scenario_from_json(
                args.scenario_json,
                world_cfg=DEFAULT_CONFIG.world,
                robot_cfg=DEFAULT_CONFIG.robot,
            )
            app._world.load_scenario(custom_scen)
            app._start_m = custom_scen.robot_start_m
            app._goal_m = custom_scen.goal_m
            app.plan_path()
            app._evaluate_safety(record_history=False)
            print(f"[DYNAMIC NAV] Loaded custom JSON scenario: {custom_scen.name!r}", flush=True)
        driver = pygame.display.get_driver()
        size = app.screen.get_size()
        wm_info = pygame.display.get_wm_info()
        print(
            f"[DYNAMIC NAV] Window created: size={size}, driver={driver!r}, wm={wm_info}",
            flush=True,
        )
        if driver in ("dummy", "offscreen"):
            print(
                "[DYNAMIC NAV] WARNING: Running with a headless SDL video driver; "
                "no visible desktop window will appear.",
                file=sys.stderr,
                flush=True,
            )

        print("[DYNAMIC NAV] Entering event loop (press Esc or close window to exit).", flush=True)
        app.run()
        print("[DYNAMIC NAV] Event loop exited cleanly.", flush=True)
    except Exception:
        print("[DYNAMIC NAV] Fatal error during application execution:", file=sys.stderr, flush=True)
        traceback.print_exc()
        pygame.quit()
        sys.exit(1)


if __name__ == "__main__":
    main()
