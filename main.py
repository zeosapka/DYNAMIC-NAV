"""Entry point for the DYNAMIC NAV autonomous mobile robot simulator."""

import os
import sys
import traceback

import pygame

from config import DEFAULT_CONFIG
from interface.app import DynamicNavApp
from interface.views import AppView


def main() -> None:
    """Initialize and run the DYNAMIC NAV application with startup diagnostics."""
    try:
        print("[DYNAMIC NAV] Initializing simulator...", flush=True)
        sdl_override = os.environ.get("SDL_VIDEODRIVER")
        if sdl_override:
            print(f"[DYNAMIC NAV] SDL_VIDEODRIVER={sdl_override!r}", flush=True)

        app = DynamicNavApp(DEFAULT_CONFIG, initial_view=AppView.HOME)
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
