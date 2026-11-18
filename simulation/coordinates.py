"""Coordinate system transformations between SI world space and screen pixels.

World space uses right-handed Cartesian coordinates in meters:
- Origin (0.0, 0.0) at the bottom-left corner of the simulation world.
- +X points right (east), +Y points up (north).

Screen space uses Pygame pixel coordinates:
- Origin (0, 0) at the top-left corner of the window.
- +px points right, +py points down.
"""

from dataclasses import dataclass


@dataclass(frozen=True)
class ScreenRect:
    """Pixel-space rectangle defining the available simulation viewport."""

    left: int
    top: int
    width: int
    height: int


class CoordinateTransform:
    """Aspect-preserving affine mapping between world meters and screen pixels."""

    def __init__(
        self,
        world_width_m: float,
        world_height_m: float,
        viewport: ScreenRect,
    ) -> None:
        if world_width_m <= 0.0 or world_height_m <= 0.0:
            raise ValueError("World dimensions must be strictly positive.")
        if viewport.width <= 0 or viewport.height <= 0:
            raise ValueError("Viewport dimensions must be strictly positive.")

        self._world_width_m = world_width_m
        self._world_height_m = world_height_m
        self._viewport = viewport

        scale_x = viewport.width / world_width_m
        scale_y = viewport.height / world_height_m
        self._pixels_per_meter = min(scale_x, scale_y)

        rendered_width_px = world_width_m * self._pixels_per_meter
        rendered_height_px = world_height_m * self._pixels_per_meter

        # Center the world map inside the viewport rectangle.
        self._origin_px_x = viewport.left + (viewport.width - rendered_width_px) * 0.5
        self._origin_px_y = (
            viewport.top + (viewport.height - rendered_height_px) * 0.5 + rendered_height_px
        )

    @property
    def pixels_per_meter(self) -> float:
        """Uniform scale factor from meters to pixels."""
        return self._pixels_per_meter

    @property
    def world_screen_bounds(self) -> tuple[int, int, int, int]:
        """Return (left, top, width, height) in pixels of the rendered world area."""
        width_px = round(self._world_width_m * self._pixels_per_meter)
        height_px = round(self._world_height_m * self._pixels_per_meter)
        left_px = round(self._origin_px_x)
        top_px = round(self._origin_px_y - height_px)
        return (left_px, top_px, width_px, height_px)

    def meters_to_pixels(self, length_m: float) -> float:
        """Convert a scalar length in meters to pixels."""
        return length_m * self._pixels_per_meter

    def pixels_to_meters(self, length_px: float) -> float:
        """Convert a scalar length in pixels to meters."""
        return length_px / self._pixels_per_meter

    def world_to_screen(self, x_m: float, y_m: float) -> tuple[int, int]:
        """Map world coordinates (meters) to integer screen pixel coordinates."""
        px = round(self._origin_px_x + x_m * self._pixels_per_meter)
        py = round(self._origin_px_y - y_m * self._pixels_per_meter)
        return (px, py)

    def screen_to_world(self, px: int, py: int) -> tuple[float, float]:
        """Map screen pixel coordinates back to world coordinates (meters)."""
        x_m = (px - self._origin_px_x) / self._pixels_per_meter
        y_m = (self._origin_px_y - py) / self._pixels_per_meter
        return (x_m, y_m)
