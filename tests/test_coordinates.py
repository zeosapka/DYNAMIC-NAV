"""Unit tests for SI world-to-screen coordinate transformations."""

import pytest

from simulation.coordinates import CoordinateTransform, ScreenRect


def test_world_screen_round_trip() -> None:
    viewport = ScreenRect(left=28, top=28, width=1064, height=844)
    tf = CoordinateTransform(world_width_m=20.0, world_height_m=15.0, viewport=viewport)

    test_points = [(0.0, 0.0), (10.0, 7.5), (20.0, 15.0), (3.25, 11.8)]
    for x_m, y_m in test_points:
        px, py = tf.world_to_screen(x_m, y_m)
        rx_m, ry_m = tf.screen_to_world(px, py)
        # Tolerance accounts for integer pixel rounding.
        tol_m = tf.pixels_to_meters(1.0)
        assert rx_m == pytest.approx(x_m, abs=tol_m)
        assert ry_m == pytest.approx(y_m, abs=tol_m)


def test_y_axis_is_inverted_on_screen() -> None:
    viewport = ScreenRect(left=0, top=0, width=1000, height=800)
    tf = CoordinateTransform(world_width_m=20.0, world_height_m=15.0, viewport=viewport)

    _, py_low = tf.world_to_screen(5.0, 2.0)
    _, py_high = tf.world_to_screen(5.0, 10.0)
    assert py_high < py_low


def test_aspect_ratio_preserved_and_positive_dimensions_enforced() -> None:
    viewport = ScreenRect(left=0, top=0, width=1200, height=600)
    tf = CoordinateTransform(world_width_m=20.0, world_height_m=15.0, viewport=viewport)

    _, _, w_px, h_px = tf.world_screen_bounds
    assert w_px / h_px == pytest.approx(20.0 / 15.0, rel=1e-2)

    with pytest.raises(ValueError):
        CoordinateTransform(world_width_m=0.0, world_height_m=10.0, viewport=viewport)
    with pytest.raises(ValueError):
        CoordinateTransform(
            world_width_m=10.0,
            world_height_m=10.0,
            viewport=ScreenRect(0, 0, 0, 100),
        )
