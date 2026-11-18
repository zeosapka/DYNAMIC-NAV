"""UI button widgets and keyboard input mapping for manual robot control."""

from collections.abc import Callable, Sequence
import pygame

from config import RobotConfig, ThemeConfig


class Button:
    """Interactive rectangular button for the engineering control sidebar."""

    def __init__(
        self,
        rect: pygame.Rect,
        label: str,
        on_click: Callable[[], None],
        enabled: bool = True,
        tooltip: str = "",
    ) -> None:
        self.rect = rect
        self.label = label
        self.on_click = on_click
        self.enabled = enabled
        self.tooltip = tooltip
        self.hovered = False

    def update_hover(self, mouse_pos: tuple[int, int]) -> None:
        """Update hover state based on current cursor position."""
        self.hovered = self.enabled and self.rect.collidepoint(mouse_pos)

    def handle_click(self, mouse_pos: tuple[int, int]) -> bool:
        """Invoke callback if enabled and clicked; return True when triggered."""
        if self.enabled and self.rect.collidepoint(mouse_pos):
            self.on_click()
            return True
        return False

    def draw(
        self,
        surface: pygame.Surface,
        font: pygame.font.Font,
        theme: ThemeConfig,
        active: bool = False,
    ) -> None:
        """Render the button onto the target Pygame surface."""
        if not self.enabled:
            bg_color = theme.button_disabled
            text_color = theme.button_text_disabled
        elif active:
            bg_color = theme.button_active
            text_color = theme.text_primary
        elif self.hovered:
            bg_color = theme.button_hover
            text_color = theme.text_primary
        else:
            bg_color = theme.button_bg
            text_color = theme.text_primary

        pygame.draw.rect(surface, bg_color, self.rect, border_radius=6)
        border_color = theme.accent if active else theme.border
        pygame.draw.rect(surface, border_color, self.rect, width=1, border_radius=6)

        text_surf = font.render(self.label, True, text_color)
        text_rect = text_surf.get_rect(center=self.rect.center)
        surface.blit(text_surf, text_rect)


class NumericField:
    """Interactive numeric text input box for the ADAS object inspector."""

    def __init__(
        self,
        key: str,
        label: str,
        rect: pygame.Rect,
        value_str: str = "",
    ) -> None:
        self.key = key
        self.label = label
        self.rect = rect
        self.text = value_str
        self.focused = False
        self.invalid = False

    def handle_click(self, mouse_pos: tuple[int, int]) -> bool:
        was_focused = self.focused
        self.focused = self.rect.collidepoint(mouse_pos)
        return self.focused or was_focused

    def handle_key(self, event: pygame.event.Event) -> tuple[bool, float | None]:
        """Process KEYDOWN when focused; returns (consumed, committed_float_or_None)."""
        if not self.focused:
            return (False, None)
        if event.key == pygame.K_RETURN or event.key == pygame.K_KP_ENTER:
            try:
                val = float(self.text.replace(",", "."))
                self.invalid = False
                self.focused = False
                return (True, val)
            except ValueError:
                self.invalid = True
                return (True, None)
        if event.key == pygame.K_ESCAPE:
            self.focused = False
            self.invalid = False
            return (True, None)
        if event.key == pygame.K_BACKSPACE:
            self.text = self.text[:-1]
            self.invalid = False
            return (True, None)
        ch = event.unicode
        if ch in "0123456789.-+,":
            self.text += ch
            self.invalid = False
            return (True, None)
        return (True, None)

    def draw(
        self,
        surface: pygame.Surface,
        font: pygame.font.Font,
        theme: ThemeConfig,
    ) -> None:
        bg = (36, 49, 68) if self.focused else theme.adas_card_bg
        border = theme.safety_collision if self.invalid else (theme.accent if self.focused else theme.adas_card_border)
        pygame.draw.rect(surface, bg, self.rect, border_radius=4)
        pygame.draw.rect(surface, border, self.rect, width=1, border_radius=4)
        lbl_surf = font.render(self.label, True, theme.text_secondary)
        surface.blit(lbl_surf, (self.rect.x + 6, self.rect.y + 4))
        val_surf = font.render(self.text, True, theme.text_primary)
        surface.blit(val_surf, (self.rect.right - val_surf.get_width() - 6, self.rect.y + 4))


class KeyboardController:
    """Maps Pygame keyboard state to SI linear and angular velocity commands."""

    def __init__(self, robot_config: RobotConfig) -> None:
        self._robot_config = robot_config

    def compute_velocity_command(
        self, pressed_keys: Sequence[bool]
    ) -> tuple[float, float]:
        """Compute (v_mps, omega_radps) from currently held keyboard keys."""
        linear_axis = 0.0
        if pressed_keys[pygame.K_w] or pressed_keys[pygame.K_UP]:
            linear_axis += 1.0
        if pressed_keys[pygame.K_s] or pressed_keys[pygame.K_DOWN]:
            linear_axis -= 1.0

        angular_axis = 0.0
        if pressed_keys[pygame.K_a] or pressed_keys[pygame.K_LEFT]:
            angular_axis += 1.0
        if pressed_keys[pygame.K_d] or pressed_keys[pygame.K_RIGHT]:
            angular_axis -= 1.0

        v_mps = linear_axis * self._robot_config.max_linear_speed_mps
        omega_radps = angular_axis * self._robot_config.max_angular_speed_radps
        return (v_mps, omega_radps)
