"""DWA (Dynamic Window Approach) interface reservation for Phase 4.

Defines empty/typed telemetry structures so the dashboard architecture is ready
for Phase 4 without implementing DWA or displaying fake data.
"""

from dataclasses import dataclass, field


@dataclass(frozen=True)
class DWATelemetryPlaceholder:
    """Reserved telemetry container for future DWA trajectory evaluation."""

    active: bool = False
    candidate_trajectories: list[list[tuple[float, float]]] = field(
        default_factory=list
    )
    selected_trajectory: list[tuple[float, float]] | None = None
    rejected_trajectories: list[list[tuple[float, float]]] = field(
        default_factory=list
    )
    selected_v_mps: float | None = None
    selected_omega_radps: float | None = None
    evaluation_score: float | None = None
    predicted_min_clearance_m: float | None = None
    min_braking_clearance_m: float | None = None
    blocked: bool = False
    waiting_state: str = "GATE_CLEAR_HOLD_SAFE"
    waiting_min_clearance_m: float | None = None
    holding_pose_m: tuple[float, float] | None = None
    reverse_active: bool = False
    fallback_priority: int | None = None
    fallback_duration_s: float = 0.0
    deadlock_state: str = "NONE"
