# AGENTS.md — Engineering Guidelines for DYNAMIC NAV

## Core Principles

1. **SI Units Only in Simulation & Algorithms**:
   - Lengths in meters (`_m`), time in seconds (`_s`), angles in radians (`_rad`), linear speeds in `m/s` (`_mps`), angular speeds in `rad/s` (`_radps`).
   - Never mix pixel units into `simulation/`, `algorithms/`, or `analysis/`.

2. **Strict Separation of Concerns**:
   - `simulation/` must never import `pygame`.
   - `interface/` handles Pygame windowing, input events, and drawing only.
   - `config.py` is the single source of truth for tunable parameters.

3. **No Hardcoded Magic Values**:
   - Add new parameters to the appropriate dataclass in `config.py`.

4. **Code Quality & Verification**:
   - Use full Python 3.10+ type hints and concise docstrings.
   - All physics and motion must be delta-time (`dt_s`) based.
   - Run `pytest -v` after every change and report only observed test results.
