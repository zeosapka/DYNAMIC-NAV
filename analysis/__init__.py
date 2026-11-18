"""Telemetry logging, metrics computation, and benchmark suite for DYNAMIC NAV."""

from analysis.benchmark import (
    CONTROLLER_DWA,
    CONTROLLER_FOLLOWER,
    export_results_csv,
    export_results_json,
    run_benchmark_suite,
    run_single_experiment,
)
from analysis.metrics import (
    RunMetrics,
    RunMetricsTracker,
    compute_instantaneous_clearance_m,
    compute_swept_clearance_m,
)

__all__ = [
    "CONTROLLER_DWA",
    "CONTROLLER_FOLLOWER",
    "RunMetrics",
    "RunMetricsTracker",
    "compute_instantaneous_clearance_m",
    "compute_swept_clearance_m",
    "export_results_csv",
    "export_results_json",
    "run_benchmark_suite",
    "run_single_experiment",
]
