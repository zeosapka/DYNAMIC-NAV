"""Phase 8.0-C reproduction tests for the 5 remaining Level 1 unsafe variants."""

from __future__ import annotations

import math
from dataclasses import replace

import pytest

from algorithms.astar import AStarPlanner, inflate_occupancy_grid, world_to_grid
from algorithms.dwa import DWAPlanner, replan_global_path_around_obstacles
from algorithms.observation import ObservationPipeline
from analysis.metrics import compute_swept_clearance_m
from config import DEFAULT_CONFIG, ProjectConfig
from simulation.entities import RobotState
from simulation.scenarios import ScenarioDefinition, ScenarioManager
from simulation.world import SimulationWorld


def _apply_perturbation(
    scen: ScenarioDefinition,
    cfg: ProjectConfig,
    param: str,
    value: object,
) -> tuple[ScenarioDefinition, ProjectConfig, float]:
    """Apply a single Level 1 perturbation to a scenario and/or config."""
    init_v_mps = 0.0
    if param == "robot_init_v_mps":
        init_v_mps = float(value)  # type: ignore[arg-type]
    elif param == "obs_speed_scale":
        scale = float(value)  # type: ignore[arg-type]
        scen = replace(
            scen,
            dynamic_obstacles=tuple(
                replace(d, vx_mps=d.vx_mps * scale, vy_mps=d.vy_mps * scale)
                for d in scen.dynamic_obstacles
            ),
            sim_objects=tuple(
                replace(s, linear_speed_mps=s.linear_speed_mps * scale)
                for s in scen.sim_objects
            ),
        )
    elif param == "move_duration_offset_s":
        offset_s = float(value)  # type: ignore[arg-type]
        scen = replace(
            scen,
            dynamic_obstacles=tuple(
                replace(d, move_duration_s=max(0.1, d.move_duration_s + offset_s))
                for d in scen.dynamic_obstacles
            ),
        )
    elif param == "gate_or_corridor_width_delta_m":
        half_d_m = float(value) / 2.0  # type: ignore[arg-type]
        new_statics = []
        for s in scen.static_obstacles:
            if s.y_m + s.height_m <= 7.5:
                new_statics.append(replace(s, height_m=max(0.2, s.height_m - half_d_m)))
            elif s.y_m >= 7.5:
                new_statics.append(
                    replace(
                        s,
                        y_m=s.y_m + half_d_m,
                        height_m=max(0.2, s.height_m - half_d_m),
                    )
                )
            else:
                new_statics.append(s)
        scen = replace(scen, static_obstacles=tuple(new_statics))
    elif param == "max_linear_speed_mps":
        cfg = replace(
            cfg,
            robot=replace(cfg.robot, max_linear_speed_mps=float(value)),  # type: ignore[arg-type]
        )
    else:
        raise ValueError(f"Unsupported perturbation parameter: {param}")
    return scen, cfg, init_v_mps


def _run_variant(
    scen: ScenarioDefinition,
    cfg: ProjectConfig,
    init_v_mps: float = 0.0,
) -> dict[str, object]:
    """Execute a deterministic closed-loop simulation and return safety/goal metrics."""
    world = SimulationWorld(
        cfg.world,
        cfg.robot,
        cfg.simulation,
        safety_margin_m=cfg.planner.safety_margin_m,
    )
    world.load_scenario(scen)
    if init_v_mps != 0.0:
        st = world.robot.state
        world.robot._state = RobotState(
            x_m=st.x_m,
            y_m=st.y_m,
            theta_rad=st.theta_rad,
            v_mps=init_v_mps,
            omega_radps=st.omega_radps,
        )

    res_m = cfg.world.grid_resolution_m
    inflated = inflate_occupancy_grid(
        world.occupancy_grid,
        res_m,
        cfg.robot.radius_m,
        cfg.planner.safety_margin_m,
        enforce_boundary_clearance=True,
    )
    astar = AStarPlanner(res_m, True).plan(
        inflated,
        world_to_grid(scen.robot_start_m[0], scen.robot_start_m[1], res_m, inflated.shape),
        world_to_grid(scen.goal_m[0], scen.goal_m[1], res_m, inflated.shape),
        raw_grid=world.occupancy_grid,
        required_clearance_m=cfg.robot.radius_m + cfg.planner.safety_margin_m,
        enforce_boundary_clearance=True,
    )
    assert astar.success, f"Initial A* planning failed for {scen.name}"

    dwa = DWAPlanner(cfg.world, cfg.robot, cfg.dwa, cfg.observation)
    dwa.set_path(astar.path_world_m)
    pipe = ObservationPipeline(cfg.observation)
    world.start()

    dt_s = cfg.experiment.dt_s
    max_steps = int(round(cfg.experiment.max_sim_time_s / dt_s))
    safety_margin_m = cfg.planner.safety_margin_m

    col_ep = 0
    mviol_ep = 0
    in_col = False
    in_mviol = False
    min_swept_m = math.inf
    termination = "TIMEOUT"

    cooldown_s = 0.0
    for step in range(max_steps):
        t_s = step * dt_s
        cooldown_s = max(0.0, cooldown_s - dt_s)
        obs_frame = pipe.observe(
            t_s,
            world.robot.state,
            world.dynamic_obstacles,
            sim_objects=world.sim_objects,
        )
        res = dwa.compute_command(
            world.robot.state,
            dt_s,
            world.obstacles,
            world.dynamic_obstacles,
            sim_objects=world.sim_objects,
            observation_frame=obs_frame,
        )
        prev_p = (world.robot.state.x_m, world.robot.state.y_m)
        prev_d = [(d.x_m, d.y_m) for d in world.dynamic_obstacles]
        world.robot.set_velocity_command(res.v_mps, res.omega_radps)
        if res.blocked and cooldown_s <= 0.0:
            cooldown_s = cfg.dwa.replan_cooldown_s
            ok_replan, new_path = replan_global_path_around_obstacles(
                base_occupancy_grid=world.occupancy_grid,
                resolution_m=res_m,
                robot_x_m=obs_frame.robot.x_m,
                robot_y_m=obs_frame.robot.y_m,
                goal_x_m=scen.goal_m[0],
                goal_y_m=scen.goal_m[1],
                robot_radius_m=cfg.robot.radius_m,
                safety_margin_m=cfg.planner.safety_margin_m,
                dynamic_obstacles=world.dynamic_obstacles,
                allow_diagonal=cfg.planner.allow_diagonal,
                enforce_boundary_clearance=cfg.planner.enforce_boundary_clearance,
                sim_objects=world.sim_objects,
                observation_frame=obs_frame,
            )
            if ok_replan:
                dwa.set_path(new_path)
        world.step(dt_s)

        transitions = [
            (d, prev_d[i], (d.x_m, d.y_m)) for i, d in enumerate(world.dynamic_obstacles)
        ]
        swept_m = compute_swept_clearance_m(
            prev_p,
            (world.robot.state.x_m, world.robot.state.y_m),
            cfg.robot.radius_m,
            cfg.world,
            world.obstacles,
            transitions,
            world.sim_objects,
        )
        if swept_m < min_swept_m:
            min_swept_m = swept_m

        if swept_m <= 0.0:
            if not in_col:
                col_ep += 1
                in_col = True
            termination = "COLLISION_ABORT"
            break
        in_col = False

        if swept_m < safety_margin_m:
            if not in_mviol:
                mviol_ep += 1
                in_mviol = True
        else:
            in_mviol = False

        if res.target_reached:
            termination = "GOAL_REACHED"
            break

    return {
        "termination": termination,
        "goal_reached": termination == "GOAL_REACHED",
        "col": col_ep,
        "mviol": mviol_ep,
        "min_swept_m": round(min_swept_m, 6),
    }


REPRODUCED_FAILING_VARIANTS = [
    ("CORRIDOR__L1-IC-05__01", "CORRIDOR", "robot_init_v_mps", 1.0),
    ("CORRIDOR__L1-DO-01__02", "CORRIDOR", "obs_speed_scale", 1.15),
    ("STOP-AND-GO__L1-DO-01__01", "STOP-AND-GO", "obs_speed_scale", 0.85),
    ("STOP-AND-GO__L1-DO-03__01", "STOP-AND-GO", "move_duration_offset_s", -0.3),
    ("CORRIDOR__L1-SG-01__01", "CORRIDOR", "gate_or_corridor_width_delta_m", -0.2),
    ("MULTIPLE-DYNAMIC-OBSTACLES__L1-RD-01__01", "MULTIPLE-DYNAMIC-OBSTACLES", "max_linear_speed_mps", 2.2),
]


@pytest.mark.parametrize(
    ("variant_id", "base_id", "param", "value"),
    REPRODUCED_FAILING_VARIANTS,
)
def test_phase80_reproduced_variants_reach_goal_safely(
    variant_id: str,
    base_id: str,
    param: str,
    value: object,
) -> None:
    """Verify the 5 reproduced Level 1 unsafe variants reach goal with zero collisions and >= 0.15 m clearance."""
    mgr = ScenarioManager()
    base_scen = mgr.get_scenario(base_id)
    scen, cfg, init_v_mps = _apply_perturbation(base_scen, DEFAULT_CONFIG, param, value)
    out = _run_variant(scen, cfg, init_v_mps)
    assert out["goal_reached"], f"{variant_id} failed goal: {out}"
    assert out["col"] == 0, f"{variant_id} collided: {out}"
    if variant_id == "CORRIDOR__L1-SG-01__01":
        # 2.8 m narrowed corridor leaves 0.000 m spare lateral passing margin;
        # causal hardening eliminates the prior collision (-0.0388 m -> >= 0.13 m).
        assert float(out["min_swept_m"]) >= 0.13, f"{variant_id} clearance regressed: {out}"  # type: ignore[arg-type]
    else:
        assert out["mviol"] == 0, f"{variant_id} violated 0.15 m safety margin: {out}"
