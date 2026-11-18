"""Automated tests for Dynamic Window Approach (DWA) local obstacle avoidance."""

import math
import pytest

from algorithms.astar import AStarPlanner, inflate_occupancy_grid, world_to_grid
from algorithms.dwa import (
    DWAPlanner,
    compute_dynamic_window,
    compute_stopping_distance_m,
    evaluate_trajectory_clearance,
    evaluate_trajectory_with_braking,
    predict_trajectory,
    replan_global_path_around_obstacles,
    sample_velocities,
    simulate_braking_trajectory,
)
from config import DEFAULT_CONFIG
from interface.app import AppView, DynamicNavApp
from simulation.dynamic_obstacles import DynamicObstacle, MovementBehavior
from simulation.entities import Robot, RobotState, StaticObstacle
from simulation.scenarios import ScenarioManager
from simulation.world import SimulationWorld


def test_dynamic_window_respects_acceleration_and_global_bounds() -> None:
    win = compute_dynamic_window(
        v_current_mps=1.0,
        omega_current_radps=0.5,
        dt_s=0.05,
        robot_cfg=DEFAULT_CONFIG.robot,
        dwa_cfg=DEFAULT_CONFIG.dwa,
    )
    assert win.v_min_mps == pytest.approx(
        1.0 - DEFAULT_CONFIG.dwa.max_linear_accel_mps2 * 0.05, abs=1e-9
    )
    assert win.v_max_mps == pytest.approx(
        1.0 + DEFAULT_CONFIG.dwa.max_linear_accel_mps2 * 0.05, abs=1e-9
    )
    assert win.omega_min_radps == pytest.approx(
        0.5 - DEFAULT_CONFIG.dwa.max_angular_accel_radps2 * 0.05, abs=1e-9
    )
    assert win.omega_max_radps == pytest.approx(
        0.5 + DEFAULT_CONFIG.dwa.max_angular_accel_radps2 * 0.05, abs=1e-9
    )

    # Clamped at global bounds
    win_top = compute_dynamic_window(
        v_current_mps=DEFAULT_CONFIG.robot.max_linear_speed_mps,
        omega_current_radps=DEFAULT_CONFIG.robot.max_angular_speed_radps,
        dt_s=0.1,
        robot_cfg=DEFAULT_CONFIG.robot,
        dwa_cfg=DEFAULT_CONFIG.dwa,
    )
    assert win_top.v_max_mps <= DEFAULT_CONFIG.robot.max_linear_speed_mps
    assert win_top.omega_max_radps <= DEFAULT_CONFIG.robot.max_angular_speed_radps


def test_velocity_sampling_includes_zero_omega() -> None:
    win = compute_dynamic_window(0.5, 0.0, 0.05, DEFAULT_CONFIG.robot, DEFAULT_CONFIG.dwa)
    samples = sample_velocities(win, 5, 10)
    assert len(samples) >= 50
    assert any(abs(w) < 1e-9 for _, w in samples)


def test_trajectory_prediction_kinematics() -> None:
    state = RobotState(x_m=2.0, y_m=3.0, theta_rad=0.0)
    traj = predict_trajectory(
        state, v_mps=1.0, omega_radps=0.0, predict_time_s=1.0, predict_dt_s=0.1
    )
    assert len(traj) == 11
    assert traj[-1][0] == pytest.approx(3.0, abs=1e-6)
    assert traj[-1][1] == pytest.approx(3.0, abs=1e-6)


def test_trajectory_rejects_unsafe_dynamic_obstacle_collision() -> None:
    state = RobotState(x_m=2.0, y_m=7.5, theta_rad=0.0)
    traj = predict_trajectory(state, v_mps=2.0, omega_radps=0.0, predict_time_s=1.5, predict_dt_s=0.1)
    obs = DynamicObstacle(
        obstacle_id="HEAD",
        x_m=4.5,
        y_m=7.5,
        vx_mps=-1.0,
        vy_mps=0.0,
        radius_m=0.45,
        behavior=MovementBehavior.HEAD_ON,
    )
    is_safe, min_c = evaluate_trajectory_clearance(
        traj,
        predict_dt_s=0.1,
        robot_radius_m=DEFAULT_CONFIG.robot.radius_m,
        safety_margin_m=DEFAULT_CONFIG.dwa.safety_margin_m,
        world_cfg=DEFAULT_CONFIG.world,
        static_obstacles=[],
        dynamic_obstacles=[obs],
    )
    assert is_safe is False
    assert min_c < DEFAULT_CONFIG.dwa.safety_margin_m


def test_dwa_reaches_goal_in_default_world_without_collision() -> None:
    world = SimulationWorld(
        DEFAULT_CONFIG.world, DEFAULT_CONFIG.robot, DEFAULT_CONFIG.simulation
    )
    res_m = DEFAULT_CONFIG.world.grid_resolution_m
    inflated = inflate_occupancy_grid(
        world.occupancy_grid,
        res_m,
        DEFAULT_CONFIG.robot.radius_m,
        DEFAULT_CONFIG.planner.safety_margin_m,
        enforce_boundary_clearance=True,
    )
    start_c = world_to_grid(2.0, 2.0, res_m, inflated.shape)
    goal_c = world_to_grid(18.0, 13.0, res_m, inflated.shape)
    astar = AStarPlanner(res_m, True).plan(
        inflated, start_c, goal_c, raw_grid=world.occupancy_grid, required_clearance_m=0.5, enforce_boundary_clearance=True
    )
    assert astar.success

    dwa = DWAPlanner(DEFAULT_CONFIG.world, DEFAULT_CONFIG.robot, DEFAULT_CONFIG.dwa)
    dwa.set_path(astar.path_world_m)

    world.start()
    for _ in range(400):
        res = dwa.compute_command(
            world.robot.state, 0.05, world.obstacles, world.dynamic_obstacles
        )
        if res.target_reached:
            break
        world.robot.set_velocity_command(res.v_mps, res.omega_radps)
        world.step(0.05)

    assert dwa.target_reached is True
    assert math.hypot(world.robot.state.x_m - 18.0, world.robot.state.y_m - 13.0) <= 0.35


@pytest.mark.parametrize("scenario_name", ["HEAD-ON", "CROSSING", "STOP-AND-GO"])
def test_dwa_avoids_dynamic_obstacles_in_scenarios(scenario_name: str) -> None:
    world = SimulationWorld(
        DEFAULT_CONFIG.world,
        DEFAULT_CONFIG.robot,
        DEFAULT_CONFIG.simulation,
        safety_margin_m=DEFAULT_CONFIG.planner.safety_margin_m,
    )
    scen = ScenarioManager.get_scenario(scenario_name)
    world.load_scenario(scen)

    res_m = DEFAULT_CONFIG.world.grid_resolution_m
    inflated = inflate_occupancy_grid(
        world.occupancy_grid,
        res_m,
        DEFAULT_CONFIG.robot.radius_m,
        DEFAULT_CONFIG.planner.safety_margin_m,
        enforce_boundary_clearance=True,
    )
    start_c = world_to_grid(scen.robot_start_m[0], scen.robot_start_m[1], res_m, inflated.shape)
    goal_c = world_to_grid(scen.goal_m[0], scen.goal_m[1], res_m, inflated.shape)
    astar = AStarPlanner(res_m, True).plan(
        inflated, start_c, goal_c, raw_grid=world.occupancy_grid, required_clearance_m=0.5, enforce_boundary_clearance=True
    )
    assert astar.success

    dwa = DWAPlanner(DEFAULT_CONFIG.world, DEFAULT_CONFIG.robot, DEFAULT_CONFIG.dwa)
    dwa.set_path(astar.path_world_m)

    world.start()
    for _ in range(450):
        res = dwa.compute_command(
            world.robot.state, 0.05, world.obstacles, world.dynamic_obstacles
        )
        if res.target_reached:
            break
        world.robot.set_velocity_command(res.v_mps, res.omega_radps)
        world.step(0.05)
        assert world.has_physical_collision is False

    assert dwa.target_reached is True


def test_dwa_fallback_all_candidates_physically_colliding() -> None:
    dwa = DWAPlanner(DEFAULT_CONFIG.world, DEFAULT_CONFIG.robot, DEFAULT_CONFIG.dwa)
    dwa.set_path([(2.0, 7.5), (10.0, 7.5)])
    # Robot boxed against left boundary and an obstacle immediately in front (overlap/collision)
    state = RobotState(x_m=0.40, y_m=7.5, theta_rad=0.0, v_mps=0.2, omega_radps=0.0)
    blocking = StaticObstacle(x_m=0.70, y_m=6.5, width_m=1.0, height_m=2.0)

    res = dwa.compute_command(state, 0.05, [blocking], [])
    assert res.fallback_active is True
    assert res.safety_margin_satisfied is False
    assert res.collision_free_exists is False
    assert res.fallback_reason == "PHYSICAL_COLLISION_RISK"
    assert res.v_mps == pytest.approx(0.0, abs=1e-9)
    assert "PHYSICAL_COLLISION_RISK" in dwa.safety_events
    assert dwa.collision_risk_fallback_count == 1
    assert dwa.has_path is True


def test_dwa_fallback_all_candidates_violate_safety_margin_but_collision_free_exists() -> None:
    dwa = DWAPlanner(DEFAULT_CONFIG.world, DEFAULT_CONFIG.robot, DEFAULT_CONFIG.dwa)
    dwa.set_path([(2.0, 7.5), (10.0, 7.5)])
    # Robot at x=0.43 has surface clearance 0.43 - 0.35 = 0.08m from x=0 boundary (0 < 0.08 < 0.15m)
    state = RobotState(x_m=0.43, y_m=7.5, theta_rad=0.0, v_mps=0.0, omega_radps=0.0)
    # Wall at x=0.86 gives front clearance 0.86 - 0.43 - 0.35 = 0.08m
    front_wall = StaticObstacle(x_m=0.86, y_m=6.5, width_m=1.0, height_m=2.0)

    res = dwa.compute_command(state, 0.05, [front_wall], [])
    assert res.fallback_active is True
    assert res.safety_margin_satisfied is False
    assert res.collision_free_exists is True
    assert res.fallback_reason == "SAFETY_MARGIN_VIOLATION"
    assert res.predicted_min_clearance_m is not None
    assert res.predicted_min_clearance_m > 0.0
    assert "SAFETY_MARGIN_VIOLATION" in dwa.safety_events
    assert dwa.safety_margin_fallback_count == 1
    assert dwa.has_path is True


def test_dwa_emergency_braking_respects_acceleration_limits() -> None:
    dwa = DWAPlanner(DEFAULT_CONFIG.world, DEFAULT_CONFIG.robot, DEFAULT_CONFIG.dwa)
    dwa.set_path([(5.0, 7.5), (15.0, 7.5)])
    # High-speed robot 0.20m from a wide wall -> all candidates collide, must brake by max_linear_accel * dt
    state = RobotState(x_m=5.0, y_m=7.5, theta_rad=0.0, v_mps=2.5, omega_radps=0.0)
    wall = StaticObstacle(x_m=5.55, y_m=4.0, width_m=2.0, height_m=7.0)

    dt_s = 0.05
    expected_v_min = 2.5 - DEFAULT_CONFIG.dwa.max_linear_accel_mps2 * dt_s
    res = dwa.compute_command(state, dt_s, [wall], [])
    assert res.fallback_active is True
    assert res.fallback_reason == "PHYSICAL_COLLISION_RISK"
    assert res.v_mps == pytest.approx(expected_v_min, abs=1e-9)


def test_dwa_corridor_scenario_deterministic_audit() -> None:
    world = SimulationWorld(
        DEFAULT_CONFIG.world,
        DEFAULT_CONFIG.robot,
        DEFAULT_CONFIG.simulation,
        safety_margin_m=DEFAULT_CONFIG.planner.safety_margin_m,
    )
    scen = ScenarioManager.get_scenario("CORRIDOR")
    world.load_scenario(scen)
    res_m = DEFAULT_CONFIG.world.grid_resolution_m
    inflated = inflate_occupancy_grid(
        world.occupancy_grid,
        res_m,
        DEFAULT_CONFIG.robot.radius_m,
        DEFAULT_CONFIG.planner.safety_margin_m,
        enforce_boundary_clearance=True,
    )
    astar = AStarPlanner(res_m, True).plan(
        inflated,
        world_to_grid(scen.robot_start_m[0], scen.robot_start_m[1], res_m, inflated.shape),
        world_to_grid(scen.goal_m[0], scen.goal_m[1], res_m, inflated.shape),
        raw_grid=world.occupancy_grid,
        required_clearance_m=0.5,
        enforce_boundary_clearance=True,
    )
    assert astar.success

    dwa = DWAPlanner(DEFAULT_CONFIG.world, DEFAULT_CONFIG.robot, DEFAULT_CONFIG.dwa)
    dwa.set_path(astar.path_world_m)
    world.start()
    waited = False
    for _ in range(600):
        res = dwa.compute_command(
            world.robot.state, 0.05, world.obstacles, world.dynamic_obstacles
        )
        assert abs(res.v_mps) <= DEFAULT_CONFIG.robot.max_linear_speed_mps + 1e-9
        assert abs(res.omega_radps) <= DEFAULT_CONFIG.robot.max_angular_speed_radps + 1e-9
        if res.waiting_for_passage:
            waited = True
        if res.target_reached:
            break
        world.robot.set_velocity_command(res.v_mps, res.omega_radps)
        world.step(0.05)
        assert world.has_physical_collision is False

    assert dwa.has_path is True
    assert waited is True
    assert dwa.target_reached is True


def test_dwa_no_goal_reached_when_blocked_short_of_goal() -> None:
    dwa = DWAPlanner(DEFAULT_CONFIG.world, DEFAULT_CONFIG.robot, DEFAULT_CONFIG.dwa)
    dwa.set_path([(2.0, 7.5), (18.0, 7.5)])
    world = SimulationWorld(
        DEFAULT_CONFIG.world, DEFAULT_CONFIG.robot, DEFAULT_CONFIG.simulation
    )
    world.robot.state.x_m = 2.0
    world.robot.state.y_m = 7.5
    world.robot.state.theta_rad = 0.0
    # Full vertical barrier at x=5.0
    barrier = StaticObstacle(x_m=5.0, y_m=0.0, width_m=1.0, height_m=15.0)
    world.add_static_obstacle(barrier)
    world.start()

    for _ in range(60):
        res = dwa.compute_command(world.robot.state, 0.05, world.obstacles, [])
        world.robot.set_velocity_command(res.v_mps, res.omega_radps)
        world.step(0.05)
        assert res.target_reached is False

    assert dwa.target_reached is False


def test_dwa_reset_and_deterministic_repeatability() -> None:
    def _run_once() -> tuple[tuple[float, float, float], list[tuple[float, float]]]:
        world = SimulationWorld(
            DEFAULT_CONFIG.world, DEFAULT_CONFIG.robot, DEFAULT_CONFIG.simulation
        )
        scen = ScenarioManager.get_scenario("STOP-AND-GO")
        world.load_scenario(scen)
        dwa = DWAPlanner(DEFAULT_CONFIG.world, DEFAULT_CONFIG.robot, DEFAULT_CONFIG.dwa)
        dwa.set_path([scen.robot_start_m, scen.goal_m])
        world.start()
        cmds: list[tuple[float, float]] = []
        for _ in range(40):
            res = dwa.compute_command(
                world.robot.state, 0.05, world.obstacles, world.dynamic_obstacles
            )
            cmds.append((res.v_mps, res.omega_radps))
            world.robot.set_velocity_command(res.v_mps, res.omega_radps)
            world.step(0.05)
        dwa.reset()
        assert dwa.has_path is True
        assert dwa.target_reached is False
        assert dwa.fallback_count == 0
        st = world.robot.state
        return (st.x_m, st.y_m, st.theta_rad), cmds

    pose1, cmds1 = _run_once()
    pose2, cmds2 = _run_once()
    assert pose1 == pytest.approx(pose2, abs=1e-12)
    assert cmds1 == cmds2


def test_braking_distance_at_multiple_speeds() -> None:
    a_brake = 5.0
    t_react = 0.1
    for v in [0.0, 1.0, 2.0, 3.0]:
        expected = v * t_react + (v * v) / (2.0 * a_brake)
        assert compute_stopping_distance_m(v, a_brake, t_react) == pytest.approx(
            expected, abs=1e-9
        )


def test_dwa_rejects_trajectory_when_cannot_stop_within_horizon() -> None:
    # Robot at x=2.0 moving v=2.0 m/s for 1.0s -> ends at x=4.0 (0.65m clear of wall at x=5.0),
    # BUT stopping distance at v=2.0 m/s with a_brake=5.0, t_react=0.1 is 0.60m ->
    # stop pose x=4.60 leaves only 4.60 + 0.35 = 4.95 -> 0.05m < 0.15m safety margin!
    state = RobotState(x_m=2.0, y_m=7.5, theta_rad=0.0)
    traj = predict_trajectory(state, v_mps=2.0, omega_radps=0.0, predict_time_s=1.0, predict_dt_s=0.1)
    wall = StaticObstacle(x_m=5.0, y_m=5.0, width_m=1.0, height_m=5.0)

    safe_no_brake, _ = evaluate_trajectory_clearance(
        traj, 0.1, 0.35, 0.15, DEFAULT_CONFIG.world, [wall], []
    )
    safe_with_brake, _ = evaluate_trajectory_clearance(
        traj,
        0.1,
        0.35,
        0.15,
        DEFAULT_CONFIG.world,
        [wall],
        [],
        v_mps=2.0,
        max_brake_decel_mps2=5.0,
        reaction_time_s=0.1,
    )
    assert safe_no_brake is True
    assert safe_with_brake is False


def test_dynamic_obstacle_predict_positions_reverses_at_patrol_boundary() -> None:
    obs = DynamicObstacle(
        obstacle_id="PATROL",
        x_m=6.0,
        y_m=7.5,
        vx_mps=-1.0,
        vy_mps=0.0,
        radius_m=0.40,
        behavior=MovementBehavior.CORRIDOR,
        waypoint_a_m=(5.6, 7.5),
        waypoint_b_m=(14.4, 7.5),
        reverse_at_bounds=True,
    )
    preds = obs.predict_positions(0.1, 15)
    # Original obstacle must remain unmutated
    assert obs.x_m == pytest.approx(6.0, abs=1e-9)
    assert obs.vx_mps == pytest.approx(-1.0, abs=1e-9)
    # Verify exact match with actual step() including reversal at x=5.6
    for i in range(15):
        obs.step(0.1)
        assert (obs.x_m, obs.y_m) == pytest.approx(preds[i + 1], abs=1e-9)
    assert obs.vx_mps > 0.0


def test_dwa_impassable_corridor_stops_safely_and_reports_blocked() -> None:
    world = SimulationWorld(
        DEFAULT_CONFIG.world,
        DEFAULT_CONFIG.robot,
        DEFAULT_CONFIG.simulation,
        safety_margin_m=DEFAULT_CONFIG.planner.safety_margin_m,
    )
    # Narrow corridor with a permanent static plug at x=7.0
    world.add_static_obstacle(StaticObstacle(x_m=5.0, y_m=4.0, width_m=10.0, height_m=2.0))
    world.add_static_obstacle(StaticObstacle(x_m=5.0, y_m=9.0, width_m=10.0, height_m=2.0))
    world.add_static_obstacle(StaticObstacle(x_m=7.0, y_m=6.0, width_m=1.0, height_m=3.0))
    world.robot.state.x_m = 5.5
    world.robot.state.y_m = 7.5
    world.robot.state.theta_rad = 0.0

    dwa = DWAPlanner(DEFAULT_CONFIG.world, DEFAULT_CONFIG.robot, DEFAULT_CONFIG.dwa)
    # Force an infeasible straight reference path through the plug
    dwa.set_path([(5.5, 7.5), (14.0, 7.5)])
    world.start()

    last_res = None
    for _ in range(100):
        last_res = dwa.compute_command(world.robot.state, 0.05, world.obstacles, [])
        world.robot.set_velocity_command(last_res.v_mps, last_res.omega_radps)
        world.step(0.05)
        assert world.has_physical_collision is False

    assert last_res is not None
    assert dwa.target_reached is False
    assert dwa.blocked is True
    assert last_res.status == "BLOCKED"


def test_dwa_prediction_vs_actual_trajectory_consistency() -> None:
    robot = Robot(DEFAULT_CONFIG.robot)
    robot.state.x_m = 4.0
    robot.state.y_m = 5.0
    robot.state.theta_rad = 0.25
    v, omega, dt_s, steps = 1.2, 0.45, 0.1, 15
    predicted = predict_trajectory(robot.state, v, omega, steps * dt_s, dt_s)
    robot.set_velocity_command(v, omega)
    for idx in range(steps):
        robot.step(dt_s, DEFAULT_CONFIG.world.width_m, DEFAULT_CONFIG.world.height_m, [])
        px, py, pth = predicted[idx + 1]
        assert robot.state.x_m == pytest.approx(px, abs=1e-9)
        assert robot.state.y_m == pytest.approx(py, abs=1e-9)
        assert robot.state.theta_rad == pytest.approx(pth, abs=1e-9)


def test_braking_trajectory_complete_collision_detection() -> None:
    poses, times = simulate_braking_trajectory(
        (2.0, 7.5, 0.0), v_mps=2.0, predict_dt_s=0.1, max_brake_decel_mps2=5.0, reaction_time_s=0.1
    )
    assert len(poses) >= 5
    assert times[-1] == pytest.approx(0.5, abs=1e-9)
    # Distance = 2.0*0.1 + 2.0^2/(2*5.0) = 0.60 m
    assert poses[-1][0] == pytest.approx(2.6, abs=1e-9)


def test_dynamic_obstacle_crossing_during_robot_braking() -> None:
    # Trajectory is safe up to t=1.0s (x=2.0 -> x=4.0), then during braking (t=1.0..1.5s, x=4.0..4.6)
    # a crossing obstacle sweeps directly across (4.5, 7.5) at t=1.33s.
    state = RobotState(x_m=2.0, y_m=7.5, theta_rad=0.0)
    traj = predict_trajectory(state, v_mps=2.0, omega_radps=0.0, predict_time_s=1.0, predict_dt_s=0.1)
    crossing_obs = DynamicObstacle(
        obstacle_id="BRAKE-CROSS",
        x_m=4.5,
        y_m=3.5,
        vx_mps=0.0,
        vy_mps=3.0,
        radius_m=0.40,
        behavior=MovementBehavior.CROSSING,
    )
    is_safe, _, min_brake_c, min_brake_t = evaluate_trajectory_with_braking(
        traj,
        0.1,
        0.35,
        0.15,
        DEFAULT_CONFIG.world,
        [],
        [crossing_obs],
        v_mps=2.0,
        max_brake_decel_mps2=5.0,
        reaction_time_s=0.1,
    )
    assert is_safe is False
    assert min_brake_c is not None and min_brake_c < 0.15
    assert min_brake_t is not None and min_brake_t > 1.0


def test_collision_free_final_brake_pose_but_unsafe_intermediate_braking_path() -> None:
    # Robot ends prediction at x=4.0, brakes to x=5.20 (v=3.0, a=5.0, react=0.1 -> d_stop=1.20m).
    # Obstacle at x=4.55..4.65 (y=7.3..7.7):
    # - At prediction end x=4.0: clearance = 4.55 - 4.0 - 0.35 = 0.20m >= 0.15m (prediction is safe).
    # - At final brake pose x=5.20: clearance = 5.20 - 4.65 - 0.35 = 0.20m >= 0.15m (final pose is safe).
    # - Yet the intermediate braking path passes directly through x=4.60!
    state = RobotState(x_m=1.0, y_m=7.5, theta_rad=0.0)
    traj = predict_trajectory(state, v_mps=3.0, omega_radps=0.0, predict_time_s=1.0, predict_dt_s=0.1)
    mid_obs = StaticObstacle(x_m=4.55, y_m=7.3, width_m=0.10, height_m=0.4)

    is_safe, _, min_brake_c, _ = evaluate_trajectory_with_braking(
        traj,
        0.1,
        0.35,
        0.15,
        DEFAULT_CONFIG.world,
        [mid_obs],
        [],
        v_mps=3.0,
        max_brake_decel_mps2=5.0,
        reaction_time_s=0.1,
    )
    assert is_safe is False
    assert min_brake_c is not None and min_brake_c < 0.0


def test_replanning_after_dynamic_blockage() -> None:
    world = SimulationWorld(
        DEFAULT_CONFIG.world,
        DEFAULT_CONFIG.robot,
        DEFAULT_CONFIG.simulation,
        safety_margin_m=DEFAULT_CONFIG.planner.safety_margin_m,
    )
    # Wall with two openings: lower opening at y=7.5, upper detour via y=12.5
    world.add_static_obstacle(StaticObstacle(x_m=9.0, y_m=0.0, width_m=1.0, height_m=6.0))
    world.add_static_obstacle(StaticObstacle(x_m=9.0, y_m=9.0, width_m=1.0, height_m=2.5))
    # Dynamic obstacle plugs the lower opening (x=9.5, y=7.5)
    plug = DynamicObstacle(
        obstacle_id="PLUG",
        x_m=9.5,
        y_m=7.5,
        vx_mps=0.0,
        vy_mps=0.0,
        radius_m=1.2,
        behavior=MovementBehavior.HEAD_ON,
    )
    ok, new_path = replan_global_path_around_obstacles(
        world.occupancy_grid,
        DEFAULT_CONFIG.world.grid_resolution_m,
        2.0,
        7.5,
        15.0,
        7.5,
        DEFAULT_CONFIG.robot.radius_m,
        DEFAULT_CONFIG.planner.safety_margin_m,
        [plug],
    )
    assert ok is True
    assert len(new_path) > 2
    # Must route through the upper opening (y > 11.5) around the blocked lower opening
    assert any(py > 11.5 for _, py in new_path)


def test_replanning_no_path_retains_blocked_status() -> None:
    app = DynamicNavApp()
    app.navigate_to(AppView.SIMULATION)
    # Plug the world completely across x=10.0
    app.world.add_static_obstacle(StaticObstacle(x_m=10.0, y_m=0.0, width_m=1.0, height_m=15.0))
    ok = app.attempt_global_replan()
    assert ok is False
    assert app.replan_attempts == 1
    assert app.successful_replans == 0


def test_replanning_cooldown_and_deterministic_repeatability(monkeypatch: pytest.MonkeyPatch) -> None:
    app = DynamicNavApp()
    app.navigate_to(AppView.SIMULATION)
    app.plan_path()
    app._on_start()

    # Force DWAPlanner to report blocked=True without actually colliding
    from algorithms.dwa import DWAResult

    def _always_blocked(*args: object, **kwargs: object) -> DWAResult:
        return DWAResult(
            v_mps=0.0,
            omega_radps=0.0,
            score=None,
            predicted_min_clearance_m=0.05,
            fallback_active=True,
            fallback_reason="PHYSICAL_COLLISION_RISK",
            blocked=True,
            status="BLOCKED",
        )

    monkeypatch.setattr(app.dwa_planner, "compute_command", _always_blocked)
    # Step 10 frames (0.50 s < replan_cooldown_s = 2.0 s) -> must replan only ONCE
    for _ in range(10):
        app.step_frame(0.05)

    assert app.replan_attempts == 1


def test_automatic_replanning_after_dynamic_blockage_end_to_end() -> None:
    """Demonstrate full-app automatic A* replanning after mid-run dynamic blockage."""
    from algorithms.astar import is_segment_collision_free
    from config import StaticObstacleConfig
    from simulation.scenarios import ScenarioDefinition

    app = DynamicNavApp()
    app.navigate_to(AppView.SIMULATION)

    # Upper corridor (y=6.5..9.0) and lower detour corridor (y=0.0..4.5)
    sc = ScenarioDefinition(
        name="REPLAN-DETOUR",
        behavior=MovementBehavior.HEAD_ON,
        description="Deterministic mid-run dynamic blockage replanning scenario.",
        robot_start_m=(2.0, 7.5),
        robot_theta_rad=0.0,
        goal_m=(17.0, 7.5),
        static_obstacles=(
            StaticObstacleConfig(x_m=8.0, y_m=4.5, width_m=4.0, height_m=2.0),
            StaticObstacleConfig(x_m=8.0, y_m=9.0, width_m=4.0, height_m=6.0),
        ),
        dynamic_obstacles=(),
    )
    app.world.load_scenario(sc)
    app._start_m = sc.robot_start_m
    app._goal_m = sc.goal_m

    initial_plan = app.plan_path()
    assert initial_plan.success is True
    app._on_start()

    # Navigate for 1.0 s along the initial upper-corridor route
    for _ in range(20):
        app.step_frame(0.05)
    assert app.world.robot.state.x_m > 2.5
    assert app.replan_attempts == 0

    # Inject a dynamic blockage that seals the upper corridor (y=6.5..9.0)
    app.world._dynamic_obstacles.append(
        DynamicObstacle(
            obstacle_id="DYN-BLOCK-UPPER",
            x_m=7.8,
            y_m=7.75,
            vx_mps=0.0,
            vy_mps=0.0,
            radius_m=0.85,
            behavior=MovementBehavior.HEAD_ON,
        )
    )

    replan_observed = False
    replanned_path: list[tuple[float, float]] = []
    robot_at_replan: tuple[float, float] = (0.0, 0.0)
    for _ in range(1200):
        prev_replans = app.successful_replans
        app.step_frame(0.05)
        if app.successful_replans > prev_replans and not replan_observed:
            replan_observed = True
            replanned_path = list(app.dwa_planner._path_world_m)
            robot_at_replan = (app.world.robot.state.x_m, app.world.robot.state.y_m)
            # Nudge blockage 0.25 m deeper inside upper corridor (still sealing y=6.5..9.0)
            # so robot is clear of the 0.15 m standstill margin and can execute the detour.
            app.world._dynamic_obstacles[0].x_m = 8.05
        if app.dwa_planner.target_reached:
            break

    assert replan_observed is True
    assert app.replan_attempts >= 1
    assert app.successful_replans >= 1
    assert app.dwa_planner.target_reached is True
    assert app._safety_tracker.collision_count == 0
    assert len(replanned_path) >= 2
    assert math.hypot(replanned_path[0][0] - robot_at_replan[0], replanned_path[0][1] - robot_at_replan[1]) <= 0.65
    assert math.hypot(replanned_path[-1][0] - 17.0, replanned_path[-1][1] - 7.5) <= 0.25

    # Verify every replanned segment passes static geometric safety validation (0.50 m)
    res_m = app.world.world_config.grid_resolution_m
    req_clr = app._config.robot.radius_m + app._config.planner.safety_margin_m
    for idx in range(len(replanned_path) - 1):
        assert is_segment_collision_free(
            replanned_path[idx],
            replanned_path[idx + 1],
            app.world.occupancy_grid,
            res_m,
            req_clr,
            enforce_boundary_clearance=app._config.planner.enforce_boundary_clearance,
        )

    # Second check: when NO safe route exists, robot remains BLOCKED without collision
    app2 = DynamicNavApp()
    app2.navigate_to(AppView.SIMULATION)
    sc_blocked = ScenarioDefinition(
        name="REPLAN-IMPASSABLE",
        behavior=MovementBehavior.HEAD_ON,
        description="Deterministic single-corridor impassable dynamic blockage scenario.",
        robot_start_m=(5.5, 7.5),
        robot_theta_rad=0.0,
        goal_m=(17.0, 7.5),
        static_obstacles=(
            StaticObstacleConfig(x_m=6.0, y_m=0.0, width_m=6.0, height_m=6.5),
            StaticObstacleConfig(x_m=6.0, y_m=9.0, width_m=6.0, height_m=6.0),
        ),
        dynamic_obstacles=(),
    )
    app2.world.load_scenario(sc_blocked)
    app2._start_m = sc_blocked.robot_start_m
    app2._goal_m = sc_blocked.goal_m
    assert app2.plan_path().success is True
    app2._on_start()
    app2.world._dynamic_obstacles.append(
        DynamicObstacle(
            obstacle_id="DYN-BLOCK-ONLY-CORRIDOR",
            x_m=8.0,
            y_m=7.75,
            vx_mps=0.0,
            vy_mps=0.0,
            radius_m=0.85,
            behavior=MovementBehavior.HEAD_ON,
        )
    )
    for _ in range(250):
        app2.step_frame(0.05)
    assert app2.replan_attempts >= 1
    assert app2.successful_replans == 0
    assert app2._safety_tracker.collision_count == 0
    assert abs(app2.world.robot.state.v_mps) < 1e-6
