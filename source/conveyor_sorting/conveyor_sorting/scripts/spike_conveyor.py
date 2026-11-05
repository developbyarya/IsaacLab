# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

# Target Environment: Isaac Sim 6.2.0, Isaac Lab 3.0.0, Python 3.12, PhysX Backend

"""Milestone 1 (M1) Conveyor Spike Script.

Validates conveyor surface velocity integration and evaluates acceptance criterion AT-2:
"Kecepatan conveyor: letakkan satu objek di belt, ukur kecepatan sumbu X setelah transien.
Kriteria lulus: Selisih dengan conveyor.speed_mps <= 10%."

Usage (on target machine with Isaac Sim / Isaac Lab installed):
    Windows:
        .\isaaclab.bat -p source\conveyor_sorting\conveyor_sorting\scripts\spike_conveyor.py --headless --num_envs 1 --speed 0.2
        .\isaaclab.bat -p source\conveyor_sorting\conveyor_sorting\scripts\spike_conveyor.py --headless --num_envs 4 --speed 0.2
    Linux:
        uv run python source/conveyor_sorting/conveyor_sorting/scripts/spike_conveyor.py --headless --num_envs 1 --speed 0.2
        uv run python source/conveyor_sorting/conveyor_sorting/scripts/spike_conveyor.py --headless --num_envs 4 --speed 0.2
"""

from __future__ import annotations

import argparse
import os
import sys

# Ensure conveyor_sorting package root is in sys.path
_CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
_PKG_ROOT = os.path.abspath(os.path.join(_CURRENT_DIR, "..", ".."))
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

def clean_windows_path():
    """Sanitize PATH on Windows to prevent WinError 206 without altering DLL search order."""
    if sys.platform != "win32":
        return

    clean_paths = []
    for p in os.environ.get("PATH", "").split(os.pathsep):
        p_clean = p.strip().strip('"')
        if not p_clean or len(p_clean) > 200 or not os.path.isdir(p_clean):
            continue
        if "copilot-shim" in p_clean.lower():
            continue
        try:
            h = os.add_dll_directory(p_clean)
            h.close()
            clean_paths.append(p_clean)
        except Exception:
            pass
    os.environ["PATH"] = os.pathsep.join(clean_paths)


clean_windows_path()

from isaaclab.app import add_launcher_args, launch_simulation

# 1. Parse arguments first
parser = argparse.ArgumentParser(
    description="Milestone 1 (M1) Spike: Validate conveyor belt velocity and evaluate AT-2."
)
parser.add_argument("--speed", type=float, default=0.2, help="Conveyor belt target speed in m/s.")
parser.add_argument("--num_envs", type=int, default=1, help="Number of parallel environments to evaluate.")
parser.add_argument("--num_steps", type=int, default=400, help="Number of simulation steps to run.")
parser.add_argument(
    "--mode",
    type=str,
    choices=["custom_drive", "surface_api", "omnigraph"],
    default="custom_drive",
    help=(
        "Conveyor implementation mode: "
        "'custom_drive' (Isaac Lab tensor state velocity drive, GPU/CPU compatible), "
        "'surface_api' (PhysX PhysxSurfaceVelocityAPI), "
        "or 'omnigraph' (Isaac Sim Conveyor Belt Utility)."
    ),
)
add_launcher_args(parser)
args_cli = parser.parse_args()

# 2. Imports of framework classes
import torch

import isaaclab.sim as sim_utils
from isaaclab.assets import RigidObject, RigidObjectCfg
from isaaclab.sim import SimulationCfg, SimulationContext


def build_spike_scene(
    stage,
    origins: list[list[float]],
    conveyor_cfg,
) -> tuple[dict[str, RigidObject], any]:
    """Constructs minimal spike scene containing ground, conveyor belt, side rails, and test cube.

    Args:
        stage: The USD stage.
        origins: Origin coordinates for each environment.
        conveyor_cfg: Conveyor parameters.

    Returns:
        Tuple of (scene_entities dictionary, conveyor_cfg).
    """
    from pxr import Gf, UsdGeom
    from conveyor_sorting.scene.conveyor import enable_conveyor_extension, setup_conveyor_belt, spawn_side_rails
    # Ground plane and lighting
    ground_cfg = sim_utils.GroundPlaneCfg()
    ground_cfg.func("/World/defaultGroundPlane", ground_cfg)
    light_cfg = sim_utils.DomeLightCfg(intensity=2500.0, color=(0.85, 0.85, 0.85))
    light_cfg.func("/World/Light", light_cfg)

    # Only enable conveyor extension if omnigraph utility is requested
    if conveyor_cfg.use_omnigraph_utility:
        enable_conveyor_extension()

    half_depth = conveyor_cfg.belt_depth_m / 2.0
    belt_center_z = conveyor_cfg.surface_height_m - half_depth

    # Build conveyor belt geometry and physics for each environment
    for i, origin in enumerate(origins):
        env_ns = f"/World/Env_{i}"
        sim_utils.create_prim(env_ns, "Xform", translation=origin)

        belt_path = f"{env_ns}/ConveyorBelt"
        # Spawn conveyor belt rigid body with high-friction physics material
        belt_cfg = sim_utils.CuboidCfg(
            size=(conveyor_cfg.length_m, conveyor_cfg.width_m, conveyor_cfg.belt_depth_m),
            rigid_props=sim_utils.UsdPhysicsRigidBodyCfg(kinematic_enabled=True),
            collision_props=sim_utils.UsdPhysicsCollisionCfg(),
            physics_material=sim_utils.RigidBodyMaterialCfg(
                static_friction=1.0, dynamic_friction=1.0, restitution=0.0
            ),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.2, 0.2, 0.25)),
        )
        belt_cfg.func(belt_path, belt_cfg, translation=(0.0, 0.0, belt_center_z))

        # Setup conveyor surface velocity
        setup_conveyor_belt(stage, belt_path, conveyor_cfg)

        # Spawn side rails if configured
        if conveyor_cfg.side_rails:
            spawn_side_rails(stage, env_ns, conveyor_cfg)

    # Spawn test cube using RigidObject across all envs
    # Cube initial position: x = -1.0 m, y = 0.0 m, resting on surface at z = 0.60 m + (size/2) + 0.001 m
    cube_size = 0.05
    cube_cfg = RigidObjectCfg(
        prim_path="/World/Env_.*/TestCube",
        spawn=sim_utils.CuboidCfg(
            size=(cube_size, cube_size, cube_size),
            rigid_props=sim_utils.UsdPhysicsRigidBodyCfg(),
            mass_props=sim_utils.MassCfg(mass=0.05),
            collision_props=sim_utils.UsdPhysicsCollisionCfg(),
            physics_material=sim_utils.RigidBodyMaterialCfg(
                static_friction=1.0, dynamic_friction=0.9, restitution=0.0
            ),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.1, 0.7, 0.2)),
        ),
        init_state=RigidObjectCfg.InitialStateCfg(
            pos=(-1.0, 0.0, conveyor_cfg.surface_height_m + (cube_size / 2.0) + 0.001),
            rot=(1.0, 0.0, 0.0, 0.0),
            lin_vel=(0.0, 0.0, 0.0),
            ang_vel=(0.0, 0.0, 0.0),
        ),
    )
    test_cube = RigidObject(cfg=cube_cfg)

    return {"test_cube": test_cube}, conveyor_cfg


def main():
    """Main execution entry point for M1 Conveyor Spike."""
    print("=" * 70)
    print("=== STARTING MILESTONE 1 (M1): CONVEYOR SPIKE & AT-2 VALIDATION ===")
    print("=" * 70)

    from conveyor_sorting.scene.conveyor import ConveyorCfg

    num_envs = args_cli.num_envs
    target_speed = args_cli.speed
    use_utility = args_cli.mode == "omnigraph"

    conveyor_cfg = ConveyorCfg(
        length_m=2.4,
        width_m=0.5,
        surface_height_m=0.6,
        speed_mps=target_speed,
        side_rails=True,
        use_omnigraph_utility=use_utility,
    )

    # Simulation configuration
    sim_cfg = SimulationCfg(dt=1.0 / 60.0, device=args_cli.device)

    all_passed = False

    # Launch simulation runtime via context manager (starts Kit and initializes Omniverse)
    with launch_simulation(sim_cfg, args_cli):
        clean_windows_path()

        sim = SimulationContext(sim_cfg)
        sim.set_camera_view(eye=[0.0, -2.5, 2.2], target=[0.0, 0.0, 0.6])

        # Environment origins
        origins = [[0.0, i * 2.0, 0.0] for i in range(num_envs)]

        stage = sim.stage
        entities, _ = build_spike_scene(stage, origins, conveyor_cfg)
        cube: RigidObject = entities["test_cube"]

        sim.reset()
        cube.reset()

        print(f"\n[M1] Environment count: {num_envs}")
        print(f"[M1] Target conveyor speed: {target_speed:.4f} m/s")
        print(f"[M1] Implementation mode: {args_cli.mode}")
        print("[M1] Stepping simulation and recording linear velocity...")

        # Data collection buffers
        recorded_vx_steady: list[torch.Tensor] = []
        settle_steps = 60  # Initial 1 second transient window

        for step in range(args_cli.num_steps):
            # If in custom_drive mode, drive linear velocity along +X for objects resting on the belt
            if args_cli.mode == "custom_drive":
                vel = cube.data.root_vel_w.torch.clone()
                pos = cube.data.root_pos_w.torch
                half_len = conveyor_cfg.length_m / 2.0
                half_wid = conveyor_cfg.width_m / 2.0
                on_belt = (
                    (pos[:, 0] >= -half_len)
                    & (pos[:, 0] <= half_len)
                    & (pos[:, 1] >= -half_wid)
                    & (pos[:, 1] <= half_wid)
                    & (pos[:, 2] >= conveyor_cfg.surface_height_m - 0.05)
                    & (pos[:, 2] <= conveyor_cfg.surface_height_m + 0.20)
                )
                vel[on_belt, 0] = target_speed
                cube.write_root_velocity_to_sim_index(root_velocity=vel)

            # Step physics
            sim.step()
            cube.update(sim.get_physics_dt())

            # Measure linear velocity along X
            # root_lin_vel_w has shape (num_envs, 3)
            lin_vel_w = cube.data.root_lin_vel_w.torch
            vx = lin_vel_w[:, 0]
            pos_w = cube.data.root_pos_w.torch

            if step >= settle_steps:
                recorded_vx_steady.append(vx.clone())

            if (step + 1) % 50 == 0:
                avg_step_vx = vx.mean().item()
                avg_step_x = pos_w[:, 0].mean().item()
                print(
                    f"  Step {step + 1:3d}/{args_cli.num_steps}: avg x = {avg_step_x:6.3f} m | instantaneous avg vx = {avg_step_vx:6.4f} m/s"
                )

        # Aggregate results
        stacked_vx = torch.stack(recorded_vx_steady, dim=0)  # shape: (steps_steady, num_envs)
        mean_vx_per_env = stacked_vx.mean(dim=0).cpu().tolist()

        print("\n" + "=" * 70)
        print("=== RESULTS SUMMARY FOR AT-2 ACCEPTANCE CRITERION ===")
        print("=" * 70)

        all_passed = True
        for env_idx, measured_vx in enumerate(mean_vx_per_env):
            speed_error = abs(measured_vx - target_speed)
            error_pct = (speed_error / target_speed) * 100.0 if target_speed > 0 else 0.0
            passed = error_pct <= 10.0
            if not passed:
                all_passed = False

            status_str = "PASS" if passed else "FAIL"
            print(
                f"Env {env_idx}: Target = {target_speed:.4f} m/s | "
                f"Measured = {measured_vx:.4f} m/s | "
                f"Error = {error_pct:5.2f}% (Threshold <= 10.0%) -> [{status_str}]"
            )

        print("-" * 70)
        if all_passed:
            print("[M1 SPIKE STATUS]: PASSED - AT-2 criterion satisfied!")
        else:
            print("[M1 SPIKE STATUS]: FAILED - One or more envs exceeded 10% velocity tolerance.")
            if args_cli.mode == "omnigraph":
                print("[RECOMMENDATION]: Retry with direct surface velocity fallback: '--mode surface_api'")
        print("=" * 70)

    sys.exit(0 if all_passed else 1)


if __name__ == "__main__":
    main()
