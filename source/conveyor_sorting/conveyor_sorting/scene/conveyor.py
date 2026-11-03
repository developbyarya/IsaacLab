# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

# Target Environment: Isaac Sim 6.2.0, Isaac Lab 3.0.0, Python 3.12, PhysX Backend

"""Conveyor belt scene definition, utility integration, and surface velocity configuration."""

from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from pxr import Usd

logger = logging.getLogger(__name__)


@dataclass
class ConveyorCfg:
    """Configuration for straight conveyor belt."""

    length_m: float = 2.4
    """Total length of the conveyor belt in meters along X axis."""

    width_m: float = 0.5
    """Width of the conveyor belt surface in meters along Y axis."""

    surface_height_m: float = 0.6
    """Height of conveyor top surface above floor in meters along Z axis."""

    speed_mps: float = 0.2
    """Linear surface speed in meters per second along +X direction."""

    side_rails: bool = True
    """Whether protective side rails are spawned on the belt edges."""

    rail_height_m: float = 0.03
    """Height of side rails above belt surface in meters."""

    rail_thickness_m: float = 0.02
    """Thickness of side rail walls in meters."""

    belt_depth_m: float = 0.10
    """Vertical thickness of the belt rigid body in meters."""

    use_omnigraph_utility: bool = True
    """If True, attempt setup via isaacsim.asset.gen.conveyor; if False, use direct PhysxSurfaceVelocityAPI."""


def enable_conveyor_extension() -> bool:
    """Attempt to enable the Isaac Sim Conveyor Belt Utility extension.

    Returns:
        True if the extension was enabled, False otherwise.
    """
    try:
        import omni.kit.app  # TODO(verify): Isaac Sim 6.2.0 extension manager

        ext_manager = omni.kit.app.get_app().get_extension_manager()
        # Extension name in Isaac Sim 6.2.0
        ext_name = "isaacsim.asset.gen.conveyor"
        if not ext_manager.is_extension_enabled(ext_name):
            success = ext_manager.set_extension_enabled_immediate(ext_name, True)
            if not success:
                # Fallback check for legacy naming
                ext_name_legacy = "omni.isaac.conveyor"  # TODO(verify)
                success = ext_manager.set_extension_enabled_immediate(ext_name_legacy, True)
            return success
        return True
    except Exception as e:
        logger.warning("Could not enable conveyor extension via omni.kit.app: %s", e)
        return False


def setup_conveyor_belt(
    stage: Usd.Stage,
    belt_prim_path: str,
    cfg: ConveyorCfg,
) -> bool:
    """Configure surface velocity on the belt prim using either OmniGraph or PhysxSurfaceVelocityAPI.

    Args:
        stage: USD stage where prim exists.
        belt_prim_path: Path to the conveyor belt rigid body prim.
        cfg: Conveyor configuration dataclass.

    Returns:
        True if surface velocity was successfully configured.
    """
    from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics  # TODO(verify): USD & PhysX schemas

    belt_prim = stage.GetPrimAtPath(belt_prim_path)
    if not belt_prim.IsValid():
        raise ValueError(f"Conveyor belt prim path '{belt_prim_path}' is invalid on stage.")

    # 1. Ensure collision and rigid body APIs are applied
    if not belt_prim.HasAPI(UsdPhysics.CollisionAPI):
        UsdPhysics.CollisionAPI.Apply(belt_prim)

    if not belt_prim.HasAPI(UsdPhysics.RigidBodyAPI):
        rb_api = UsdPhysics.RigidBodyAPI.Apply(belt_prim)
        # Conveyor frame/belt should be kinematic so it stays fixed in place
        rb_api.CreateKinematicEnabledAttr().Set(True)

    applied_via_utility = False

    if cfg.use_omnigraph_utility:
        try:
            # Isaac Sim 6.2.0 conveyor utility function
            try:
                from isaacsim.asset.gen.conveyor import create_conveyor_belt  # TODO(verify)
            except ImportError:
                from omni.isaac.conveyor import create_conveyor_belt  # TODO(verify): legacy fallback

            prim_name = belt_prim_path.split("/")[-1]
            create_conveyor_belt(stage, belt_prim, prim_name)  # TODO(verify)
            logger.info("Configured conveyor belt via create_conveyor_belt utility.")
            applied_via_utility = True
        except Exception as err:
            logger.warning(
                "Failed to configure conveyor via create_conveyor_belt utility: %s. Falling back to PhysxSurfaceVelocityAPI.",
                err,
            )

    # 2. Fallback or direct configuration using native PhysxSurfaceVelocityAPI
    # This guarantees surface motion even if OmniGraph action graph fails or does not clone in multi-env
    if not applied_via_utility:
        surface_api = PhysxSchema.PhysxSurfaceVelocityAPI.Apply(belt_prim)
        # Velocity in local space along +X axis
        surface_api.CreateSurfaceVelocityAttr().Set(Gf.Vec3f(cfg.speed_mps, 0.0, 0.0))
        surface_api.CreateSurfaceVelocityLocalSpaceAttr().Set(True)
        surface_api.CreateSurfaceVelocityEnabledAttr().Set(True)
        logger.info(
            "Configured conveyor belt surface velocity directly via PhysxSurfaceVelocityAPI with speed %s m/s.",
            cfg.speed_mps,
        )

    return True


def spawn_side_rails(
    stage: Usd.Stage,
    parent_prim_path: str,
    cfg: ConveyorCfg,
) -> tuple[str, str]:
    """Spawn static side rails on the conveyor edges if enabled.

    Args:
        stage: USD stage.
        parent_prim_path: Parent prim path (e.g. env prim or conveyor base).
        cfg: Conveyor configuration dataclass.

    Returns:
        Tuple of (left_rail_path, right_rail_path).
    """
    from pxr import Gf, UsdGeom, UsdPhysics  # TODO(verify)

    left_path = f"{parent_prim_path}/LeftRail"
    right_path = f"{parent_prim_path}/RightRail"

    half_width = cfg.width_m / 2.0
    half_thick = cfg.rail_thickness_m / 2.0
    rail_y_offset = half_width + half_thick
    rail_z = cfg.surface_height_m + (cfg.rail_height_m / 2.0)

    for path, y_pos in [(left_path, -rail_y_offset), (right_path, rail_y_offset)]:
        cube = UsdGeom.Cube.Define(stage, path)
        cube.CreateSizeAttr().Set(1.0)
        # Scale: length along X, thickness along Y, height along Z
        cube.AddScaleOp().Set(Gf.Vec3d(cfg.length_m, cfg.rail_thickness_m, cfg.rail_height_m))
        cube.AddTranslateOp().Set(Gf.Vec3d(0.0, y_pos, rail_z))

        prim = stage.GetPrimAtPath(path)
        UsdPhysics.CollisionAPI.Apply(prim)

    return left_path, right_path
