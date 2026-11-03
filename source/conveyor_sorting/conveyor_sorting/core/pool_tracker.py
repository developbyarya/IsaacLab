# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

# Target Environment: Isaac Sim 6.2.0, Isaac Lab 3.0.0, Python 3.12, PhysX Backend

"""Object pool tracker and state manager (pure Python, simulator-independent)."""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class PoolCfg:
    """Configuration for conveyor object pool and despawn criteria."""

    max_active: int = 5
    """Maximum number of simultaneously active objects (size of pre-allocated pool)."""

    shapes: list[str] = field(
        default_factory=lambda: ["cube", "sphere", "cylinder", "capsule"]
    )
    """List of geometric shape names used across the pool."""

    parking_offset_xy_m: tuple[float, float] = (0.0, 1.5)
    """Parking zone (x, y) coordinates for inactive objects, out of reach and camera view."""

    parking_z_m: float = -0.50
    """Z coordinate for inactive objects in the parking zone (beneath workspace)."""

    conveyor_length_m: float = 2.4
    """Conveyor belt total length in meters."""

    surface_height_m: float = 0.6
    """Conveyor belt surface height from floor in meters."""

    despawn_margin_m: float = 0.0
    """Margin before conveyor end for despawning. End threshold = (L/2 - margin_m)."""

    despawn_min_height_m: float = 0.2
    """Height drop beneath belt surface that triggers a despawn (fall-off threshold)."""


@dataclass
class ActiveObjectRecord:
    """Record of an active object currently tracked on the conveyor."""

    pool_index: int
    """Index in the pre-allocated pool."""

    shape: str
    """Shape name of the object."""

    spawned_time_s: float
    """Simulation time when the object was spawned."""

    position: tuple[float, float, float] = (0.0, 0.0, 0.0)
    """Latest known (x, y, z) position in meters."""

    orientation_quat_wxyz: tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)
    """Latest known orientation quaternion (w, x, y, z)."""


@dataclass
class TransitionEventLog:
    """Logged transition event for spawn or despawn."""

    event_type: str
    """Type of transition: 'spawn' or 'despawn'."""

    sim_time_s: float
    """Simulation timestamp in seconds."""

    pool_index: int
    """Pool index of the object."""

    shape: str
    """Shape of the object."""

    reason: str = ""
    """Reason for transition (e.g. 'scheduled', 'boundary_reached', 'fell_below_threshold')."""


class ObjectPoolTracker:
    """Tracks active and inactive objects in the pre-allocated pool per environment.

    Maintains lifecycle states without deleting or instantiating USD prims at runtime.
    """

    def __init__(self, cfg: PoolCfg):
        """Initialize tracker with pool configuration.

        Args:
            cfg: Pool and despawn configuration.
        """
        self.cfg = cfg

        # Assign a shape to each pool index cyclicly
        num_shapes = len(self.cfg.shapes)
        self.object_shapes: list[str] = [
            self.cfg.shapes[i % num_shapes] for i in range(self.cfg.max_active)
        ]

        # Inactive indices available for spawn
        self.inactive_indices: list[int] = list(range(self.cfg.max_active))
        # Active object records mapped by pool index
        self.active_objects: dict[int, ActiveObjectRecord] = {}

        # Counters and event logs per FR-11
        self.total_spawned_count: int = 0
        self.total_despawned_count: int = 0
        self.transition_logs: list[TransitionEventLog] = []

    def reset(self) -> None:
        """Reset the pool to initial state: all objects inactive, counters zeroed."""
        self.inactive_indices = list(range(self.cfg.max_active))
        self.active_objects.clear()
        self.total_spawned_count = 0
        self.total_despawned_count = 0
        self.transition_logs.clear()

    @property
    def num_active(self) -> int:
        """Return the number of currently active objects."""
        return len(self.active_objects)

    @property
    def is_pool_exhausted(self) -> bool:
        """Return True if all pool objects are active and none are available for spawn."""
        return len(self.inactive_indices) == 0

    def get_active_x_positions(self) -> list[float]:
        """Return the list of X coordinates of all currently active objects."""
        return [record.position[0] for record in self.active_objects.values()]

    def update_object_pose(
        self,
        pool_index: int,
        position: tuple[float, float, float],
        quat_wxyz: tuple[float, float, float, float],
    ) -> None:
        """Update the tracked pose of an active object.

        Args:
            pool_index: Index in the object pool.
            position: New (x, y, z) position in meters.
            quat_wxyz: New (w, x, y, z) quaternion.
        """
        if pool_index in self.active_objects:
            record = self.active_objects[pool_index]
            record.position = position
            record.orientation_quat_wxyz = quat_wxyz

    def allocate_spawn(
        self,
        sim_time_s: float,
        spawn_pos: tuple[float, float, float],
        spawn_quat_wxyz: tuple[float, float, float, float],
    ) -> ActiveObjectRecord | None:
        """Allocate an inactive object from the pool and register it as active.

        Per FR-6, if no inactive object is available, spawn is delayed and returns None.

        Args:
            sim_time_s: Current simulation timestamp.
            spawn_pos: Initial (x, y, z) position.
            spawn_quat_wxyz: Initial (w, x, y, z) quaternion.

        Returns:
            The newly created ActiveObjectRecord, or None if pool was exhausted.
        """
        if not self.inactive_indices:
            return None

        pool_idx = self.inactive_indices.pop(0)
        shape = self.object_shapes[pool_idx]

        record = ActiveObjectRecord(
            pool_index=pool_idx,
            shape=shape,
            spawned_time_s=sim_time_s,
            position=spawn_pos,
            orientation_quat_wxyz=spawn_quat_wxyz,
        )
        self.active_objects[pool_idx] = record
        self.total_spawned_count += 1

        self.transition_logs.append(
            TransitionEventLog(
                event_type="spawn",
                sim_time_s=sim_time_s,
                pool_index=pool_idx,
                shape=shape,
                reason="scheduled",
            )
        )
        return record

    def check_despawn_conditions(
        self, sim_time_s: float
    ) -> list[tuple[int, str]]:
        """Evaluate despawn criteria for all active objects per FR-7.

        Criteria:
        1. Object passed conveyor end: x >= (conveyor_length_m / 2.0 - despawn_margin_m)
        2. Object fell below minimum height: z < (surface_height_m - despawn_min_height_m)

        Args:
            sim_time_s: Current simulation time.

        Returns:
            List of tuples (pool_index, despawn_reason) for objects that met despawn criteria.
        """
        end_threshold_x = (self.cfg.conveyor_length_m / 2.0) - self.cfg.despawn_margin_m
        fall_threshold_z = self.cfg.surface_height_m - self.cfg.despawn_min_height_m

        despawn_list: list[tuple[int, str]] = []

        for pool_idx, record in list(self.active_objects.items()):
            x, y, z = record.position
            if x >= end_threshold_x:
                despawn_list.append((pool_idx, "passed_conveyor_end"))
            elif z < fall_threshold_z:
                despawn_list.append((pool_idx, "fell_below_minimum_height"))

        return despawn_list

    def deallocate_despawn(
        self, pool_index: int, sim_time_s: float, reason: str
    ) -> None:
        """Deactivate an active object, return its index to the pool, and log despawn.

        Args:
            pool_index: Index of object to deactivate.
            sim_time_s: Current simulation time.
            reason: Reason for despawn.
        """
        if pool_index not in self.active_objects:
            return

        record = self.active_objects.pop(pool_index)
        self.inactive_indices.append(pool_index)
        self.total_despawned_count += 1

        self.transition_logs.append(
            TransitionEventLog(
                event_type="despawn",
                sim_time_s=sim_time_s,
                pool_index=pool_index,
                shape=record.shape,
                reason=reason,
            )
        )

    def get_parking_pose(self, pool_index: int) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
        """Return the parking pose for an inactive object.

        Slightly staggers objects in the parking zone along Y to prevent collision overlapping.

        Args:
            pool_index: Object index in pool.

        Returns:
            ((x, y, z), (qw, qx, qy, qz)) parking pose tuple.
        """
        base_x, base_y = self.cfg.parking_offset_xy_m
        stagger_y = base_y + pool_index * 0.15
        return ((base_x, stagger_y, self.cfg.parking_z_m), (1.0, 0.0, 0.0, 0.0))

    def get_info_dict(self) -> dict:
        """Generate summary statistics for the environment's `info` return per FR-11.

        Returns:
            Dictionary containing counts and per-object active details.
        """
        active_details = []
        for idx, rec in self.active_objects.items():
            active_details.append({
                "id": idx,
                "shape": rec.shape,
                "spawned_time_s": rec.spawned_time_s,
                "position": rec.position,
                "orientation_quat_wxyz": rec.orientation_quat_wxyz,
            })

        return {
            "num_spawned": self.total_spawned_count,
            "num_despawned": self.total_despawned_count,
            "num_active": len(self.active_objects),
            "active_objects": active_details,
        }
