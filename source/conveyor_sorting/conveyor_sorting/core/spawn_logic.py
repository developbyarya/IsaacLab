# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

# Target Environment: Isaac Sim 6.2.0, Isaac Lab 3.0.0, Python 3.12, PhysX Backend

"""Spawn timing, position sampling, and clearance check logic (pure Python, simulator-independent)."""

from __future__ import annotations

import math
import random
from dataclasses import dataclass


@dataclass
class SpawnCfg:
    """Configuration for conveyor object spawning."""

    interval_s: tuple[float, float] = (2.5, 2.5)
    """Interval between spawn events in seconds: (min, max). If min == max, fixed interval."""

    lateral_range_m: tuple[float, float] = (-0.15, 0.15)
    """Lateral position range relative to belt centerline (Y-axis) in meters."""

    min_gap_m: float = 0.10
    """Minimum clearance along X-axis from spawn point to nearest active object in meters."""

    spawn_x_m: float = -1.20
    """X coordinate where objects spawn on the conveyor (default: -L/2 = -1.2 m)."""

    surface_z_m: float = 0.60
    """Conveyor surface height above floor in meters."""

    spawn_height_offset_m: float = 0.04
    """Vertical clearance above conveyor surface when dropping spawned object."""


@dataclass
class SpawnCandidate:
    """Candidate spawn state for an object."""

    position: tuple[float, float, float]
    """Spawn position (x, y, z) in meters."""

    orientation_quat_wxyz: tuple[float, float, float, float]
    """Orientation quaternion (w, x, y, z) with random yaw rotation."""

    linear_velocity: tuple[float, float, float] = (0.0, 0.0, 0.0)
    """Initial linear velocity in m/s (zero by default per FR-6)."""

    angular_velocity: tuple[float, float, float] = (0.0, 0.0, 0.0)
    """Initial angular velocity in rad/s (zero by default)."""


class SpawnCalculator:
    """Calculates spawn timing, clearance validation, and pose sampling."""

    def __init__(self, cfg: SpawnCfg, seed: int = 0):
        """Initialize the spawn calculator with configuration and random generator.

        Args:
            cfg: Spawn configuration parameters.
            seed: Random seed for reproducible lateral position, yaw, and intervals.
        """
        self.cfg = cfg
        self._rng = random.Random(seed)
        self._next_spawn_time_s: float = self._sample_next_interval(0.0)

    def set_seed(self, seed: int) -> None:
        """Reset random number generator seed.

        Args:
            seed: Seed value.
        """
        self._rng = random.Random(seed)
        self._next_spawn_time_s = self._sample_next_interval(0.0)

    def reset(self, current_time_s: float = 0.0) -> None:
        """Reset spawn schedule to initial state at the specified time.

        Args:
            current_time_s: Current simulation time in seconds.
        """
        self._next_spawn_time_s = self._sample_next_interval(current_time_s)

    def is_spawn_due(self, current_time_s: float) -> bool:
        """Check whether scheduled time for the next spawn has arrived.

        Args:
            current_time_s: Current simulation time in seconds.

        Returns:
            True if current time is greater than or equal to scheduled spawn time.
        """
        return current_time_s >= self._next_spawn_time_s

    def is_spawn_area_clear(self, active_object_x_positions: list[float]) -> bool:
        """Check whether the spawn zone has sufficient clearance along X-axis.

        Per FR-6, spawn is postponed if another object is still within min_gap_m of spawn_x_m.

        Args:
            active_object_x_positions: List of X coordinates of currently active objects.

        Returns:
            True if all active objects are at least min_gap_m away from spawn_x_m.
        """
        for x in active_object_x_positions:
            if abs(x - self.cfg.spawn_x_m) < self.cfg.min_gap_m:
                return False
        return True

    def sample_candidate(self) -> SpawnCandidate:
        """Sample a new object pose with random lateral position and random yaw.

        Returns:
            SpawnCandidate containing position, orientation quaternion (w, x, y, z), and velocities.
        """
        lat_min, lat_max = self.cfg.lateral_range_m
        y = self._rng.uniform(lat_min, lat_max)
        x = self.cfg.spawn_x_m
        z = self.cfg.surface_z_m + self.cfg.spawn_height_offset_m

        # Sample random yaw angle around Z axis in [-pi, pi]
        yaw = self._rng.uniform(-math.pi, math.pi)
        qw = math.cos(yaw / 2.0)
        qz = math.sin(yaw / 2.0)
        quat_wxyz = (qw, 0.0, 0.0, qz)

        return SpawnCandidate(
            position=(x, y, z),
            orientation_quat_wxyz=quat_wxyz,
            linear_velocity=(0.0, 0.0, 0.0),
            angular_velocity=(0.0, 0.0, 0.0),
        )

    def advance_schedule(self, current_time_s: float) -> float:
        """Advance the scheduled spawn time to the next interval.

        Args:
            current_time_s: Current simulation time in seconds.

        Returns:
            The newly scheduled spawn time in seconds.
        """
        self._next_spawn_time_s = self._sample_next_interval(current_time_s)
        return self._next_spawn_time_s

    def _sample_next_interval(self, base_time_s: float) -> float:
        """Sample duration from interval range and add to base time.

        Args:
            base_time_s: Base timestamp in seconds.

        Returns:
            Next scheduled timestamp in seconds.
        """
        min_dt, max_dt = self.cfg.interval_s
        if min_dt == max_dt:
            dt = min_dt
        else:
            dt = self._rng.uniform(min_dt, max_dt)
        return base_time_s + dt
