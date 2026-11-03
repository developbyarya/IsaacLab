# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

# Target Environment: Isaac Sim 6.2.0, Isaac Lab 3.0.0, Python 3.12, PhysX Backend

"""Core decoupled logic for conveyor object pooling, spawn scheduling, and despawn tracking."""

from .pool_tracker import ActiveObjectRecord, ObjectPoolTracker, PoolCfg
from .spawn_logic import SpawnCalculator, SpawnCfg, SpawnCandidate

__all__ = [
    "ActiveObjectRecord",
    "ObjectPoolTracker",
    "PoolCfg",
    "SpawnCalculator",
    "SpawnCandidate",
    "SpawnCfg",
]
