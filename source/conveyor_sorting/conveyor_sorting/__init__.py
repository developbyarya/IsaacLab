# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

# Target Environment: Isaac Sim 6.2.0, Isaac Lab 3.0.0, Python 3.12, PhysX Backend

"""Isaac Lab Conveyor Sorting Environment package."""

from .core.pool_tracker import ActiveObjectRecord, ObjectPoolTracker, PoolCfg
from .core.spawn_logic import SpawnCalculator, SpawnCandidate, SpawnCfg
from .scene.conveyor import ConveyorCfg, setup_conveyor_belt

__version__ = "0.1.0"

__all__ = [
    "ActiveObjectRecord",
    "ConveyorCfg",
    "ObjectPoolTracker",
    "PoolCfg",
    "SpawnCalculator",
    "SpawnCandidate",
    "SpawnCfg",
    "setup_conveyor_belt",
]
