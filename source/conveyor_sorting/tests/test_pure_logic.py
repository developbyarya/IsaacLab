# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

# Target Environment: Isaac Sim 6.2.0, Isaac Lab 3.0.0, Python 3.12, PhysX Backend

"""Unit tests for decoupled spawn and pool tracking logic (runs without Isaac simulator)."""

import math
import unittest

from conveyor_sorting.core.pool_tracker import ObjectPoolTracker, PoolCfg
from conveyor_sorting.core.spawn_logic import SpawnCalculator, SpawnCfg


class TestSpawnLogic(unittest.TestCase):
    """Test suite for spawn scheduling, clearance check, and pose sampling."""

    def setUp(self):
        self.cfg = SpawnCfg(
            interval_s=(2.0, 3.0),
            lateral_range_m=(-0.15, 0.15),
            min_gap_m=0.10,
            spawn_x_m=-1.20,
            surface_z_m=0.60,
            spawn_height_offset_m=0.04,
        )
        self.calc = SpawnCalculator(self.cfg, seed=42)

    def test_fixed_interval_schedule(self):
        fixed_cfg = SpawnCfg(interval_s=(2.5, 2.5))
        calc = SpawnCalculator(fixed_cfg, seed=0)
        self.assertAlmostEqual(calc._next_spawn_time_s, 2.5)
        next_t = calc.advance_schedule(current_time_s=2.5)
        self.assertAlmostEqual(next_t, 5.0)

    def test_clearance_gap_check(self):
        # Spawn point at -1.20 with min gap 0.10.
        # Object at -1.15 is only 0.05 m away -> blocked
        self.assertFalse(self.calc.is_spawn_area_clear([-1.15]))
        # Object at -1.05 is 0.15 m away -> clear
        self.assertTrue(self.calc.is_spawn_area_clear([-1.05]))
        # No active objects -> clear
        self.assertTrue(self.calc.is_spawn_area_clear([]))

    def test_sample_candidate_bounds(self):
        for _ in range(50):
            candidate = self.calc.sample_candidate()
            x, y, z = candidate.position
            self.assertEqual(x, -1.20)
            self.assertGreaterEqual(y, -0.15)
            self.assertLessEqual(y, 0.15)
            self.assertAlmostEqual(z, 0.64)

            # Check valid quaternion norm
            qw, qx, qy, qz = candidate.orientation_quat_wxyz
            norm = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
            self.assertAlmostEqual(norm, 1.0, places=5)

    def test_reproducibility_with_seed(self):
        calc1 = SpawnCalculator(self.cfg, seed=123)
        calc2 = SpawnCalculator(self.cfg, seed=123)

        c1 = [calc1.sample_candidate() for _ in range(10)]
        c2 = [calc2.sample_candidate() for _ in range(10)]

        for item1, item2 in zip(c1, c2):
            self.assertEqual(item1.position, item2.position)
            self.assertEqual(item1.orientation_quat_wxyz, item2.orientation_quat_wxyz)


class TestPoolTracker(unittest.TestCase):
    """Test suite for pre-allocated pool tracking, despawn triggers, and info reporting."""

    def setUp(self):
        self.cfg = PoolCfg(
            max_active=5,
            shapes=["cube", "sphere", "cylinder", "capsule"],
            conveyor_length_m=2.4,
            surface_height_m=0.6,
            despawn_margin_m=0.0,
            despawn_min_height_m=0.2,
        )
        self.tracker = ObjectPoolTracker(self.cfg)

    def test_initial_state(self):
        self.assertEqual(self.tracker.num_active, 0)
        self.assertFalse(self.tracker.is_pool_exhausted)
        self.assertEqual(len(self.tracker.inactive_indices), 5)
        self.assertEqual(len(self.tracker.object_shapes), 5)

    def test_allocate_spawn_up_to_max(self):
        records = []
        for i in range(5):
            rec = self.tracker.allocate_spawn(
                sim_time_s=float(i),
                spawn_pos=(-1.2, 0.0, 0.64),
                spawn_quat_wxyz=(1.0, 0.0, 0.0, 0.0),
            )
            self.assertIsNotNone(rec)
            records.append(rec)

        self.assertEqual(self.tracker.num_active, 5)
        self.assertTrue(self.tracker.is_pool_exhausted)

        # 6th attempt should return None (pool exhausted per FR-6)
        overflow_rec = self.tracker.allocate_spawn(
            sim_time_s=5.0,
            spawn_pos=(-1.2, 0.0, 0.64),
            spawn_quat_wxyz=(1.0, 0.0, 0.0, 0.0),
        )
        self.assertIsNone(overflow_rec)

    def test_despawn_passed_end_conveyor(self):
        # Conveyor end is at x = +1.20
        self.tracker.allocate_spawn(
            sim_time_s=1.0,
            spawn_pos=(-1.2, 0.0, 0.64),
            spawn_quat_wxyz=(1.0, 0.0, 0.0, 0.0),
        )
        # Move object beyond +1.20
        self.tracker.update_object_pose(0, (1.25, 0.0, 0.64), (1.0, 0.0, 0.0, 0.0))

        despawn_items = self.tracker.check_despawn_conditions(sim_time_s=10.0)
        self.assertEqual(len(despawn_items), 1)
        idx, reason = despawn_items[0]
        self.assertEqual(idx, 0)
        self.assertEqual(reason, "passed_conveyor_end")

        # Deallocate and verify return to pool
        self.tracker.deallocate_despawn(idx, sim_time_s=10.0, reason=reason)
        self.assertEqual(self.tracker.num_active, 0)
        self.assertIn(0, self.tracker.inactive_indices)

    def test_despawn_fall_below_minimum_height(self):
        # Surface is 0.60, min fall height is 0.20 -> threshold is 0.40
        self.tracker.allocate_spawn(
            sim_time_s=1.0,
            spawn_pos=(-1.2, 0.0, 0.64),
            spawn_quat_wxyz=(1.0, 0.0, 0.0, 0.0),
        )
        # Drop object below 0.40 m
        self.tracker.update_object_pose(0, (0.0, 0.0, 0.35), (1.0, 0.0, 0.0, 0.0))

        despawn_items = self.tracker.check_despawn_conditions(sim_time_s=5.0)
        self.assertEqual(len(despawn_items), 1)
        self.assertEqual(despawn_items[0][1], "fell_below_minimum_height")

    def test_info_dict_reporting(self):
        self.tracker.allocate_spawn(1.0, (-1.2, 0.0, 0.64), (1.0, 0.0, 0.0, 0.0))
        self.tracker.allocate_spawn(2.0, (-1.2, 0.05, 0.64), (1.0, 0.0, 0.0, 0.0))
        info = self.tracker.get_info_dict()

        self.assertEqual(info["num_spawned"], 2)
        self.assertEqual(info["num_despawned"], 0)
        self.assertEqual(info["num_active"], 2)
        self.assertEqual(len(info["active_objects"]), 2)


if __name__ == "__main__":
    unittest.main()
