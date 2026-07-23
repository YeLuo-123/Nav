#!/usr/bin/env python3
"""Regression tests for temporal clearing of projected 3D obstacles."""

import numpy as np

from tools.s2_registered_cloud_mapper import ElevatedObstacleLayer, OccupancyAccumulator


def main() -> None:
    accumulator = OccupancyAccumulator(
        resolution=0.05,
        map_size_m=12.0,
        hit_log_odds=0.85,
        miss_log_odds=0.40,
    )
    accumulator.initialize(0.0, 0.0)
    layer = ElevatedObstacleLayer(
        accumulator.size,
        accumulator.resolution,
        hit_log_odds=0.70,
        miss_log_odds=0.20,
    )
    table = np.asarray([[1.0, 0.0]], dtype=np.float32)
    person_old = np.asarray([[2.0, 0.5]], dtype=np.float32)

    # Both objects become occupied after repeated observations.
    for _ in range(3):
        layer.integrate(accumulator, (0.0, 0.0), np.vstack((table, person_old)), 0.0, 8.0)
    table_cell = accumulator.cell(*table[0])
    person_cell = accumulator.cell(*person_old[0])
    assert table_cell is not None and person_cell is not None
    assert layer.occupied(1.10)[table_cell]
    assert layer.occupied(1.10)[person_cell]

    # The table stays fixed while the person moves.  The old person cell is
    # observable but no longer hit, so it must disappear without erasing the table.
    for step in range(8):
        person_now = np.asarray([[2.0, 0.8 + 0.1 * step]], dtype=np.float32)
        layer.integrate(accumulator, (0.0, 0.0), np.vstack((table, person_now)), 0.0, 8.0)
    occupied = layer.occupied(1.10)
    assert occupied[table_cell]
    assert not occupied[person_cell]
    print("dynamic elevated obstacle clearing: PASS")


if __name__ == "__main__":
    main()
