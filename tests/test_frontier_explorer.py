#!/usr/bin/env python3

import numpy as np

from tools.s2_frontier_explorer import approach_cell, connected_components, frontier_mask


def main() -> None:
    grid = np.full((12, 12), -1, dtype=np.int16)
    grid[3:9, 3:9] = 0
    grid[5, 5] = 100
    mask = frontier_mask(grid)
    assert not mask[5, 5]
    assert int(mask.sum()) == 20
    components = connected_components(mask)
    assert len(components) == 1
    assert len(components[0]) == 20
    selected = approach_cell(
        grid, np.asarray([3, 6]), (8.0, 6.0), standoff_cells=2.0, clearance_cells=0
    )
    assert selected == (5, 6)
    # Clearance forces the goal away from both obstacles and unknown space.
    selected = approach_cell(
        grid, np.asarray([3, 6]), (8.0, 6.0), standoff_cells=2.0, clearance_cells=1
    )
    assert selected is not None
    row, col = selected
    assert np.all(grid[row - 1 : row + 2, col - 1 : col + 2] == 0)
    print("frontier exploration geometry: PASS")


if __name__ == "__main__":
    main()
