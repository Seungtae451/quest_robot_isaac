"""Check full rotated cube footprints against the requested exclusion zone."""
import math

import numpy as np

from config import tabletop_config as cfg
from simulation.tabletop import sample_cube_poses, table_surface_height


def test_random_cubes_stay_on_box_sides_and_in_quarter_to_two_thirds_band(monkeypatch):
    monkeypatch.setattr(cfg, "CUBE_COUNT", 8)
    half = cfg.CUBE_SIZE / 2
    table_x, table_y, _ = cfg.TABLE_CENTER
    length, width, _ = cfg.TABLE_SIZE
    # Independently check each actual corner, including random cube yaw.
    right_edge_of_box = cfg.BOX_CENTER_XY[1] - cfg.BOX_SIZE[1] / 2
    left_edge_of_box = cfg.BOX_CENTER_XY[1] + cfg.BOX_SIZE[1] / 2
    used_sides = set()
    for seed in range(128):
        poses = sample_cube_poses(seed)
        assert len(poses) == 8
        bounds = []
        for (x, y, z), (w, _, _, qz) in poses:
            yaw = 2 * math.atan2(qz, w)
            c, s = math.cos(yaw), math.sin(yaw)
            corners = np.array([(x + c * dx - s * dy, y + s * dx + c * dy)
                                for dx in (-half, half) for dy in (-half, half)])
            # The near quarter must stay empty, including rotated corners.
            assert corners[:, 0].min() >= table_x - length / 2 + length / 4 + cfg.CUBE_TABLE_EDGE_MARGIN - 1e-12
            # The far third must stay empty, including rotated cube corners.
            assert corners[:, 0].max() <= table_x - length / 2 + length * (2 / 3) - cfg.CUBE_TABLE_EDGE_MARGIN + 1e-12
            assert corners[:, 1].min() >= table_y - width / 2 + cfg.CUBE_TABLE_EDGE_MARGIN - 1e-12
            assert corners[:, 1].max() <= table_y + width / 2 - cfg.CUBE_TABLE_EDGE_MARGIN + 1e-12
            if y < cfg.BOX_CENTER_XY[1]:
                assert corners[:, 1].max() <= right_edge_of_box - cfg.BOX_SPAWN_CLEARANCE + 1e-12
                used_sides.add("right")
            else:
                assert corners[:, 1].min() >= left_edge_of_box + cfg.BOX_SPAWN_CLEARANCE - 1e-12
                used_sides.add("left")
            assert z - half > table_surface_height()
            bounds.append((corners.min(axis=0), corners.max(axis=0)))
        for i, (lo, hi) in enumerate(bounds):
            for other_lo, other_hi in bounds[i + 1:]:
                assert np.any(hi <= other_lo) or np.any(other_hi <= lo)
    assert used_sides == {"left", "right"}


def test_seed_reproduces_layout_and_changes_positions():
    assert sample_cube_poses(42) == sample_cube_poses(42)
    assert sample_cube_poses(42) != sample_cube_poses(43)
