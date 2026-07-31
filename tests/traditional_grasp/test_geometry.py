from __future__ import annotations

import numpy as np
import pytest

from rpent_traditional_grasp.config import PerceptionConfig
from rpent_traditional_grasp.geometry import estimate_bottle_center
from rpent_traditional_grasp.models import Detection


def test_label_band_estimates_front_depth_and_geometric_center() -> None:
    height, width = 240, 320
    mask = np.zeros((height, width), dtype=bool)
    mask[50:210, 140:180] = True
    depth = np.full((height, width), np.nan, dtype=np.float32)
    depth[mask] = 0.6
    depth[100:105, 150:155] = 1.7
    projection = np.array(
        [[600.0, 0.0, 160.0, 0.0], [0.0, 600.0, 120.0, 0.0], [0, 0, 1, 0]]
    )
    detection = Detection("bottle", 0.91, (140, 50, 180, 210))

    estimate = estimate_bottle_center(
        detection,
        mask,
        depth,
        projection,
        np.eye(4),
        PerceptionConfig(),
    )

    assert estimate.front_depth_m == pytest.approx(0.6, abs=1e-6)
    assert estimate.diameter_m == pytest.approx(0.04, abs=0.003)
    assert estimate.center_camera_m[2] > estimate.front_center_camera_m[2]
    assert estimate.depth_mad_m == pytest.approx(0.0)


def test_label_band_rejects_unstable_depth() -> None:
    mask = np.zeros((200, 200), dtype=bool)
    mask[20:180, 80:120] = True
    depth = np.full((200, 200), np.nan, dtype=np.float32)
    alternating = np.indices((160, 40)).sum(axis=0) % 2
    depth[20:180, 80:120] = 0.4 + alternating * 0.3
    projection = np.array(
        [[600.0, 0, 100.0], [0, 600.0, 100.0], [0, 0, 1.0]]
    )

    with pytest.raises(ValueError, match="深度离散过大"):
        estimate_bottle_center(
            Detection("bottle", 0.9, (80, 20, 120, 180)),
            mask,
            depth,
            projection,
            np.eye(4),
            PerceptionConfig(),
        )
