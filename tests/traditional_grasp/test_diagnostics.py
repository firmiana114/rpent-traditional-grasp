from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from rpent_traditional_grasp.diagnostics import load_arm_chain_geometry


def test_exported_chain_geometry_has_expected_torso_reach_bound() -> None:
    root = Path(__file__).resolve().parents[2]

    left = load_arm_chain_geometry(
        root / "robot/chains/g1_left_arm.chain",
        "left",
    )
    right = load_arm_chain_geometry(
        root / "robot/chains/g1_right_arm.chain",
        "right",
    )

    assert left.root_frame == right.root_frame == "torso_link"
    np.testing.assert_allclose(
        left.shoulder_body_xyz_m,
        [0.0039563, 0.10022, 0.24778],
    )
    np.testing.assert_allclose(
        right.shoulder_body_xyz_m,
        [0.0039563, -0.10021, 0.24778],
    )
    assert left.serial_length_upper_bound_m == pytest.approx(
        0.4603940645,
        abs=1e-10,
    )
    assert right.serial_length_upper_bound_m == pytest.approx(
        left.serial_length_upper_bound_m,
        abs=1e-12,
    )
