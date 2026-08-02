"""Tests for the self-collision pre-filter that reuses the parent's checker."""

from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np
import pytest

from rpent_traditional_grasp.collision import (
    SubprocessSelfCollisionChecker,
    build_collision_checker,
)
from rpent_traditional_grasp.config import ResourceConfig
from rpent_traditional_grasp.models import IKPath

WORKER = Path(__file__).resolve().parents[2] / "scripts" / "collision_worker.py"


def _path(arm: str = "left", points: int = 3) -> IKPath:
    return IKPath(
        arm=arm,
        joint_names=tuple(f"{arm}_j{i}" for i in range(7)),
        positions=[np.zeros(7) for _ in range(points)],
        waypoint_names=["pregrasp", "grasp", "retreat"][:points],
        score=0.0,
        max_joint_step_rad=0.0,
    )


def _fake_parent(tmp_path: Path, *, safe: bool) -> Path:
    """Write a stand-in for the parent package so no pinocchio is needed."""
    package = tmp_path / "robots" / "air_robot"
    package.mkdir(parents=True)
    (tmp_path / "robots" / "__init__.py").write_text("")
    (package / "__init__.py").write_text("")
    (package / "collision.py").write_text(
        "class _Model:\n"
        "    collisionPairs = [1, 2, 3]\n"
        "\n"
        "class PinocchioSelfCollisionChecker:\n"
        "    urdf_sha256 = 'deadbeef'\n"
        "    sample_step_rad = 0.05\n"
        "    collision_model = _Model()\n"
        "    def __init__(self, urdf=None):\n"
        "        self.urdf = urdf\n"
        "    def check_path(self, *, arm_side, positions, current_q):\n"
        "        assert len(current_q) == 14\n"
        "        assert arm_side in ('left', 'right')\n"
        f"        safe = {safe!r}\n"
        "        if safe:\n"
        "            return {'safe': True, 'detail': 'clean',\n"
        "                    'sample_count': len(positions)}\n"
        "        return {'safe': False, 'detail': 'self collision: a <-> b',\n"
        "                'sample_index': 2, 'sample_count': len(positions)}\n",
        encoding="utf-8",
    )
    return tmp_path


def _checker(tmp_path: Path, *, safe: bool) -> SubprocessSelfCollisionChecker:
    return SubprocessSelfCollisionChecker(
        sys.executable,
        _fake_parent(tmp_path, safe=safe),
        lambda: np.zeros(14),
        worker_script=WORKER,
    )


def test_worker_reports_collision_free_path(tmp_path: Path) -> None:
    with _checker(tmp_path, safe=True) as checker:
        safe, detail = checker.check_path(_path())
    assert safe is True
    assert detail == "clean"


def test_worker_reports_collision_with_sample_index(tmp_path: Path) -> None:
    with _checker(tmp_path, safe=False) as checker:
        safe, detail = checker.check_path(_path())
    assert safe is False
    assert "self collision: a <-> b" in detail
    # The sample index is what makes a rejection actionable rather than opaque.
    assert "sample 2" in detail


def test_handshake_exposes_parent_checker_metadata(tmp_path: Path) -> None:
    with _checker(tmp_path, safe=True) as checker:
        assert checker.metadata["checker"] == "PinocchioSelfCollisionChecker"
        assert checker.metadata["urdf_sha256"] == "deadbeef"
        assert checker.metadata["collision_pair_count"] == 3


def test_check_path_rejects_malformed_joint_state(tmp_path: Path) -> None:
    checker = SubprocessSelfCollisionChecker(
        sys.executable,
        _fake_parent(tmp_path, safe=True),
        lambda: np.zeros(7),
        worker_script=WORKER,
    )
    with pytest.raises(ValueError, match="14"):
        checker.check_path(_path())
    checker.close()


def test_builder_returns_none_when_unconfigured() -> None:
    resources = ResourceConfig()
    assert resources.collision_checker_python == ""
    assert build_collision_checker(resources, lambda: np.zeros(14)) is None


def test_builder_degrades_instead_of_raising_on_bad_interpreter(
    tmp_path: Path,
) -> None:
    resources = ResourceConfig(
        collision_checker_python=str(tmp_path / "missing-python"),
        collision_checker_repo=str(tmp_path),
    )
    # Planning must survive a missing checker; the parent still gates execution.
    assert build_collision_checker(resources, lambda: np.zeros(14)) is None


def test_worker_survives_a_bad_request(tmp_path: Path) -> None:
    checker = _checker(tmp_path, safe=True)
    checker._start()
    reply = checker._request(json.dumps({"arm_side": "left"}))
    assert reply["ok"] is False
    # A malformed request must not take the worker down with it.
    safe, _ = checker.check_path(_path())
    assert safe is True
    checker.close()
