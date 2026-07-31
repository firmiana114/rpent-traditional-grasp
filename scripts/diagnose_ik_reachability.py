#!/usr/bin/env python3
"""Diagnose exact-pose and position-only IK without commanding robot motion."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path

import numpy as np

from rpent_traditional_grasp.config import TraditionalGraspConfig
from rpent_traditional_grasp.diagnostics import diagnose_ik_reachability
from rpent_traditional_grasp.ik import TracIKProcess
from rpent_traditional_grasp.logging import configure_logging, get_logger
from rpent_traditional_grasp.models import BottleEstimate

logger = get_logger("diagnose_ik")


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Compare exact fixed-orientation IK, diagnostic position-only IK "
            "and the continuous path. This command never controls the robot."
        )
    )
    parser.add_argument("--config", default="thor.example.json")
    parser.add_argument(
        "--target-body-xyz-m",
        nargs=3,
        type=float,
        required=True,
        metavar=("X", "Y", "Z"),
    )
    parser.add_argument("--left-seed-rad", nargs=7, type=float)
    parser.add_argument("--right-seed-rad", nargs=7, type=float)
    parser.add_argument(
        "--seed-source",
        default="zero_default",
        help="Human-readable provenance included in the report.",
    )
    parser.add_argument("--ik-timeout-s", type=float, default=0.5)
    parser.add_argument(
        "--max-adaptive-subdivisions",
        type=int,
        help=(
            "Override continuous-path adaptive subdivision depth. Use 0 for "
            "a bounded first-failure diagnostic."
        ),
    )
    parser.add_argument("--skip-continuous", action="store_true")
    parser.add_argument("--output-json")
    return parser


def main() -> int:
    args = _parser().parse_args()
    configure_logging()
    root = Path(__file__).resolve().parents[1]
    config = TraditionalGraspConfig.from_json(args.config)
    if args.ik_timeout_s <= 0.0:
        raise ValueError("--ik-timeout-s 必须大于 0")
    config.planner.ik_timeout_s = args.ik_timeout_s
    if args.max_adaptive_subdivisions is not None:
        if args.max_adaptive_subdivisions < 0:
            raise ValueError("--max-adaptive-subdivisions 不能为负数")
        config.planner.max_adaptive_subdivisions = (
            args.max_adaptive_subdivisions
        )
    target = np.asarray(args.target_body_xyz_m, dtype=np.float64)
    estimate = _estimate(target)
    seeds = {
        "left": np.asarray(args.left_seed_rad or np.zeros(7)),
        "right": np.asarray(args.right_seed_rad or np.zeros(7)),
    }
    logger.info(
        "开始无运动 IK 诊断: target=[%.4f,%.4f,%.4f] seed_source=%s",
        *target,
        args.seed_source,
    )
    try:
        with (
            TracIKProcess(
                root / config.resources.left_ik_binary,
                root / "robot/chains/g1_left_arm.chain",
                "left",
                args.ik_timeout_s,
                config.planner.ik_tolerance,
                response_timeout_s=max(2.0, args.ik_timeout_s * 4.0),
            ) as left,
            TracIKProcess(
                root / config.resources.right_ik_binary,
                root / "robot/chains/g1_right_arm.chain",
                "right",
                args.ik_timeout_s,
                config.planner.ik_tolerance,
                response_timeout_s=max(2.0, args.ik_timeout_s * 4.0),
            ) as right,
        ):
            chain_files = {
                "left": root / "robot/chains/g1_left_arm.chain",
                "right": root / "robot/chains/g1_right_arm.chain",
            }
            report = diagnose_ik_reachability(
                estimate=estimate,
                solvers={"left": left, "right": right},
                seeds=seeds,
                seed_source=args.seed_source,
                config=config.planner,
                chain_files=chain_files,
                include_continuous=not args.skip_continuous,
            )
        _emit(report, args.output_json)
        return 0
    except Exception as exc:
        logger.exception(
            "无运动 IK 诊断失败: target=%s seed_source=%s",
            target.tolist(),
            args.seed_source,
        )
        _emit(
            {
                "schema_version": 1,
                "stage": "ik_reachability_diagnostic",
                "motion_commanded": False,
                "success": False,
                "error": f"{type(exc).__name__}: {exc}",
            },
            args.output_json,
        )
        return 2


def _estimate(target: np.ndarray) -> BottleEstimate:
    return BottleEstimate(
        class_name="bottle",
        confidence=1.0,
        bbox_xyxy=(0, 0, 1, 1),
        center_uv=(0.0, 0.0),
        front_center_camera_m=np.zeros(3),
        center_camera_m=np.zeros(3),
        center_body_m=target,
        axis_camera=np.array([0.0, 1.0, 0.0]),
        diameter_m=0.060958079929906046,
        front_depth_m=0.5673544406890869,
        depth_mad_m=0.007152736186981201,
        valid_depth_pixels=1071,
    )


def _emit(report: dict[str, object], output_json: str | None) -> None:
    rendered = json.dumps(report, ensure_ascii=False, indent=2)
    if output_json:
        path = Path(output_json)
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_suffix(path.suffix + ".tmp")
        try:
            temporary.write_text(rendered + "\n", encoding="utf-8")
            os.replace(temporary, path)
        except OSError as exc:
            logger.exception("写入 IK 诊断结果失败: path=%s", path)
            raise RuntimeError(f"无法写入 IK 诊断 JSON: {path}") from exc
        logger.info("IK 诊断结果已写入: path=%s", path)
    print(rendered)


if __name__ == "__main__":
    raise SystemExit(main())
