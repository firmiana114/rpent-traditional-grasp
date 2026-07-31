#!/usr/bin/env python3
"""Run no-motion stereo-image-to-object or gripper XYZ acceptance tests."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
from typing import Any

from rpent_traditional_grasp.logging import configure_logging, get_logger
from rpent_traditional_grasp.thor import build_thor_shadow_api
from rpent_traditional_grasp.xyz import (
    build_gripper_xyz_report,
    build_xyz_report,
)

logger = get_logger("image_to_xyz")


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Read one synchronized stereo image pair and emit object or gripper XYZ. "
            "This command never opens the online camera or sends robot motion."
        ),
    )
    parser.add_argument("--left-image", required=True)
    parser.add_argument("--right-image", required=True)
    parser.add_argument("--config", default="thor.example.json")
    parser.add_argument("--target", default="bottle")
    parser.add_argument(
        "--result-kind",
        choices=["object_xyz", "gripper_xyz"],
        default="object_xyz",
        help="Emit the object center or the final gripper TCP target.",
    )
    parser.add_argument(
        "--arm",
        choices=["auto", "left", "right"],
        default="auto",
        help="Gripper arm selection for gripper_xyz output.",
    )
    parser.add_argument(
        "--bbox",
        nargs=4,
        type=float,
        metavar=("X1", "Y1", "X2", "Y2"),
        help="Optional target box; bypasses YOLO-World detection.",
    )
    parser.add_argument(
        "--bbox-format",
        choices=["auto", "pixel", "norm01"],
        default="auto",
    )
    parser.add_argument(
        "--expected-body-xyz-m",
        nargs=3,
        type=float,
        metavar=("X", "Y", "Z"),
        help="Optional measured body-frame truth used for pass/fail.",
    )
    parser.add_argument(
        "--expected-gripper-xyz-m",
        nargs=3,
        type=float,
        metavar=("X", "Y", "Z"),
        help="Optional measured final gripper TCP truth used for pass/fail.",
    )
    parser.add_argument(
        "--tolerance-m",
        type=float,
        default=0.03,
        help="Maximum Euclidean XYZ error when ground truth is supplied.",
    )
    parser.add_argument(
        "--output-json",
        help="Optional path for an atomic, diagnostics-free JSON result.",
    )
    return parser


def main() -> int:
    parser = _build_parser()
    args = parser.parse_args()
    if (
        args.result_kind == "object_xyz"
        and args.expected_gripper_xyz_m is not None
    ):
        parser.error("--expected-gripper-xyz-m 仅用于 gripper_xyz")
    if args.result_kind == "gripper_xyz" and args.expected_body_xyz_m is not None:
        parser.error("--expected-body-xyz-m 仅用于 object_xyz")
    configure_logging()
    logger.info(
        "开始图片到 XYZ 测试: kind=%s left=%s right=%s target=%s bbox=%s",
        args.result_kind,
        args.left_image,
        args.right_image,
        args.target,
        args.bbox is not None,
    )
    try:
        with build_thor_shadow_api(
            args.config,
            left_image=args.left_image,
            right_image=args.right_image,
            perception_only=True,
        ) as api:
            if args.result_kind == "gripper_xyz":
                result = api.preview_pick_object_xyz(
                    object_prompt=args.target,
                    arm_side=args.arm,
                    bbox=args.bbox,
                    bbox_format=args.bbox_format,
                )
            else:
                result = api.search_object(
                    object_prompt=args.target,
                    bbox=args.bbox,
                    bbox_format=args.bbox_format,
                )
            if not result.get("found"):
                report: dict[str, Any] = {
                    "schema_version": 1,
                    "stage": (
                        "stereo_image_to_gripper_xyz"
                        if args.result_kind == "gripper_xyz"
                        else "stereo_image_to_xyz"
                    ),
                    "success": False,
                    "reason": result.get("reason", "target_not_found"),
                    (
                        "pick_object"
                        if args.result_kind == "gripper_xyz"
                        else "search_object"
                    ): result,
                }
            else:
                estimate = api.context.estimate
                if estimate is None:
                    raise RuntimeError("搜索成功但缺少瓶体几何估计")
                common = {
                    "estimate": estimate,
                    "left_image": Path(args.left_image),
                    "right_image": Path(args.right_image),
                    "target": args.target,
                    "stereo_calibration_validated": (
                        api.config.safety.stereo_calibration_validated
                    ),
                    "camera_to_body_validated": (
                        api.config.safety.camera_to_body_validated
                    ),
                    "tolerance_m": args.tolerance_m,
                }
                if args.result_kind == "gripper_xyz":
                    report = build_gripper_xyz_report(
                        **common,
                        requested_arm=args.arm,
                        planner_config=api.config.planner,
                        gripper_tcp_calibration_validated=(
                            api.config.safety.gripper_tcp_calibration_validated
                        ),
                        expected_gripper_xyz_m=args.expected_gripper_xyz_m,
                    )
                    if (
                        result.get("final_tcp_body_xyz_m")
                        != report["gripper_target"]["final_tcp_body_xyz_m"]
                    ):
                        raise RuntimeError(
                            "pick_object 与结构化报告的最终夹爪 XYZ 不一致"
                        )
                    report["entrypoint"] = "pick_object"
                else:
                    report = build_xyz_report(
                        **common,
                        expected_body_xyz_m=args.expected_body_xyz_m,
                    )
    except Exception as exc:
        logger.exception(
            "图片到 XYZ 测试失败: kind=%s left=%s right=%s target=%s",
            args.result_kind,
            args.left_image,
            args.right_image,
            args.target,
        )
        report = {
            "schema_version": 1,
            "stage": (
                "stereo_image_to_gripper_xyz"
                if args.result_kind == "gripper_xyz"
                else "stereo_image_to_xyz"
            ),
            "success": False,
            "reason": "pipeline_error",
            "error": f"{type(exc).__name__}: {exc}",
        }
        _emit_report(report, args.output_json)
        return 2

    _emit_report(report, args.output_json)
    return 0 if report["success"] else 1


def _emit_report(report: dict[str, Any], output_json: str | None) -> None:
    rendered = json.dumps(report, ensure_ascii=False, indent=2)
    if output_json:
        path = Path(output_json)
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_suffix(path.suffix + ".tmp")
        try:
            temporary.write_text(rendered + "\n", encoding="utf-8")
            os.replace(temporary, path)
        except OSError as exc:
            logger.exception("写入图片到 XYZ 结果失败: path=%s", path)
            raise RuntimeError(f"无法写入 XYZ JSON: {path}") from exc
        logger.info("图片到 XYZ 结构化结果已写入: path=%s", path)
    print(rendered)


if __name__ == "__main__":
    raise SystemExit(main())
