#!/usr/bin/env python3
"""Run real Thor perception and IK without authorizing robot motion."""

from __future__ import annotations

import argparse
import json

from rpent_traditional_grasp.logging import configure_logging
from rpent_traditional_grasp.thor import build_thor_shadow_api


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--config",
        default="thor.example.json",
        help="Traditional grasp JSON configuration.",
    )
    parser.add_argument("--target", default="bottle")
    parser.add_argument("--arm", choices=["auto", "left", "right"], default="auto")
    parser.add_argument(
        "--operation",
        choices=["search", "pick"],
        default="search",
        help="pick runs perception and IK but uses a simulated arm.",
    )
    parser.add_argument("--host", default="192.168.123.164")
    parser.add_argument("--port", type=int, default=55555)
    parser.add_argument(
        "--left-image",
        help="Existing left image. Must be provided together with --right-image.",
    )
    parser.add_argument(
        "--right-image",
        help="Existing right image. Must be provided together with --left-image.",
    )
    parser.add_argument(
        "--online-camera",
        action="store_true",
        help="Explicitly opt in to online ZMQ capture instead of image files.",
    )
    args = parser.parse_args()
    if bool(args.left_image) != bool(args.right_image):
        parser.error("--left-image and --right-image must be provided together")
    if args.online_camera and args.left_image:
        parser.error("--online-camera cannot be combined with image files")
    if not args.online_camera and not args.left_image:
        parser.error(
            "provide --left-image and --right-image; online capture requires "
            "--online-camera"
        )
    configure_logging()
    with build_thor_shadow_api(
        args.config,
        host=args.host,
        port=args.port,
        left_image=args.left_image,
        right_image=args.right_image,
        online_camera=args.online_camera,
    ) as api:
        if args.operation == "search":
            result = api.search_object(object_prompt=args.target)
        else:
            result = api.pick_object(
                object_prompt=args.target,
                arm_side=args.arm,
            )
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result.get("success") or result.get("planned") else 1


if __name__ == "__main__":
    raise SystemExit(main())
