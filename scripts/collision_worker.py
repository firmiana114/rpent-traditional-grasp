#!/usr/bin/env python3
"""Persistent worker exposing the parent project's self-collision checker.

The planning service and the parent project run in different interpreters and
only the parent's has pinocchio/hpp-fcl, so the authoritative checker is reached
over a line-delimited JSON pipe instead of an import. Reusing the parent's own
class is deliberate: a pre-filter that disagreed with the gate that actually
vetoes execution would be worse than no pre-filter at all.

Protocol: one JSON request per stdin line, one JSON reply per stdout line.
  request  {"positions": [[7 floats], ...], "arm_side": "left", "current_q": [14 floats]}
  reply    {"ok": true, "result": {...parent evidence dict...}}
           {"ok": false, "error": "..."}
  "QUIT" on its own line exits.
This process never commands motion; it only evaluates geometry.
"""

from __future__ import annotations

import argparse
import importlib
import json
import sys
import traceback


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Serve the parent project's self-collision checker over stdio. "
            "This command never moves the robot."
        ),
    )
    parser.add_argument(
        "--repo",
        required=True,
        help="parent project root to place on sys.path",
    )
    parser.add_argument("--module", default="robots.air_robot.collision")
    parser.add_argument("--class-name", default="PinocchioSelfCollisionChecker")
    parser.add_argument(
        "--urdf",
        default="",
        help="collision URDF; empty uses the checker's own default",
    )
    return parser


def main() -> int:
    args = _build_parser().parse_args()
    sys.path.insert(0, args.repo)
    try:
        module = importlib.import_module(args.module)
        checker_class = getattr(module, args.class_name)
        checker = checker_class(args.urdf or None)
    except Exception as exc:
        print(f"ERR {type(exc).__name__}: {exc}", flush=True)
        traceback.print_exc(file=sys.stderr)
        return 2

    ready = {
        "checker": args.class_name,
        "urdf_sha256": getattr(checker, "urdf_sha256", ""),
        "collision_pair_count": len(checker.collision_model.collisionPairs),
        "sample_step_rad": getattr(checker, "sample_step_rad", None),
    }
    print("READY " + json.dumps(ready, ensure_ascii=False), flush=True)

    for line in sys.stdin:
        command = line.strip()
        if not command or command == "QUIT":
            break
        try:
            request = json.loads(command)
            result = checker.check_path(
                arm_side=str(request["arm_side"]),
                positions=[[float(v) for v in row] for row in request["positions"]],
                current_q=[float(v) for v in request["current_q"]],
            )
            reply: dict[str, object] = {"ok": True, "result": result}
        except Exception as exc:
            reply = {"ok": False, "error": f"{type(exc).__name__}: {exc}"}
            traceback.print_exc(file=sys.stderr)
        print(json.dumps(reply, ensure_ascii=False), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
