#!/usr/bin/env python3
"""Export the exact G1 torso-to-TCP arm chains for the standalone solver."""

from __future__ import annotations

import argparse
import hashlib
import logging
import xml.etree.ElementTree as ET
from pathlib import Path

EXPECTED_URDF_SHA256 = (
    "8bbf006633fc50b616f665c7a970780cc296577a0adfd7d28b049e751c238735"
)
LOGGER = logging.getLogger("export_g1_chains")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("urdf", type=Path)
    parser.add_argument("output_dir", type=Path)
    parser.add_argument("--tip-offset-m", type=float, default=0.05)
    parser.add_argument(
        "--allow-unknown-urdf",
        action="store_true",
        help="Allow a URDF hash other than the Thor-verified model.",
    )
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="[%(name)s] %(message)s")
    export_chains(
        args.urdf,
        args.output_dir,
        args.tip_offset_m,
        args.allow_unknown_urdf,
    )


def export_chains(
    urdf_path: Path,
    output_dir: Path,
    tip_offset_m: float,
    allow_unknown_urdf: bool = False,
) -> None:
    try:
        urdf_bytes = urdf_path.read_bytes()
        digest = hashlib.sha256(urdf_bytes).hexdigest()
        if digest != EXPECTED_URDF_SHA256 and not allow_unknown_urdf:
            raise ValueError(
                "G1 URDF 哈希与 Thor 已核实版本不一致: "
                f"actual={digest} expected={EXPECTED_URDF_SHA256}"
            )
        root = ET.fromstring(urdf_bytes)
    except (OSError, ET.ParseError, ValueError) as exc:
        LOGGER.exception("读取 G1 URDF 失败: path=%s", urdf_path)
        raise RuntimeError(f"无法导出 G1 运动链: {urdf_path}") from exc

    joints_by_child = {
        joint.find("child").attrib["link"]: joint
        for joint in root.findall("joint")
        if joint.find("child") is not None
    }
    output_dir.mkdir(parents=True, exist_ok=True)
    for arm in ("left", "right"):
        chain = _trace_chain(
            joints_by_child,
            base_link="torso_link",
            tip_link=f"{arm}_wrist_yaw_link",
        )
        movable = [joint for joint in chain if joint.attrib["type"] != "fixed"]
        if len(movable) != 7:
            raise RuntimeError(
                f"{arm} 运动链必须有 7 个活动关节，实际 {len(movable)}"
            )
        output_path = output_dir / f"g1_{arm}_arm.chain"
        lines = [
            "RPENT_G1_CHAIN_V1",
            "BASE torso_link",
            f"ARM {arm}",
        ]
        lines.extend(_joint_line(joint) for joint in chain)
        lines.append(
            f"FIXED {arm}_tcp {arm}_tcp_link "
            f"{tip_offset_m:.17g} 0 0 0 0 0"
        )
        lines.append("END")
        output_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
        LOGGER.info(
            "已导出 G1 七轴运动链: arm=%s joints=%d output=%s urdf_sha256=%s",
            arm,
            len(movable),
            output_path,
            digest,
        )


def _trace_chain(
    joints_by_child: dict[str, ET.Element],
    base_link: str,
    tip_link: str,
) -> list[ET.Element]:
    chain: list[ET.Element] = []
    current = tip_link
    visited: set[str] = set()
    while current != base_link:
        if current in visited:
            raise RuntimeError(f"URDF 运动链存在环: link={current}")
        visited.add(current)
        joint = joints_by_child.get(current)
        if joint is None:
            raise RuntimeError(
                f"无法从 {tip_link} 回溯到 {base_link}: missing_child={current}"
            )
        chain.append(joint)
        parent = joint.find("parent")
        if parent is None:
            raise RuntimeError(f"关节缺少 parent: {joint.attrib.get('name')}")
        current = parent.attrib["link"]
    chain.reverse()
    return chain


def _joint_line(joint: ET.Element) -> str:
    name = joint.attrib["name"]
    type_ = joint.attrib["type"]
    child_element = joint.find("child")
    if child_element is None:
        raise RuntimeError(f"关节缺少 child: {name}")
    child = child_element.attrib["link"]
    origin = joint.find("origin")
    xyz = _triple(origin.attrib.get("xyz") if origin is not None else None, "0 0 0")
    rpy = _triple(origin.attrib.get("rpy") if origin is not None else None, "0 0 0")
    if type_ == "fixed":
        return f"FIXED {name} {child} {' '.join((*xyz, *rpy))}"
    if type_ not in {"revolute", "continuous"}:
        raise RuntimeError(f"不支持的关节类型: name={name} type={type_}")
    axis_element = joint.find("axis")
    axis = _triple(
        axis_element.attrib.get("xyz") if axis_element is not None else None,
        "1 0 0",
    )
    if type_ == "continuous":
        lower, upper = "-3.141592653589793", "3.141592653589793"
    else:
        limit = joint.find("limit")
        if limit is None or "lower" not in limit.attrib or "upper" not in limit.attrib:
            raise RuntimeError(f"活动关节缺少上下限: {name}")
        lower, upper = limit.attrib["lower"], limit.attrib["upper"]
    return (
        f"REVOLUTE {name} {child} {' '.join((*xyz, *rpy, *axis))} "
        f"{lower} {upper}"
    )


def _triple(value: str | None, default: str) -> tuple[str, str, str]:
    parts = (value or default).split()
    if len(parts) != 3:
        raise RuntimeError(f"预期三个数值，实际: {value!r}")
    return parts[0], parts[1], parts[2]


if __name__ == "__main__":
    main()
