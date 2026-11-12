#!/usr/bin/env python3
"""Inspect host ports and saved 0.84-s1 feedback without opening hardware.

This is an offline diagnostic, not a recorder, robot adapter or deployment gate.
It deliberately has no serial, camera, SDK, torch or network dependencies.
"""

import argparse
import glob
import json
import math
from pathlib import Path
import platform


JOINT_FIELDS = ("b", "s", "e", "t", "r", "g")
JOINT_NAMES = ("base", "shoulder", "elbow", "wrist_pitch", "wrist_roll", "gripper")


def finite_number(value):
    if type(value) not in (int, float):
        return False
    try:
        return math.isfinite(value)
    except OverflowError:
        return False


def inspect_feedback(packet):
    """Report only what one saved guarded-firmware packet actually contains.

    Joint positions retain native firmware radians, including gripper angle.
    Loads (tB etc.) and supply voltage (v) are never interpreted as velocities.
    A file cannot establish the age of physical feedback or camera alignment.
    """
    if not isinstance(packet, dict):
        raise ValueError("Expected one JSON object containing a T=1051 response")
    problems = []
    if type(packet.get("T")) is not int or packet["T"] != 1051:
        problems.append("Expected response T=1051, not request T=105")
    if packet.get("firmware") != "0.84-s1":
        problems.append("Expected the inventoried guarded firmware 0.84-s1")
    for flag in ("feedback_valid", "motion_allowed"):
        if packet.get(flag) is not True:
            problems.append(f"{flag} is not true")
    supply = packet.get("motion_supply_v")
    if not finite_number(supply) or supply <= 0:
        problems.append("motion_supply_v must be a finite positive number")
    invalid = [key for key in JOINT_FIELDS if not finite_number(packet.get(key))]
    if invalid:
        problems.append("Missing/nonfinite joint radians: " + ", ".join(invalid))
    return {
        "saved_position_packet_ok": not problems,
        "problems": problems,
        "joint_names": list(JOINT_NAMES),
        "qpos_radians": None if invalid else [packet[key] for key in JOINT_FIELDS],
        "qvel": None,
        "freshness_verified": False,
        "flash_capture_compatible": False,
        "limitation": "0.84-s1 does not export joint velocities; saved feedback cannot verify live timing.",
    }


def host_report():
    patterns = (
        "/dev/cu.usbserial*", "/dev/cu.usbmodem*", "/dev/cu.SLAB*",
        "/dev/ttyUSB*", "/dev/ttyACM*",
    )
    return {
        "os": platform.system(),
        "architecture": platform.machine(),
        "python": platform.python_version(),
        "serial_port_candidates": sorted({port for pattern in patterns for port in glob.glob(pattern)}),
        "ports_opened": False,
        "device_identity_verified": False,
        "note": "Port names are candidates only. Match USB serial and MAC to arms.json before operation.",
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--feedback", type=Path, help="Saved single JSON feedback object; never a device path")
    args = parser.parse_args(argv)
    report = {"host": host_report(), "robot_deployment_ready": False}
    status = 0
    if args.feedback:
        # Prevent accidentally blocking on a device, FIFO or directory.
        if not args.feedback.is_file():
            parser.error("--feedback must name a regular saved JSON file")
        try:
            packet = json.loads(args.feedback.read_text(encoding="utf-8"))
            report["feedback"] = inspect_feedback(packet)
        except (OSError, ValueError) as exc:
            parser.error(str(exc))
        if not report["feedback"]["saved_position_packet_ok"]:
            status = 2
    print(json.dumps(report, indent=2, allow_nan=False))
    return status


if __name__ == "__main__":
    raise SystemExit(main())
