"""Pure RoArm JSON helpers for offline integration development.

No transport, controller, recorder or policy runner is implemented here.
Caller-supplied bounds are test inputs until measured on the actual hardware.
"""

from dataclasses import dataclass
import json

from inspect_setup import JOINT_FIELDS, finite_number, inspect_feedback


COMMAND_FIELDS = ("base", "shoulder", "elbow", "wrist", "roll", "hand")


@dataclass(frozen=True)
class SavedPosition:
    qpos_radians: tuple
    supply_voltage: float
    firmware: str
    # Legacy firmware exports no velocity, sample sequence or sample time.
    qvel: None = None


def decode_saved_feedback(packet):
    """Decode one saved guarded packet, without asserting live freshness."""
    report = inspect_feedback(packet)
    if not report["saved_position_packet_ok"]:
        raise ValueError("; ".join(report["problems"]))
    return SavedPosition(tuple(float(packet[key]) for key in JOINT_FIELDS),
                         float(packet["motion_supply_v"]), packet["firmware"])


def _vector(value, name):
    if not isinstance(value, (list, tuple)) or len(value) != 6:
        raise ValueError(f"{name} requires exactly six native joint values")
    if not all(finite_number(item) for item in value):
        raise ValueError(f"{name} requires finite numeric values")
    return tuple(float(item) for item in value)


def encode_joint_target(target, *, limits, reference, max_delta, speed, acceleration):
    """Construct a T=102 JSON line without sending or queuing it.

    All six joint limits and allowed steps must be supplied explicitly. No ARX
    width conversion, wrapping, padding, clipping or default pose is applied.
    ``reference`` must eventually come from fresh measured state in the live
    adapter; this pure function cannot verify its origin or timestamp.
    """
    target = _vector(target, "target")
    reference = _vector(reference, "reference")
    max_delta = _vector(max_delta, "max_delta")
    if not isinstance(limits, (list, tuple)) or len(limits) != 6:
        raise ValueError("limits requires six explicit (minimum, maximum) pairs")
    for index, (value, previous, delta, bounds) in enumerate(zip(target, reference, max_delta, limits)):
        if (not isinstance(bounds, (list, tuple)) or len(bounds) != 2
                or not all(finite_number(item) for item in bounds) or bounds[0] >= bounds[1]):
            raise ValueError(f"Invalid limits for joint {index}")
        if not bounds[0] <= value <= bounds[1] or not bounds[0] <= previous <= bounds[1]:
            raise ValueError(f"Target/reference outside limits for joint {index}")
        if delta <= 0 or abs(value - previous) > delta:
            raise ValueError(f"Target step exceeds max_delta for joint {index}")
    # The documented SDK ranges exclude zero; firmware zero can select a fast
    # native mode and must never become an implicit default during integration.
    if type(speed) is not int or not 1 <= speed <= 4096:
        raise ValueError("speed must be an integer from 1 to 4096")
    if type(acceleration) is not int or not 1 <= acceleration <= 254:
        raise ValueError("acceleration must be an integer from 1 to 254")
    command = dict(T=102, **dict(zip(COMMAND_FIELDS, target)), spd=speed, acc=acceleration)
    return json.dumps(command, separators=(",", ":"), allow_nan=False) + "\n"
