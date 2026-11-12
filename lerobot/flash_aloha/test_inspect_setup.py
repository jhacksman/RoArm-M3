"""Offline regressions for false-valid feedback and joint ordering."""

import copy
import unittest

from inspect_setup import inspect_feedback


def position_packet():
    # Synthetic format fixture, not evidence of a connected or tested arm.
    return dict(T=1051, firmware="0.84-s1", feedback_valid=True,
                motion_allowed=True, motion_supply_v=12.0,
                b=0.1, s=0.2, e=0.3, t=0.4, r=0.5, g=0.6)


class FeedbackTests(unittest.TestCase):
    def test_native_order_and_gripper_are_preserved(self):
        packet = position_packet()
        before = copy.deepcopy(packet)
        report = inspect_feedback(packet)
        self.assertEqual(report["qpos_radians"], [0.1, 0.2, 0.3, 0.4, 0.5, 0.6])
        self.assertTrue(report["saved_position_packet_ok"])
        self.assertEqual(packet, before)
        self.assertFalse(report["flash_capture_compatible"])
        self.assertFalse(report["freshness_verified"])

    def test_invalid_guard_flags_are_rejected(self):
        for name in ("feedback_valid", "motion_allowed"):
            for value in (False, None, 1, "true"):
                with self.subTest(name=name, value=value):
                    packet = position_packet()
                    packet[name] = value
                    self.assertFalse(inspect_feedback(packet)["saved_position_packet_ok"])

    def test_missing_null_boolean_and_nonfinite_positions_are_rejected(self):
        for value in (None, True, "0.1", float("nan"), float("inf"), float("-inf"), 10**400):
            with self.subTest(value=value):
                packet = position_packet()
                packet["b"] = value
                report = inspect_feedback(packet)
                self.assertFalse(report["saved_position_packet_ok"])
                self.assertIsNone(report["qpos_radians"])
        packet = position_packet()
        del packet["g"]
        self.assertFalse(inspect_feedback(packet)["saved_position_packet_ok"])

    def test_voltage_is_neither_velocity_nor_proof_of_readiness(self):
        packet = position_packet()
        packet.update(v=1200, tB=100, tS=100, qvel=[0.0] * 6)
        report = inspect_feedback(packet)
        self.assertIsNone(report["qvel"])
        self.assertFalse(report["flash_capture_compatible"])

    def test_wrong_response_or_firmware_is_rejected(self):
        for name, value in (("T", 105), ("T", 1051.0), ("T", True), ("firmware", "0.84")):
            with self.subTest(name=name, value=value):
                packet = position_packet()
                packet[name] = value
                self.assertFalse(inspect_feedback(packet)["saved_position_packet_ok"])

    def test_zero_or_invalid_supply_is_rejected(self):
        for value in (0, -1, None, True, float("nan"), float("inf")):
            with self.subTest(value=value):
                packet = position_packet()
                packet["motion_supply_v"] = value
                self.assertFalse(inspect_feedback(packet)["saved_position_packet_ok"])

    def test_non_object_is_rejected(self):
        for value in ([], None, 0):
            with self.subTest(value=value), self.assertRaises(ValueError):
                inspect_feedback(value)


if __name__ == "__main__":
    unittest.main()
