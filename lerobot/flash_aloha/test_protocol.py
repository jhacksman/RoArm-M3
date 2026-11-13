"""Regression cases for the numeric firmware protocol, entirely offline."""

from dataclasses import FrozenInstanceError
import json
from pathlib import Path
import unittest

from protocol import decode_saved_feedback, encode_joint_target


class ProtocolTests(unittest.TestCase):
    def setUp(self):
        self.packet = json.loads((Path(__file__).parent / "fixtures/position.synthetic.json").read_text())
        # Synthetic bounds intentionally differ from actual hardware limits.
        self.options = dict(limits=[(-2.0, 2.0)] * 6, reference=[0.0] * 6,
                            max_delta=[1.0] * 6, speed=100, acceleration=10)

    def test_saved_state_is_immutable_and_has_no_fabricated_velocity(self):
        state = decode_saved_feedback(self.packet)
        self.assertEqual(state.qpos_radians, (0.1, 0.2, 0.3, 0.4, 0.5, 0.6))
        self.assertIsNone(state.qvel)
        self.packet["g"] = 9
        self.assertEqual(state.qpos_radians[-1], 0.6)
        with self.assertRaises(FrozenInstanceError):
            state.firmware = "other"

    def test_invalid_guarded_state_is_not_decoded(self):
        self.packet["feedback_valid"] = False
        with self.assertRaises(ValueError):
            decode_saved_feedback(self.packet)

    def test_target_matches_firmware_numeric_command_and_native_gripper(self):
        line = encode_joint_target([0.1, 0.2, 0.3, 0.4, 0.5, 0.6], **self.options)
        self.assertTrue(line.endswith("\n"))
        self.assertEqual(json.loads(line), {
            "T": 102, "base": 0.1, "shoulder": 0.2, "elbow": 0.3,
            "wrist": 0.4, "roll": 0.5, "hand": 0.6, "spd": 100, "acc": 10,
        })

    def test_arx_sized_incomplete_and_nonfinite_vectors_fail(self):
        for vector in ([0] * 7, [0] * 5, [0] * 12, [0, 0, 0, 0, 0, True],
                       [0, 0, 0, 0, 0, float("nan")]):
            with self.subTest(vector=vector), self.assertRaises(ValueError):
                encode_joint_target(vector, **self.options)

    def test_target_and_reference_must_both_be_inside_explicit_limits(self):
        for values in (([2.1] * 6, [0] * 6), ([0] * 6, [2.1] * 6)):
            with self.subTest(values=values), self.assertRaises(ValueError):
                encode_joint_target(values[0], **dict(self.options, reference=values[1]))

    def test_large_steps_are_rejected_instead_of_clipped(self):
        with self.assertRaises(ValueError):
            encode_joint_target([1.1] * 6, **self.options)

    def test_limits_steps_and_native_speed_options_must_be_explicit_and_valid(self):
        variants = (
            {"limits": None}, {"limits": [(-2, 2)] * 5}, {"limits": [(2, -2)] * 6},
            {"max_delta": [0] * 6}, {"max_delta": [float("inf")] * 6},
            {"speed": 0}, {"speed": True}, {"speed": 4097}, {"speed": 1.5},
            {"acceleration": 0}, {"acceleration": True}, {"acceleration": 255},
        )
        for variant in variants:
            with self.subTest(variant=variant), self.assertRaises(ValueError):
                encode_joint_target([0] * 6, **dict(self.options, **variant))


if __name__ == "__main__":
    unittest.main()
