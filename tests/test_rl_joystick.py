from __future__ import annotations

import unittest
from unittest.mock import patch

import numpy as np

from src.locomotion import VelocityCommand
from src.rl.joystick_control import (
    NeutralResidualGate,
    _RLModeRouter,
    _VelocityMailbox,
)
from src.rl.stance import STANDARD_STANDING_DEGREES


class _FakeModeRobot:
    def __init__(self) -> None:
        self.initialized = False
        self.mode_name = "walk"
        self.controller = None
        self.profile = None

    def change_mode(self) -> str:
        self.mode_name = {
            "walk": "drive",
            "drive": "climb",
            "climb": "walk",
        }[self.mode_name]
        return self.mode_name

    def forward(self) -> None:  # pragma: no cover - neutral test path
        pass

    backward = forward
    left = forward
    right = forward


class NeutralResidualGateTests(unittest.TestCase):
    def test_neutral_start_ignores_policy_bias(self) -> None:
        gate = NeutralResidualGate()

        action = gate.apply([0.0, 0.0, 0.0], np.ones(18), dt=0.02)

        np.testing.assert_array_equal(action, np.zeros(18))

    def test_active_command_passes_policy_action(self) -> None:
        gate = NeutralResidualGate()
        expected = np.linspace(-1.0, 1.0, 18, dtype=np.float32)

        action = gate.apply([0.2, 0.0, 0.0], expected, dt=0.02)

        np.testing.assert_allclose(action, expected)

    def test_neutral_command_decays_previous_action_to_exact_zero(self) -> None:
        gate = NeutralResidualGate(decay_seconds=0.05)
        gate.apply([0.2, 0.0, 0.0], np.ones(18), dt=0.02)

        actions = [
            gate.apply([0.0, 0.0, 0.0], np.ones(18), dt=0.02)
            for _ in range(20)
        ]

        self.assertLess(float(np.linalg.norm(actions[0])), np.sqrt(18.0))
        np.testing.assert_array_equal(actions[-1], np.zeros(18))


class RLModeRouterTests(unittest.TestCase):
    def test_r_cycles_rl_walk_drive_climb_and_back_to_policy(self) -> None:
        fake_robot = _FakeModeRobot()
        mailbox = _VelocityMailbox()
        with patch("src.rl.joystick_control.SCONE", return_value=fake_robot):
            router = _RLModeRouter(
                object(),
                STANDARD_STANDING_DEGREES,
                mailbox,
            )
        try:
            router.apply_command(VelocityCommand(vx=0.2), 0.02)
            self.assertEqual(mailbox.read().vx, 0.2)

            self.assertTrue(router.handle_key("r"))
            self.assertEqual(router.control_name(), "old/drive")
            self.assertFalse(router.policy_active())

            self.assertTrue(router.handle_key("r"))
            self.assertEqual(router.control_name(), "old/climb")

            self.assertTrue(router.handle_key("r"))
            self.assertEqual(router.control_name(), "rl/walk")
            self.assertTrue(router.policy_active())
            self.assertTrue(router.consume_resume_pending())
            self.assertFalse(router.consume_resume_pending())
        finally:
            router.close()


if __name__ == "__main__":
    unittest.main()
