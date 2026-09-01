from __future__ import annotations

import unittest
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

from src.locomotion import VelocityCommand
from src.rl.joystick_control import (
    NeutralResidualGate,
    SconeHybridControlConfig,
    SconeHybridController,
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


class SconeHybridControllerTests(unittest.TestCase):
    class _FakeEnv:
        _motion_profile = object()
        model_path = object()
        control_dt = 0.02
        _phase = 0.25
        default_degrees = np.arange(18, dtype=np.float64)

        def __init__(self) -> None:
            self.override = None

        def set_reference_override(
            self,
            degrees,
            *,
            blend=1.0,
            unwrapped_lower=False,
        ) -> None:
            self.override = (degrees, blend, unwrapped_lower)

    class _FakeGait:
        continuous_roll_degrees = np.full(6, 360.0)

        def __init__(self, *args, **kwargs) -> None:
            self.reset_calls = []
            self.step_calls = []

        def reset(self, **kwargs) -> None:
            self.reset_calls.append(kwargs)

        def set_continuous_roll_degrees(self, values) -> None:
            self.roll_seed = np.asarray(values)

        def step(self, command, dt):
            self.step_calls.append((command, dt))
            return SimpleNamespace(
                converged=True,
                failed_legs=(),
                motor_degrees=np.array([220.0] * 12 + [620.0] * 6),
            )

    def test_yaw_and_slow_translation_remain_ppo_only(self) -> None:
        env = self._FakeEnv()
        with patch("src.rl.joystick_control.SconeGait", self._FakeGait):
            hybrid = SconeHybridController(env)
        action = np.linspace(-1.0, 1.0, 18, dtype=np.float32)

        yaw_action = hybrid.apply(VelocityCommand(yaw_rate=0.8), action)
        slow_action = hybrid.apply(VelocityCommand(vx=0.08), action)

        np.testing.assert_array_equal(yaw_action, action)
        np.testing.assert_array_equal(slow_action, action)
        self.assertEqual(hybrid.last_blend, 0.0)
        self.assertEqual(hybrid.gait.step_calls, [])

    def test_fast_translation_uses_point_support_hybrid_reference(self) -> None:
        env = self._FakeEnv()
        with patch("src.rl.joystick_control.SconeGait", self._FakeGait):
            hybrid = SconeHybridController(env)
        action = np.ones(18, dtype=np.float32)

        output = hybrid.apply(VelocityCommand(vx=0.5), action)

        np.testing.assert_array_equal(output, np.zeros(18, dtype=np.float32))
        self.assertEqual(hybrid.last_blend, 1.0)
        self.assertEqual(hybrid.control_name(), "scone-gait/hybrid/roll-1.0turn")
        self.assertEqual(env.override[1], 1.0)
        self.assertTrue(env.override[2])
        np.testing.assert_array_equal(
            env.override[0],
            np.array([220.0] * 12 + [620.0] * 6),
        )

    def test_transition_band_is_smooth(self) -> None:
        config = SconeHybridControlConfig(
            hybrid_start_speed=0.10,
            hybrid_full_speed=0.20,
        )

        self.assertEqual(config.hybrid_blend(VelocityCommand(vx=0.10)), 0.0)
        self.assertAlmostEqual(
            config.hybrid_blend(VelocityCommand(vx=0.15)),
            0.5,
        )
        self.assertEqual(config.hybrid_blend(VelocityCommand(vx=0.20)), 1.0)


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
