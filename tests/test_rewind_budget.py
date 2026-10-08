"""End-to-end command invariants for the opt-in simulation controller."""
import unittest
import numpy as np
from src.locomotion import SconeGaitV2, SconeGaitV2Config


class RewindBudgetTests(unittest.TestCase):
    def test_sector_rate_and_excursion_survive_reversal_and_pause(self):
        for dt in (0.01, 0.02, 0.037):
            gait = SconeGaitV2(config=SconeGaitV2Config(
                limit_reindex_rate=True, anticipate_reindex_budget=True,
                command_time_constant=0.0))
            gait.reset(phase=0.71)
            previous = gait.sector_degrees
            budget = 540 * 0.2 / 1.875
            for command in ((.45, 0, 0), (0, 0, 0), (-.45, 0, 0),
                            (0, .25, 0), (0, 0, .9), (.18, 0, .35)):
                for _ in range(int(1.3 / dt)):
                    sample = gait.step(command, dt)
                    current = gait.sector_degrees
                    self.assertLessEqual(float(np.max(abs(current - previous))) / dt,
                                         540 + 1e-7)
                    self.assertLessEqual(float(np.max(abs(current))), budget + 1e-7)
                    self.assertTrue(np.isfinite(sample.motor_degrees).all())
                    previous = current

    def test_public_benchmark_factory_exposes_opt_in_variants(self):
        from benchmark.controllers import ROLE_CONFIGS, CONTROLLER_CHOICES
        self.assertTrue(set(ROLE_CONFIGS) <= set(CONTROLLER_CHOICES))
        self.assertFalse(ROLE_CONFIGS['role-split-scone'].limit_reindex_rate)
        self.assertTrue(ROLE_CONFIGS['rewind-budget-scone'].limit_reindex_rate)

    def test_lookahead_rewinds_before_high_speed_residual_becomes_unreachable(self):
        from benchmark.revision_study import run_job
        job=dict(id=0, suite='regression', code='B', command=[.45,0,0],
                 phase=0., geometry='decomposed-arc', physics_dt=.002, stress={})
        row,_=run_job(job)
        self.assertTrue(row['completed'])
        self.assertEqual(row['sector_rate_violation_frames'],0)
        self.assertGreater(row['displacement_x_m'],1.0)


if __name__ == '__main__':
    unittest.main()
