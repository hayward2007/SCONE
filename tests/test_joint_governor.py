import unittest
import numpy as np
from benchmark.joint_governor import JointTargetGovernor, DEGREES_PER_COUNT, degrees_to_counts


class JointGovernorTests(unittest.TestCase):
    def test_quantized_reversal_and_hold_obey_all_joint_limits(self):
        for scale in (.5, .8, 1.):
            g = JointTargetGovernor(np.full(18, 2048), scale)
            for target in [np.arange(18)*180+100, np.full(18, 4096), np.zeros(18), np.full(18, 2048)]:
                for _ in range(300):
                    before=g.raw.copy();out=g.advance(target)
                    self.assertTrue(np.all(abs(out-before)*DEGREES_PER_COUNT/.02 <= g.limits_dps+1e-9))
                    self.assertTrue(np.all(abs(target-out) <= abs(target-before)))
                    if g.at_target:break
                self.assertTrue(g.at_target)
                np.testing.assert_array_equal(g.advance(target),target)

    def test_passthrough_for_reachable_step_and_input_atomicity(self):
        g=JointTargetGovernor(np.full(18,2048),.8)
        target=np.full(18,2050);np.testing.assert_array_equal(g.advance(target),target)
        self.assertEqual(g.last_alpha,1.)
        for invalid in [np.full(18,np.nan),np.full(18,4097),np.full(18,1.5)]:
            with self.assertRaises(ValueError):g.advance(invalid)
            np.testing.assert_array_equal(g.raw,target)
        np.testing.assert_array_equal(degrees_to_counts(np.full(18,180.)),np.full(18,2048))

    def test_inner_profile_is_nonzero_and_below_requested_limit(self):
        class Profile:
            def __init__(self):self.commands={}
            def _speed_to_radians_per_second(self,i,v):return np.radians(v*(.114 if i<=6 else .229)*6)
            def set_speed(self,i,v):self.commands[i]=v
        g=JointTargetGovernor(np.full(18,2048),.8);controller=Profile()
        effective=g.configure_inner_profile(controller)
        self.assertTrue(np.all(effective<=g.limits_dps+1e-9))
        self.assertTrue(all(v>0 for v in controller.commands.values()))

    def test_full_simulated_goal_and_inner_profile_bounds(self):
        from benchmark.joint_limit_study import design, run_job, command_at
        jobs=design()
        self.assertEqual(len(jobs),176)
        sequence=[[0,.18,0,0],[6,0,0,0],[12,-.18,0,0]]
        np.testing.assert_array_equal(command_at(sequence,5.98),[.18,0,0])
        np.testing.assert_array_equal(command_at(sequence,6),[0,0,0])
        np.testing.assert_array_equal(command_at(sequence,12),[-.18,0,0])
        job=next(j for j in jobs if j['code']=='B' and j['mode']=='governed'
                 and j['scale']==.8 and j['sequence'][0][1]==.45)
        result,_=run_job({**job,'horizon_s':2.})
        self.assertTrue(result['completed'])
        self.assertEqual(result['issued_rate_violation_frames'],0)
        self.assertLessEqual(result['inner_limit_excess_dps'],1e-8)
        self.assertLess(result['planner_time_fraction'],1.)
        self.assertGreater(result['planner_time_fraction'],0.)
        self.assertGreater(max(result['joint_tracking_rms_deg']),0.)


if __name__=='__main__':unittest.main()
