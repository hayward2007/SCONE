"""Guard the stair experiment against false completion and confounded pairs."""
from benchmark.stair_simplification import design, fixed_config, top_supported, halt_stable


import unittest

class StairSimplificationTests(unittest.TestCase):
    def test_top_requires_rear_clearance_and_loaded_support(self):
        assert not top_supported(1.05,1.0,6,1.)
        assert not top_supported(1.13,1.0,2,1.)
        assert not top_supported(1.13,1.0,6,.49)
        assert top_supported(1.13,1.0,3,.8)


    def test_halt_rejects_motion_and_loss_of_support(self):
        assert not halt_stable(True,.06,0.)
        assert not halt_stable(True,0.,.21)
        assert not halt_stable(False,0.,0.)
        assert halt_stable(True,.01,.05)


    def test_posture_ablation_changes_only_brace(self):
        from dataclasses import asdict
        a,b=map(asdict,(fixed_config(195),fixed_config(180)))
        changed={k for k in a if a[k]!=b[k]}
        assert changed=={'neutral_front_stage1_degrees','medium_front_stage1_degrees','tall_front_stage1_degrees'}
        assert a['synchronized_phase_degrees']==a['tall_synchronized_phase_degrees']==90
        assert a['phase_velocity']==a['easy_phase_velocity']==200


    def test_full_factorial_contains_every_matched_geometry_posture_pair(self):
        jobs=design()
        assert len(jobs)==102 and len({j['id'] for j in jobs})==102
        keys={(j['h'],j['d'],j['pose']) for j in jobs if j['suite']=='factorial'}
        assert len(keys)==18
        for key in keys:
            group=[j for j in jobs if j['suite']=='factorial' and (j['h'],j['d'],j['pose'])==key]
            assert {(j['geometry'],j['control']) for j in group}=={(g,c) for g in ['decomposed-arc','closed-wheel'] for c in ['fixed','neutral']}
