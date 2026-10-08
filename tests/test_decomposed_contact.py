import json, unittest
from pathlib import Path
import numpy as np
from scipy.spatial import ConvexHull
from benchmark.model_variants import transform_for_contact_geometry,CLOSED_WHEEL_CENTER
from benchmark.common import SimulationTrial,MetricsRecorder
from src.simulation.core.model import load_model

class DecomposedContactTests(unittest.TestCase):
    def test_cavity_is_outside_every_piece(self):
        payload=json.loads((Path(__file__).resolve().parents[1]/'benchmark/assets/tire_coacd_2mm.json').read_text())
        centre=np.array(CLOSED_WHEEL_CENTER)
        for p in payload['parts']:
            equations=ConvexHull(p['vertices']).equations
            self.assertGreater(float((equations[:,:3]@centre+equations[:,3]).max()),.001)
    def test_preserves_inertia_and_counts_pieces_as_tires(self):
        original=load_model(floating_base=True)
        with SimulationTrial(contact_geometry='decomposed-arc') as trial:
            np.testing.assert_array_equal(original.body_mass,trial.model.body_mass)
            np.testing.assert_array_equal(original.body_inertia,trial.model.body_inertia)
            recorder=MetricsRecorder(trial,[0,0,0])
            self.assertEqual(len(recorder.tire_geom_ids),6*25)
            self.assertEqual(len(set(trial.model.geom_bodyid[list(recorder.tire_geom_ids)])),6)
