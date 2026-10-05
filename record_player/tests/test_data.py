"""Run: record_player/.venv/Scripts/python -m unittest discover -s record_player/tests -p test_*.py.
Uses small synthetic Parquet fixtures; never modifies original recordings.
"""
import math
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import data


class DataTests(unittest.TestCase):
    """Validate conversions whose mistakes would change interpretation of player behavior."""

    def test_signed_gear_recovery(self):
        """Recover negative gearing from recording columns, not from a possibly changed profile."""
        trace = dict(dial_pos_rad=[[math.pi]*6, [2*math.pi]*6], dial_robot_deg=[[-18]*6, [-36]*6])  # Known -0.1 ratio.
        self.assertEqual(data.infer_ratios(trace), [-.1]*6)
        self.assertEqual(data.infer_ratios({'dial_pos_rad':[], 'dial_robot_deg':[]}), [None]*6)

    def test_alignment_gaps(self):
        """Exclude pre-stream and stale samples from lead statistics instead of treating them as zero."""
        trace = {'t':[0, 1], 'q':[[2]*6, [3]*6]}  # Deliberate one-second gap.
        values = data.aligned(trace, 'q', np.array([-.1, .1, .6, 1.1]))  # Four sample-age cases.
        self.assertTrue(np.isnan(values[0]).all())
        self.assertTrue(np.isnan(values[2]).all())
        np.testing.assert_equal(values[[1,3]], [[2]*6,[3]*6])

    def test_parquet_timestamp_precision_and_sorting(self):
        """Subtract int64 wall times before converting to seconds, preserving nanosecond ordering."""
        with tempfile.TemporaryDirectory() as directory:
            origin = 1791023824000000000  # Epoch nanoseconds too large for exact JS numbers.
            source = pa.table({'ts_wall_ns':[origin+100000000,origin,origin+20000000], 'value':[3,1,2]})  # Out-of-order packets.
            pq.write_table(source, Path(directory)/'sample.parquet')
            trace = data.stream(Path(directory),'sample.parquet',origin,[])  # Retains last sample in each 50ms bin.
            self.assertEqual(trace['t'],[.02,.1])
            self.assertEqual(trace['value'],[2,3])

    def test_empty_skeleton_frames_are_not_carried_forward(self):
        """A zero-person camera frame must clear older skeletons."""
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory)  # Disposable synthetic camera recording.
            (folder/'skeleton/a').mkdir(parents=True)
            pq.write_table(pa.table({'frame_id':[1,2], 'ts_unix_ms':[1000,1100], 'people_count':[1,0]}),folder/'skeleton/a/frames.parquet')
            pq.write_table(pa.table({'frame_id':[1], 'ts_unix_ms':[1000], 'person_id':[123], 'kp_x':[[1.]*17], 'kp_y':[[2.]*17], 'kp_score':[[.9]*17]}),folder/'skeleton/a/skeleton.parquet')
            trace = data.skeleton(folder,'a',1000000000,[])  # Frame-aligned pose result.
            self.assertEqual(trace['t'],[0,.1])
            self.assertEqual(len(trace['people'][0]),1)
            self.assertEqual(trace['people'][1],[])

    def test_scene_units_and_hidden_obstructions(self):
        """Use native body scale and keep ceiling/player blockers hidden by default."""
        scene = data.scene()  # Actual curated assets; no physics libraries required.
        self.assertEqual(sum(joint['type']=='revolute' for joint in scene['joints']),6)
        self.assertTrue(scene['tool'])
        for body in scene['bodies']:
            self.assertEqual(body['hidden'], 'player' in body['name'] or body['name']=='ceiling')
            self.assertLess(max(abs(value) for mesh in body['meshes'] for value in mesh['positions']),20)

    def test_recording_id_cannot_escape_root(self):
        """Reject malformed API recording keys before touching the filesystem."""
        for game_id in ['../../config','2026-10-03/../../x','C:/Windows']:
            with self.assertRaises(ValueError):
                data.game_path(game_id)


if __name__ == '__main__':
    unittest.main()
