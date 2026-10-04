"""Task success, latest checkpoint selection and closed-loop metric accounting."""
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from config import tabletop_config as cfg
from dataset.openpi_checkpoint import resolve_checkpoint
from simulation.inference_task import PlacementSuccess, EvaluationResults, bounded_action
from simulation.tabletop import table_surface_height


class PolicyEvalTests(unittest.TestCase):
    def test_last_checkpoint_must_be_committed(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            for name,complete in [('9000',True),('42000',True),('50000',False)]:
                checkpoint=root/name
                (checkpoint/'params').mkdir(parents=True)
                (checkpoint/'_CHECKPOINT_METADATA').write_text(json.dumps({'commit_timestamp_nsecs':1 if complete else None}))
            self.assertEqual(resolve_checkpoint('latest',root).name,'42000')
            with self.assertRaises(ValueError):
                resolve_checkpoint(root/'50000',root)

    def test_only_inside_bottom_contact_counts(self):
        success=PlacementSuccess()
        center=np.array([*cfg.BOX_CENTER_XY,table_surface_height()+cfg.BOX_BOTTOM_THICKNESS+cfg.CUBE_SIZE/2])
        rotation=[1,0,0,0]
        weight=[0,0,.1962]
        self.assertFalse(success.update(center,rotation,[0,0,0],1/60))
        self.assertFalse(success.update(center+[0,0,.1],rotation,weight,1/60))
        outside=center.copy()
        outside[1]+=cfg.BOX_SIZE[1]/2
        self.assertFalse(success.update(outside,rotation,weight,1/60))
        self.assertTrue(success.update(center,rotation,weight,1/60))

    def test_rotated_corner_must_not_cross_inner_wall(self):
        success=PlacementSuccess()
        center=np.array([*cfg.BOX_CENTER_XY,table_surface_height()+cfg.BOX_BOTTOM_THICKNESS+cfg.CUBE_SIZE/2])
        center[0]+=cfg.BOX_SIZE[0]/2-cfg.BOX_WALL_THICKNESS-.017
        self.assertTrue(success.update(center,[1,0,0,0],[0,0,.2],1/60))
        yaw=np.pi/4
        self.assertFalse(success.update(center,[np.cos(yaw/2),0,0,np.sin(yaw/2)],[0,0,.2],1/60))

    def test_optional_contact_hold_resets_after_separation(self):
        success=PlacementSuccess(.05)
        center=[*cfg.BOX_CENTER_XY,table_surface_height()+cfg.BOX_BOTTOM_THICKNESS+cfg.CUBE_SIZE/2]
        self.assertFalse(success.update(center,[1,0,0,0],[0,0,.2],.03))
        self.assertFalse(success.update(center,[1,0,0,0],[0,0,0],.01))
        self.assertFalse(success.update(center,[1,0,0,0],[0,0,.2],.03))
        self.assertTrue(success.update(center,[1,0,0,0],[0,0,.2],.02))

    def test_joint_and_gripper_limits_and_invalid_predictions(self):
        action=np.full(16,2.)
        clipped,changed=bounded_action(action,np.tile([-1.,1.],(14,1)))
        self.assertTrue(changed)
        np.testing.assert_array_equal(clipped,np.ones(16))
        action[0]=np.nan
        with self.assertRaises(ValueError):
            bounded_action(action,np.tile([-1.,1.],(14,1)))

    def test_100_trials_and_interrupted_denominator(self):
        with tempfile.TemporaryDirectory() as directory:
            results=EvaluationResults(Path(directory)/'evaluation',{'requested_trials':100})
            for index in range(100):
                results.record(trial_index=index,outcome='success' if index<73 else 'timeout',elapsed_sim_seconds=5.)
            summary=results.summary('complete')
            self.assertEqual(summary['completed_trials'],100)
            self.assertEqual(summary['success_rate_percent'],73.)
            self.assertEqual(summary['failures'],27)
            results.record(trial_index=100,outcome='interrupted',elapsed_sim_seconds=1.)
            self.assertEqual(results.summary('interrupted')['completed_trials'],100)
            self.assertEqual(len((results.directory/'trials.jsonl').read_text().splitlines()),101)

    def test_manual_failure_is_a_completed_failure(self):
        with tempfile.TemporaryDirectory() as directory:
            results=EvaluationResults(Path(directory)/'evaluation',{'requested_trials':100})
            results.record(trial_index=0,outcome='success',elapsed_sim_seconds=5.)
            results.record(trial_index=1,outcome='manual_failure',elapsed_sim_seconds=2.)
            summary=results.summary('running')
            self.assertEqual(summary['completed_trials'],2)
            self.assertEqual(summary['failures'],1)
            self.assertEqual(summary['success_rate_percent'],50.)
            self.assertEqual(summary['outcomes']['manual_failure'],1)


if __name__=='__main__':
    unittest.main()
