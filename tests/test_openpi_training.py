"""GPU training must stop before initialization when the recorder is active."""
from pathlib import Path
import json
from types import SimpleNamespace
import tempfile
import unittest

from scripts.train_openpi_f14 import require_isaac_stopped, append_training_metrics


class IsaacGuardTests(unittest.TestCase):
    def test_metric_history_survives_resume(self):
        with tempfile.TemporaryDirectory() as directory:
            config = SimpleNamespace(checkpoint_dir=Path(directory))
            append_training_metrics(config, {'loss': 0.12}, 50)
            append_training_metrics(config, {'loss': 0.08}, 100)
            rows = [json.loads(line) for line in
                    (config.checkpoint_dir/'training_metrics.jsonl').read_text().splitlines()]
            self.assertEqual([row['step'] for row in rows], [50, 100])
            self.assertEqual([row['loss'] for row in rows], [0.12, 0.08])

    def test_recording_receiver_blocks_training(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            process = root / '12345'
            process.mkdir()
            (process / 'cmdline').write_bytes(
                b'python\0/home/user/scripts/run_isaac_teleop.py\0--record\0')
            with self.assertRaisesRegex(RuntimeError, 'PID 12345'):
                require_isaac_stopped(root)

    def test_other_python_process_is_allowed(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            process = root / '12345'
            process.mkdir()
            (process / 'cmdline').write_bytes(b'python\0other_script.py\0')
            require_isaac_stopped(root)


if __name__ == '__main__':
    unittest.main()
