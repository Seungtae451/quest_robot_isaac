"""Compatibility entry point for the GR00T N1.7-style F14 open-loop evaluator.

The earlier single-observation plot was replaced by the official trajectory
protocol. Use --model-path/--checkpoint, --traj-ids, --steps and
--execution-horizon; see OPENPI_TRAINING.md.
"""
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from scripts.open_loop_eval_f14 import main

if __name__ == '__main__':
    main()
