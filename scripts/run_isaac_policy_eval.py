"""Run with Isaac Lab's isaaclab.sh -p; see OPENPI_TRAINING.md."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from simulation.isaac_policy_eval import main

if __name__=='__main__':
    main()
