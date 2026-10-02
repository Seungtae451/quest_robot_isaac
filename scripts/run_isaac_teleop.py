"""Terminal 1 entry point; run with Isaac Lab 2.3.2's isaaclab.sh -p."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from simulation.isaac_teleop_app import main

if __name__ == "__main__":
    main()
