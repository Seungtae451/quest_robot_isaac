"""Terminal 2 entry point: Quest + Pinocchio, without any Isaac imports."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from teleop.quest_teleop_server import main

if __name__ == "__main__":
    main()
