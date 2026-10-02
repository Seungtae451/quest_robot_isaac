"""Launch real F14 HOME/gripper/wrist/camera validation and save RGB snapshots.

Run through isaaclab.sh -p. All normal Isaac launcher arguments are accepted.
Default output: outputs/camera_check/{front,left_wrist,right_wrist,quest_composite}.png
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from simulation.isaac_teleop_app import main

if __name__ == "__main__":
    import faulthandler
    faulthandler.dump_traceback_later(60, repeat=True)
    defaults = ["--smoke-test", "--camera-debug"]
    if "--snapshot-dir" not in sys.argv:
        defaults += ["--snapshot-dir", str(Path(__file__).resolve().parents[1] / "outputs/camera_check")]
    main(defaults + sys.argv[1:])
