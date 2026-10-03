"""Local LeRobot v3 collection defaults; no Hub upload is performed."""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SESSION_PORT = 5006
DATASET_ROOT = ROOT / "datasets/f14_cube_pickplace"
REPO_ID = "local/f14_cube_pickplace"
TASK = "Pick up the red cube and place it in the center box."
WRITER_PYTHON = ROOT / ".venv-lerobot/bin/python"
QUEUE_FRAMES = 16
RESET_SETTLE_SECONDS = 0.5
RESET_TIMEOUT_SECONDS = 90.0
RESET_OPEN_SECONDS = 1.0

