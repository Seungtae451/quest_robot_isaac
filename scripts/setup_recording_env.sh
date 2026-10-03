#!/usr/bin/env bash
# Run after activating env_isaaclab. Only creates a local writer environment.
set -euo pipefail
project_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
isaac_python=${F14_ISAAC_PYTHON:-python}
"$isaac_python" -m venv --system-site-packages "$project_dir/.venv-lerobot"
"$project_dir/.venv-lerobot/bin/python" -m pip install --no-deps -r "$project_dir/requirements-recording.txt"
"$project_dir/.venv-lerobot/bin/python" -c 'from lerobot.datasets.lerobot_dataset import LeRobotDataset; import av; av.codec.Codec("libx264", "w"); print("LeRobot v3 writer ready")'
