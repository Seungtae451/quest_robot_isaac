"""Operator-only Quest AR settings. Never used by policy evaluation/recorders."""
from pathlib import Path

HTTPS_PORT = 8012
SCENE_ENDPOINT = "tcp://127.0.0.1:5557"
SCENE_HZ = 30.0
SCENE_TIMEOUT = 0.5
ATTACH_RADIUS = 0.05  # metres, controller grip origin to measured dof7 link origin
ASSET_ROOT = Path(__file__).resolve().parents[1] / "outputs/quest_ar/assets"
WEB_ROOT = Path(__file__).resolve().parents[1] / "web/quest_ar"
THREE_VERSION = "0.180.0"
