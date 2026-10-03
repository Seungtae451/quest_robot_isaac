"""Lightweight verified-spawn cache and camera geometry; no Pinocchio/Isaac.

Heavy IK runs in a separate interpreter before Kit starts, not in physics or
the episode reset loop. Cache validity follows every relevant configuration.
"""
import hashlib
import json
from pathlib import Path
import subprocess
import sys

import numpy as np

from robot import f14_config as robot
from config import tabletop_config as table
from config import teleop_config as teleop

CACHE_PATH = robot.PROJECT_ROOT / "outputs/arm_workspace/ik_spawn_pool.json"
_loaded = None


def workspace_signature():
    settings = {"version": 1, "urdf_sha256": hashlib.sha256(robot.F14_URDF_PATH.read_bytes()).hexdigest(),
        "home": robot.HOME_Q.tolist(), "left_rot": robot.LEFT_EE_DOWN_ROT.tolist(),
        "right_rot": robot.RIGHT_EE_DOWN_ROT.tolist(),
        "table": {name: value for name, value in vars(table).items()
                  if name.isupper() and name not in ("CUBE_COUNT", "TABLE_COLOR", "BOX_COLOR", "CUBE_COLOR", "CUBE_MASS")},
        "camera": {name: getattr(teleop, name) for name in (
            "BODY_CAMERA_OFFSET", "BODY_CAMERA_ROT", "BODY_CAMERA_WIDTH", "BODY_CAMERA_HEIGHT",
            "BODY_CAMERA_FOCAL_LENGTH", "CAMERA_HORIZONTAL_APERTURE")},
        "ik": {name: getattr(teleop, name) for name in ("IK_MAX_ITER", "IK_EPS", "IK_DT", "IK_DAMPING")}}
    return hashlib.sha256(json.dumps(settings, sort_keys=True).encode()).hexdigest()


def ensure_spawn_pool():
    global _loaded
    signature = workspace_signature()
    if _loaded is not None and _loaded[0] == signature:
        return _loaded[1]
    try:
        data = json.loads(CACHE_PATH.read_text())
    except (FileNotFoundError, json.JSONDecodeError):
        data = None
    if data is None or data.get("signature") != signature:
        print("Building IK-verified cube spawn pool before scene startup...", flush=True)
        subprocess.run([sys.executable, str(robot.PROJECT_ROOT / "scripts/build_ik_spawn_pool.py")],
                       cwd=robot.PROJECT_ROOT, check=True)
        data = json.loads(CACHE_PATH.read_text())
    if data.get("signature") != signature or not data.get("candidates"):
        raise RuntimeError("No valid IK spawn pool. Run scripts/build_ik_spawn_pool.py; never falling back to unchecked positions.")
    _loaded = (signature, data)
    return data


def grasp_wrist_position(x, y, side, height):
    rotation = (robot.LEFT_EE_DOWN_ROT, robot.RIGHT_EE_DOWN_ROT)[side]
    return np.array([x, y, height]) - rotation @ np.asarray(table.GRASP_LOCAL_OFFSET)


def camera_pixels(points):
    """Ideal body camera pixels, +X forward/+Y left/+Z up camera convention."""
    w,x,y,z = teleop.BODY_CAMERA_ROT
    q = np.array([w,x,y,z], dtype=float);q /= np.linalg.norm(q);w,x,y,z=q
    rotation = np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
        [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
        [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
    local = (np.asarray(points)-teleop.BODY_CAMERA_OFFSET) @ rotation
    focal = teleop.BODY_CAMERA_WIDTH * teleop.BODY_CAMERA_FOCAL_LENGTH / teleop.CAMERA_HORIZONTAL_APERTURE
    depth = local[:,0]
    uv = np.column_stack((teleop.BODY_CAMERA_WIDTH/2-focal*local[:,1]/depth,
                          teleop.BODY_CAMERA_HEIGHT/2-focal*local[:,2]/depth))
    return uv, depth


def cube_is_in_camera(x, y, surface):
    # Worst yaw footprint; include the airborne spawn height as well as settled.
    radius = table.CUBE_SIZE / np.sqrt(2)
    corners = [[x+dx,y+dy,z] for dx in (-radius,radius) for dy in (-radius,radius)
               for z in (surface,surface+table.CUBE_SIZE+table.CUBE_DROP_GAP)]
    uv,depth=camera_pixels(corners)
    margin=table.SPAWN_CAMERA_MARGIN
    return bool(np.all(depth>.01) and np.all(uv>np.array([teleop.BODY_CAMERA_WIDTH,teleop.BODY_CAMERA_HEIGHT])*margin)
        and np.all(uv<np.array([teleop.BODY_CAMERA_WIDTH,teleop.BODY_CAMERA_HEIGHT])*(1-margin)))
