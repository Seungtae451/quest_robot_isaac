"""Three actual VLA RGB sensors, also the ONLY source of Quest camera images.

CameraCfg and Camera are from the installed Isaac Lab 2.3.2 checkout. Camera
prims are children of the actual USD rigid links, so wrists follow articulation
motion automatically. No fake operator camera or pose-following loop is used.
"""
from pathlib import Path

import cv2
import numpy as np
import omni.usd
from pxr import Usd, UsdPhysics
import isaaclab.sim as sim_utils
from isaaclab.sensors import Camera, CameraCfg

from config import teleop_config as cfg


def rigid_link_path(name):
    stage = omni.usd.get_context().get_stage()
    root = stage.GetPrimAtPath("/World/F14")
    matches = [p for p in Usd.PrimRange(root) if p.GetName() == name and p.HasAPI(UsdPhysics.RigidBodyAPI)]
    if len(matches) != 1:
        raise RuntimeError(f"Expected one rigid link named {name}; found {[str(p.GetPath()) for p in matches]}")
    return str(matches[0].GetPath())


def create_cameras(fps=cfg.CAMERA_FPS, debug=False):
    mounts = [
        ("front", "base_link", cfg.BODY_CAMERA_OFFSET, cfg.BODY_CAMERA_ROT, cfg.BODY_CAMERA_WIDTH, cfg.BODY_CAMERA_HEIGHT),
        ("left_wrist", "left_dof7_link", cfg.LEFT_WRIST_CAMERA_OFFSET, cfg.LEFT_WRIST_CAMERA_ROT, cfg.WRIST_CAMERA_WIDTH, cfg.WRIST_CAMERA_HEIGHT),
        ("right_wrist", "right_dof7_link", cfg.RIGHT_WRIST_CAMERA_OFFSET, cfg.RIGHT_WRIST_CAMERA_ROT, cfg.WRIST_CAMERA_WIDTH, cfg.WRIST_CAMERA_HEIGHT),
    ]
    cameras = {}
    for name, link, pos, rot, width, height in mounts:
        path = f"{rigid_link_path(link)}/{name}_camera"
        camera_cfg = CameraCfg(
            prim_path=path, update_period=1. / fps, width=width, height=height,
            data_types=["rgb"], update_latest_camera_pose=debug,
            # world convention here specifies the CAMERA axes, not the frame
            # in which pos is measured; pos is always relative to parent link.
            offset=CameraCfg.OffsetCfg(pos=pos, rot=rot, convention="world"),
            spawn=sim_utils.PinholeCameraCfg(
                focal_length=18., horizontal_aperture=24., clipping_range=(.01, 100.)),
        )
        cameras[name] = Camera(camera_cfg)
        print(f"Camera {name}: {path}, {width}x{height}, parent offset={pos}, wxyz={rot}")
    return cameras


def raw_observation(cameras, state):
    """Borrow GPU tensor views for this frame. A recorder retaining past frames
    must clone them because Isaac reuses sensor buffers on subsequent updates.
    No resize, overlay, color swap, or CPU copy is applied to these tensors.
    """
    return {"observation.state": state,
            **{f"observation.images.{name}": camera.data.output["rgb"] for name, camera in cameras.items()}}


def display_rgb_copies(observation):
    """One GPU->CPU copy per sensor, at CAMERA rate, only for display/export."""
    return {name.rsplit(".", 1)[-1]: np.ascontiguousarray(value[0].detach().cpu().numpy()[..., :3]).copy()
            for name, value in observation.items() if name.startswith("observation.images.")}


def save_snapshots(directory, images, composite, suffix=""):
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    for name, rgb in {**images, "quest_composite": composite}.items():
        path = directory / f"{name}{suffix}.png"
        if not cv2.imwrite(str(path), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)):
            raise RuntimeError(f"Could not save {path}")
