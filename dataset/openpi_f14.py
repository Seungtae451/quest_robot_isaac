"""F14 16D absolute drive targets -> OpenPI inputs; import in OpenPI's env only."""
import dataclasses

import numpy as np

from openpi import transforms
from openpi.training import config

CAMERA_MAP = {"front": "base_0_rgb", "left_wrist": "left_wrist_0_rgb", "right_wrist": "right_wrist_0_rgb"}


@dataclasses.dataclass(frozen=True)
class F14Inputs:
    def __call__(self, data):
        images = {}
        for source, target in CAMERA_MAP.items():
            image = np.asarray(data["images"][source])
            if image.ndim != 3 or image.shape[0] != 3:
                raise ValueError(f"Expected LeRobot CHW RGB: {source}, {image.shape}")
            if np.issubdtype(image.dtype, np.floating):
                image = np.rint(np.clip(image, 0., 1.) * 255).astype(np.uint8)
            images[target] = np.transpose(image, (1, 2, 0))
        state = np.asarray(data["state"], np.float32)
        if state.shape != (16,) or not np.isfinite(state).all():
            raise ValueError("F14 state must have 14 arm joints and two gripper open ratios")
        result = {"image": images, "image_mask": {name: np.True_ for name in images},
                  "state": state, "prompt": data["prompt"]}
        if "actions" in data:
            actions = np.asarray(data["actions"], np.float32)
            if actions.ndim != 2 or actions.shape[-1] != 16 or not np.isfinite(actions).all():
                raise ValueError("F14 action chunks must be [horizon,16]")
            result["actions"] = actions
        return result


@dataclasses.dataclass(frozen=True)
class F14Outputs:
    def __call__(self, data):
        return {"actions": np.asarray(data["actions"])[..., :16]}


@dataclasses.dataclass(frozen=True)
class LeRobotF14DataConfig(config.DataConfigFactory):
    """Preserve F14 joint signs/order and absolute actions, without Aloha conversion."""
    def create(self, assets_dirs, model_config):
        repack = transforms.RepackTransform({
            "images": {name: f"observation.images.{name}" for name in CAMERA_MAP},
            "state": "observation.state", "actions": "action", "prompt": "prompt"})
        return dataclasses.replace(self.create_base_config(assets_dirs, model_config),
            repack_transforms=transforms.Group(inputs=[repack]),
            data_transforms=transforms.Group(inputs=[F14Inputs()], outputs=[F14Outputs()]),
            model_transforms=config.ModelTransformFactory()(model_config),
            action_sequence_keys=("action",), prompt_from_task=True)
