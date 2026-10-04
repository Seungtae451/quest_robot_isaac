"""Spawn the canonical F14 USD and map semantic commands by joint NAME.

Import only after AppLauncher. HOME is written as initial state once; during
operation only drive targets are changed. Imported USD gains are retained.
"""
import torch
import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import Articulation, ArticulationCfg

from robot.f14_config import ARM_JOINT_NAMES, GRIPPER_JOINT_NAMES, F14_USD_PATH, HOME_ACTION, USD_ARM_SIGNS
from robot.gripper import closure_to_joints
from config import teleop_config as cfg
from simulation.tabletop import spawn_tabletop


def create_scene(camera_debug=False, scene_seed=None, *, cube_contact_sensors=False):
    if not F14_USD_PATH.is_file():
        raise FileNotFoundError(F14_USD_PATH)
    # Procedural ground avoids an external asset-server dependency during
    # startup; it is a static collision surface whose top is exactly z=0.
    ground = sim_utils.CuboidCfg(
        size=(20., 20., .02), collision_props=sim_utils.CollisionPropertiesCfg(),
        visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(.22, .24, .27)))
    ground.func("/World/Ground", ground, translation=(0., 0., -.01))
    spawn_tabletop(scene_seed, contact_sensors=cube_contact_sensors)
    light = sim_utils.DomeLightCfg(intensity=3000.)
    light.func("/World/Light", light)
    robot = Articulation(ArticulationCfg(
        prim_path="/World/F14",
        spawn=sim_utils.UsdFileCfg(usd_path=str(F14_USD_PATH)),
        init_state=ArticulationCfg.InitialStateCfg(pos=(0., 0., 0.)),
        actuators={
            "arms": ImplicitActuatorCfg(joint_names_expr=ARM_JOINT_NAMES, stiffness=None, damping=None),
            "fingers": ImplicitActuatorCfg(joint_names_expr=GRIPPER_JOINT_NAMES,
                                           stiffness=cfg.GRIPPER_STIFFNESS, damping=cfg.GRIPPER_DAMPING),
        },
    ))
    from robot.appearance import apply_isaac_materials
    apply_isaac_materials()
    if camera_debug:
        # Optional landmarks are camera diagnostics, separate from the
        # collidable tabletop task cube and its collection box.
        for index, color in enumerate(((1., .05, .05), (.05, 1., .05), (.05, .05, 1.))):
            box = sim_utils.CuboidCfg(size=(.07, .07, .07), visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=color))
            box.func(f"/World/CameraLandmark{index}", box, translation=(.65, .25 - index * .25, .4))
    return robot


def resolve_joint_ids(robot):
    groups = []
    for names in (ARM_JOINT_NAMES, GRIPPER_JOINT_NAMES):
        indices, resolved = robot.find_joints(names, preserve_order=True)
        if resolved != names or len(indices) != len(names):
            raise RuntimeError(f"F14 name mapping mismatch: {resolved}")
        groups.append(indices)
        print("Joint mapping:", dict(zip(resolved, indices)))
    if robot.num_joints != 18 or robot.num_bodies != 19:
        raise RuntimeError(f"Expected canonical F14 18 joints/19 bodies, got {robot.num_joints}/{robot.num_bodies}")
    return tuple(groups)


def apply_action(robot, action, arm_ids, gripper_ids):
    # set_joint_position_target updates Isaac Lab's target BUFFER. The actual
    # PhysX drive write happens in write_data_to_sim, before the physics step.
    arm = torch.as_tensor(action[:14] * USD_ARM_SIGNS, dtype=torch.float32, device=robot.device).unsqueeze(0)
    fingers = torch.as_tensor(closure_to_joints(*action[14:]), dtype=torch.float32, device=robot.device).unsqueeze(0)
    # USD uses 0.042489517 rather than the rounded semantic 0.0425 m. Respect
    # the actual physical limits, a ~10 micrometre endpoint correction only.
    limits = robot.data.joint_pos_limits[:, gripper_ids]
    fingers = torch.clamp(fingers, min=limits[..., 0], max=limits[..., 1])
    robot.set_joint_position_target(arm, joint_ids=arm_ids)
    robot.set_joint_position_target(fingers, joint_ids=gripper_ids)


def initialize_home(robot, arm_ids, gripper_ids):
    positions = robot.data.default_joint_pos.clone()
    positions[:, arm_ids] = torch.as_tensor(HOME_ACTION[:14] * USD_ARM_SIGNS, device=robot.device, dtype=positions.dtype)
    fingers = torch.as_tensor(closure_to_joints(0., 0.), device=robot.device, dtype=positions.dtype)
    limits = robot.data.joint_pos_limits[:, gripper_ids]
    positions[:, gripper_ids] = torch.clamp(fingers, min=limits[..., 0], max=limits[..., 1])
    robot.write_joint_state_to_sim(positions, torch.zeros_like(positions))
    robot.reset()
    apply_action(robot, HOME_ACTION, arm_ids, gripper_ids)
    robot.write_data_to_sim()


def semantic_state(robot, arm_ids, gripper_ids):
    """Measured 16D state tensor, left/right arm then left/right closure.

This stays on the simulation device. A dataset recorder can clone it together
with the same RGB tensors used by the operator view, at the camera timestamp.
"""
    q = robot.data.joint_pos
    fingers = q[:, gripper_ids].reshape(-1, 2, 2)
    from robot.f14_config import GRIPPER_WIDTH
    closure = (1. - (fingers[..., 1] - fingers[..., 0]) / (2 * GRIPPER_WIDTH)).clamp(0., 1.)
    signs = torch.as_tensor(USD_ARM_SIGNS, dtype=q.dtype, device=q.device)
    return torch.cat((q[:, arm_ids] * signs, closure), dim=-1)
