import argparse

from isaaclab.app import AppLauncher


# =========================================================
# Launch Isaac Sim
# =========================================================

parser = argparse.ArgumentParser(
    description="Test F14 HOME pose in Isaac Lab."
)

AppLauncher.add_app_launcher_args(parser)

args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app


# Isaac imports must come after AppLauncher
import torch

import isaaclab.sim as sim_utils

from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import Articulation, ArticulationCfg
from isaaclab.sim import SimulationContext


# =========================================================
# USER CONFIG
# =========================================================

F14_USD_PATH = (
    "/home/cocelo-server01/stkim_ws/quest_robot_isaac/"
    "assets/f14/"
    "FlaminGO_14Dof_Arm_Robot_v2_isaac/"
    "FlaminGO_14Dof_Arm_Robot_v2_isaac.usd"
)

LEFT_HOME = [
    -0.57,
     0.40,
     0.22,
    -0.95,
     0.24,
    -0.79,
    -0.27,
]

RIGHT_HOME = [
    -0.57,
    -0.40,
     0.22,
     0.95,
    -0.24,
     0.79,
    -0.27,
]


LEFT_ARM_JOINTS = [
    "left_dof1_joint",
    "left_dof2_joint",
    "left_dof3_joint",
    "left_dof4_joint",
    "left_dof5_joint",
    "left_dof6_joint",
    "left_dof7_joint",
]

RIGHT_ARM_JOINTS = [
    "right_dof1_joint",
    "right_dof2_joint",
    "right_dof3_joint",
    "right_dof4_joint",
    "right_dof5_joint",
    "right_dof6_joint",
    "right_dof7_joint",
]

ARM_JOINTS = (
    LEFT_ARM_JOINTS
    + RIGHT_ARM_JOINTS
)


LEFT_EE_BODY = "left_dof7_link"
RIGHT_EE_BODY = "right_dof7_link"


# =========================================================
# F14 CONFIG
# =========================================================

F14_CFG = ArticulationCfg(

    prim_path="/World/F14",

    spawn=sim_utils.UsdFileCfg(
        usd_path=F14_USD_PATH,
    ),

    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.0),
    ),

    actuators={

        # Keep the position-drive gains imported from USD
        "all_joints": ImplicitActuatorCfg(
            joint_names_expr=[".*"],
            stiffness=None,
            damping=None,
        ),
    },
)


# =========================================================
# Scene
# =========================================================

def design_scene():

    # Ground
    ground_cfg = sim_utils.GroundPlaneCfg()
    ground_cfg.func(
        "/World/defaultGroundPlane",
        ground_cfg,
    )

    # Light
    light_cfg = sim_utils.DomeLightCfg(
        intensity=3000.0,
        color=(0.75, 0.75, 0.75),
    )

    light_cfg.func(
        "/World/Light",
        light_cfg,
    )

    # Robot
    robot = Articulation(
        cfg=F14_CFG
    )

    return robot


# =========================================================
# Main
# =========================================================

def main():

    # -----------------------------------------------------
    # Simulation
    # -----------------------------------------------------

    sim_cfg = sim_utils.SimulationCfg(
        dt=1.0 / 120.0,
        device=args_cli.device,
    )

    sim = SimulationContext(
        sim_cfg
    )

    # Camera
    sim.set_camera_view(
        eye=[1.8, 1.4, 1.3],
        target=[0.3, 0.0, 0.5],
    )

    robot = design_scene()

    # Important:
    # initialize physics / articulation handles
    sim.reset()

    print("\n========================================")
    print("F14 loaded")
    print("========================================")


    # -----------------------------------------------------
    # Print all joints
    # -----------------------------------------------------

    print("\n===== JOINTS =====")

    for i, name in enumerate(
        robot.joint_names
    ):
        print(
            f"{i:2d}: {name}"
        )


    print(
        "\nNumber of joints:",
        robot.num_joints,
    )


    # -----------------------------------------------------
    # Print all bodies
    # -----------------------------------------------------

    print("\n===== BODIES =====")

    for i, name in enumerate(
        robot.body_names
    ):
        print(
            f"{i:2d}: {name}"
        )


    # -----------------------------------------------------
    # Resolve F14 arm joint IDs by NAME
    # -----------------------------------------------------

    arm_joint_ids, resolved_names = (
        robot.find_joints(
            ARM_JOINTS,
            preserve_order=True,
        )
    )

    print(
        "\n===== ARM JOINT MAPPING ====="
    )

    for name, idx in zip(
        resolved_names,
        arm_joint_ids,
    ):
        print(
            f"{name:25s} -> {idx}"
        )


    if len(arm_joint_ids) != 14:
        raise RuntimeError(
            f"Expected 14 arm joints, "
            f"but found {len(arm_joint_ids)}"
        )


    # -----------------------------------------------------
    # Resolve EE bodies
    # -----------------------------------------------------

    left_ee_ids, left_names = (
        robot.find_bodies(
            [LEFT_EE_BODY],
            preserve_order=True,
        )
    )

    right_ee_ids, right_names = (
        robot.find_bodies(
            [RIGHT_EE_BODY],
            preserve_order=True,
        )
    )


    if len(left_ee_ids) != 1:
        raise RuntimeError(
            "Could not find left EE body."
        )

    if len(right_ee_ids) != 1:
        raise RuntimeError(
            "Could not find right EE body."
        )


    left_ee_id = left_ee_ids[0]
    right_ee_id = right_ee_ids[0]


    print(
        "\nLeft EE:",
        left_names[0],
        "id =",
        left_ee_id,
    )

    print(
        "Right EE:",
        right_names[0],
        "id =",
        right_ee_id,
    )


    # -----------------------------------------------------
    # Build HOME configuration
    # -----------------------------------------------------

    # start from whatever the USD defines
    q_home = (
        robot.data.default_joint_pos.clone()
    )

    q_vel = torch.zeros_like(
        q_home
    )


    arm_home = torch.tensor(
        LEFT_HOME + RIGHT_HOME,
        dtype=torch.float32,
        device=robot.device,
    )


    # q_home shape:
    # [num_envs, num_joints]
    q_home[
        0,
        arm_joint_ids
    ] = arm_home


    # -----------------------------------------------------
    # Write HOME immediately
    # -----------------------------------------------------

    robot.write_joint_state_to_sim(
        q_home,
        q_vel,
    )

    robot.reset()


    # -----------------------------------------------------
    # Also set it as controller target
    # -----------------------------------------------------

    robot.set_joint_position_target(
        q_home
    )

    robot.write_data_to_sim()


    # -----------------------------------------------------
    # Run some physics so position drives settle
    # -----------------------------------------------------

    sim_dt = sim.get_physics_dt()

    print(
        "\nMoving F14 to HOME..."
    )

    for _ in range(240):

        robot.set_joint_position_target(
            q_home
        )

        robot.write_data_to_sim()

        sim.step()

        robot.update(
            sim_dt
        )


    # -----------------------------------------------------
    # Read actual joint values
    # -----------------------------------------------------

    actual_q = (
        robot.data.joint_pos[
            0,
            arm_joint_ids
        ]
        .detach()
        .cpu()
        .numpy()
    )


    print(
        "\n===== TARGET ARM Q ====="
    )

    print(
        arm_home
        .detach()
        .cpu()
        .numpy()
    )


    print(
        "\n===== ACTUAL ARM Q ====="
    )

    print(
        actual_q
    )


    # -----------------------------------------------------
    # EE positions
    # -----------------------------------------------------

    left_pos_world = (
        robot.data.body_pos_w[
            0,
            left_ee_id
        ]
        .detach()
        .cpu()
        .numpy()
    )

    right_pos_world = (
        robot.data.body_pos_w[
            0,
            right_ee_id
        ]
        .detach()
        .cpu()
        .numpy()
    )


    print(
        "\n===== ISAAC EE POSITIONS ====="
    )

    print(
        "Left EE :",
        left_pos_world,
    )

    print(
        "Right EE:",
        right_pos_world,
    )


    # -----------------------------------------------------
    # MuJoCo reference
    # -----------------------------------------------------

    mujoco_left = torch.tensor(
        [
            0.42102236,
            0.26728693,
            0.53370183,
        ]
    )

    mujoco_right = torch.tensor(
        [
            0.42102386,
           -0.26728430,
            0.53370183,
        ]
    )


    isaac_left = torch.tensor(
        left_pos_world
    )

    isaac_right = torch.tensor(
        right_pos_world
    )


    left_error = torch.linalg.norm(
        isaac_left
        - mujoco_left
    )

    right_error = torch.linalg.norm(
        isaac_right
        - mujoco_right
    )


    print(
        "\n===== MUJOCO REFERENCE ====="
    )

    print(
        "MuJoCo Left :",
        mujoco_left.numpy()
    )

    print(
        "MuJoCo Right:",
        mujoco_right.numpy()
    )


    print(
        "\n===== POSITION ERROR ====="
    )

    print(
        "Left error :",
        left_error.item(),
        "m"
    )

    print(
        "Right error:",
        right_error.item(),
        "m"
    )


    print(
        "\n========================================"
    )

    print(
        "Keep simulator open..."
    )

    print(
        "Ctrl+C or close the window to exit."
    )


    # -----------------------------------------------------
    # Keep robot at HOME
    # -----------------------------------------------------

    while simulation_app.is_running():

        robot.set_joint_position_target(
            q_home
        )

        robot.write_data_to_sim()

        sim.step()

        robot.update(
            sim_dt
        )


# =========================================================

if __name__ == "__main__":

    main()

    simulation_app.close()
