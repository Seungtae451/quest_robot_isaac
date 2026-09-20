import argparse
import sys
import time
from pathlib import Path

import numpy as np


# =========================================================
# Project path
# =========================================================

PROJECT_ROOT = Path(__file__).resolve().parents[1]

sys.path.insert(
    0,
    str(PROJECT_ROOT / "robot"),
)


# =========================================================
# Isaac Sim launch
# =========================================================

from isaaclab.app import AppLauncher


parser = argparse.ArgumentParser(
    description="Quest 3 dual-arm teleoperation for F14 in Isaac Sim."
)

AppLauncher.add_app_launcher_args(parser)

args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)

simulation_app = app_launcher.app


# =========================================================
# Imports after Isaac launch
# =========================================================

import torch

import isaaclab.sim as sim_utils

from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import Articulation, ArticulationCfg
from isaaclab.sim import SimulationContext

from televuer import TeleVuerWrapper

from f14_ik import F14IK


# =========================================================
# Paths
# =========================================================

F14_USD_PATH = (
    "/home/cocelo-server01/stkim_ws/quest_robot_isaac/"
    "assets/f14/"
    "FlaminGO_14Dof_Arm_Robot_v2_isaac/"
    "FlaminGO_14Dof_Arm_Robot_v2_isaac.usd"
)

F14_URDF_PATH = (
    "/home/cocelo-server01/stkim_ws/quest_robot_isaac/"
    "assets/f14/F14_URDF_rev_2_0_0/urdf/"
    "FlaminGO_14Dof_Arm_Robot_v2_mujoco.urdf"
)

# =========================================================
# F14 HOME
# =========================================================

LEFT_HOME = np.array([
    -0.57,
     0.40,
     0.22,
    -0.95,
     0.24,
    -0.79,
    -0.27,
], dtype=np.float64)


RIGHT_HOME = np.array([
    -0.57,
    -0.40,
     0.22,
     0.95,
    -0.24,
     0.79,
    -0.27,
], dtype=np.float64)


HOME_Q = np.concatenate([
    LEFT_HOME,
    RIGHT_HOME,
])


# =========================================================
# Joint names
# =========================================================

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


# =========================================================
# Teleoperation settings
# =========================================================

PHYSICS_HZ = 60.0

CONTROL_HZ = 30.0

CONTROL_DECIMATION = int(
    PHYSICS_HZ / CONTROL_HZ
)


# Same behavior that worked in MuJoCo
POSITION_SCALE = 1.0


MAX_DELTA = np.array([
    0.30,
    0.30,
    0.30,
])


DEADBAND = 0.005


FILTER_ALPHA = 0.2


# =========================================================
# Helper functions
# =========================================================

def apply_deadband(v, threshold):

    result = v.copy()

    result[
        np.abs(result) < threshold
    ] = 0.0

    return result


def clip_delta(delta):

    return np.clip(
        delta,
        -MAX_DELTA,
        MAX_DELTA,
    )


# =========================================================
# F14 Isaac config
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

        "all_joints": ImplicitActuatorCfg(

            joint_names_expr=[
                ".*"
            ],

            # use USD drive settings
            stiffness=None,
            damping=None,
        ),
    },
)


# =========================================================
# Scene
# =========================================================

def create_scene():

    ground_cfg = (
        sim_utils.GroundPlaneCfg()
    )

    ground_cfg.func(
        "/World/defaultGroundPlane",
        ground_cfg,
    )


    light_cfg = (
        sim_utils.DomeLightCfg(
            intensity=3000.0,
            color=(
                0.75,
                0.75,
                0.75,
            ),
        )
    )

    light_cfg.func(
        "/World/Light",
        light_cfg,
    )


    robot = Articulation(
        cfg=F14_CFG
    )

    return robot


# =========================================================
# Main
# =========================================================

def main():

    # -----------------------------------------------------
    # Isaac simulation
    # -----------------------------------------------------

    sim_cfg = sim_utils.SimulationCfg(

        dt=1.0 / PHYSICS_HZ,

        device=args_cli.device,
    )


    sim = SimulationContext(
        sim_cfg
    )


    sim.set_camera_view(
        eye=[
            1.8,
            1.4,
            1.3,
        ],

        target=[
            0.3,
            0.0,
            0.5,
        ],
    )


    robot = create_scene()


    sim.reset()


    sim_dt = sim.get_physics_dt()


    print(
        "\n======================================"
    )

    print(
        "F14 Isaac Teleoperation"
    )

    print(
        "======================================"
    )


    # -----------------------------------------------------
    # Resolve joints by name
    # -----------------------------------------------------

    arm_joint_ids, arm_joint_names = (
        robot.find_joints(
            ARM_JOINTS,
            preserve_order=True,
        )
    )


    if len(arm_joint_ids) != 14:

        raise RuntimeError(
            f"Expected 14 arm joints, "
            f"found {len(arm_joint_ids)}"
        )


    print(
        "\n===== ARM JOINT MAPPING ====="
    )


    for name, idx in zip(
        arm_joint_names,
        arm_joint_ids,
    ):

        print(
            f"{name:25s} -> {idx}"
        )


    # -----------------------------------------------------
    # Set F14 HOME
    # -----------------------------------------------------

    q_home_all = (
        robot.data.default_joint_pos.clone()
    )


    q_vel_all = torch.zeros_like(
        q_home_all
    )


    home_tensor = torch.tensor(

        HOME_Q,

        dtype=torch.float32,

        device=robot.device,

    ).unsqueeze(0)


    q_home_all[
        :,
        arm_joint_ids
    ] = home_tensor


    robot.write_joint_state_to_sim(
        q_home_all,
        q_vel_all,
    )


    robot.reset()


    robot.set_joint_position_target(
        q_home_all
    )


    robot.write_data_to_sim()


    print(
        "\nMoving F14 to HOME..."
    )


    for _ in range(120):

        robot.set_joint_position_target(
            q_home_all
        )

        robot.write_data_to_sim()

        sim.step()

        robot.update(
            sim_dt
        )


    # -----------------------------------------------------
    # Pinocchio IK
    # -----------------------------------------------------

    print(
        "\nLoading F14 IK..."
    )


    ik = F14IK(
        F14_URDF_PATH
    )


    left_home, right_home = (
        ik.forward_kinematics(
            HOME_Q
        )
    )


    print(
        "\n===== HOME EE ====="
    )

    print(
        "Left :",
        left_home.translation
    )

    print(
        "Right:",
        right_home.translation
    )


    q_current = HOME_Q.copy()


    # -----------------------------------------------------
    # TeleVuer
    # -----------------------------------------------------

    print(
        "\nStarting TeleVuer..."
    )


    tv = TeleVuerWrapper(

        use_hand_tracking=False,

        display_mode="pass-through",
    )


    print(
        "Waiting for Quest controller tracking..."
    )


    while (
        simulation_app.is_running()
    ):

        data = tv.get_tele_data()

        if data.motion_data_ready:
            break

        time.sleep(
            0.1
        )


    print(
        "\nController tracking detected."
    )


    print(
        "Hold both controllers still "
        "at the desired starting position..."
    )


    time.sleep(
        2.0
    )


    # -----------------------------------------------------
    # Quest calibration
    # -----------------------------------------------------

    left_samples = []
    right_samples = []


    for _ in range(30):

        data = tv.get_tele_data()


        left_samples.append(

            data.left_wrist_pose[
                :3,
                3
            ].copy()

        )


        right_samples.append(

            data.right_wrist_pose[
                :3,
                3
            ].copy()

        )


        time.sleep(
            0.02
        )


    left_start = np.mean(
        left_samples,
        axis=0,
    )


    right_start = np.mean(
        right_samples,
        axis=0,
    )


    print(
        "\nCalibration complete."
    )


    print(
        "Quest LEFT start :",
        left_start
    )


    print(
        "Quest RIGHT start:",
        right_start
    )


    print(
        "\nTeleoperation ACTIVE"
    )


    print(
        "Position-only dual-arm control."
    )


    # -----------------------------------------------------
    # Filters
    # -----------------------------------------------------

    left_filtered = np.zeros(
        3,
        dtype=np.float64,
    )


    right_filtered = np.zeros(
        3,
        dtype=np.float64,
    )


    # current Isaac target
    q_target_isaac = home_tensor.clone()


    physics_count = 0


    try:

        # =================================================
        # Main physics loop
        # =================================================

        while (
            simulation_app.is_running()
        ):

            frame_start = time.perf_counter()


            # ---------------------------------------------
            # Run teleop controller at 30 Hz
            # ---------------------------------------------

            if (
                physics_count
                % CONTROL_DECIMATION
                == 0
            ):

                data = tv.get_tele_data()


                if data.motion_data_ready:

                    # =====================================
                    # Quest positions
                    # =====================================

                    left_current = (

                        data.left_wrist_pose[
                            :3,
                            3
                        ]

                    )


                    right_current = (

                        data.right_wrist_pose[
                            :3,
                            3
                        ]

                    )


                    # =====================================
                    # Relative motion
                    # =====================================

                    left_delta = (

                        left_current
                        - left_start

                    )


                    right_delta = (

                        right_current
                        - right_start

                    )


                    # =====================================
                    # Deadband
                    # =====================================

                    left_delta = apply_deadband(

                        left_delta,
                        DEADBAND,

                    )


                    right_delta = apply_deadband(

                        right_delta,
                        DEADBAND,

                    )


                    # =====================================
                    # Scale
                    # =====================================

                    left_delta *= (
                        POSITION_SCALE
                    )


                    right_delta *= (
                        POSITION_SCALE
                    )


                    # =====================================
                    # Workspace clamp
                    # =====================================

                    left_delta = clip_delta(
                        left_delta
                    )


                    right_delta = clip_delta(
                        right_delta
                    )


                    # =====================================
                    # Low-pass filtering
                    # =====================================

                    left_filtered = (

                        FILTER_ALPHA
                        * left_delta

                        + (
                            1.0
                            - FILTER_ALPHA
                        )
                        * left_filtered

                    )


                    right_filtered = (

                        FILTER_ALPHA
                        * right_delta

                        + (
                            1.0
                            - FILTER_ALPHA
                        )
                        * right_filtered

                    )


                    # =====================================
                    # EE targets
                    # =====================================

                    target_left = (
                        left_home.copy()
                    )


                    target_right = (
                        right_home.copy()
                    )


                    target_left.translation = (

                        left_home.translation
                        + left_filtered

                    )


                    target_right.translation = (

                        right_home.translation
                        + right_filtered

                    )


                    # =====================================
                    # Dual-arm IK
                    # =====================================

                    q_solution, success = (
                        ik.solve(

                            target_left,
                            target_right,

                            q_current,

                            max_iter=20,

                            eps=2e-4,

                            dt=0.3,

                            damping=1e-4,

                            verbose=False,
                        )
                    )


                    if success:

                        q_current = (
                            q_solution.copy()
                        )


                        q_target_isaac = (
                            torch.tensor(

                                q_current,

                                dtype=torch.float32,

                                device=robot.device,

                            ).unsqueeze(0)
                        )


                    print(

                        "\r"
                        f"L Δxyz="
                        f"{np.round(left_filtered, 3)} | "

                        f"R Δxyz="
                        f"{np.round(right_filtered, 3)} | "

                        f"IK={success}",

                        end="",

                        flush=True,
                    )


            # ---------------------------------------------
            # Apply 14 arm targets to Isaac
            # ---------------------------------------------

            robot.set_joint_position_target(

                q_target_isaac,

                joint_ids=arm_joint_ids,

            )


            robot.write_data_to_sim()


            # ---------------------------------------------
            # Physics
            # ---------------------------------------------

            sim.step()


            robot.update(
                sim_dt
            )


            physics_count += 1


            # ---------------------------------------------
            # Keep approximately real-time
            # ---------------------------------------------

            elapsed = (
                time.perf_counter()
                - frame_start
            )


            sleep_time = (
                sim_dt
                - elapsed
            )


            if sleep_time > 0:

                time.sleep(
                    sleep_time
                )


    except KeyboardInterrupt:

        print(
            "\nStopping..."
        )


    finally:

        tv.close()


# =========================================================
# Entry
# =========================================================

if __name__ == "__main__":

    try:

        main()

    finally:

        simulation_app.close()
