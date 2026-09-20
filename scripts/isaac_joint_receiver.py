import argparse
import socket
import struct
import time

import numpy as np


# ============================================================
# Isaac launch
# ============================================================

from isaaclab.app import AppLauncher


parser = argparse.ArgumentParser(
    description="F14 Isaac UDP joint target receiver"
)

AppLauncher.add_app_launcher_args(
    parser
)

args_cli = parser.parse_args()


app_launcher = AppLauncher(
    args_cli
)

simulation_app = (
    app_launcher.app
)


# ============================================================
# Isaac imports
# ============================================================

import torch

import isaaclab.sim as sim_utils

from isaaclab.actuators import (
    ImplicitActuatorCfg,
)

from isaaclab.assets import (
    Articulation,
    ArticulationCfg,
)

from isaaclab.sim import (
    SimulationContext,
)


# ============================================================
# Network
# ============================================================

UDP_IP = "127.0.0.1"
UDP_PORT = 5005

MAGIC = b"F14Q"

PACKET_FMT = "!4sId14f"

PACKET_SIZE = struct.calcsize(
    PACKET_FMT
)


# ============================================================
# Simulation
# ============================================================

PHYSICS_HZ = 60.0

COMMAND_TIMEOUT = 0.5


# ============================================================
# USD
# ============================================================

F14_USD_PATH = (
    "/home/cocelo-server01/stkim_ws/quest_robot_isaac/"
    "assets/f14/"
    "FlaminGO_14Dof_Arm_Robot_v2_isaac/"
    "FlaminGO_14Dof_Arm_Robot_v2_isaac.usd"
)


# ============================================================
# HOME
# ============================================================

LEFT_HOME = np.array([
    -0.57,
     0.40,
     0.22,
    -0.95,
     0.24,
    -0.79,
    -0.27,
])


RIGHT_HOME = np.array([
    -0.57,
    -0.40,
     0.22,
     0.95,
    -0.24,
     0.79,
    -0.27,
])


HOME_Q = np.concatenate([
    LEFT_HOME,
    RIGHT_HOME,
])


# ============================================================
# Joint names
# ============================================================

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


# ============================================================
# Articulation
# ============================================================

F14_CFG = ArticulationCfg(

    prim_path="/World/F14",

    spawn=sim_utils.UsdFileCfg(
        usd_path=F14_USD_PATH,
    ),

    init_state=(
        ArticulationCfg.InitialStateCfg(
            pos=(0.0, 0.0, 0.0),
        )
    ),

    actuators={

        "all_joints":
        ImplicitActuatorCfg(

            joint_names_expr=[
                ".*"
            ],

            stiffness=None,
            damping=None,
        )
    },
)


# ============================================================
# Scene
# ============================================================

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


# ============================================================
# Main
# ============================================================

def main():

    # --------------------------------------------------------
    # Simulation
    # --------------------------------------------------------

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


    # --------------------------------------------------------
    # Joint mapping
    # --------------------------------------------------------

    arm_joint_ids, arm_names = (
        robot.find_joints(
            ARM_JOINTS,
            preserve_order=True,
        )
    )


    print("\n===== ARM JOINT MAPPING =====")

    for name, idx in zip(
        arm_names,
        arm_joint_ids,
    ):

        print(
            f"{name:25s} -> {idx}"
        )


    if len(arm_joint_ids) != 14:

        raise RuntimeError(
            "Expected exactly 14 arm joints."
        )


    # --------------------------------------------------------
    # HOME
    # --------------------------------------------------------

    q_all = (
        robot.data.default_joint_pos.clone()
    )

    qd_all = torch.zeros_like(
        q_all
    )


    home_tensor = torch.tensor(

        HOME_Q,

        dtype=torch.float32,

        device=robot.device,

    ).unsqueeze(0)


    q_all[
        :,
        arm_joint_ids
    ] = home_tensor


    robot.write_joint_state_to_sim(
        q_all,
        qd_all,
    )


    robot.reset()


    robot.set_joint_position_target(
        q_all
    )

    robot.write_data_to_sim()


    print("\nMoving F14 to HOME...")


    for _ in range(120):

        robot.set_joint_position_target(
            q_all
        )

        robot.write_data_to_sim()

        sim.step()

        robot.update(
            sim_dt
        )


    # --------------------------------------------------------
    # UDP receiver
    # --------------------------------------------------------

    sock = socket.socket(
        socket.AF_INET,
        socket.SOCK_DGRAM,
    )


    sock.setsockopt(
        socket.SOL_SOCKET,
        socket.SO_REUSEADDR,
        1,
    )


    sock.bind(
        (
            UDP_IP,
            UDP_PORT,
        )
    )


    sock.setblocking(
        False
    )


    print(
        f"\nListening on "
        f"{UDP_IP}:{UDP_PORT}"
    )

    print(
        "Waiting for Quest IK sender..."
    )


    latest_q = HOME_Q.copy()

    latest_seq = 0

    last_receive_time = None

    last_print_time = 0.0


    try:

        while simulation_app.is_running():

            loop_start = time.perf_counter()


            # =================================================
            # Drain UDP queue
            # Keep only newest command
            # =================================================

            while True:

                try:

                    packet, _ = (
                        sock.recvfrom(
                            4096
                        )
                    )

                except BlockingIOError:

                    break


                if len(packet) != PACKET_SIZE:
                    continue


                unpacked = struct.unpack(
                    PACKET_FMT,
                    packet,
                )


                magic = unpacked[0]

                if magic != MAGIC:
                    continue


                seq = unpacked[1]

                sender_timestamp = (
                    unpacked[2]
                )


                q_received = np.array(
                    unpacked[3:],
                    dtype=np.float32,
                )


                if (
                    q_received.shape
                    != (14,)
                ):
                    continue


                if not np.all(
                    np.isfinite(
                        q_received
                    )
                ):
                    continue


                latest_q = q_received

                latest_seq = seq

                last_receive_time = (
                    time.monotonic()
                )


            # =================================================
            # Convert latest command to tensor
            # =================================================

            q_target = torch.tensor(

                latest_q,

                dtype=torch.float32,

                device=robot.device,

            ).unsqueeze(0)


            # =================================================
            # Apply arm target
            # =================================================

            robot.set_joint_position_target(

                q_target,

                joint_ids=arm_joint_ids,

            )


            # Isaac Lab requires this after target update.
            robot.write_data_to_sim()


            # =================================================
            # Physics
            # =================================================

            sim.step()


            robot.update(
                sim_dt
            )


            # =================================================
            # Status
            # =================================================

            now = time.monotonic()


            if (
                now - last_print_time
                > 0.2
            ):

                if last_receive_time is None:

                    state = "WAITING"

                elif (
                    now
                    - last_receive_time
                    > COMMAND_TIMEOUT
                ):

                    state = "HOLD"

                else:

                    state = "ACTIVE"


                print(

                    "\r"
                    f"state={state:7s} | "
                    f"seq={latest_seq:07d} | "
                    f"q0={latest_q[0]:+.3f} | "
                    f"q7={latest_q[7]:+.3f}",

                    end="",

                    flush=True,
                )


                last_print_time = now


            # =================================================
            # Real-time pacing
            # =================================================

            elapsed = (
                time.perf_counter()
                - loop_start
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

        print("\nStopping receiver...")


    finally:

        sock.close()


if __name__ == "__main__":

    try:

        main()

    finally:

        simulation_app.close()
