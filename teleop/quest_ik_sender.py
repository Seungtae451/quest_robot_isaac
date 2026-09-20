import socket
import struct
import sys
import time
from pathlib import Path

import numpy as np


# ============================================================
# Project path
# ============================================================

PROJECT_ROOT = Path(__file__).resolve().parents[1]

sys.path.insert(
    0,
    str(PROJECT_ROOT / "robot"),
)


from televuer import TeleVuerWrapper
from f14_ik import F14IK


# ============================================================
# Network
# ============================================================

DEST_IP = "127.0.0.1"
DEST_PORT = 5005

MAGIC = b"F14Q"

# magic + sequence + timestamp + 14 float32
PACKET_FMT = "!4sId14f"


# ============================================================
# URDF
# ============================================================

F14_URDF_PATH = (
    "/home/cocelo-server01/stkim_ws/quest_robot_isaac/"
    "assets/f14/F14_URDF_rev_2_0_0/urdf/"
    "FlaminGO_14Dof_Arm_Robot_v2_mujoco.urdf"
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


# ============================================================
# Teleop settings
# ============================================================

CONTROL_HZ = 30.0

POSITION_SCALE = 1.0

MAX_DELTA = np.array([
    0.30,
    0.30,
    0.30,
])

DEADBAND = 0.005

FILTER_ALPHA = 0.2


# ============================================================
# Helpers
# ============================================================

def apply_deadband(v):

    result = v.copy()

    result[
        np.abs(result) < DEADBAND
    ] = 0.0

    return result


def clip_delta(v):

    return np.clip(
        v,
        -MAX_DELTA,
        MAX_DELTA,
    )


# ============================================================
# Main
# ============================================================

def main():

    print("\n======================================")
    print("F14 Quest IK Sender")
    print("======================================")

    # --------------------------------------------------------
    # IK
    # --------------------------------------------------------

    print("\nLoading F14 IK...")

    ik = F14IK(
        F14_URDF_PATH
    )

    left_home, right_home = (
        ik.forward_kinematics(
            HOME_Q
        )
    )

    print("Left HOME EE :", left_home.translation)
    print("Right HOME EE:", right_home.translation)


    q_current = HOME_Q.copy()


    # --------------------------------------------------------
    # UDP
    # --------------------------------------------------------

    sock = socket.socket(
        socket.AF_INET,
        socket.SOCK_DGRAM,
    )

    print(
        f"\nUDP target: "
        f"{DEST_IP}:{DEST_PORT}"
    )


    # --------------------------------------------------------
    # TeleVuer
    # --------------------------------------------------------

    tv = TeleVuerWrapper(
        use_hand_tracking=False,
        display_mode="pass-through",
    )

    print(
        "\nWaiting for Quest controller tracking..."
    )


    while True:

        data = tv.get_tele_data()

        if data.motion_data_ready:
            break

        time.sleep(0.1)


    print("\nQuest tracking detected.")

    print(
        "Hold both controllers still "
        "at the desired starting pose..."
    )

    time.sleep(2.0)


    # --------------------------------------------------------
    # Calibration
    # --------------------------------------------------------

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

        time.sleep(0.02)


    left_start = np.mean(
        left_samples,
        axis=0,
    )

    right_start = np.mean(
        right_samples,
        axis=0,
    )


    print("\nCalibration complete.")

    print(
        "Quest LEFT start :",
        left_start
    )

    print(
        "Quest RIGHT start:",
        right_start
    )


    # --------------------------------------------------------
    # Filters
    # --------------------------------------------------------

    left_filtered = np.zeros(
        3,
        dtype=np.float64,
    )

    right_filtered = np.zeros(
        3,
        dtype=np.float64,
    )


    seq = 0

    period = 1.0 / CONTROL_HZ


    print("\nSending F14 joint targets...")
    print("Ctrl+C to stop.\n")


    try:

        while True:

            loop_start = time.perf_counter()

            data = tv.get_tele_data()


            if not data.motion_data_ready:

                print(
                    "\rQuest tracking LOST",
                    end="",
                    flush=True,
                )

                time.sleep(0.02)

                continue


            # =================================================
            # Controller position
            # =================================================

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


            # =================================================
            # Relative displacement
            # =================================================

            left_delta = (
                left_current
                - left_start
            )

            right_delta = (
                right_current
                - right_start
            )


            left_delta = apply_deadband(
                left_delta
            )

            right_delta = apply_deadband(
                right_delta
            )


            left_delta *= POSITION_SCALE
            right_delta *= POSITION_SCALE


            left_delta = clip_delta(
                left_delta
            )

            right_delta = clip_delta(
                right_delta
            )


            # =================================================
            # Low-pass
            # =================================================

            left_filtered = (
                FILTER_ALPHA
                * left_delta
                + (1.0 - FILTER_ALPHA)
                * left_filtered
            )

            right_filtered = (
                FILTER_ALPHA
                * right_delta
                + (1.0 - FILTER_ALPHA)
                * right_filtered
            )


            # =================================================
            # EE target
            # =================================================

            target_left = left_home.copy()
            target_right = right_home.copy()


            target_left.translation = (
                left_home.translation
                + left_filtered
            )

            target_right.translation = (
                right_home.translation
                + right_filtered
            )


            # =================================================
            # IK
            # =================================================

            q_solution, success = ik.solve(

                target_left,
                target_right,

                q_current,

                max_iter=20,

                eps=2e-4,

                dt=0.3,

                damping=1e-4,

                verbose=False,
            )


            if success:

                q_current = (
                    q_solution.copy()
                )


                # =============================================
                # UDP packet
                # =============================================

                seq += 1

                packet = struct.pack(
                    PACKET_FMT,
                    MAGIC,
                    seq,
                    time.time(),
                    *q_current.astype(
                        np.float32
                    ),
                )


                sock.sendto(
                    packet,
                    (
                        DEST_IP,
                        DEST_PORT,
                    ),
                )


            print(
                "\r"
                f"seq={seq:07d} | "
                f"L={np.round(left_filtered, 3)} | "
                f"R={np.round(right_filtered, 3)} | "
                f"IK={success}",
                end="",
                flush=True,
            )


            elapsed = (
                time.perf_counter()
                - loop_start
            )


            sleep_time = (
                period
                - elapsed
            )


            if sleep_time > 0:
                time.sleep(
                    sleep_time
                )


    except KeyboardInterrupt:

        print("\nStopping sender...")


    finally:

        tv.close()
        sock.close()


if __name__ == "__main__":
    main()
