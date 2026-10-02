"""Process A: Quest controller snapshots -> relative SE(3) -> IK -> UDP 16D.

An independent video SUB receives Isaac's three-camera dashboard and forwards
it to TeleVuer. This process NEVER imports Isaac Sim. TeleVuer itself starts
its normal WebSocket child process and shared-memory image writer.
"""
import argparse
from contextlib import ExitStack, closing
import select
import sys
import time

import numpy as np

from config import teleop_config as cfg
from robot.f14_config import F14_URDF_PATH, HOME_Q
from robot.f14_ik import F14IK
from robot.gripper import normalize_input
from teleop.action_protocol import ActionSender
from teleop.xr_pose import RelativePoseMapper, average_pose, samples_are_still


def parser():
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument("--host-ip", help="Company PC LAN IP printed in the Quest URL; autodetected if omitted")
    result.add_argument("--action-port", type=int, default=cfg.ACTION_UDP_PORT)
    result.add_argument("--video-endpoint", default=cfg.VIDEO_ZMQ_ENDPOINT)
    result.add_argument("--xr-mode", choices=("immersive", "ego"), default="immersive")
    result.add_argument("--gripper-input", choices=("trigger", "squeeze"), default="trigger")
    result.add_argument("--trigger-encoding", choices=("auto", "legacy-inverted-10", "standard"), default="auto")
    result.add_argument("--no-filter", action="store_true")
    result.add_argument("--position-only", action="store_true", help="Diagnostic: retain the working translation-only behavior")
    result.add_argument("--verbose", action="store_true")
    return result


def recenter_requested():
    # Canonical terminal input intentionally needs R then Enter; no background
    # keyboard thread or global key grab is required, and Ctrl+C stays normal.
    if sys.stdin.isatty() and select.select([sys.stdin], [], [], 0)[0]:
        return sys.stdin.readline().strip().lower() == "r"
    return False


def main(argv=None):
    args = parser().parse_args(argv)
    # Late imports keep --help independent of TeleVuer server startup.
    from teleop.televuer_adapter import QuestInterface, detect_host_ip, trigger_encoding
    from teleop.xr_video import VideoSubscriber, send_image_to_xr, waiting_image

    encoding = trigger_encoding(args.trigger_encoding) if args.gripper_input == "trigger" else "standard"
    ik = F14IK(F14_URDF_PATH)
    print("HOME EE positions:", [p.translation for p in ik.forward_kinematics(HOME_Q)])
    print(f"Gripper input={args.gripper_input}, encoding={encoding}; semantic 0=open, 1=closed")
    print("R + Enter = recalibrate at held robot pose. Ctrl+C = clean shutdown.")
    try:
        with ExitStack() as stack:
            # TeleVuer forks internally. Start it BEFORE constructing ZMQ's
            # background I/O threads, avoiding fork-after-ZMQ initialization.
            tv = stack.enter_context(closing(QuestInterface(args.xr_mode)))
            tv.print_url(args.host_ip or detect_host_ip())
            sender = stack.enter_context(closing(ActionSender(cfg.ACTION_UDP_HOST, args.action_port)))
            video = stack.enter_context(closing(VideoSubscriber(args.video_endpoint)))
            placeholder = waiting_image()
            send_image_to_xr(tv, placeholder)
            print("Waiting for Quest controller tracking and Isaac receiver...")
            mappers = None
            samples = [[], []]
            settle_start = None
            last_serial = -1
            last_log = 0.
            q_current = HOME_Q.copy()
            grippers = np.zeros(2)
            frames = 0
            fps_start = time.monotonic()
            previous_state = None
            stale_image_sent = False
            while True:
                start = time.monotonic()
                if not tv.tvuer.process.is_alive():
                    raise RuntimeError("TeleVuer child exited; check port 8012 and SSL errors above")
                restarted = sender.poll_feedback()
                data, fresh, serial = tv.snapshot()
                if restarted or recenter_requested():
                    mappers, settle_start, samples = None, None, [[], []]
                    print("Receiver connected/restarted or recenter requested; calibrating at held target.")
                bgr = video.receive()
                if bgr is not None:
                    send_image_to_xr(tv, bgr)
                    frames += 1
                    stale_image_sent = False
                elif video.age > cfg.VIDEO_TIMEOUT and not stale_image_sent:
                    send_image_to_xr(tv, waiting_image("Isaac video unavailable / stale"))
                    stale_image_sent = True

                if not fresh or not sender.ready:
                    # No keepalive action is sent on tracking/receiver loss.
                    # After COMMAND_TIMEOUT the simulator holds the LAST target.
                    mappers, settle_start, samples = None, None, [[], []]
                    state = "TRACKING LOST / WAITING" if not fresh else "WAITING FOR ISAAC"
                elif mappers is None:
                    state = "CALIBRATING"
                    if settle_start is None:
                        settle_start = start
                        q_current = sender.held_action[:14].astype(float).copy()
                        grippers = sender.held_action[14:].astype(float).copy()
                        print("Quest websocket connected; Tracking ready.")
                        print("Hold both controllers and head still; release triggers. Settling for 2 seconds...")
                    elif start - settle_start >= cfg.CALIBRATION_SETTLE_SECONDS and serial != last_serial:
                        samples[0].append(data.left_wrist_pose.copy())
                        samples[1].append(data.right_wrist_pose.copy())
                        if len(samples[0]) >= cfg.CALIBRATION_SAMPLES:
                            if not all(samples_are_still(s) for s in samples):
                                print("Calibration motion detected; hold still and retry.")
                                samples = [[], []]
                            else:
                                # During the sampling interval any final in-
                                # flight command has reached Isaac. Anchor at
                                # that fresh held target, not a reply captured
                                # while the operator was still moving before R.
                                q_current = sender.held_action[:14].astype(float).copy()
                                grippers = sender.held_action[14:].astype(float).copy()
                                anchors = ik.forward_kinematics(q_current)
                                mappers = [RelativePoseMapper(average_pose(s), anchor, not args.no_filter)
                                           for s, anchor in zip(samples, anchors)]
                                print("Calibration complete. Teleoperation ACTIVE.")
                else:
                    targets = [mapper.target(pose) for mapper, pose in zip(mappers, (data.left_wrist_pose, data.right_wrist_pose))]
                    if args.position_only:
                        for target, mapper in zip(targets, mappers):
                            target.rotation = mapper.anchor.rotation.copy()
                    solution, success = ik.solve(
                        *targets, q_current, max_iter=cfg.IK_MAX_ITER, eps=cfg.IK_EPS,
                        dt=cfg.IK_DT, damping=cfg.IK_DAMPING)
                    if success and np.isfinite(solution).all():
                        q_current = solution
                    state = "ACTIVE" if success else "IK FAIL: holding last valid arm q"
                    raw = [getattr(data, f"{side}_ctrl_{args.gripper_input}Value") for side in ("left", "right")]
                    closure = np.array([normalize_input(value, encoding) for value in raw])
                    alpha = 1. if args.no_filter else cfg.GRIPPER_FILTER_ALPHA
                    grippers += alpha * (closure - grippers)
                    # IK failure holds both arms; valid gripper input remains
                    # independent. Stale tracking stops ALL action transmission.
                    _, still_fresh, _ = tv.snapshot()
                    if still_fresh and sender.ready:
                        sender.send(np.concatenate((q_current, grippers)))
                last_serial = serial
                if state != previous_state and not state.startswith(("ACTIVE", "IK FAIL")):
                    print(state)
                previous_state = state
                interval = .25 if args.verbose else cfg.LOG_INTERVAL
                if start - last_log >= interval:
                    info = ""
                    if mappers:
                        info = (f" Lxyz={mappers[0].delta.round(3)} Rxyz={mappers[1].delta.round(3)}"
                                f" Lrot={mappers[0].angle_degrees:.1f}deg Rrot={mappers[1].angle_degrees:.1f}deg")
                        if args.verbose:
                            info += f" rawL={mappers[0].raw_delta.round(3)} rawR={mappers[1].raw_delta.round(3)} residual={ik.last_error:.3g}"
                    print(f"{state} seq={sender.sequence} IK={'OK' if state == 'ACTIVE' else 'FAIL' if state.startswith('IK FAIL') else '-'}"
                          f"{info} grip={grippers.round(2)} video={frames / max(start-fps_start, 1e-6):.1f}fps age={video.age:.2f}s")
                    last_log, fps_start, frames = start, start, 0
                time.sleep(max(0., 1. / cfg.CONTROL_HZ - (time.monotonic() - start)))
    except KeyboardInterrupt:
        print("\nQuest stopped; Isaac will HOLD its last valid target.")


if __name__ == "__main__":
    main()
