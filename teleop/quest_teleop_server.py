"""Process A: Quest XYZ + grippers -> downward EE pose IK -> UDP 16D.

An independent video SUB receives Isaac's three-camera dashboard and forwards
it to TeleVuer. This process NEVER imports Isaac Sim. TeleVuer itself starts
its normal WebSocket child process and shared-memory image writer.
"""
import argparse
import json
from contextlib import ExitStack, closing
import select
import sys
import time
from pathlib import Path

import numpy as np

from config import teleop_config as cfg
from robot.f14_config import F14_URDF_PATH, HOME_Q
from robot.f14_ik import F14IK
from robot.gripper import normalize_input
from teleop.action_protocol import ActionSender
from teleop.xr_pose import DownwardPoseMapper, average_pose, samples_are_still
from teleop.input_timing import timed_filter_alpha
from teleop.episode_protocol import EpisodeClient
from config import recording_config as recording_cfg


def parser():
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument("--host-ip", help="Company PC LAN IP printed in the Quest URL; autodetected if omitted")
    result.add_argument("--action-port", type=int, default=cfg.ACTION_UDP_PORT)
    result.add_argument("--session-port", type=int, default=recording_cfg.SESSION_PORT)
    result.add_argument("--video-endpoint", default=cfg.VIDEO_ZMQ_ENDPOINT)
    result.add_argument("--xr-mode", choices=("immersive", "ego"), default="immersive")
    result.add_argument("--gripper-input", choices=("trigger", "squeeze"), default="trigger")
    result.add_argument("--trigger-encoding", choices=("auto", "legacy-inverted-10", "standard"), default="auto")
    result.add_argument("--no-filter", action="store_true")
    result.add_argument("--position-only", action="store_true", help="Compatibility option: XYZ with fixed downward grippers is now always enabled")
    result.add_argument("--verbose", action="store_true")
    result.add_argument("--wrist-axis-test", action="store_true", help="Observe controller rotation vs fixed downward targets; N + Enter starts each trial (rotation never commands the robot)")
    result.add_argument("--wrist-test-dir", type=Path, default=Path("outputs/wrist_axis_check"))
    result.add_argument("--timing-output", type=Path, default=Path("outputs/quest_input_check/runtime.jsonl"),
                        help="Append input-rate, processing-latency and IK timing measurements")
    return result


def terminal_command():
    # Canonical terminal input intentionally needs R then Enter; no background
    # keyboard thread or global key grab is required, and Ctrl+C stays normal.
    if sys.stdin.isatty() and select.select([sys.stdin], [], [], 0)[0]:
        return sys.stdin.readline().strip().lower()
    return ""


def main(argv=None):
    args = parser().parse_args(argv)
    # Late imports keep --help independent of TeleVuer server startup.
    from teleop.televuer_adapter import QuestInterface, detect_host_ip, trigger_encoding
    from teleop.xr_video import VideoSubscriber, send_image_to_xr, waiting_image

    encoding = trigger_encoding(args.trigger_encoding) if args.gripper_input == "trigger" else "standard"
    ik = F14IK(F14_URDF_PATH)
    print("HOME EE positions:", [p.translation for p in ik.forward_kinematics(HOME_Q)])
    print("Control: controller XYZ displacement + grippers; both EE orientations fixed vertically downward.")
    print(f"Gripper input={args.gripper_input}, encoding={encoding}; semantic 0=open, 1=closed")
    print("R + Enter = recalibrate at held robot pose. Ctrl+C = clean shutdown.")
    wrist_test = None
    if args.wrist_axis_test:
        from teleop.wrist_axis_test import WristAxisTest
        wrist_test = WristAxisTest(args.wrist_test_dir)
    try:
        with ExitStack() as stack:
            # TeleVuer forks internally. Start it BEFORE constructing ZMQ's
            # background I/O threads, avoiding fork-after-ZMQ initialization.
            tv = stack.enter_context(closing(QuestInterface(args.xr_mode)))
            tv.print_url(args.host_ip or detect_host_ip())
            sender = stack.enter_context(closing(ActionSender(cfg.ACTION_UDP_HOST, args.action_port)))
            episodes = stack.enter_context(closing(EpisodeClient(cfg.ACTION_UDP_HOST, args.session_port)))
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
            previous_recording_state = None
            stale_image_sent = False
            last_control_time = None
            loops = solves = skipped_events = 0
            solve_times, input_ages = [], []
            args.timing_output.parent.mkdir(parents=True, exist_ok=True)
            timing_file = stack.enter_context(args.timing_output.open("a", buffering=1))
            while True:
                start = time.monotonic()
                loops += 1
                if not tv.tvuer.process.is_alive():
                    raise RuntimeError("TeleVuer child exited; check port 8012 and SSL errors above")
                restarted = sender.poll_feedback()
                episodes.poll()
                data, fresh, serial = tv.snapshot()
                for button, press_poses in tv.recording_button_events(with_poses=True):
                    can_start = (press_poses is not None and mappers is not None and fresh and sender.ready and not restarted
                                 and episodes.status.get("start_allowed", False)) if episodes.available else False
                    if episodes.available and (button != "a" or episodes.status["state"] != "READY" or can_start):
                        if episodes.button(button) and button == "a" and episodes.status["state"] == "READY":
                            # The press pose, not an earlier waiting/calibration
                            # pose, is zero. Moving controllers before A cannot
                            # become a jump when the writer acknowledges start.
                            q_current = sender.measured_state[:14].astype(float).copy()
                            grippers = sender.measured_state[14:].astype(float).copy()
                            anchors = ik.forward_kinematics(q_current)
                            mappers = [DownwardPoseMapper(pose.copy(), anchor, not args.no_filter, side=side)
                                for side, pose, anchor in zip(("left", "right"),
                                    press_poses, anchors)]
                            last_control_time = start
                            print("New episode neutral captured at A press: "
                                  f"Lxyz={np.round(press_poses[0,:3,3], 4)} "
                                  f"Rxyz={np.round(press_poses[1,:3,3], 4)}")
                if episodes.available and episodes.status["state"] != previous_recording_state:
                    previous_recording_state = episodes.status["state"]
                    print(f"Episode: {previous_recording_state}; frames={episodes.status['frames']} "
                          f"saved={episodes.status['saved_episodes']} {episodes.status.get('error', '')}")
                command = terminal_command()
                if command == "n" and wrist_test and mappers is not None and fresh and sender.ready:
                    wrist_test.request_start()
                if restarted or command == "r":
                    if wrist_test:
                        wrist_test.interrupt("recalibration / receiver restarted")
                    mappers, settle_start, samples = None, None, [[], []]
                    print("Receiver connected/restarted or recenter requested; calibrating at measured robot pose.")
                bgr = video.receive()
                if bgr is not None:
                    send_image_to_xr(tv, bgr)
                    frames += 1
                    stale_image_sent = False
                elif video.age > cfg.VIDEO_TIMEOUT and not stale_image_sent:
                    send_image_to_xr(tv, waiting_image("Isaac video unavailable / stale"))
                    stale_image_sent = True

                if episodes.blocks_teleop and not episodes.preparing_episode:
                    # Isaac owns braking/reset and gripper hold between episodes.
                    # Buttons remain usable in REVIEW; no stale IK is sent.
                    mappers, settle_start, samples = None, None, [[], []]
                    state = f"EPISODE {episodes.status['state']}"
                elif not fresh or not sender.ready:
                    if wrist_test:
                        wrist_test.interrupt("tracking / feedback lost")
                    # No keepalive action is sent on tracking/receiver loss.
                    # After COMMAND_TIMEOUT the simulator brakes at its current
                    # trajectory position instead of chasing an old IK goal.
                    mappers, settle_start, samples = None, None, [[], []]
                    state = "TRACKING LOST / WAITING" if not fresh else "WAITING FOR ISAAC"
                elif mappers is None:
                    state = "CALIBRATING"
                    # Stop an in-flight old IK goal immediately while the new
                    # neutral pose is being collected; keep the gripper state.
                    sender.send(np.concatenate((sender.measured_state[:14], grippers)), hold_arms=True)
                    if settle_start is None:
                        settle_start = start
                        q_current = sender.measured_state[:14].astype(float).copy()
                        grippers = sender.measured_state[14:].astype(float).copy()
                        print("Quest websocket connected; Tracking ready.")
                        print("Hold both controllers and head still; release triggers. Settling for 2 seconds...")
                    elif start - settle_start >= cfg.CALIBRATION_SETTLE_SECONDS and serial != last_serial:
                        samples[0].append(data.left_wrist_pose.copy())
                        samples[1].append(data.right_wrist_pose.copy())
                        if len(samples[0]) >= cfg.CALIBRATION_SAMPLES:
                            if not all(samples_are_still(s, check_rotation=False) for s in samples):
                                print("Calibration motion detected; hold still and retry.")
                                samples = [[], []]
                            else:
                                # Recalibrate at the latest measured pose, never
                                # a distant IK destination still being pursued.
                                q_current = sender.measured_state[:14].astype(float).copy()
                                grippers = sender.measured_state[14:].astype(float).copy()
                                anchors = ik.forward_kinematics(q_current)
                                mappers = [DownwardPoseMapper(average_pose(s), anchor, not args.no_filter, side=side)
                                           for side, s, anchor in zip(("left", "right"), samples, anchors)]
                                last_control_time = start
                                print("Calibration complete. HOME HOLD; A starts recording and teleoperation."
                                      if episodes.preparing_episode else "Calibration complete. Teleoperation ACTIVE.")
                elif episodes.preparing_episode:
                    # Keep tracking freshness for A eligibility, with the actual
                    # open grippers held and no IK or trigger input applied.
                    sender.send(sender.measured_state, hold_arms=True)
                    state = "HOME HOLD / A TO RECORD" if episodes.status["state"] == "READY" else "STARTING RECORDING / HOME HOLD"
                elif serial != last_serial:
                    # Consume the newest event once. No repeated filtering/IK
                    # or action keepalives for a controller event that stopped.
                    control_dt = 1. / cfg.FILTER_REFERENCE_HZ if last_control_time is None else start - last_control_time
                    last_control_time = start
                    skipped_events += max(0, serial - last_serial - 1)
                    input_ages.append(max(0., start - tv.last_snapshot_controller_time) * 1000)
                    targets = [mapper.target(pose, dt=control_dt) for mapper, pose in zip(mappers, (data.left_wrist_pose, data.right_wrist_pose))]
                    solve_started = time.perf_counter()
                    solution, success = ik.solve(
                        *targets, sender.measured_state[:14], max_iter=cfg.IK_MAX_ITER, eps=cfg.IK_EPS,
                        dt=cfg.IK_DT, damping=cfg.IK_DAMPING)
                    solve_times.append((time.perf_counter() - solve_started) * 1000)
                    solves += 1
                    success = success and np.isfinite(solution).all()
                    if wrist_test:
                        wrist_test.update(
                            (data.left_wrist_pose, data.right_wrist_pose), targets,
                            ik.forward_kinematics(sender.measured_state[:14]), success,
                            time.monotonic() - sender.last_feedback)
                    if success:
                        q_current = solution
                    else:
                        q_current = sender.measured_state[:14].astype(float).copy()
                    state = "ACTIVE" if success else "IK FAIL: braking arms"
                    raw = [getattr(data, f"{side}_ctrl_{args.gripper_input}Value") for side in ("left", "right")]
                    closure = np.array([normalize_input(value, encoding) for value in raw])
                    if args.gripper_input == "trigger":
                        closure = 1.0 - closure
                    alpha = 1. if args.no_filter else timed_filter_alpha(cfg.GRIPPER_FILTER_ALPHA, control_dt, cfg.FILTER_REFERENCE_HZ)
                    grippers += alpha * (closure - grippers)
                    # IK failure explicitly brakes both arms; gripper remains
                    # independent. Stale tracking stops ALL action transmission.
                    _, still_fresh, _ = tv.snapshot()
                    if still_fresh and sender.ready:
                        sender.send(np.concatenate((q_current, grippers)), hold_arms=not success)
                last_serial = serial
                if state != previous_state and not state.startswith(("ACTIVE", "IK FAIL")):
                    print(state)
                previous_state = state
                interval = .25 if args.verbose else cfg.LOG_INTERVAL
                if start - last_log >= interval:
                    elapsed = max(start - fps_start, 1e-6)
                    timing = tv.input_timing()
                    stats = {"monotonic_time": start, "state": state,
                             "input": timing, "control_tick_hz": loops / elapsed,
                             "ik_hz": solves / elapsed, "skipped_events": skipped_events,
                             "ik_p50_ms": float(np.median(solve_times)) if solve_times else None,
                             "ik_p95_ms": float(np.percentile(solve_times, 95)) if solve_times else None,
                             "input_age_p95_ms": float(np.percentile(input_ages, 95)) if input_ages else None}
                    timing_file.write(json.dumps(stats) + "\n")
                    info = ""
                    if mappers:
                        info = (f" Lxyz={mappers[0].delta.round(3)} Rxyz={mappers[1].delta.round(3)}"
                                " orientation=DOWN_FIXED")
                        if args.verbose:
                            info += f" rawL={mappers[0].raw_delta.round(3)} rawR={mappers[1].raw_delta.round(3)} residual={ik.last_error:.3g}"
                        if state.startswith("IK FAIL") and ik.last_diagnostics:
                            diagnostic = ik.last_diagnostics
                            info += (f" posErrL/R_mm={np.round(diagnostic['position_error_mm'], 2)}"
                                     f" rotErrL/R_deg={np.round(diagnostic['rotation_error_deg'], 2)}"
                                     f" nearLimits={diagnostic['near_limits']} iter={diagnostic['iterations']}")
                    print(f"{state} seq={sender.sequence} IK={'OK' if state == 'ACTIVE' else 'FAIL' if state.startswith('IK FAIL') else '-'}"
                          f"{info} grip={grippers.round(2)} video={frames / elapsed:.1f}fps age={video.age:.2f}s"
                          f" input={timing['controller']['hz']:.1f}Hz solve={stats['ik_hz']:.1f}Hz"
                          f" ikP95={stats['ik_p95_ms']}ms inputAgeP95={stats['input_age_p95_ms']}ms skipped={skipped_events}")
                    last_log, fps_start, frames = start, start, 0
                    loops = solves = skipped_events = 0
                    solve_times, input_ages = [], []
                time.sleep(max(0., 1. / cfg.CONTROL_HZ - (time.monotonic() - start)))
    except KeyboardInterrupt:
        if wrist_test:
            wrist_test.interrupt("operator stopped")
        print("\nQuest stopped; Isaac will HOLD its last valid target.")


if __name__ == "__main__":
    main()
