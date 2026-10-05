"""Process A: fixed-world Quest AR grips -> XYZ grasp-tip differential IK -> UDP 16D.

AR mirrors measured Isaac links/cubes over a separate read-only stream. Legacy
2D TeleVuer remains opt-in. This process NEVER imports Isaac Sim.
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
from config import table_assist_config as assist_cfg
from robot.f14_config import F14_URDF_PATH, HOME_Q, GRIPPER_CONTROL_OFFSET
from robot.f14_ik import F14IK
from robot.differential_ik import DifferentialIK
from robot.gripper import normalize_input
from teleop.action_protocol import ActionSender
from teleop.xr_pose import TranslationPoseMapper, WorldPoseMapper, front_facing_controls, average_pose, samples_are_still
from teleop.input_timing import timed_filter_alpha
from teleop.motion_speed import MotionSpeedMonitor
from teleop.episode_protocol import EpisodeClient
from teleop.ar_start import start_decision, start_message
from config import recording_config as recording_cfg


def parser():
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument("--host-ip", help="Company PC LAN IP printed in the Quest URL; autodetected if omitted")
    result.add_argument("--action-port", type=int, default=cfg.ACTION_UDP_PORT)
    result.add_argument("--session-port", type=int, default=recording_cfg.SESSION_PORT)
    result.add_argument("--video-endpoint", default=cfg.VIDEO_ZMQ_ENDPOINT)
    result.add_argument("--xr-mode", choices=("ar", "immersive", "ego"), default="ar",
                        help="ar: fixed-world passthrough scene; immersive/ego: legacy 2D camera panel")
    from config import ar_config
    result.add_argument("--ar-port", type=int, default=ar_config.HTTPS_PORT)
    result.add_argument("--ar-scene-endpoint", default=ar_config.SCENE_ENDPOINT)
    result.add_argument("--gripper-input", choices=("trigger", "squeeze"), default="trigger")
    result.add_argument("--trigger-encoding", choices=("auto", "legacy-inverted-10", "standard"), default="auto")
    result.add_argument("--no-filter", action="store_true")
    result.add_argument("--position-only", action="store_true", help="Ignore controller rotation (default follows position and rotation)")
    result.add_argument("--verbose", action="store_true")
    result.add_argument("--wrist-axis-test", action="store_true", help="Observe controller rotation vs XYZ-only targets; N + Enter starts each trial (rotation never commands the robot)")
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
    from teleop.xr_tls import detect_host_ip
    ar_mode = args.xr_mode == "ar"
    if ar_mode:
        from teleop.quest_ar_adapter import QuestARInterface
        if args.trigger_encoding not in ("auto", "standard"):
            parser().error("AR uses native WebXR analog input; --trigger-encoding must be auto or standard")
        encoding = "standard"
    else:
        from teleop.televuer_adapter import QuestInterface, trigger_encoding
        from teleop.xr_video import VideoSubscriber, send_image_to_xr, waiting_image
        encoding = trigger_encoding(args.trigger_encoding) if args.gripper_input == "trigger" else "standard"
    ik = F14IK(F14_URDF_PATH, ee_offset=GRIPPER_CONTROL_OFFSET)
    reachable = DifferentialIK(ik, position_only=args.position_only, rotating_geometry=True)
    PoseMapper = TranslationPoseMapper if args.position_only else WorldPoseMapper
    print("HOME EE positions:", [p.translation for p in ik.forward_kinematics(HOME_Q)])
    print("Control: controller XYZ displacement + grippers; grasp-tip XYZ only, orientation FREE; right hand -> left arm, left hand -> right arm.")
    print(f"Translation mapping: robot XYZ signs={cfg.TRANSLATION_AXIS_SIGNS}; AR world-aligned XYZ (no extra inversion); source={Path(__file__).resolve()}")
    print("IK: Pinocchio + ProxQP; independent arms, bounded differential joint references.")
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
            interface = QuestARInterface(args.ar_scene_endpoint, args.ar_port) if ar_mode else QuestInterface(args.xr_mode)
            tv = stack.enter_context(closing(interface))
            tv.print_url(args.host_ip or detect_host_ip())
            sender = stack.enter_context(closing(ActionSender(cfg.ACTION_UDP_HOST, args.action_port)))
            episodes = stack.enter_context(closing(EpisodeClient(cfg.ACTION_UDP_HOST, args.session_port)))
            video = None
            if not ar_mode:
                video = stack.enter_context(closing(VideoSubscriber(args.video_endpoint)))
                send_image_to_xr(tv, waiting_image())
            print("Waiting for Quest controller tracking and Isaac receiver...")
            reach_result = None
            ar_active = False
            ar_epoch = tv.control_epoch if ar_mode else None
            last_ar_start = None
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
            solve_times, input_ages, loop_times = [], [], []
            motion_speed = MotionSpeedMonitor()
            args.timing_output.parent.mkdir(parents=True, exist_ok=True)
            timing_file = stack.enter_context(args.timing_output.open("a", buffering=1))
            while True:
                start = time.monotonic()
                loops += 1
                if not (tv.is_alive if ar_mode else tv.tvuer.process.is_alive()):
                    raise RuntimeError("Quest server exited; check HTTPS port and SSL errors above")
                restarted = sender.poll_feedback()
                episodes.poll()
                data, fresh, serial = tv.snapshot()
                data = front_facing_controls(data)
                if ar_mode:
                    if tv.control_epoch != ar_epoch:
                        ar_epoch = tv.control_epoch
                        if ar_active and episodes.available and episodes.status['state'] == 'RECORDING':
                            episodes.button('b')
                        ar_active = False
                        mappers = None
                    if ar_active and (not fresh or restarted or not sender.ready):
                        if episodes.available and episodes.status['state'] == 'RECORDING':
                            episodes.button('b')
                        ar_active = False
                    if episodes.available and episodes.status['state'] not in ('RECORDING', 'STARTING', 'READY'):
                        ar_active = False
                for button, press_poses in tv.recording_button_events(with_poses=True):
                    if press_poses is not None:
                        press_poses = press_poses[::-1].copy()  # physical hands -> robot arms
                    can_start = (press_poses is not None and mappers is not None and fresh and sender.ready and not restarted
                                 and episodes.status.get("start_allowed", False)) if episodes.available else False
                    if ar_mode:
                        attachment = tv.attachment_status(press_poses, sender.boot_time)
                        attached = not attachment['reasons']
                        decision = start_decision(attachment, fresh=fresh, receiver_ready=sender.ready,
                            restarted=restarted, mapper_ready=mappers is not None, episode=episodes.status,
                            episode_available=episodes.available,
                            command_busy=episodes.pending is not None or bool(episodes.queued))
                        can_start = decision['allowed']
                        if button == 'a' and not ar_active and (episodes.status is None or episodes.status['state'] == 'READY'):
                            last_ar_start = {**decision, 'time': start, 'input_sequence': serial,
                                             'message': start_message(decision)}
                            print('AR start ' + ('accepted: ' if can_start else 'rejected: ') + last_ar_start['message'])
                        if not episodes.available and episodes.status is None:
                            if button in ('b', 'x'):
                                ar_active = False
                            elif button == 'a' and not ar_active and attached and fresh and sender.ready and not restarted:
                                q_current = sender.measured_state[:14].astype(float).copy()
                                reachable.reset(q_current)
                                grippers = sender.measured_state[14:].astype(float).copy()
                                mappers = [PoseMapper(pose.copy(), anchor, not args.no_filter, side=side)
                                    for side, pose, anchor in zip(('left', 'right'), press_poses, ik.forward_kinematics(q_current))]
                                last_control_time = start
                                ar_active = True
                    if episodes.available and (button != "a" or episodes.status["state"] != "READY" or can_start):
                        if episodes.button(button) and button == "a" and episodes.status["state"] == "READY":
                            # The press pose, not an earlier waiting/calibration
                            # pose, is zero. Moving controllers before A cannot
                            # become a jump when the writer acknowledges start.
                            q_current = sender.measured_state[:14].astype(float).copy()
                            reachable.reset(q_current)
                            grippers = sender.measured_state[14:].astype(float).copy()
                            anchors = ik.forward_kinematics(q_current)
                            mappers = [PoseMapper(pose.copy(), anchor, not args.no_filter, side=side)
                                for side, pose, anchor in zip(("left", "right"),
                                    press_poses, anchors)]
                            last_control_time = start
                            if ar_mode:
                                ar_active = True
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
                    if ar_mode:
                        if ar_active and episodes.available and episodes.status['state'] == 'RECORDING':
                            episodes.button('b')
                        ar_active = False
                        print("Isaac/reset updated or R requested; HOLD. Reattach both grasp tips with A.")
                    else:
                        print("Receiver connected/restarted or recenter requested; calibrating at measured robot pose.")
                bgr = video.receive() if video else None
                if bgr is not None:
                    send_image_to_xr(tv, bgr)
                    frames += 1
                    stale_image_sent = False
                elif video and video.age > cfg.VIDEO_TIMEOUT and not stale_image_sent:
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
                    if ar_mode and not tv.placed:
                        state = "AR PLACE WORLD / A TO LOCK"
                elif mappers is None:
                    state = "AR LINK TIPS / A TO RECORD" if ar_mode else "CALIBRATING"
                    # Stop an in-flight old IK goal immediately while the new
                    # neutral pose is being collected; keep the gripper state.
                    sender.send(np.concatenate((sender.measured_state[:14], grippers)), hold_arms=True)
                    if ar_mode:
                        q_current = sender.measured_state[:14].astype(float).copy()
                        reachable.reset(q_current)
                        grippers = sender.measured_state[14:].astype(float).copy()
                        mappers = [PoseMapper(pose.copy(), anchor, not args.no_filter, side=side)
                            for side, pose, anchor in zip(('left', 'right'),
                                (data.left_wrist_pose, data.right_wrist_pose), ik.forward_kinematics(q_current))]
                        last_control_time = start
                    elif settle_start is None:
                        settle_start = start
                        q_current = sender.measured_state[:14].astype(float).copy()
                        grippers = sender.measured_state[14:].astype(float).copy()
                        print("Quest websocket connected; Tracking ready.")
                        print("Hold both controllers and head still; release triggers. Settling for 2 seconds...")
                    elif start - settle_start >= cfg.CALIBRATION_SETTLE_SECONDS and serial != last_serial:
                        samples[0].append(data.left_wrist_pose.copy())
                        samples[1].append(data.right_wrist_pose.copy())
                        if len(samples[0]) >= cfg.CALIBRATION_SAMPLES:
                            if not all(samples_are_still(s, check_rotation=not args.position_only) for s in samples):
                                print("Calibration motion detected; hold still and retry.")
                                samples = [[], []]
                            else:
                                # Recalibrate at the latest measured pose, never
                                # a distant IK destination still being pursued.
                                q_current = sender.measured_state[:14].astype(float).copy()
                                reachable.reset(q_current)
                                grippers = sender.measured_state[14:].astype(float).copy()
                                anchors = ik.forward_kinematics(q_current)
                                mappers = [PoseMapper(average_pose(s), anchor, not args.no_filter, side=side)
                                           for side, s, anchor in zip(("left", "right"), samples, anchors)]
                                last_control_time = start
                                print("Calibration complete. HOME HOLD; A starts recording and teleoperation."
                                      if episodes.preparing_episode else "Calibration complete. Teleoperation ACTIVE.")
                elif episodes.preparing_episode:
                    # Keep tracking freshness for A eligibility, with the actual
                    # open grippers held and no IK or trigger input applied.
                    sender.send(sender.measured_state, hold_arms=True)
                    state = "HOME HOLD / A TO RECORD" if episodes.status["state"] == "READY" else "STARTING RECORDING / HOME HOLD"
                elif ar_mode and not ar_active:
                    # Never resume after loss/reset just because fresh poses
                    # returned. A new explicit attachment is required.
                    sender.send(sender.measured_state, hold_arms=True)
                    state = "AR LINK TIPS / A TO START" if episodes.status is None else "AR PAUSED / B TO REVIEW"
                elif serial != last_serial:
                    # Consume the newest event once. No repeated filtering/IK
                    # or action keepalives for a controller event that stopped.
                    control_dt = 1. / cfg.FILTER_REFERENCE_HZ if last_control_time is None else start - last_control_time
                    last_control_time = start
                    skipped_events += max(0, serial - last_serial - 1)
                    input_ages.append(max(0., start - tv.last_snapshot_controller_time) * 1000)
                    targets = [mapper.target(pose, dt=control_dt) for mapper, pose in zip(mappers, (data.left_wrist_pose, data.right_wrist_pose))]
                    motion_speed.observe('controller', tv.last_snapshot_controller_time,
                        [data.left_wrist_pose[:3, 3], data.right_wrist_pose[:3, 3]])
                    motion_speed.observe('filtered_target', start, [t.translation for t in targets])
                    table_limited = [False, False]
                    table_ready = True
                    qp_feedback = None
                    if assist_cfg.ENABLED and episodes.available and episodes.status["state"] == "RECORDING":
                        from robot.table_approach import limit_targets
                        targets, table_limited, table_ready = limit_targets(
                            targets, sender.table_feedback, sender.boot_time, start, tcp_only=True)
                        if table_ready:
                            qp_feedback = sender.table_feedback
                        else:
                            reachable.reset(sender.measured_state[:14])
                    solve_started = time.perf_counter()
                    reach_result = reachable.solve(
                        *targets, sender.measured_state[:14], dt=control_dt, feedback=qp_feedback)
                    solve_times.append((time.perf_counter() - solve_started) * 1000)
                    motion_speed.observe('guarded_target', start, [t.translation for t in targets])
                    motion_speed.observe('qp_reference', start, reach_result.diagnostics['reference_position_m'])
                    motion_speed.observe('measured_robot', sender.last_feedback, reach_result.diagnostics['measured_position_m'])
                    solves += 1
                    solution, success = reach_result.q, reach_result.success
                    success = success and table_ready
                    if wrist_test:
                        wrist_test.update(
                            (data.left_wrist_pose, data.right_wrist_pose), targets,
                            ik.forward_kinematics(sender.measured_state[:14]), success,
                            time.monotonic() - sender.last_feedback)
                    if success:
                        q_current = solution
                    else:
                        q_current = sender.measured_state[:14].astype(float).copy()
                    state = ("ACTIVE / QP ARM BRAKE" if reach_result.mode == "QP_PARTIAL" else "ACTIVE") if success else "IK HOLD: no feasible step"
                    if not table_ready:
                        state = "TABLE/BOX ASSIST WAITING: braking arms"
                    elif any(table_limited):
                        state += " / TABLE/BOX LIMIT " + "/".join(side for side, limited in zip(("L", "R"), table_limited) if limited)
                    raw = [getattr(data, f"{side}_ctrl_{args.gripper_input}Value") for side in ("left", "right")]
                    closure = np.array([normalize_input(value, encoding) for value in raw])
                    if args.gripper_input == "trigger":
                        closure = 1.0 - closure
                    alpha = 1. if args.no_filter else timed_filter_alpha(cfg.GRIPPER_FILTER_ALPHA, control_dt, cfg.FILTER_REFERENCE_HZ)
                    grippers += alpha * (closure - grippers)
                    # A rejected QP brakes that arm; if both fail, request
                    # receiver braking. Tracking loss stops all transmission.
                    _, still_fresh, _ = tv.snapshot()
                    if still_fresh and sender.ready:
                        sender.send(np.concatenate((q_current, grippers)), hold_arms=not success,
                                    direct_reference=success)
                if not state.startswith(('ACTIVE', 'IK HOLD')):
                    motion_speed.reset()  # Never differentiate across A/recalibration/reset.
                last_serial = serial
                if state != previous_state and not state.startswith(("ACTIVE", "IK HOLD")):
                    print(state)
                previous_state = state
                if ar_mode:
                    attachment = tv.attachment_status(np.stack((data.left_wrist_pose, data.right_wrist_pose)), sender.boot_time)
                    ar_gate = start_decision(attachment, fresh=fresh, receiver_ready=sender.ready,
                        restarted=restarted, mapper_ready=mappers is not None, episode=episodes.status,
                        episode_available=episodes.available,
                        command_busy=episodes.pending is not None or bool(episodes.queued))
                    tv.update_status(state, episodes.status if episodes.available else None,
                                     attachment['distances_m'], ar_gate, last_ar_start)
                interval = .25 if args.verbose else cfg.LOG_INTERVAL
                if start - last_log >= interval:
                    elapsed = max(start - fps_start, 1e-6)
                    timing = tv.input_timing()
                    stats = {"monotonic_time": start, "state": state,
                             "input": timing, "control_tick_hz": loops / elapsed,
                             "ik_hz": solves / elapsed, "skipped_events": skipped_events,
                             "motion_speed": motion_speed.summary(start),
                             "ik_follow": ({"mode": reach_result.mode, "fraction": reach_result.fraction,
                                            "attempts": reach_result.attempts,
                                            "remaining_mm": reach_result.remaining_mm,
                                            "diagnostics": reach_result.diagnostics}
                                           if reach_result and state.startswith(("ACTIVE", "IK HOLD")) else None),
                             "ik_p50_ms": float(np.median(solve_times)) if solve_times else None,
                             "ik_p95_ms": float(np.percentile(solve_times, 95)) if solve_times else None,
                             "ik_max_ms": float(np.max(solve_times)) if solve_times else None,
                             "loop_p95_ms": float(np.percentile(loop_times, 95)) if loop_times else None,
                             "loop_max_ms": float(np.max(loop_times)) if loop_times else None,
                             "input_age_p95_ms": float(np.percentile(input_ages, 95)) if input_ages else None}
                    if mappers:
                        stats['translation_mapping'] = {
                            'signs': list(cfg.TRANSLATION_AXIS_SIGNS),
                            'controller_delta_robot_m': [((pose[:3,3]-mapper.start[:3,3]).tolist())
                                for mapper,pose in zip(mappers,(data.left_wrist_pose,data.right_wrist_pose))],
                            'mapped_delta_m': [mapper.raw_delta.tolist() for mapper in mappers],
                            'filtered_delta_m': [mapper.delta.tolist() for mapper in mappers]}
                    if ar_mode:
                        stats['ar_start'] = {'current': ar_gate, 'last_press': last_ar_start}
                    timing_file.write(json.dumps(stats) + "\n")
                    info = ""
                    if mappers:
                        info = (f" Lxyz={mappers[0].delta.round(3)} Rxyz={mappers[1].delta.round(3)}"
                                " orientation=CONTROLLER tcp=GRIPPER_MIDDLE hand_map=R_TO_L/L_TO_R")
                        if state.startswith("ACTIVE") and reach_result:
                            info += (f" step={reach_result.mode} fraction={reach_result.fraction:.3f}"
                                     f" remainingL/R_mm={np.round(reach_result.remaining_mm, 1)}"
                                     f" attempts={reach_result.attempts}")
                            info += f" qpL/R={reach_result.diagnostics['arm_status']}"
                        speed = stats['motion_speed']
                        for key, label in (('controller', 'hand'), ('measured_robot', 'robot')):
                            if speed.get(key):
                                info += f" {label}L/R_cm_s={np.round(speed[key]['mean_cm_s'], 1)}"
                        if args.verbose:
                            info += f" rawL={mappers[0].raw_delta.round(3)} rawR={mappers[1].raw_delta.round(3)} residual={ik.last_error:.3g}"
                        if state.startswith("IK HOLD") and reach_result and reach_result.diagnostics:
                            diagnostic = reach_result.diagnostics
                            info += (f" posErrL/R_mm={np.round(diagnostic['position_error_mm'], 2)}"
                                     f" rotErrL/R_deg={np.round(diagnostic['rotation_error_deg'], 2)}"
                                     f" nearLimits={diagnostic['near_limits']} iter={diagnostic['iterations']}")
                    ik_status = ('PARTIAL' if state.startswith('ACTIVE / QP ARM BRAKE') else
                                 'OK' if state.startswith('ACTIVE') else
                                 'HOLD' if state.startswith('IK HOLD') else '-')
                    if ar_mode and (episodes.preparing_episode or episodes.status is None):
                        info += ' attach/start=' + start_message(ar_gate)
                    print(f"{state} seq={sender.sequence} IK={ik_status}"
                          f"{info} grip={grippers.round(2)} video={frames / elapsed:.1f}fps age={video.age if video else 0.:.2f}s"
                          f" input={timing['controller']['hz']:.1f}Hz solve={stats['ik_hz']:.1f}Hz"
                          f" ikP95={stats['ik_p95_ms']}ms inputAgeP95={stats['input_age_p95_ms']}ms skipped={skipped_events}")
                    last_log, fps_start, frames = start, start, 0
                    loops = solves = skipped_events = 0
                    solve_times, input_ages, loop_times = [], [], []
                loop_times.append((time.monotonic() - start) * 1000.)
                time.sleep(max(0., 1. / cfg.CONTROL_HZ - (time.monotonic() - start)))
    except KeyboardInterrupt:
        if wrist_test:
            wrist_test.interrupt("operator stopped")
        print("\nQuest stopped; Isaac will HOLD its last valid target.")


if __name__ == "__main__":
    main()
