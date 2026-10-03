"""Process B: F14 drives + 3 RGB observations + asynchronous Quest video.

AppLauncher MUST run before scene/camera imports. No TeleVuer, Vuer, or
Pinocchio is imported here. UDP targets remain held on packet/tracking loss;
camera buffers feed both a dataset-ready observation and a display composite.
"""
import argparse
import json
from pathlib import Path
import signal
import time
import traceback

import numpy as np

from config import teleop_config as cfg
from robot.f14_config import HOME_ACTION, EE_BODY_NAMES, USD_ARM_SIGNS
from teleop.action_protocol import ActionReceiver
from robot.joint_motion import JointMotionLimiter


def parser():
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument("--action-port", type=int, default=cfg.ACTION_UDP_PORT)
    result.add_argument("--video-endpoint", default=cfg.VIDEO_ZMQ_ENDPOINT)
    result.add_argument("--camera-fps", type=float, default=cfg.CAMERA_FPS)
    result.add_argument("--steps", type=int, default=0, help="Exit after N physics steps; 0 runs continuously")
    result.add_argument("--snapshot-dir", type=Path, help="Save raw RGB and display PNGs for camera alignment")
    result.add_argument("--camera-debug", action="store_true", help="Add colored landmarks and update camera poses")
    result.add_argument("--scene-seed", type=int, help="Repeat a tabletop cube layout; omitted means fresh random placement")
    result.add_argument("--smoke-test", action="store_true", help="360-step HOME/finger/wrist/camera validation, no UDP actuation")
    return result


def run(simulation_app, args, observation_callback=None, stop_requested=lambda: False):
    """Optional callback(observation, action_16, sim_time) is the recorder seam.

The callback receives borrowed GPU tensors; clone if retaining them. It must
return quickly. Training, disk recording, and task logic are intentionally not
part of this teleoperation loop.
    """
    import torch
    import isaaclab.sim as sim_utils
    from simulation.f14_scene import create_scene, resolve_joint_ids, initialize_home, apply_action, semantic_state
    from simulation.cameras import create_cameras, raw_observation, display_rgb_copies, save_snapshots
    from simulation.quest_view_compositor import compose_quest_view
    from teleop.xr_video import VideoPublisher

    sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=1. / cfg.PHYSICS_HZ, device=args.device))
    sim.set_camera_view(eye=[1.8, 1.4, 1.3], target=[.3, 0., .5])
    receiver = publisher = None
    cameras = {}
    try:
        robot = create_scene(args.camera_debug, args.scene_seed)
        cameras = create_cameras(args.camera_fps, args.camera_debug or args.smoke_test)
        sim.reset()
        # Lab 2.3.2's camera XformPrimView lazily seeds Fabric transforms from
        # USD on its FIRST pose read. Do that while the articulation still has
        # its USD pose; doing it after HOME can overwrite the wrist camera's
        # world transform with the stale zero-joint pose. Subsequent reads are
        # Fabric-only and naturally follow their parent link.
        for camera in cameras.values():
            camera.update(0., force_recompute=True)
        arm_ids, finger_ids = resolve_joint_ids(robot)
        print("USD finger limits:", robot.data.joint_pos_limits[0, finger_ids].cpu().numpy())
        ee_ids, ee_names = robot.find_bodies(EE_BODY_NAMES, preserve_order=True)
        if ee_names != EE_BODY_NAMES:
            raise RuntimeError(f"EE lookup failed: {ee_names}")
        initialize_home(robot, arm_ids, finger_ids)
        limits = robot.data.joint_pos_limits[0, arm_ids].cpu().numpy()
        limits = np.sort(limits * USD_ARM_SIGNS[:, None], axis=1)
        receiver = ActionReceiver(cfg.ACTION_UDP_HOST, args.action_port, limits)
        measured_state = semantic_state(robot, arm_ids, finger_ids)
        measured_action = measured_state[0].cpu().numpy()
        receiver.update_feedback(measured_action)
        motion = JointMotionLimiter(measured_action[:14], cfg.ARM_MAX_VELOCITY,
                                    cfg.ARM_MAX_ACCELERATION, cfg.ARM_MAX_TRACKING_ERROR)
        publisher = VideoPublisher(args.video_endpoint)
        print(f"F14 ready: UDP {cfg.ACTION_UDP_HOST}:{args.action_port}; video PUB {args.video_endpoint}")
        print("HOME arms, both grippers OPEN. UDP WAITING until calibrated Quest action arrives.")
        print(f"Arm motion limits: speed={cfg.ARM_MAX_VELOCITY} rad/s, acceleration={cfg.ARM_MAX_ACCELERATION} rad/s^2")
        dt = sim.get_physics_dt()
        frame_dt = 1. / args.camera_fps
        frame_accumulator = 0.
        action = HOME_ACTION.copy()
        last_log = time.monotonic()
        frame_count = 0
        steps = 0
        snapshot_saved = False
        checks = {}
        home_camera_poses = None
        while simulation_app.is_running() and not stop_requested() and (args.steps == 0 or steps < args.steps):
            loop_start = time.monotonic()
            if args.smoke_test:
                # Independent gripper phases also catch a left/right swap.
                goal = HOME_ACTION.copy()
                if 90 <= steps < 180:
                    goal[14] = 1.
                elif 180 <= steps < 270:
                    goal[15] = 1.
                elif steps >= 270:
                    goal[0] += .08
                    goal[7] += .08
                arms_enabled = True
            else:
                receiver.poll()
                goal = receiver.action
                arms_enabled = receiver.arms_enabled
            # The UDP action is only the IK destination. A continuous drive
            # reference advances from measured startup joints on EVERY physics
            # step. New goals/IK recovery never reset reference or velocity.
            action = goal.copy()
            action[:14] = motion.step(goal[:14], measured_action[:14], dt, arms_enabled)
            apply_action(robot, action, arm_ids, finger_ids)
            robot.write_data_to_sim()
            # Physics is 60 Hz. Rendering and GPU->CPU camera copies are only
            # scheduled at camera rate; a delayed loop never plays catch-up.
            sim.step(render=False)
            robot.update(dt)
            measured_state = semantic_state(robot, arm_ids, finger_ids)
            measured_action = measured_state[0].cpu().numpy()
            receiver.update_feedback(measured_action)
            steps += 1
            frame_accumulator += dt
            for camera in cameras.values():
                camera.update(dt)
            if frame_accumulator + 1e-9 >= frame_dt:
                frame_accumulator %= frame_dt
                sim.render()
                captured = time.monotonic()
                observation = raw_observation(cameras, measured_state)
                if observation_callback:
                    observation_callback(observation, action.copy(), steps * dt)
                images = display_rgb_copies(observation)
                status = f"UDP {receiver.state} ARM {'FOLLOW' if arms_enabled else 'BRAKE/HOLD'} seq={receiver.latest.sequence if receiver.latest else 0}"
                composite = compose_quest_view(images, status)
                publisher.submit(composite, captured)
                frame_count += 1
                if args.snapshot_dir and steps >= 60 and not snapshot_saved:
                    save_snapshots(args.snapshot_dir, images, composite)
                    if args.camera_debug or args.smoke_test:
                        poses = {name: {"pos_w": cam.data.pos_w[0].cpu().tolist(),
                                        "quat_w_world": cam.data.quat_w_world[0].cpu().tolist()}
                                 for name, cam in cameras.items()}
                        poses["ee"] = {"pos_w": robot.data.body_pos_w[0, ee_ids].cpu().tolist(),
                                       "quat_w": robot.data.body_quat_w[0, ee_ids].cpu().tolist()}
                        (args.snapshot_dir / "camera_poses.json").write_text(json.dumps(poses, indent=2) + "\n")
                        from isaaclab.utils.math import combine_frame_transforms
                        for name, body_id in zip(("left_wrist", "right_wrist"), ee_ids):
                            offset = torch.tensor(cameras[name].cfg.offset.pos, device=robot.device).unsqueeze(0)
                            expected, _ = combine_frame_transforms(robot.data.body_pos_w[:, body_id], robot.data.body_quat_w[:, body_id], offset)
                            discrepancy = float(torch.linalg.norm(cameras[name].data.pos_w - expected))
                            if discrepancy > .001:
                                raise AssertionError(f"{name} mount offset mismatch: {discrepancy} m")
                    print("RGB snapshot shapes:", {k: v.shape for k, v in images.items()})
                    snapshot_saved = True
                if args.smoke_test and steps >= 60 and home_camera_poses is None:
                    home_camera_poses = {name: (cam.data.pos_w.clone(), cam.data.quat_w_world.clone()) for name, cam in cameras.items()}
                if args.smoke_test and steps >= 358:
                    for name, rgb in images.items():
                        if np.std(rgb.astype(float)) < 1.:
                            raise AssertionError(f"{name}: camera image is blank")
                    moved = {name: float(torch.linalg.norm(cam.data.pos_w - home_camera_poses[name][0])) for name, cam in cameras.items()}
                    if moved["front"] > 1e-5 or min(moved["left_wrist"], moved["right_wrist"]) < .001:
                        raise AssertionError(f"Camera parenting validation failed: {moved}")
                    checks["camera_motion_m"] = moved
                    checks["rgb_shapes"] = {k: list(v.shape) for k, v in images.items()}
                    if args.snapshot_dir:
                        save_snapshots(args.snapshot_dir, images, composite, "_moved")
            if args.smoke_test and steps in (80, 170, 260):
                measured = semantic_state(robot, arm_ids, finger_ids)[0].cpu().numpy()
                arm_error = float(np.max(np.abs(measured[:14] - action[:14])))
                grip_error = float(np.max(np.abs(measured[14:] - action[14:])))
                if arm_error > .01 or grip_error > .03:
                    raise AssertionError(f"F14 target check step={steps}: arm={arm_error}, gripper={grip_error}; measured={measured.tolist()}; fingers={robot.data.joint_pos[0, finger_ids].cpu().tolist()}")
                checks[f"step_{steps}"] = {"arm_error_rad": arm_error, "gripper_error": grip_error}
                if steps == 80:
                    actual = robot.data.body_pos_w[0, ee_ids].cpu().numpy()
                    reference = np.array([[.42102236, .26728693, .53370183], [.42102386, -.26728430, .53370183]])
                    errors = np.linalg.norm(actual - reference, axis=1)
                    if np.max(errors) > .001:
                        raise AssertionError(f"HOME FK mismatch: {errors}")
                    checks["home_fk_error_m"] = errors.tolist()
                print("Smoke check:", steps, checks[f"step_{steps}"])
            now = time.monotonic()
            if now - last_log >= cfg.LOG_INTERVAL:
                print(f"UDP {receiver.state} seq={receiver.latest.sequence if receiver.latest else 0} age={receiver.age:.3f}s "
                      f"qL0={action[0]:+.3f} qR0={action[7]:+.3f} grip={action[14:].round(2)} "
                      f"arm={'FOLLOW' if arms_enabled else 'BRAKE/HOLD'} lag_limit={bool(motion.tracking_limited.any())} "
                      f"camera={frame_count / (now-last_log):.1f}fps PUB={'ERROR: '+publisher.error if publisher.error else 'RUNNING'} "
                      f"rejected={receiver.rejected}")
                last_log, frame_count = now, 0
            time.sleep(max(0., dt - (time.monotonic() - loop_start)))
        if args.smoke_test:
            if steps < 360 or "camera_motion_m" not in checks:
                raise RuntimeError("Smoke test interrupted before all checks completed")
            print("ISAAC SMOKE TEST PASSED", json.dumps(checks, indent=2))
            if args.snapshot_dir:
                (args.snapshot_dir / "validation.json").write_text(json.dumps(checks, indent=2) + "\n")
    finally:
        if publisher:
            publisher.close()
        if receiver:
            receiver.close()
        # Camera's destructor detaches annotators/render products. Release it
        # while Kit is still alive, then let the outer launcher close the app.
        cameras.clear()
        camera = None  # Release the loop variable while Kit is still alive.
        import gc
        gc.collect()
        sim.clear_all_callbacks()
        # Lab 2.3.2 has a standalone STOP callback that renders forever while
        # paused. clear_instance unsubscribes it BEFORE stopping the timeline;
        # otherwise an exception or Ctrl+C during a headless run can hang here.
        sim.clear_instance()
        sim.stop()


def main(argv=None):
    from isaaclab.app import AppLauncher
    arguments = parser()
    AppLauncher.add_app_launcher_args(arguments)
    args = arguments.parse_args(argv)
    if not 0 < args.camera_fps <= cfg.PHYSICS_HZ:
        arguments.error(f"--camera-fps must be in (0, {cfg.PHYSICS_HZ}]")
    if args.steps < 0:
        arguments.error("--steps must be nonnegative")
    if args.smoke_test:
        args.steps = max(args.steps, 360)
    # Required even in --headless: camera rendering must stay enabled.
    args.enable_cameras = True
    # Keep Kit's supported default fast shutdown after explicitly releasing
    # our sockets/threads/sensors. Full extension teardown on this installed
    # 5.1.0 build segfaulted after otherwise successful camera validation.
    launcher = AppLauncher(args)
    # SimulationApp installs a SIGINT handler that unloads Kit plugins before
    # Python finally blocks run. A Python KeyboardInterrupt can ALSO be caught
    # and swallowed by a native physics callback. Set a flag instead; the main
    # loop exits at its next boundary and closes our resources before Kit.
    stopping = False

    def request_stop(signum, frame):
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    try:
        run(launcher.app, args, stop_requested=lambda: stopping)
        print("Isaac teleoperation stopped; sockets, video thread and sensors released.")
    except KeyboardInterrupt:
        print("\nStopping Isaac teleoperation.")
    except Exception:
        # Kit's close may terminate the interpreter before an unhandled error
        # is printed; preserve the actual error BEFORE entering app.close().
        traceback.print_exc()
        raise
    finally:
        launcher.app.close()


if __name__ == "__main__":
    main()
