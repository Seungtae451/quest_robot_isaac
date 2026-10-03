"""Real Isaac replay: distant IK -> failed IK hold -> recovery -> packet loss.

Uses the normal receiver/trajectory loop on isolated ports. Checks every
physics-step drive target, measured speed, braking, and runtime state writes.
Run: python scripts/test_joint_motion.py --headless --device cuda:0
"""
import json
from pathlib import Path
import subprocess
import sys
import traceback

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def make_goals():
    # This helper runs in a separate process, keeping Pinocchio out of Isaac.
    from robot.f14_config import HOME_Q, HOME_ACTION, F14_URDF_PATH
    from robot.f14_ik import F14IK
    from config import teleop_config as cfg
    ik = F14IK(F14_URDF_PATH)
    cases = {"home": HOME_ACTION.tolist()}
    for label, shoulder in (("raised", -.60), ("forward", -.15)):
        # Raise the hands away from the physical table/box. This replay checks
        # IK retargeting independently of wall impacts/contact impulses.
        seed = HOME_Q.copy()
        seed[[0, 7]] = shoulder
        targets = list(ik.forward_kinematics(seed))
        q, ok = ik.solve(*targets, seed, max_iter=cfg.IK_MAX_ITER, eps=cfg.IK_EPS,
                         dt=cfg.IK_DT, damping=cfg.IK_DAMPING)
        assert ok, (label, ik.last_error)
        cases[label] = np.r_[q, 0., 0.].tolist()
    targets = list(ik.forward_kinematics(HOME_Q))
    targets[0].translation[0] += 10.
    _, ok = ik.solve(*targets, HOME_Q, max_iter=cfg.IK_MAX_ITER, eps=cfg.IK_EPS,
                     dt=cfg.IK_DT, damping=cfg.IK_DAMPING)
    assert not ok
    print(json.dumps(cases))


def main():
    goals_process = subprocess.run([sys.executable, __file__, "--make-goals"],
                                   capture_output=True, text=True, check=True)
    goals = json.loads(goals_process.stdout.splitlines()[-1])
    from isaaclab.app import AppLauncher
    from simulation.isaac_teleop_app import parser, run
    arguments = parser()
    arguments.add_argument("--output-dir", type=Path, default=ROOT / "outputs/joint_motion_check")
    AppLauncher.add_app_launcher_args(arguments)
    args = arguments.parse_args(sys.argv[1:] + ["--steps", "900", "--scene-seed", "12",
                                              "--action-port", "15025",
                                              "--video-endpoint", "tcp://127.0.0.1:15576"])
    args.enable_cameras = True
    args.output_dir.mkdir(parents=True, exist_ok=True)
    launcher = AppLauncher(args)
    sender = None
    try:
        from config import teleop_config as cfg
        from robot.f14_config import USD_ARM_SIGNS, HOME_Q
        from teleop.action_protocol import ActionSender
        import simulation.f14_scene as scene

        sender = ActionSender("127.0.0.1", args.action_port)
        samples, state_writes = [], []
        home_initialized = False
        original_apply, original_create = scene.apply_action, scene.create_scene

        def create_scene(*args, **kwargs):
            robot = original_create(*args, **kwargs)
            original_write = robot.write_joint_state_to_sim

            def write_state(*args, **kwargs):
                state_writes.append(len(samples))
                return original_write(*args, **kwargs)

            robot.write_joint_state_to_sim = write_state
            return robot

        def apply(robot, action, arm_ids, finger_ids):
            nonlocal home_initialized
            if not home_initialized:
                home_initialized = True
                return original_apply(robot, action, arm_ids, finger_ids)
            step = len(samples)
            if step < 120:
                phase, goal, hold = "home", goals["home"], False
            elif step < 240:
                phase, goal, hold = "large_ik", goals["raised"], False
            elif step < 300:
                phase, goal, hold = "ik_failure", goals["raised"], True
            elif step < 480:
                phase, goal, hold = "ik_recovery", goals["forward"], False
            elif step < 570:
                phase, goal, hold = "packet_loss", None, False
            else:
                phase, goal, hold = "resume_home", goals["home"], False
            if goal is not None and step % 2 == 0:
                sender.send(goal, hold_arms=hold)
            sender.poll_feedback()
            samples.append({"step": step, "phase": phase, "drive_q": action[:14].tolist(),
                            "measured_q": (robot.data.joint_pos[0, arm_ids].cpu().numpy() * USD_ARM_SIGNS).tolist(),
                            "measured_velocity": (robot.data.joint_vel[0, arm_ids].cpu().numpy() * USD_ARM_SIGNS).tolist()})
            if step in (0, 119, 299, 479, 899):
                print("Motion sample", samples[-1], flush=True)
            return original_apply(robot, action, arm_ids, finger_ids)

        scene.create_scene, scene.apply_action = create_scene, apply
        run(launcher.app, args)
        (args.output_dir / "samples.json").write_text(json.dumps(samples) + "\n")
        assert len(samples) == 900, len(samples)
        dt = 1 / cfg.PHYSICS_HZ
        drive = np.array([s["drive_q"] for s in samples])
        measured = np.array([s["measured_q"] for s in samples])
        measured_v = np.array([s["measured_velocity"] for s in samples])
        command_v = np.diff(drive, axis=0) / dt
        command_a = np.diff(command_v, axis=0) / dt
        peak_v, peak_a = float(np.max(np.abs(command_v))), float(np.max(np.abs(command_a)))
        # Compare measured position changes too: a state jump cannot be hidden
        # behind a smooth command or an instantaneous solver velocity report.
        actual_position_v = np.diff(measured, axis=0) / dt
        peak_actual_v = float(np.max(np.abs(actual_position_v[120:])))
        assert peak_v <= cfg.ARM_MAX_VELOCITY + 1e-5, peak_v
        assert peak_a <= cfg.ARM_MAX_ACCELERATION + .003, peak_a
        assert peak_actual_v <= cfg.ARM_MAX_VELOCITY + .015, peak_actual_v
        assert np.max(np.abs(np.diff(measured[120:], axis=0))) <= cfg.ARM_MAX_VELOCITY * dt + .0003
        np.testing.assert_allclose(drive[285:300], drive[285][None, :] + np.zeros((15, 14)), atol=1e-6)
        assert np.max(np.abs(drive[299] - np.array(goals["raised"][:14]))) > .05
        # Recovery must start from that held trajectory, not the newly solved
        # destination (which is far away on at least one joint).
        assert np.max(np.abs(drive[301] - drive[300])) < .001
        np.testing.assert_allclose(drive[565:570], drive[565][None, :] + np.zeros((5, 14)), atol=1e-6)
        np.testing.assert_allclose(measured[-1], HOME_Q, atol=.002)
        assert state_writes == [0], state_writes
        result = {"passed": True, "physics_steps": len(samples),
                  "max_drive_velocity_rad_s": peak_v, "max_drive_acceleration_rad_s2": peak_a,
                  "max_measured_velocity_rad_s": peak_actual_v,
                  "max_instantaneous_solver_velocity_rad_s": float(np.max(np.abs(measured_v[120:]))),
                  "max_recovery_first_step_rad": float(np.max(np.abs(drive[301] - drive[300]))),
                  "runtime_joint_state_writes": 0, "initial_home_state_writes": len(state_writes),
                  "failed_ik_brakes": True, "packet_loss_brakes": True,
                  "return_home_max_error_rad": float(np.max(np.abs(measured[-1] - HOME_Q)))}
        (args.output_dir / "validation.json").write_text(json.dumps(result, indent=2) + "\n")
        print("JOINT MOTION CHECK PASSED", json.dumps(result, indent=2), flush=True)
    except Exception:
        traceback.print_exc()
        raise
    finally:
        if sender:
            sender.close()
        launcher.app.close()


if __name__ == "__main__":
    if sys.argv[1:] == ["--make-goals"]:
        make_goals()
    else:
        main()
