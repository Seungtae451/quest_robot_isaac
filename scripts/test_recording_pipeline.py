"""Actual Isaac RGB/physics + Quest WSS A/B/X + official LeRobot reload.

Uses isolated UDP/video ports and synthetic controller events, not a headset.
Output: outputs/recording_pipeline_check. Uses test WSS port 18012, leaving the
operator's 8012 server running. Run with env_isaaclab and a CUDA GPU.
"""
import asyncio
from contextlib import closing
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import time

import aiohttp
import msgpack
import numpy as np
import pinocchio as pin

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))
from test_pipeline import connect, stop
from teleop.action_protocol import ActionSender
from teleop.episode_protocol import EpisodeClient
from robot.f14_config import HOME_Q, F14_URDF_PATH, GRIPPER_FORWARD_AXIS, LEFT_EE_DOWN_ROT, RIGHT_EE_DOWN_ROT
from robot.f14_ik import F14IK

WSS_PORT = 18012


async def feed(ws, monitor, episodes, seconds, displacement=0., buttons=(), rotations=None, measured=None):
    until = time.monotonic() + seconds
    while time.monotonic() < until:
        poses = []
        for index, x in enumerate((-.25, .25)):
            pose = np.eye(4)
            pose[:3, 3] = (x, 1.2, -.35 - displacement)
            if rotations is not None:
                pose[:3, :3] = pin.exp3(np.asarray(rotations[index]))
            poses.append(pose.flatten(order="F").tolist())
        head = np.eye(4); head[1, 3] = 1.5
        left = {"triggerValue": .25, "squeezeValue": 0., "aButton": "x" in buttons}
        right = {"triggerValue": .25, "squeezeValue": 0., "aButton": "a" in buttons, "bButton": "b" in buttons}
        for message in (
            {"etype": "CAMERA_MOVE", "value": {"camera": {"matrix": head.flatten(order="F").tolist()}}},
            {"etype": "CONTROLLER_MOVE", "value": {"left": poses[0], "right": poses[1],
                                                     "leftState": left, "rightState": right}}):
            await ws.send_bytes(msgpack.packb(message, use_bin_type=True))
        monitor.poll_feedback(); episodes.poll()
        if measured is not None and monitor.ready:
            measured.append(monitor.measured_state.copy())
        await asyncio.sleep(1 / 60)


async def wait_for(ws, monitor, episodes, state, timeout=40, displacement=0.):
    until = time.monotonic() + timeout
    while time.monotonic() < until:
        await feed(ws, monitor, episodes, .1, displacement)
        if episodes.status and episodes.status["state"] == "ERROR":
            raise RuntimeError(episodes.status)
        if episodes.available and episodes.status["state"] == state:
            return
    raise RuntimeError(f"Waiting for {state}: {episodes.status}")


async def click(ws, monitor, episodes, button, displacement=0.):
    await feed(ws, monitor, episodes, .12, displacement)
    await feed(ws, monitor, episodes, .18, displacement, (button,))
    await feed(ws, monitor, episodes, .12, displacement)


async def run():
    with socket.socket() as probe:
        probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        probe.bind(("127.0.0.1", WSS_PORT))
    output = ROOT / "outputs/recording_pipeline_check" / time.strftime("%Y%m%d_%H%M%S")
    output.mkdir(parents=True)
    dataset_root = output / "dataset"
    env = dict(os.environ, PYTHONUNBUFFERED="1")
    sim = quest = None
    result = {"physical_headset": False, "dataset_root": str(dataset_root)}
    with (output / "isaac.log").open("w") as sim_log, (output / "quest.log").open("w") as quest_log:
        try:
            sim = subprocess.Popen([sys.executable, "-u", "scripts/run_isaac_teleop.py", "--headless", "--device", "cuda:0",
                "--record", "--scene-seed", "12", "--action-port", "15045", "--session-port", "15046",
                "--video-endpoint", "tcp://127.0.0.1:15596", "--dataset-root", str(dataset_root),
                "--snapshot-dir", str(output / "images"),
                "--repo-id", "local/f14_recording_test"], cwd=ROOT, env=env, stdout=sim_log, stderr=subprocess.STDOUT)
            with closing(ActionSender("127.0.0.1", 15045)) as monitor, closing(EpisodeClient("127.0.0.1", 15046)) as episodes:
                deadline = time.monotonic() + 70
                while time.monotonic() < deadline:
                    monitor.poll_feedback(); episodes.poll()
                    if episodes.available and episodes.status["state"] == "ERROR":
                        raise RuntimeError(episodes.status)
                    if monitor.ready and episodes.available and episodes.status["state"] == "READY":
                        break
                    if sim.poll() is not None:
                        raise RuntimeError("Isaac startup failed; see isaac.log")
                    await asyncio.sleep(.05)
                assert monitor.ready and episodes.available, "Isaac recording startup timed out"
                # Upstream TeleVuer doesn't expose port configuration. Change
                # only the Vuer constructor default inside this test child.
                bootstrap = ("from functools import partial; from vuer import Vuer; "
                    "import televuer.televuer as backend; "
                    f"backend.Vuer=partial(Vuer,port={WSS_PORT}); "
                    "from teleop.quest_teleop_server import main; main()")
                quest = subprocess.Popen([sys.executable, "-u", "-c", bootstrap, "--host-ip", "127.0.0.1",
                    "--action-port", "15045", "--session-port", "15046", "--video-endpoint", "tcp://127.0.0.1:15596",
                    "--timing-output", str(output / "timing.jsonl")], cwd=ROOT, env=env, stdout=quest_log, stderr=subprocess.STDOUT)
                async with aiohttp.ClientSession() as session:
                    ws = await connect(session, WSS_PORT)
                    await feed(ws, monitor, episodes, 5.)
                    # Controller motion and trigger input before A must leave
                    # both arms at HOME and both physical grippers open.
                    await feed(ws, monitor, episodes, 2., displacement=-.09)
                    np.testing.assert_allclose(monitor.measured_state[:14], HOME_Q, atol=.015)
                    assert np.min(monitor.measured_state[14:]) > .97
                    assert episodes.status["state"] == "READY" and episodes.status["frames"] == 0
                    result["before_a_home_hold_including_grippers"] = True
                    initial_boot = monitor.boot_time
                    await click(ws, monitor, episodes, "a", displacement=-.09)
                    await wait_for(ws, monitor, episodes, "RECORDING", displacement=-.09)
                    await feed(ws, monitor, episodes, .6, displacement=-.09)
                    np.testing.assert_allclose(monitor.measured_state[:14], HOME_Q, atol=.015)
                    result["a_press_controller_pose_is_neutral"] = True
                    # Pure rotations must not command either arm, and the
                    # actual physical jaws still face down. Then translate
                    # while feeding different controller rotations per arm.
                    still = monitor.measured_state[:14].copy()
                    await feed(ws, monitor, episodes, 1., displacement=-.09,
                               rotations=([.9, -.4, .7], [-.6, .8, -.9]))
                    np.testing.assert_allclose(monitor.measured_state[:14], still, atol=.004)
                    result["controller_rotations_do_not_move_arms"] = True
                    trajectory = []
                    await feed(ws, monitor, episodes, 3., displacement=-.125,
                               rotations=([-.7, .9, -.2], [.9, -.3, .8]), measured=trajectory)
                    moved = monitor.measured_state[:14].copy()
                    assert np.max(np.abs(moved - HOME_Q)) > .02, "Robot did not move during episode"
                    ik = F14IK(F14_URDF_PATH)
                    tilt = [float(np.rad2deg(np.arccos(np.clip(np.dot(
                        pose.rotation @ GRIPPER_FORWARD_AXIS, [0, 0, -1]), -1, 1))))
                        for state in trajectory for pose in ik.forward_kinematics(state[:14])]
                    orientation = [float(np.rad2deg(np.linalg.norm(pin.log3(rotation.T @ pose.rotation))))
                        for pose, rotation in zip(ik.forward_kinematics(moved), (LEFT_EE_DOWN_ROT, RIGHT_EE_DOWN_ROT))]
                    assert max(orientation) < .5, orientation
                    assert max(tilt) < 3., max(tilt)  # Bounded drives have finite tracking error.
                    result["measured_downward_orientation_error_deg"] = orientation
                    result["motion_max_grasp_axis_tilt_deg"] = max(tilt)
                    # Deliberately unreachable (+1 m from neutral) request:
                    # bounded feasible steps must move forward, not accept a
                    # failed candidate or freeze at the first failed full goal.
                    before_far = ik.forward_kinematics(monitor.measured_state[:14])
                    await feed(ws, monitor, episodes, 4., displacement=.91, measured=trajectory)
                    after_far = ik.forward_kinematics(monitor.measured_state[:14])
                    assert after_far[0].translation[0] > before_far[0].translation[0] + .015
                    assert np.isfinite(monitor.measured_state).all()
                    for pose in after_far:
                        assert np.rad2deg(np.arccos(np.clip(np.dot(pose.rotation @ GRIPPER_FORWARD_AXIS,
                                                                  [0,0,-1]), -1, 1))) < 3.
                    await feed(ws, monitor, episodes, 4., displacement=-.125, measured=trajectory)
                    returned = ik.forward_kinematics(monitor.measured_state[:14])
                    assert returned[0].translation[0] < after_far[0].translation[0] - .01
                    result["unreachable_request_follows_feasible_steps_and_retreats"] = True
                    await click(ws, monitor, episodes, "b", displacement=-.125)
                    await wait_for(ws, monitor, episodes, "REVIEW", displacement=-.125)
                    frames = episodes.status["frames"]
                    assert frames >= 30, frames
                    assert not dataset_root.exists(), "B prematurely saved"
                    pending = dataset_root.parent / ".dataset_pending" / episodes.status["session_id"]
                    first_start = np.load(pending / "000000/vectors.npy")[:16]
                    np.testing.assert_allclose(first_start[:14], HOME_Q, atol=.015)
                    assert np.min(first_start[14:]) > .97
                    # The videos already exist and have all frames at B; A
                    # needs no PNG reread / full-episode encoding pass.
                    prepared = list((pending / "lerobot_commit").glob("tmp*/*.mp4"))
                    assert len(prepared) == 3, prepared
                    count_videos = subprocess.run([str(ROOT / ".venv-lerobot/bin/python"), "-c",
                        "import av,sys; expected=int(sys.argv[1]); "
                        "counts=[av.open(p).streams.video[0].frames for p in sys.argv[2:]]; "
                        "assert counts == [expected]*3, counts",
                        str(frames), *(str(path) for path in prepared)], capture_output=True, text=True, timeout=10)
                    assert count_videos.returncode == 0, count_videos.stderr
                    result["three_videos_encoded_before_save"] = True
                    stopped = monitor.measured_state.copy()
                    await feed(ws, monitor, episodes, .8, displacement=-.1)
                    np.testing.assert_allclose(monitor.measured_state[:14], stopped[:14], atol=.004)
                    await click(ws, monitor, episodes, "a", displacement=-.035)
                    await wait_for(ws, monitor, episodes, "RESETTING", displacement=-.035)
                    await wait_for(ws, monitor, episodes, "READY", timeout=50, displacement=-.035)
                    assert monitor.boot_time != initial_boot
                    np.testing.assert_allclose(monitor.measured_state[:14], HOME_Q, atol=.015)
                    info = json.loads((dataset_root / "meta/info.json").read_text())
                    assert info["total_episodes"] == 1 and info["total_frames"] == frames
                    result.update(saved_frames=frames, stopped_motion_preserved=True, slow_home_reset=True)
                    # Keep a different controller neutral after reset; fresh
                    # calibration must not resurrect the preceding IK target.
                    await feed(ws, monitor, episodes, 5., displacement=-.035)
                    np.testing.assert_allclose(monitor.measured_state[:14], HOME_Q, atol=.016)
                    # Move again AFTER waiting calibration, then press A. Every
                    # episode must zero at this new pose, not the reset pose.
                    await feed(ws, monitor, episodes, .8, displacement=.055)
                    await click(ws, monitor, episodes, "a", displacement=.055)
                    await wait_for(ws, monitor, episodes, "RECORDING", displacement=.055)
                    await feed(ws, monitor, episodes, .6, displacement=.055)
                    np.testing.assert_allclose(monitor.measured_state[:14], HOME_Q, atol=.015)
                    result["every_a_press_recaptures_neutral"] = True
                    await feed(ws, monitor, episodes, 1., displacement=.070)
                    await click(ws, monitor, episodes, "b", displacement=.070)
                    await wait_for(ws, monitor, episodes, "REVIEW", displacement=.070)
                    pending = dataset_root.parent / ".dataset_pending" / episodes.status["session_id"]
                    second_start = np.load(pending / "000000/vectors.npy")[:16]
                    np.testing.assert_allclose(second_start, first_start, atol=.005)
                    result["episodes_share_home_start_state"] = True
                    await click(ws, monitor, episodes, "x", displacement=-.02)
                    await wait_for(ws, monitor, episodes, "READY", timeout=50, displacement=-.02)
                    info_after = json.loads((dataset_root / "meta/info.json").read_text())
                    assert info_after["total_episodes"] == 1 and info_after["total_frames"] == frames
                    result["x_discards_without_modifying_saved_data"] = True
                    await ws.close()
            stop(quest); stop(sim)
            assert sim.returncode == quest.returncode == 0
            log = (output / "isaac.log").read_text()
            assert "Traceback" not in log
            assert log.count("Episode reset: fresh cube poses") == 2
            assert "Episode reset started; dataset state=SAVING" in log
            assert log.index("Episode reset started; dataset state=SAVING") < log.index("Episode saved;")
            result["home_reset_overlaps_save"] = True
            result["two_new_cube_layouts"] = True
            # Official reload checks all three actual sensor streams and labels.
            command = [str(ROOT / ".venv-lerobot/bin/python"), str(ROOT / "scripts/validate_lerobot_dataset.py"),
                       "--root", str(dataset_root), "--repo-id", "local/f14_recording_test"]
            check = subprocess.run(command, cwd=ROOT, env=env, capture_output=True, text=True, timeout=40)
            (output / "reload.log").write_text(check.stdout + check.stderr)
            assert check.returncode == 0, check.stdout + check.stderr
            result["official_reload"] = True
            result["passed"] = True
            (output / "validation.json").write_text(json.dumps(result, indent=2))
            print("RECORDING PIPELINE PASSED", json.dumps(result, indent=2), flush=True)
        finally:
            stop(quest); stop(sim)


if __name__ == "__main__":
    asyncio.run(run())
