"""Bounded end-to-end test with REAL Isaac + REAL Quest server, synthetic WSS input.

Requires GPU/env_isaaclab and a free TeleVuer port 8012. Test action/video ports
are isolated (15005/15556). Logs and result JSON go to outputs/pipeline_check.
This proves the software path; it does not replace a physical headset test.
"""
import asyncio
from contextlib import closing
import json
import os
from pathlib import Path
import signal
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
from robot.f14_config import HOME_Q, F14_URDF_PATH, LEFT_EE_DOWN_ROT, RIGHT_EE_DOWN_ROT
from robot.f14_ik import F14IK
from teleop.action_protocol import ActionSender
from teleop.xr_video import VideoSubscriber


def stop(process):
    """Signal only a subprocess created by this test, never other sessions."""
    if process and process.poll() is None:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=15)
        except subprocess.TimeoutExpired:
            process.terminate()
            process.wait(timeout=10)


async def connect(session, port=8012):
    deadline = time.monotonic() + 20
    while time.monotonic() < deadline:
        try:
            return await session.ws_connect(f"wss://127.0.0.1:{port}/", ssl=False)
        except (aiohttp.ClientError, OSError):
            await asyncio.sleep(.1)
    raise RuntimeError("Quest WSS did not start")


async def feed(ws, monitor, duration, displacement=0., rotation=0., grippers=(0., 0.), neutral_shift=0.):
    until = time.monotonic() + duration
    basis = np.array([[0., 0., -1.], [-1., 0., 0.], [0., 1., 0.]])
    while time.monotonic() < until:
        poses = []
        for side_x in (-.25, .25):
            pose = np.eye(4)
            pose[:3, 3] = [side_x, 1.2, -.35 - displacement - neutral_shift]
            pose[:3, :3] = basis.T @ pin.exp3(np.array([0., 0., rotation])) @ basis
            poses.append(pose.flatten(order="F").tolist())
        head = np.eye(4); head[1, 3] = 1.5
        messages = [
            {"etype": "CAMERA_MOVE", "value": {"camera": {"matrix": head.flatten(order="F").tolist()}}},
            {"etype": "CONTROLLER_MOVE", "value": {"left": poses[0], "right": poses[1],
             "leftState": {"triggerValue": 1. - grippers[0], "squeezeValue": 0.},
             "rightState": {"triggerValue": 1. - grippers[1], "squeezeValue": 0.}}},
        ]
        for message in messages:
            await ws.send_bytes(msgpack.packb(message, use_bin_type=True))
        monitor.poll_feedback()
        await asyncio.sleep(1 / 30)


async def run():
    # Refuse to interfere with an existing operator session on the fixed WSS port.
    with socket.socket() as probe:
        probe.bind(("127.0.0.1", 8012))
    output = ROOT / "outputs/pipeline_check"
    output.mkdir(parents=True, exist_ok=True)
    timing_path = output / "timing.jsonl"
    timing_path.unlink(missing_ok=True)
    sim = quest = None
    result = {}
    env = dict(os.environ, PYTHONUNBUFFERED="1")
    with (output / "isaac.log").open("w") as sim_log, (output / "quest.log").open("w") as quest_log:
        try:
            sim = subprocess.Popen([sys.executable, "-u", "scripts/run_isaac_teleop.py", "--headless", "--device", "cuda:0",
                                    "--scene-seed", "12",
                                    "--action-port", "15005", "--video-endpoint", "tcp://127.0.0.1:15556"],
                                   cwd=ROOT, env=env, stdout=sim_log, stderr=subprocess.STDOUT)
            with closing(ActionSender("127.0.0.1", 15005)) as monitor, closing(VideoSubscriber("tcp://127.0.0.1:15556")) as video:
                deadline = time.monotonic() + 60
                frame = None
                while time.monotonic() < deadline:
                    monitor.poll_feedback()
                    received = video.receive()
                    if received is not None:
                        frame = received
                    if monitor.ready and frame is not None:
                        break
                    if sim.poll() is not None:
                        raise RuntimeError("Isaac exited during startup; see isaac.log")
                    await asyncio.sleep(.05)
                assert monitor.ready and frame is not None, "Isaac UDP/video startup timeout"
                result["isaac_video_shape"] = list(frame.shape)
                print("Pipeline: Isaac ready, actual ZMQ video received", flush=True)

                def start_quest():
                    return subprocess.Popen([sys.executable, "-u", "scripts/run_quest_teleop.py", "--host-ip", "127.0.0.1",
                                             "--action-port", "15005", "--video-endpoint", "tcp://127.0.0.1:15556", "--no-filter",
                                             "--timing-output", str(timing_path)],
                                            cwd=ROOT, env=env, stdout=quest_log, stderr=subprocess.STDOUT)

                quest = start_quest()
                async with aiohttp.ClientSession() as session:
                    ws = await connect(session)
                    await feed(ws, monitor, 5.)
                    # HOME is held by physical drives; the new wrist HOME can
                    # settle about 1 mrad from nominal under gravity. Recenter
                    # must preserve that measured pose, not force nominal q.
                    np.testing.assert_allclose(monitor.measured_state[:14], HOME_Q, atol=.002)
                    result["initial_home_max_error_rad"] = float(np.max(np.abs(monitor.measured_state[:14] - HOME_Q)))
                    neutral_measured_q = monitor.measured_state[:14].copy()
                    # Allow the bounded drive trajectory to reach the IK goal;
                    # feedback now reports measured joints, not the destination.
                    await feed(ws, monitor, 6., -.04, .02, (.6, .2))
                    action = monitor.measured_state.copy()
                    np.testing.assert_allclose(action[14:], [.6, .2], atol=.03)
                    ik = F14IK(F14_URDF_PATH)
                    home = ik.forward_kinematics(neutral_measured_q)
                    actual = ik.forward_kinematics(action[:14])
                    translation_errors = []
                    rotation_errors = []
                    for rotation, anchor, pose in zip((LEFT_EE_DOWN_ROT, RIGHT_EE_DOWN_ROT), home, actual):
                        translation_errors.append(float(np.linalg.norm(pose.translation - anchor.translation - [-.04, 0., 0.])))
                        rotation_errors.append(float(np.linalg.norm(pin.log3(rotation.T @ pose.rotation))))
                    assert max(translation_errors) < .001 and max(rotation_errors) < .001, (translation_errors, rotation_errors)
                    result["measured_fk_translation_errors_m"] = translation_errors
                    result["measured_fk_rotation_errors_rad"] = rotation_errors
                    result["received_closures"] = action[14:].tolist()
                    print("Pipeline: WSS -> XYZ/downward IK -> bounded drive -> measured Isaac pose verified", flush=True)
                    # A genuinely unreachable controller pose requests arm
                    # braking; returning to a different reachable pose must
                    # start gradually from the held measured state.
                    await feed(ws, monitor, 1.5, 1., .02, (.6, .2))
                    np.testing.assert_allclose(monitor.measured_state[:14], action[:14], atol=.002)
                    assert "IK FAIL" in (output / "quest.log").read_text()
                    stopped = monitor.measured_state.copy()
                    await feed(ws, monitor, .15, -.02, .02, (.6, .2))
                    recovery_step = float(np.max(np.abs(monitor.measured_state[:14] - stopped[:14])))
                    assert recovery_step < .04, recovery_step
                    await feed(ws, monitor, 6., -.02, .02, (.6, .2))
                    action = monitor.measured_state.copy()
                    for anchor, actual_pose in zip(home, ik.forward_kinematics(action[:14])):
                        assert np.linalg.norm(actual_pose.translation - anchor.translation - [-.02, 0., 0.]) < .001
                    result["failed_ik_hold_and_slow_recovery"] = True
                    result["recovery_first_150ms_max_joint_change_rad"] = recovery_step
                    # Consume a server image event as proof the real Isaac video
                    # also traversed Quest's SUB/render_to_xr/WSS uplink path.
                    image_received = False
                    for _ in range(80):
                        message = await ws.receive(timeout=3.)
                        if message.type == aiohttp.WSMsgType.BINARY and b"ImageBackground" in message.data:
                            image_received = True
                            break
                    assert image_received
                    result["wss_image_uplink"] = True
                    await ws.close()
                    await asyncio.sleep(.8)
                    monitor.poll_feedback(); await asyncio.sleep(.1); monitor.poll_feedback()
                    np.testing.assert_allclose(monitor.measured_state[:14], action[:14], atol=.002)
                    np.testing.assert_allclose(monitor.measured_state[14:], action[14:], atol=.03)
                    result["tracking_loss_holds_target"] = True
                    stop(quest)
                    result["quest_first_exit"] = quest.returncode
                    assert sim.poll() is None, "Isaac died with Quest"
                    # A fresh sender with a very different controller neutral
                    # must anchor at the receiver's held robot pose, not HOME.
                    quest = start_quest()
                    ws = await connect(session)
                    await feed(ws, monitor, 5., grippers=(.6, .2), neutral_shift=.3)
                    np.testing.assert_allclose(monitor.measured_state[:14], action[:14], atol=.002)
                    np.testing.assert_allclose(monitor.measured_state[14:], action[14:], atol=.03)
                    result["sender_restart_no_home_jump"] = True
                    stop(sim)
                    result["isaac_exit"] = sim.returncode
                    await feed(ws, monitor, 1., grippers=(.6, .2), neutral_shift=.3)
                    assert quest.poll() is None, "Quest died with Isaac"
                    result["simulator_exit_keeps_quest_alive"] = True
                    # Start Isaac while Quest remains alive. The changed boot
                    # timestamp must stop control and recenter at new HOME.
                    sim = subprocess.Popen([sys.executable, "-u", "scripts/run_isaac_teleop.py", "--headless", "--device", "cuda:0",
                                            "--scene-seed", "12",
                                            "--action-port", "15005", "--video-endpoint", "tcp://127.0.0.1:15556"],
                                           cwd=ROOT, env=env, stdout=sim_log, stderr=subprocess.STDOUT)
                    old_boot = monitor.boot_time
                    deadline = time.monotonic() + 60
                    while monitor.boot_time == old_boot and time.monotonic() < deadline:
                        await feed(ws, monitor, .25, neutral_shift=.3)
                        if sim.poll() is not None:
                            raise RuntimeError("Restarted Isaac exited; see isaac.log")
                    assert monitor.boot_time != old_boot
                    await feed(ws, monitor, 5., neutral_shift=.3)
                    np.testing.assert_allclose(monitor.measured_state[:14], HOME_Q, atol=.002)
                    result["restarted_home_max_error_rad"] = float(np.max(np.abs(monitor.measured_state[:14] - HOME_Q)))
                    restarted_home = monitor.measured_state[:14].copy()
                    await feed(ws, monitor, 1., neutral_shift=.3)
                    np.testing.assert_allclose(monitor.measured_state[:14], restarted_home, atol=.0003)
                    np.testing.assert_allclose(monitor.measured_state[14:], [0., 0.], atol=.03)
                    result["isaac_restart_while_quest_alive_recalibrates_home"] = True
                    await ws.close()
            stop(quest)
            stop(sim)
            result["quest_final_exit"] = quest.returncode
            result["isaac_final_exit"] = sim.returncode
            assert result["quest_first_exit"] == result["quest_final_exit"] == result["isaac_exit"] == result["isaac_final_exit"] == 0
            timings = [json.loads(line) for line in timing_path.read_text().splitlines()]
            active = [r for r in timings if r["state"] == "ACTIVE" and r["ik_hz"] > 5
                      and r["input"]["controller"]["samples"] >= 20]
            assert active
            # The WSS driver emits about 30Hz. A 60Hz polling loop must not
            # perform about 60 solves/sec on duplicate controller samples.
            assert all(r["ik_hz"] <= r["input"]["controller"]["hz"] * 1.4 for r in active)
            result["fresh_events_only_ik"] = True
            result["median_control_tick_hz"] = float(np.median([r["control_tick_hz"] for r in active]))
            result["median_ik_hz"] = float(np.median([r["ik_hz"] for r in active]))
            (output / "validation.json").write_text(json.dumps(result, indent=2) + "\n")
            print("PIPELINE TEST PASSED", json.dumps(result, indent=2), flush=True)
        finally:
            stop(quest)
            stop(sim)


if __name__ == "__main__":
    asyncio.run(run())
