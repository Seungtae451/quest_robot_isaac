"""Episode buttons, acknowledged retries, reset fencing, and real v3 exports."""
from contextlib import closing
import json
from pathlib import Path
import time
import os
import subprocess
from types import SimpleNamespace

import numpy as np
import pytest

from teleop.recording_buttons import RecordingButtons
from teleop.episode_protocol import EpisodeServer, EpisodeClient, REQUEST, encode
from teleop.action_protocol import ActionReceiver, ActionSender, pack_action
from robot.f14_config import HOME_ACTION
from robot.joint_motion import JointMotionLimiter
from simulation.episode_reset import EpisodeReset
from dataset.recording import EpisodeRecorder
from config import recording_config as cfg


def test_button_edges_include_short_presses_and_require_release_after_loss():
    buttons = RecordingButtons()
    buttons.update({}, {"aButton": True}, 1.)  # connecting while held is not a press
    assert buttons.drain(1.1) == []
    buttons.update({}, {}, 1.2)
    buttons.update({}, {"aButton": True}, 1.3)
    buttons.update({}, {}, 1.31)  # whole click between parent snapshots
    assert buttons.drain(1.32) == ["a"]
    assert buttons.drain(1.33) == []
    buttons.update({"aButton": True}, {"aButton": True, "bButton": True}, 1.4)
    assert buttons.drain(1.41) == ["x"]
    buttons.update({}, {}, 2., valid=False)
    buttons.update({}, {"bButton": True}, 2.1)
    assert buttons.drain(2.2) == []
    buttons.update({}, {}, 2.3)
    buttons.update({}, {"bButton": True}, 2.4)
    assert buttons.drain(3.) == []  # stale buttons cannot start/save later


def test_retried_buttons_and_prior_generations_never_repeat_actions():
    calls = []
    recorder = SimpleNamespace(generation="generation-one", button=calls.append,
        status=lambda: {"state": "READY", "generation": recorder.generation, "frames": 0, "saved_episodes": 0})
    with closing(EpisodeServer("127.0.0.1", 0, recorder)) as server:
        with closing(EpisodeClient("127.0.0.1", server.socket.getsockname()[1])) as client:
            client.poll(); server.poll(); client.poll()
            assert client.available
            assert client.button("a")
            payload = encode(REQUEST, client.pending)
            for _ in range(3):
                client.socket.sendto(payload, client.destination)
            server.poll(); client.poll()
            assert calls == ["a"] and client.pending is None
            recorder.generation = "generation-two"
            client.socket.sendto(encode(REQUEST, {"id": "delayed", "op": "a", "generation": "generation-one"}), client.destination)
            server.poll()
            assert calls == ["a"]


def test_environment_reset_fences_queued_goals_and_changes_feedback_boot():
    with closing(ActionReceiver("127.0.0.1", 0)) as receiver:
        with closing(ActionSender("127.0.0.1", receiver.socket.getsockname()[1])) as sender:
            old_boot = receiver.boot_time
            old_packet = pack_action(HOME_ACTION, 3)
            measured = HOME_ACTION.copy(); measured[14:] = [.5, .7]
            receiver.reset_session(measured)
            sender.socket.sendto(old_packet, sender.destination)
            assert not receiver.poll()
            assert receiver.latest is None and receiver.rejected == 1
            assert receiver.boot_time > old_boot
            np.testing.assert_allclose(receiver.action, measured)
            sender.send(measured); assert receiver.poll()


def test_home_reset_moves_with_existing_velocity_limits():
    measured = HOME_ACTION.copy(); measured[:14] += .4
    motion = JointMotionLimiter(measured[:14], .25, .5, .05)
    reset = EpisodeReset()
    positions = [measured[:14].copy()]
    for _ in range(1200):
        goal, enabled = reset.goal(measured, 1 / 60)
        measured[:14] = motion.step(goal[:14], measured[:14], 1 / 60, enabled)
        positions.append(measured[:14].copy())
        reset.update(measured, motion.velocity, 1 / 60)
        if reset.phase == "RESPAWN":
            reset.cubes_respawned()
        if reset.phase == "COMPLETE":
            break
    assert reset.phase == "COMPLETE" and reset.error is None
    velocity = np.diff(positions, axis=0) * 60
    assert np.max(np.abs(velocity)) <= .25 + 1e-10
    assert np.max(np.abs(np.diff(velocity, axis=0) * 60)) <= .5 + 1e-9
    np.testing.assert_allclose(measured[:14], HOME_ACTION[:14], atol=.015)


def test_home_reset_rejects_jittering_positions_even_with_stopped_reference():
    reset = EpisodeReset()
    reset.phase = "HOME"
    measured = HOME_ACTION.copy()
    for index in range(100):
        measured[0] = HOME_ACTION[0] + (.01 if index % 2 else -.01)
        reset.update(measured, np.zeros(14), 1 / 60)
    assert reset.phase == "HOME"
    measured[:14] = HOME_ACTION[:14]
    for _ in range(32):
        reset.update(measured, np.zeros(14), 1 / 60)
    assert reset.phase == "RESPAWN"


@pytest.mark.parametrize("state", ["READY", "STARTING", "STOPPING", "REVIEW", "SAVING", "RESETTING", "RECORDING"])
def test_only_recording_permits_operator_motion(state):
    client = EpisodeClient.__new__(EpisodeClient)
    client.status, client.last_status = {"state": state}, time.monotonic()
    assert client.blocks_teleop == (state != "RECORDING")
    assert client.preparing_episode == (state in ("READY", "STARTING"))
    recorder = EpisodeRecorder.__new__(EpisodeRecorder)
    recorder.state = state
    assert recorder.blocks_teleop == (state != "RECORDING")


def test_home_completion_waits_for_background_save_without_restarting_reset():
    recorder = EpisodeRecorder.__new__(EpisodeRecorder)
    recorder.state = "SAVING"
    recorder.reset_requested = True
    recorder._reset_started = recorder._environment_ready = False
    recorder.generation = "old"
    recorder.reset_started()
    assert not recorder.reset_requested
    recorder.reset_complete()
    assert recorder.state == "SAVING" and recorder.generation == "old"
    assert recorder._environment_ready
    recorder.state = "RESETTING"  # the saved acknowledgement arrives later
    recorder.reset_complete()
    assert recorder.state == "READY" and recorder.generation != "old"


def test_start_eligibility_from_numpy_measurements_is_json_serializable():
    recorder = EpisodeRecorder.__new__(EpisodeRecorder)
    recorder.state, recorder.generation = "READY", "test"
    recorder.frames = recorder.saved_episodes = 0
    recorder.notice = recorder.error = ""
    recorder.session_id = None
    recorder.start_allowed = np.bool_(True)
    response = json.loads(encode(REQUEST, recorder.status())[4:])
    assert response["start_allowed"] is True


def test_encoder_overload_rejects_episode_instead_of_silently_dropping_frames():
    from scripts.lerobot_writer import LiveEncoder
    encoder = SimpleNamespace(_dropped_frames={"front": 1}, feed_frame=lambda *args: None)
    live = LiveEncoder(encoder)
    live.counts = {"front": 0}
    with pytest.raises(RuntimeError, match="overloaded"):
        live.feed_frame("front", np.zeros((32, 48, 3), np.uint8))


def await_state(recorder, state, timeout=30):
    until = time.monotonic() + timeout
    while time.monotonic() < until:
        recorder.poll()
        if recorder.state == "ERROR":
            pytest.fail(f"{recorder.error}\n{recorder.log_path.read_text()[-6000:]}")
        if recorder.state == state:
            return
        time.sleep(.01)
    pytest.fail(f"Waiting for {state}: {recorder.status()}")


@pytest.mark.skipif(not cfg.WRITER_PYTHON.exists(), reason="Run scripts/setup_recording_env.sh for official dataset integration")
def test_official_lerobot_save_discard_and_resume(tmp_path):
    root = tmp_path / "f14_dataset"
    shapes = {name: (32, 48, 3) for name in ("front", "left_wrist", "right_wrist")}
    images = {name: np.full(shape, (240, 20, 20), np.uint8) for name, shape in shapes.items()}
    state = HOME_ACTION.copy(); state[14:] = .45  # contact prevents full closure
    action = HOME_ACTION.copy(); action[14:] = 0.  # still a valid action/state pair

    def frames(recorder, count):
        for k in range(count + 1):
            recorder.capture(images, state, action, k / 30)
            recorder.poll()
            time.sleep(.01)
        assert recorder.frames == count

    with closing(EpisodeRecorder(root, "local/f14_test", "Put the cube in the box.", 30, shapes)) as recorder:
        await_state(recorder, "READY")
        recorder.start_allowed = True
        recorder.button("a"); await_state(recorder, "RECORDING")
        frames(recorder, 12)
        recorder.button("b"); await_state(recorder, "REVIEW")
        assert not root.exists()  # B did not commit a dataset
        recorder.button("a"); await_state(recorder, "RESETTING")
        info = json.loads((root / "meta/info.json").read_text())
        assert info["codebase_version"] == "v3.0" and info["total_episodes"] == 1
        assert info["total_frames"] == 12
        before_discard = {p.relative_to(root): p.read_bytes() for p in root.rglob("*") if p.is_file()}
        recorder.reset_complete()
        recorder.button("a"); await_state(recorder, "RECORDING")
        frames(recorder, 6)
        recorder.button("b"); await_state(recorder, "REVIEW")
        recorder.button("x"); await_state(recorder, "RESETTING")
        assert json.loads((root / "meta/info.json").read_text())["total_episodes"] == 1
        assert {p.relative_to(root): p.read_bytes() for p in root.rglob("*") if p.is_file()} == before_discard
        recorder.reset_complete()
    # Restart the writer and append without overwriting the earlier episode.
    with closing(EpisodeRecorder(root, "local/f14_test", "Put the cube in the box.", 30, shapes)) as recorder:
        await_state(recorder, "READY")
        assert recorder.saved_episodes == 1
        recorder.start_allowed = True
        recorder.button("a"); await_state(recorder, "RECORDING")
        frames(recorder, 8)
        recorder.button("b"); await_state(recorder, "REVIEW")
        recorder.button("a"); await_state(recorder, "RESETTING")
    info = json.loads((root / "meta/info.json").read_text())
    assert info["total_episodes"] == 2 and info["total_frames"] == 20
    stats = json.loads((root / "meta/stats.json").read_text())
    assert "q01" in stats["action"] and "q99" in stats["observation.state"]


@pytest.mark.skipif(not cfg.WRITER_PYTHON.exists(), reason="Requires the isolated official LeRobot writer")
def test_failed_save_rolls_back_metadata_and_preserves_previous_episode(tmp_path):
    # Inject a failure after the official API has written an episode, which
    # exercises rollback of metadata and new Parquet/video files, not a mock
    # dataset implementation. Run only in the isolated writer environment.
    program = '''
import json, uuid, sys
from pathlib import Path
import numpy as np
from scripts.lerobot_writer import Writer
from lerobot.datasets.lerobot_dataset import LeRobotDataset
root = Path(sys.argv[1])
shapes = {name: (32,48,3) for name in ('front','left_wrist','right_wrist')}
w = Writer({'root':str(root),'repo_id':'local/rollback_test','task':'Put cube in box.','fps':30,'shapes':shapes})
def prepare():
    w.start(uuid.uuid4().hex)
    for k in range(5):
        vectors = np.r_[np.zeros(32), k/30].astype(np.float32)
        pixels = np.full((32,48,3),(240,20,20),np.uint8).tobytes()
        w.add({'session_id':w.session_id,'index':k}, [vectors.tobytes(),pixels,pixels,pixels])
    w.recording = False
original = LeRobotDataset.save_episode
def failed(*a, **kw):
    original(*a, **kw)
    raise OSError('Injected post-write failure')
prepare()
LeRobotDataset.save_episode = failed
try: w.save()
except OSError: pass
else: raise AssertionError('Expected injected failure')
assert not root.exists()
assert (w.spool/'000000/vectors.npy').exists()
LeRobotDataset.save_episode = original
w.discard(); prepare(); w.save()
before = {str(p.relative_to(root)):p.read_bytes() for p in root.rglob('*') if p.is_file()}
prepare(); LeRobotDataset.save_episode = failed
try: w.save()
except OSError: pass
else: raise AssertionError('Expected injected failure')
after = {str(p.relative_to(root)):p.read_bytes() for p in root.rglob('*') if p.is_file()}
assert before == after
assert w.saved_episodes == 1
LeRobotDataset.save_episode = original
w.discard(); prepare(); w.save()
assert json.loads((root/'meta/info.json').read_text())['total_episodes'] == 2
print('Official save rollback and recovery passed')
'''
    environment = dict(os.environ, HF_HOME=str(tmp_path / "hf_cache"), HF_HUB_OFFLINE="1", HF_DATASETS_OFFLINE="1")
    result = subprocess.run([str(cfg.WRITER_PYTHON), "-c", program, str(tmp_path / "dataset")],
                            cwd=cfg.ROOT, env=environment, capture_output=True, text=True, timeout=40)
    assert result.returncode == 0, result.stdout + result.stderr
