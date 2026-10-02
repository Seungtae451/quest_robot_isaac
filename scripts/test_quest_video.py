"""Quest video-only test: live Isaac composite or a synthetic color dashboard.

--pattern does not need Isaac. Red LEFT, green BODY, blue RIGHT make channel
swaps obvious in the headset. --self-test additionally injects actual WSS
controller events and checks freshness, BGR->RGB shared memory and timeout,
without hardware. Synthetic images are confined to this diagnostic script.
"""
import argparse
import asyncio
from contextlib import closing
from pathlib import Path
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from teleop.televuer_adapter import QuestInterface, detect_host_ip
from teleop.xr_video import VideoSubscriber, send_image_to_xr, waiting_image
from simulation.quest_view_compositor import compose_quest_view


def test_pattern():
    return compose_quest_view({"left_wrist": np.full((240, 320, 3), (240, 20, 20), np.uint8),
                               "front": np.full((480, 640, 3), (20, 240, 20), np.uint8),
                               "right_wrist": np.full((240, 320, 3), (20, 20, 240), np.uint8)}, "Synthetic video diagnostic")


async def websocket_self_test(tv):
    import aiohttp
    import msgpack
    # Self-signed LOCAL test endpoint: do not disable TLS verification for any
    # remote service. The headset still uses the LAN certificate trust workflow.
    url = f"wss://127.0.0.1:{tv.tvuer.vuer.port}/"
    async with aiohttp.ClientSession() as session:
        ws = None
        for _ in range(50):
            try:
                ws = await session.ws_connect(url, ssl=False)
                break
            except (aiohttp.ClientError, OSError):
                await asyncio.sleep(.1)
        if ws is None:
            raise RuntimeError("Local WSS server did not start")
        try:
            pose = np.eye(4)
            pose[:3, 3] = [.2, 1.2, -.3]
            matrix = pose.flatten(order="F").tolist()
            head = np.eye(4)
            head[1, 3] = 1.5
            for _ in range(8):
                for event in [
                    {"etype": "CAMERA_MOVE", "value": {"camera": {"matrix": head.flatten(order="F").tolist()}}},
                    {"etype": "CONTROLLER_MOVE", "value": {"left": matrix, "right": matrix,
                        "leftState": {"triggerValue": .75, "squeezeValue": .2},
                        "rightState": {"triggerValue": .25, "squeezeValue": .8}}},
                ]:
                    await ws.send_bytes(msgpack.packb(event, use_bin_type=True))
                await asyncio.sleep(.04)
            data, fresh, serial = tv.snapshot()
            assert fresh and serial >= 8, (fresh, serial)
            assert abs(data.left_ctrl_triggerValue - 2.5) < 1e-9
            assert abs(data.right_ctrl_triggerValue - 7.5) < 1e-9
            # The WSS return path must contain at least one ImageBackground.
            found_image = False
            for _ in range(25):
                message = await ws.receive(timeout=3.)
                if message.type == aiohttp.WSMsgType.BINARY and b"ImageBackground" in message.data:
                    found_image = True
                    break
            assert found_image, "No image message on WSS"
        finally:
            await ws.close()
        await asyncio.sleep(.6)
        assert not tv.snapshot()[1], "Disconnected client incorrectly still fresh"
        print("WSS PASS: image uplink, controller/head events, trigger conversion, disconnect watchdog")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host-ip")
    parser.add_argument("--pattern", action="store_true")
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument("--xr-mode", choices=("immersive", "ego"), default="immersive")
    parser.add_argument("--video-endpoint", default="tcp://127.0.0.1:5556")
    args = parser.parse_args()
    try:
        with closing(QuestInterface(args.xr_mode)) as tv:
            tv.print_url(args.host_ip or detect_host_ip())
            pattern = cv2.cvtColor(test_pattern(), cv2.COLOR_RGB2BGR)
            send_image_to_xr(tv, pattern)
            if args.self_test:
                asyncio.run(websocket_self_test(tv))
                np.testing.assert_array_equal(tv.tvuer.img2display[350, 50], [240, 20, 20])
                print("Video PASS: RGB/BGR boundary and shared display buffer")
                return
            with closing(VideoSubscriber(args.video_endpoint)) as sub:
                while tv.tvuer.process.is_alive():
                    frame = pattern if args.pattern else sub.receive()
                    if frame is not None:
                        send_image_to_xr(tv, frame)
                    elif sub.age > 1.:
                        send_image_to_xr(tv, waiting_image())
                    time.sleep(1 / 30)
                raise RuntimeError("TeleVuer child stopped")
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
