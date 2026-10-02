"""UDP loopback diagnostic or deliberate HOME/gripper test for running Isaac.

Default is an isolated ephemeral-port test. --send --action-port 5005 sends
HOME arms with independently selected grippers to the simulator for --seconds.
Do not run --send alongside a live Quest sender.
"""
import argparse
from contextlib import closing
from pathlib import Path
import sys
import time

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from robot.f14_config import HOME_ACTION
from teleop.action_protocol import ActionReceiver, ActionSender


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--send", action="store_true")
    parser.add_argument("--action-port", type=int, default=5005)
    parser.add_argument("--left-gripper", type=float, default=0.)
    parser.add_argument("--right-gripper", type=float, default=0.)
    parser.add_argument("--seconds", type=float, default=3.)
    args = parser.parse_args()
    action = HOME_ACTION.copy()
    action[14:] = args.left_gripper, args.right_gripper
    if args.send:
        with closing(ActionSender("127.0.0.1", args.action_port)) as sender:
            until = time.monotonic() + args.seconds
            while time.monotonic() < until:
                sender.send(action)
                time.sleep(1 / 30)
            print(f"Sent {sender.sequence} HOME/gripper actions; receiver should HOLD next.")
    else:
        with closing(ActionReceiver("127.0.0.1", 0)) as receiver:
            with closing(ActionSender("127.0.0.1", receiver.socket.getsockname()[1])) as sender:
                assert receiver.state == "WAITING"
                sender.poll_feedback()
                receiver.poll()
                assert sender.poll_feedback() and sender.ready
                for _ in range(10):
                    sender.send(action)
                assert receiver.poll()
                np.testing.assert_allclose(receiver.action, action)
                assert receiver.latest.sequence == 10 and receiver.state == "ACTIVE"
                time.sleep(.55)
                assert receiver.state == "HOLD"
                np.testing.assert_allclose(receiver.action, action)
                print("UDP PASS: 16D, newest of 10 packets, WAITING -> ACTIVE -> HOLD, held target unchanged")


if __name__ == "__main__":
    main()
