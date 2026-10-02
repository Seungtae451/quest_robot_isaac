"""Quest-only pose/trigger/squeeze and event freshness diagnostic; no Isaac."""
import argparse
from contextlib import closing
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from teleop.televuer_adapter import QuestInterface, detect_host_ip, trigger_encoding
from robot.gripper import normalize_input


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host-ip")
    args = parser.parse_args()
    encoding = trigger_encoding()
    try:
        with closing(QuestInterface("pass-through")) as tv:
            tv.print_url(args.host_ip or detect_host_ip())
            while True:
                data, fresh, serial = tv.snapshot()
                if not tv.tvuer.process.is_alive():
                    raise RuntimeError("TeleVuer child stopped")
                print(f"fresh={fresh} event={serial}")
                if fresh:
                    for side in ("left", "right"):
                        raw = getattr(data, f"{side}_ctrl_triggerValue")
                        print(side, "pose=\n", getattr(data, f"{side}_wrist_pose").round(3),
                              "trigger(raw/closure)=", raw, normalize_input(raw, encoding),
                              "squeeze=", getattr(data, f"{side}_ctrl_squeezeValue"))
                time.sleep(.5)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
