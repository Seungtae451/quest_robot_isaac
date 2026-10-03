"""Measure real Quest WSS arrivals; no Isaac import or robot action packets."""
import argparse
from contextlib import closing
import json
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from teleop.televuer_adapter import QuestInterface, detect_host_ip


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host-ip")
    parser.add_argument("--seconds", type=float, default=20.)
    parser.add_argument("--wait-timeout", type=float, default=180.)
    parser.add_argument("--output", type=Path, default=Path("outputs/quest_input_check/measurement.json"))
    args = parser.parse_args()
    if args.seconds <= 0 or args.wait_timeout <= 0:
        parser.error("measurement durations must be positive")
    samples = []
    started = None
    connected_at = time.monotonic()
    last_print = 0.
    result = {"source": "physical Quest browser WSS event arrivals", "complete": False}
    try:
        with closing(QuestInterface("pass-through")) as tv:
            tv.print_url(args.host_ip or detect_host_ip())
            print("No robot actions. Enter VR with both controllers; move gently for 20 seconds.", flush=True)
            while True:
                if not tv.tvuer.process.is_alive():
                    raise RuntimeError("Quest server exited")
                _, fresh, _ = tv.snapshot()
                now = time.monotonic()
                if fresh and started is None:
                    started = now
                    print("MEASUREMENT START", flush=True)
                if now - last_print >= 1.:
                    timing = tv.input_timing(window=args.seconds)
                    timing.update(elapsed_s=None if started is None else now - started, fresh=fresh)
                    samples.append(timing)
                    print(f"fresh={fresh} controller={timing['controller']['hz']:.1f}Hz "
                          f"head={timing['head']['hz']:.1f}Hz "
                          f"controller_p95={timing['controller']['interval_p95_ms']}ms", flush=True)
                    last_print = now
                if started is not None and now - started >= args.seconds:
                    result.update(complete=True, final=tv.input_timing(window=args.seconds))
                    break
                if started is None and now - connected_at >= args.wait_timeout:
                    result["reason"] = "No fresh physical Quest input before timeout"
                    break
                time.sleep(.01)
    except KeyboardInterrupt:
        result["reason"] = "operator stopped"
    finally:
        result["samples"] = samples
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, indent=2) + "\n")
        print(f"Saved: {args.output}", flush=True)


if __name__ == "__main__":
    main()
