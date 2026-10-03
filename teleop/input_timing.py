"""Rate/jitter estimates from server-side event arrival timestamps."""
import numpy as np


def timed_filter_alpha(alpha, dt, reference_hz=30.):
    """Keep a per-reference-frame low-pass response constant in wall time."""
    if not np.isfinite(dt) or dt < 0:
        raise ValueError("Filter dt must be finite and nonnegative")
    return float(1. - (1. - alpha) ** (dt * reference_hz))


def event_timing(timestamps, now, window=5.):
    times = np.sort(np.asarray(timestamps, dtype=float))
    times = times[(times > 0.) & (times <= now) & (times >= now - window)]
    intervals = np.diff(times)
    intervals = intervals[intervals > 0.]
    return {
        "samples": int(len(times)),
        "hz": float(len(intervals) / intervals.sum()) if len(intervals) else 0.,
        "interval_p50_ms": float(np.median(intervals) * 1000) if len(intervals) else None,
        "interval_p95_ms": float(np.percentile(intervals, 95) * 1000) if len(intervals) else None,
        "interval_max_ms": float(intervals.max() * 1000) if len(intervals) else None,
    }
