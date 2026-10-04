"""Read-only Cartesian speed estimates in wall time, outside dataset records."""
from collections import deque

import numpy as np


class MotionSpeedMonitor:
    def __init__(self, sample_seconds=.1, window_seconds=1., gap_seconds=.5):
        self.sample_seconds = sample_seconds
        self.window_seconds = window_seconds
        self.gap_seconds = gap_seconds
        self.reset()

    def reset(self):
        self.streams = {}

    def observe(self, name, timestamp, positions):
        positions = np.asarray(positions, float)
        if positions.shape != (2, 3) or not np.isfinite(positions).all() or not np.isfinite(timestamp):
            raise ValueError('Expected finite timestamps and left/right XYZ positions')
        stream = self.streams.get(name)
        if stream is None or timestamp - stream['last_time'] > self.gap_seconds:
            self.streams[name] = {'last_time': timestamp, 'base_time': timestamp,
                                 'base': positions.copy(), 'samples': deque()}
            return
        if timestamp <= stream['last_time']:
            return  # Cached feedback is not a new motion sample.
        stream['last_time'] = timestamp
        dt = timestamp - stream['base_time']
        if dt >= self.sample_seconds:
            speed = np.linalg.norm(positions - stream['base'], axis=1) / dt * 100.
            stream['samples'].append((timestamp, dt, speed))
            stream['base_time'], stream['base'] = timestamp, positions.copy()

    def summary(self, now):
        result = {}
        for name, stream in self.streams.items():
            samples = stream['samples']
            while samples and samples[0][0] < now - self.window_seconds:
                samples.popleft()
            if now - stream['last_time'] > self.gap_seconds or not samples:
                result[name] = None
                continue
            weights = np.array([sample[1] for sample in samples])
            speeds = np.array([sample[2] for sample in samples])
            result[name] = {'mean_cm_s': np.average(speeds, axis=0, weights=weights).tolist(),
                            'p95_cm_s': np.percentile(speeds, 95, axis=0).tolist(),
                            'samples': len(samples)}
        return result
