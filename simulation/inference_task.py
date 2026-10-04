"""Simulator-independent success geometry and closed-loop run statistics."""
from collections import Counter
import json
import math
from pathlib import Path
import time

import numpy as np

from config import tabletop_config as cfg
from simulation.tabletop import table_surface_height


def cube_inside_box(position, quaternion):
    """Require the entire oriented cube footprint inside the inner walls."""
    x, y, z = np.asarray(position, dtype=float)
    q = np.asarray(quaternion, dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all() or np.linalg.norm(q) < 1e-9:
        return False
    w, a, b, c = q/np.linalg.norm(q)
    rotation = np.array([[1-2*(b*b+c*c), 2*(a*b-w*c), 2*(a*c+w*b)],
                         [2*(a*b+w*c), 1-2*(a*a+c*c), 2*(b*c-w*a)],
                         [2*(a*c-w*b), 2*(b*c+w*a), 1-2*(a*a+b*b)]])
    extent = np.abs(rotation) @ np.full(3, cfg.CUBE_SIZE/2)
    bx, by = cfg.BOX_CENTER_XY
    half_x = cfg.BOX_SIZE[0]/2-cfg.BOX_WALL_THICKNESS
    half_y = cfg.BOX_SIZE[1]/2-cfg.BOX_WALL_THICKNESS
    floor = table_surface_height()+cfg.BOX_BOTTOM_THICKNESS
    # Contact offset can create a force shortly before the geometric surfaces meet.
    return bool(np.isfinite([x,y,z]).all()
                and abs(x-bx)+extent[0] <= half_x+1e-6
                and abs(y-by)+extent[1] <= half_y+1e-6
                and abs(z-extent[2]-floor) <= .004)


class PlacementSuccess:
    def __init__(self, hold_seconds=0., minimum_force=.001):
        self.hold_seconds = hold_seconds
        self.minimum_force = minimum_force
        self.contact_seconds = 0.

    def update(self, position, quaternion, bottom_force, dt):
        touching = (cube_inside_box(position, quaternion)
                    and np.isfinite(bottom_force).all()
                    and np.linalg.norm(bottom_force) >= self.minimum_force)
        self.contact_seconds = self.contact_seconds+dt if touching else 0.
        return bool(touching and self.contact_seconds+1e-9 >= self.hold_seconds)


def bounded_action(action, limits):
    action = np.asarray(action, dtype=float)
    if action.shape != (16,) or not np.isfinite(action).all():
        raise ValueError('Expected 16 finite model action values.')
    result = action.copy()
    result[:14] = np.clip(result[:14], limits[:, 0], limits[:, 1])
    result[14:] = np.clip(result[14:], 0., 1.)
    return result, bool(np.any(result != action))


class EvaluationResults:
    def __init__(self, directory, config):
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=False)
        self.config = config
        self.trials = []
        self.started = time.monotonic()

    def record(self, **trial):
        self.trials.append(trial)
        with (self.directory/'trials.jsonl').open('a') as stream:
            stream.write(json.dumps(trial, allow_nan=False)+'\n')
        self.summary('running')

    def summary(self, status):
        completed = [trial for trial in self.trials if trial['outcome'] in ('success','timeout','cube_fell','manual_failure')]
        successes = sum(trial['outcome']=='success' for trial in completed)
        count = len(completed)
        rate = successes/count if count else None
        interval = None
        if count:
            z=1.96
            center=(rate+z*z/(2*count))/(1+z*z/count)
            radius=z*math.sqrt(rate*(1-rate)/count+z*z/(4*count*count))/(1+z*z/count)
            interval=[center-radius, center+radius]
        success_times = [trial['elapsed_sim_seconds'] for trial in completed if trial['outcome']=='success']
        result = {'status': status, 'config': self.config, 'completed_trials': count,
            'successes': successes, 'failures': count-successes,
            'success_rate': rate, 'success_rate_percent': rate*100 if rate is not None else None,
            'success_rate_wilson_95_percent': [x*100 for x in interval] if interval else None,
            'outcomes': dict(Counter(trial['outcome'] for trial in self.trials)),
            'mean_success_sim_seconds': float(np.mean(success_times)) if success_times else None,
            'wall_seconds': time.monotonic()-self.started}
        temporary=self.directory/'summary.tmp'
        temporary.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
        temporary.replace(self.directory/'summary.json')
        return result
