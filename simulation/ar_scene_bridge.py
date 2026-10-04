"""Copy measured transforms for the Quest mirror; adds no USD prims/cameras."""
import time

from config import tabletop_config as table_cfg
from config import ar_config
from simulation.tabletop import box_parts
from teleop.ar_scene import ScenePublisher


class ARSceneBridge:
    def __init__(self, robot, cubes, endpoint=ar_config.SCENE_ENDPOINT):
        self.robot, self.cubes = robot, cubes
        self.publisher = ScenePublisher(endpoint)
        self.last_submit = 0.
        self.sequence = 0
        self.geometry = {
            'table': {'size': table_cfg.TABLE_SIZE, 'position': table_cfg.TABLE_CENTER,
                      'color': table_cfg.TABLE_COLOR},
            'box': [{'name': n, 'size': size, 'position': pos, 'color': table_cfg.BOX_COLOR}
                    for n, size, pos in box_parts()],
            'cube_size': table_cfg.CUBE_SIZE, 'cube_color': table_cfg.CUBE_COLOR,
        }

    def update(self, boot_time, episode=None, reset_count=0):
        now = time.monotonic()
        if now - self.last_submit < 1. / ar_config.SCENE_HZ:
            return
        # Rigid LINK frames, not centres of mass and not commanded FK. This
        # preserves USD axis conventions and physical finger contact offsets.
        positions = self.robot.data.body_pos_w[0].cpu().tolist()
        rotations = self.robot.data.body_quat_w[0].cpu().tolist()
        roots = self.cubes.data.root_state_w[:, :7].cpu().tolist()
        self.sequence += 1
        self.publisher.submit({
            'version': 1, 'timestamp': now, 'sequence': self.sequence,
            'boot_time': boot_time, 'reset_count': reset_count,
            'links': {name: p + q for name, p, q in zip(self.robot.body_names, positions, rotations)},
            'cubes': roots, 'geometry': self.geometry,
            'episode': episode or {'state': 'UNRECORDED', 'start_allowed': True},
        })
        self.last_submit = now

    def close(self):
        self.publisher.close()
