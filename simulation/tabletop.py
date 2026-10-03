"""Random tabletop layout and an open, collidable collection box.

Geometry/sampling functions have no Isaac imports, so the exclusion rule can
be checked independently. Spawning requires a running Isaac application.
"""
import math
import random

from config import tabletop_config as cfg


def table_surface_height():
    return cfg.TABLE_CENTER[2] + cfg.TABLE_SIZE[2] / 2


def cube_spawn_regions():
    """Left/right raw XY regions; the full cube must stay in one region."""
    x, y, _ = cfg.TABLE_CENTER
    length, width, _ = cfg.TABLE_SIZE
    fraction = cfg.CUBE_SPAWN_LENGTH_FRACTION
    start_fraction = cfg.CUBE_SPAWN_START_FRACTION
    if not 0 <= start_fraction < fraction <= 1:
        raise ValueError("Cube spawn fractions must satisfy 0 <= start < end <= 1")
    bx, by = cfg.BOX_CENTER_XY
    bl, bw, _ = cfg.BOX_SIZE
    if not (x - length / 2 <= bx - bl / 2 < bx + bl / 2 <= x + length / 2
            and y - width / 2 <= by - bw / 2 < by + bw / 2 <= y + width / 2):
        raise ValueError("Collection box must fit on the table")
    xmin = x - length / 2 + length * start_fraction + cfg.CUBE_TABLE_EDGE_MARGIN
    xmax = x - length / 2 + length * fraction - cfg.CUBE_TABLE_EDGE_MARGIN
    ymin = y - width / 2 + cfg.CUBE_TABLE_EDGE_MARGIN
    ymax = y + width / 2 - cfg.CUBE_TABLE_EDGE_MARGIN
    regions = [(xmin, xmax, ymin, min(ymax, by - bw / 2 - cfg.BOX_SPAWN_CLEARANCE)),
               (xmin, xmax, max(ymin, by + bw / 2 + cfg.BOX_SPAWN_CLEARANCE), ymax)]
    return [r for r in regions if r[1] > r[0] and r[3] > r[2]]


def sample_cube_poses(seed=None):
    """Fresh random XY/yaw on each startup, or reproducible with a seed.

    Account for the rotated footprint, not just the cube centre. Conservative
    AABB separation also prevents overlap if CUBE_COUNT is increased.
    """
    rng = random.Random(seed)
    regions = cube_spawn_regions()
    verified = None
    if cfg.CUBE_SPAWN_REQUIRE_IK:
        from robot.spawn_workspace import ensure_spawn_pool
        verified = ensure_spawn_pool()["candidates"]
    if not any(xmax - xmin > cfg.CUBE_SIZE and ymax - ymin > cfg.CUBE_SIZE
               for xmin, xmax, ymin, ymax in regions):
        raise ValueError("Cube does not fit on either side of the collection box")
    poses, footprints = [], []
    for _ in range(cfg.CUBE_COUNT):
        for attempt in range(1000):
            yaw = rng.uniform(-math.pi, math.pi)
            radius = cfg.CUBE_SIZE / 2 * (abs(math.cos(yaw)) + abs(math.sin(yaw)))
            allowed = [r for r in regions if r[1] - r[0] > 2 * radius and r[3] - r[2] > 2 * radius]
            if not allowed:
                continue
            if verified is not None:
                candidate = rng.choice(verified)
                x, y = candidate["x"], candidate["y"]
                if not any(xmin+radius<=x<=xmax-radius and ymin+radius<=y<=ymax-radius
                           for xmin,xmax,ymin,ymax in allowed):
                    continue
            else:
                weights = [(r[1] - r[0] - 2 * radius) * (r[3] - r[2] - 2 * radius) for r in allowed]
                xmin, xmax, ymin, ymax = rng.choices(allowed, weights=weights, k=1)[0]
                x = rng.uniform(xmin + radius, xmax - radius)
                y = rng.uniform(ymin + radius, ymax - radius)
            if any(abs(x - px) < radius + pr + cfg.CUBE_SEPARATION
                   and abs(y - py) < radius + pr + cfg.CUBE_SEPARATION
                   for px, py, pr in footprints):
                continue
            z = table_surface_height() + cfg.CUBE_SIZE / 2 + cfg.CUBE_DROP_GAP
            poses.append(((x, y, z), (math.cos(yaw / 2), 0., 0., math.sin(yaw / 2))))
            footprints.append((x, y, radius))
            break
        else:
            raise ValueError("Could not place all cubes without overlap; reduce CUBE_COUNT")
    return poses


def box_parts():
    """Five cuboids forming a hollow box; each tuple is (name, size, pos)."""
    x, y = cfg.BOX_CENTER_XY
    length, width, height = cfg.BOX_SIZE
    wall, bottom = cfg.BOX_WALL_THICKNESS, cfg.BOX_BOTTOM_THICKNESS
    if not (0 < wall < min(length, width) / 2 and 0 < bottom < height):
        raise ValueError("Invalid collection box wall/bottom thickness")
    z = table_surface_height()
    wall_height = height - bottom
    wall_z = z + bottom + wall_height / 2
    return [
        ("bottom", (length, width, bottom), (x, y, z + bottom / 2)),
        ("front", (wall, width, wall_height), (x + (length - wall) / 2, y, wall_z)),
        ("back", (wall, width, wall_height), (x - (length - wall) / 2, y, wall_z)),
        ("left", (length - 2 * wall, wall, wall_height), (x, y + (width - wall) / 2, wall_z)),
        ("right", (length - 2 * wall, wall, wall_height), (x, y - (width - wall) / 2, wall_z)),
    ]


def spawn_tabletop(seed=None):
    import isaaclab.sim as sim_utils

    poses = sample_cube_poses(seed)
    parts = box_parts()
    material = sim_utils.RigidBodyMaterialCfg(
        static_friction=0.8, dynamic_friction=0.6, restitution=0.0)
    table = sim_utils.CuboidCfg(
        size=cfg.TABLE_SIZE, collision_props=sim_utils.CollisionPropertiesCfg(),
        physics_material=material,
        visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=cfg.TABLE_COLOR))
    table.func("/World/Table", table, translation=cfg.TABLE_CENTER)
    for name, size, pos in parts:
        wall = sim_utils.CuboidCfg(
            size=size, collision_props=sim_utils.CollisionPropertiesCfg(),
            physics_material=material,
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=cfg.BOX_COLOR))
        wall.func(f"/World/CollectionBox/{name}", wall, translation=pos)
    cube = sim_utils.CuboidCfg(
        size=(cfg.CUBE_SIZE,) * 3,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False, solver_position_iteration_count=8,
            solver_velocity_iteration_count=2, max_depenetration_velocity=1.),
        mass_props=sim_utils.MassPropertiesCfg(mass=cfg.CUBE_MASS),
        collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=0.002, rest_offset=0.0),
        physics_material=material,
        visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=cfg.CUBE_COLOR, roughness=0.4))
    for index, (pos, rot) in enumerate(poses):
        cube.func(f"/World/Cubes/Cube_{index}", cube, translation=pos, orientation=rot)
        print(f"Tabletop cube {index}: position={pos}, yaw quaternion={rot}")
    print(f"Tabletop: open box at {cfg.BOX_CENTER_XY}; left/right cube spawn XY bounds={cube_spawn_regions()}")
