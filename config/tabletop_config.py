"""Tabletop geometry in robot/world axes: +X forward, +Y left, +Z up."""

# Preserve the table already placed in f14_scene.py (surface z = 0.30 m).
TABLE_CENTER = (0.55, 0.0, 0.28)
TABLE_SIZE = (0.60, 0.80, 0.04)
TABLE_COLOR = (0.45, 0.30, 0.18)

# Open collection box: bottom and four static collision walls, no lid.
BOX_CENTER_XY = TABLE_CENTER[:2]  # middle of the tabletop
BOX_SIZE = (0.18, 0.18, 0.04)  # outside dimensions, including bottom
BOX_WALL_THICKNESS = 0.01
BOX_BOTTOM_THICKNESS = 0.008
BOX_COLOR = (0.12, 0.40, 0.70)

CUBE_COUNT = 1
CUBE_SIZE = 0.03
CUBE_MASS = 0.02  # kg
CUBE_COLOR = (1.0, 0.02, 0.02)
CUBE_TABLE_EDGE_MARGIN = 0.015
# Measured from the near (body-side, -X) table edge: use only 1/4 to 2/3.
CUBE_SPAWN_START_FRACTION = 1 / 4
CUBE_SPAWN_LENGTH_FRACTION = 2 / 3
CUBE_SEPARATION = 0.01
CUBE_DROP_GAP = 0.004

# Cubes spawn on either side (+Y/-Y) of the box, with this clearance.
# Reserve the central strip across the full table length.
BOX_SPAWN_CLEARANCE = 0.02
