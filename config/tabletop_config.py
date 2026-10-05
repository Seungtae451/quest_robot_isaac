"""Tabletop geometry in robot/world axes: +X forward, +Y left, +Z up."""

# Original table location before the 10 cm move toward the body (51dba6f).
# Preserve surface z = 0.30 m and the same dimensions.
TABLE_CENTER = (0.55, 0.0, 0.28)
TABLE_SIZE = (0.60, 0.80, 0.04)
TABLE_COLOR = (0.45, 0.30, 0.18)

# Open collection box: bottom and four static collision walls, no lid.
BOX_CENTER_XY = TABLE_CENTER[:2]  # middle of the tabletop
BOX_SIZE = (0.18, 0.18, 0.04)  # outside dimensions, including bottom
BOX_WALL_THICKNESS = 0.01
BOX_BOTTOM_THICKNESS = 0.008
BOX_COLOR = (0.55, 0.80, 0.55)  # light green

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

# Restore the original continuous XY/yaw sampling over the regions above.
# The later fixed-downward IK grid is not used by this free-orientation setup.
# Offline grid settings below remain available for explicit reachability work.
CUBE_SPAWN_REQUIRE_IK = False
IK_SPAWN_GRID_STEP = 0.015  # exact verified XY locations; no unchecked jitter
IK_SPAWN_PATH_STEP = 0.015  # Cartesian waypoint spacing for continuity checks
IK_SPAWN_MAX_JOINT_STEP = 0.20  # reject branch jumps (rad per waypoint)
IK_SPAWN_JOINT_MARGIN = 0.03  # keep ~1.7 deg away from URDF limits
GRASP_LOCAL_OFFSET = (0.0, -0.225, -0.0225)  # wrist -> finger grasp center (m)
GRASP_APPROACH_HEIGHT = 0.04
GRASP_LIFT_HEIGHT = 0.06
BOX_RELEASE_GAP = 0.01
CUBE_GRIPPER_BOX_CLEARANCE = 0.08  # center clearance beyond box edge for open jaws
SPAWN_CAMERA_MARGIN = 0.05  # cube corners must lie inside the central 90% image
