"""One place for rates, filters, network endpoints, and camera mounting geometry.

Distances are metres, angles radians, and quaternions are (w, x, y, z).
Neither process imports the other's runtime through this configuration module.
"""

CONTROL_HZ = 60.0  # consume newest Quest events, at most once per tick
QUEST_INPUT_HZ = 60.0  # requested browser controller stream rate (measured separately)
FEEDBACK_HZ = 60.0  # measured Isaac joints for the IK seed
FILTER_REFERENCE_HZ = 30.0  # preserve existing smoothing in seconds
PHYSICS_HZ = 60.0
CAMERA_FPS = 30.0

# Mapping gains are deliberately fixed at one. Filters affect transient response,
# not steady-state gain. Unreachable targets are rejected by IK, never rescaled.
POSITION_SCALE = 1.0
TRANSLATION_AXIS_SIGNS = (1.0, 1.0, 1.0)  # AR inverse-world transform already preserves displayed motion
ORIENTATION_SCALE = 1.0  # legacy offline mapping diagnostics only
POSITION_DEADBAND = 0.001
POSITION_FILTER_ALPHA = 0.8
ROTATION_FILTER_ALPHA = 0.8  # live world rotation smoothing
GRIPPER_FILTER_ALPHA = 0.3
CONTROLLER_TO_EE_ROT = ((1., 0., 0.), (0., 1., 0.), (0., 0., 1.))
# Legacy wrist-axis diagnostics; live control ignores controller rotation.
# Fixed downward EE rotations live in robot/f14_config.py beside HOME.
# Map controller LOCAL [twist X, bend Y, sideways Z] to each wrist link.
# The hand frame is [forward, left, up]: forward is link -Y for both arms,
# left/up are link -Z/+X on the left, and +Z/-X on the mirrored right.
# Columns are those three hand axes expressed in the corresponding EE link.
LEFT_CONTROLLER_TO_EE_ROT = ((0., 0., 1.), (-1., 0., 0.), (0., -1., 0.))
RIGHT_CONTROLLER_TO_EE_ROT = ((0., 0., -1.), (-1., 0., 0.), (0., 1., 0.))

# Legacy 2D mode only; default AR uses a fixed local-floor/world transform.
# This matches the inspected installed wrapper's default. Its positions are
# relative to the head, with head yaw removed, then shifted to a waist origin.
# Do not add another OpenXR axis swap. Keep the head still for mapping tests.
ARM_REFERENCE_MODE = "head_yaw"
CALIBRATION_SETTLE_SECONDS = 2.0
CALIBRATION_SAMPLES = 45
CALIBRATION_POSITION_TOLERANCE = 0.015
CALIBRATION_ROTATION_TOLERANCE = 0.10
IK_MAX_ITER = 100
IK_EPS = 2e-4
IK_DT = 0.3
IK_DAMPING = 1e-4
IK_LIMIT_MARGIN = 0.2617993877991494  # 15 degrees; avoid outward motion near URDF limits
IK_LIMIT_GAIN = 0.30  # nullspace joint-limit repulsion, rad per solver time unit
# Follow from measured FK in small Cartesian steps; shrink on IK failure.
IK_FOLLOW_MAX_STEP = 0.04  # m, per solve (receiver still limits joint speed)
IK_FOLLOW_MAX_JOINT_STEP = 0.20  # rad; reject a distant IK branch
IK_FOLLOW_RETRIES = 4
IK_FOLLOW_RETRY_ITER = 50
IK_FOLLOW_BUDGET_MS = 12.0  # stop starting retries after this elapsed budget

# Live Quest uses differential IK with two independent ProxQP problems.
# The iterative IK options above remain for offline reachability/spawn checks.
QP_POSITION_GAIN = 10.0  # s^-1, Cartesian error feedback
QP_ORIENTATION_GAIN = 10.0  # s^-1, controller-relative rotation feedback
QP_NONLINEAR_ROTATION_TOLERANCE = 0.002  # rad, transient linearization allowance (~0.11 deg)
QP_ORIENTATION_WEIGHT = 1.0  # reduce prior 3x angular priority; keep full pose task
QP_MAX_CARTESIAN_SPEED = 0.75  # m/s (75 cm/s), fast teleop reference
QP_LIMIT_GAIN = 0.10  # rad/s, redundant joint-limit avoidance
QP_REGULARIZATION = 1e-4  # positive definite objective, also near singularities
QP_MAX_ITER = 100  # small 7-variable QPs; leave room for active-set changes
QP_MAX_INNER_ITER = 50  # ProxQP's default 1500 is excessive for live control
QP_COLD_RETRY_BUDGET_MS = 4.0  # start at most one cold retry per nonconverged arm
QP_MAX_DT = 0.05  # never integrate a long pause in one step
QP_LAG_HORIZON = 0.02  # seconds; solver also uses at least actual control dt

# Both differential references and legacy destinations obey these limits.
# are always enforced by the Isaac receiver, including with --no-filter.
ARM_MAX_VELOCITY = 3.14  # rad/s (~180 deg/s)
ARM_MAX_ACCELERATION = 10.0  # rad/s^2, fast bounded response
ARM_MAX_TRACKING_ERROR = 0.05  # rad; brake if a drive cannot follow its reference

# The canonical USD's finger drives are only ~2.3-2.8 N/m and cannot hold the
# ~0.1 kg fingers against gravity: a real GPU test drifted to half closure.
# Override ONLY finger drive gains in the runtime articulation configuration;
# keep all validated arm gains and source USD/URDF files unchanged.
GRIPPER_STIFFNESS = 2000.0  # N/m, about 0.5 mm deflection under 1 N gravity
GRIPPER_DAMPING = 30.0     # N s/m, approximately critical for a 0.1 kg finger

ACTION_UDP_HOST = "127.0.0.1"
ACTION_UDP_PORT = 5005
VIDEO_ZMQ_ENDPOINT = "tcp://127.0.0.1:5556"
COMMAND_TIMEOUT = 0.5
TRACKING_TIMEOUT = 0.5
VIDEO_TIMEOUT = 1.0
JPEG_QUALITY = 80
LOG_INTERVAL = 1.0

BODY_CAMERA_WIDTH, BODY_CAMERA_HEIGHT = 640, 480
WRIST_CAMERA_WIDTH, WRIST_CAMERA_HEIGHT = 320, 240
QUEST_VIEW_WIDTH, QUEST_VIEW_HEIGHT = 1280, 720
# Head-following video plane in Quest, independent of the robot camera mounts.
QUEST_SCREEN_DISTANCE = 1.5  # metres; immersive upstream default was 1.0
QUEST_SCREEN_HEIGHT = 1.0    # metres; farther plane also reduces visual crowding
PANEL_GAP = 12

# CameraCfg convention="world": camera +X looks forward and +Z is image up.
# Offsets below are expressed in the PARENT LINK, not the global world frame.
# Restore the pre-table-move body camera: 85 cm high, 8 cm forward,
# aimed 50 degrees down, with the original 18 mm focal length (51dba6f).
BODY_CAMERA_OFFSET = (0.25, 0.0, 0.95)
BODY_CAMERA_ROT = (0.8538568606, 0.0, 0.5948227868, 0.0)
BODY_CAMERA_FOCAL_LENGTH = 18.0
WRIST_CAMERA_FOCAL_LENGTH = 18.0
CAMERA_HORIZONTAL_APERTURE = 24.0

# URDF finger origins extend along wrist LOCAL -Y. Sliders use left -Z/right
# +Z. Camera forward is local -Y, not robot/world +X. The image-up/mount axes
# are left local +X and right local -X, pointing up at the original HOME.
# Quaternion columns map camera [forward, left, up] to these parent-link axes.
# Tilt 25 degrees down in CAMERA coordinates to include the fingertips. The
# quaternions are the above axis alignment multiplied by camera Ry(+25 deg).
LEFT_WRIST_CAMERA_OFFSET = (0.055, -0.03, 0.0)
RIGHT_WRIST_CAMERA_OFFSET = (-0.055, -0.03, 0.0)
LEFT_WRIST_CAMERA_ROT = (0.3799281966, -0.3799281966, 0.5963678105, -0.5963678105)
RIGHT_WRIST_CAMERA_ROT = (0.5963678105, 0.5963678105, -0.3799281966, -0.3799281966)
