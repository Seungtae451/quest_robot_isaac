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
ORIENTATION_SCALE = 1.0
POSITION_DEADBAND = 0.005
POSITION_FILTER_ALPHA = 0.2
ROTATION_FILTER_ALPHA = 0.2
GRIPPER_FILTER_ALPHA = 0.3
CONTROLLER_TO_EE_ROT = ((1., 0., 0.), (0., 1., 0.), (0., 0., 1.))
# Map controller LOCAL [twist X, bend Y, sideways Z] to each wrist link.
# The hand frame is [forward, left, up]: forward is link -Y for both arms,
# left/up are link -Z/+X on the left, and +Z/-X on the mirrored right.
# Columns are those three hand axes expressed in the corresponding EE link.
LEFT_CONTROLLER_TO_EE_ROT = ((0., 0., 1.), (-1., 0., 0.), (0., -1., 0.))
RIGHT_CONTROLLER_TO_EE_ROT = ((0., 0., -1.), (-1., 0., 0.), (0., 1., 0.))

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

# IK produces a destination, never an instantaneous drive target. These limits
# are always enforced by the Isaac receiver, including with --no-filter.
ARM_MAX_VELOCITY = 0.25  # rad/s (~14.3 deg/s)
ARM_MAX_ACCELERATION = 0.50  # rad/s^2 (~28.6 deg/s^2)
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
MAIN_VIEW_FRACTION = 0.64
PANEL_GAP = 12

# CameraCfg convention="world": camera +X looks forward and +Z is image up.
# Offsets below are expressed in the PARENT LINK, not the global world frame.
# The base_link axes are robot +X forward, +Y left, +Z up. +50 deg about Y
# tilts the body camera down, 20 degrees below the previous 30-degree view.
BODY_CAMERA_OFFSET = (0.08, 0.0, 0.85)
BODY_CAMERA_ROT = (0.9063077870, 0.0, 0.4226182617, 0.0)

# URDF finger origins extend along wrist LOCAL -Y. Sliders use left -Z/right
# +Z. Camera forward is local -Y, not robot/world +X. At HOME, left local +X
# and right local -X point upward. Use those as image-up and mounting offsets
# so the mirrored wrists do not produce an upside-down right-hand image.
# Quaternion columns map camera [forward, left, up] to these parent-link axes.
# Tilt 25 degrees down in CAMERA coordinates to include the fingertips. The
# quaternions are the above axis alignment multiplied by camera Ry(+25 deg).
LEFT_WRIST_CAMERA_OFFSET = (0.055, -0.03, 0.0)
RIGHT_WRIST_CAMERA_OFFSET = (-0.055, -0.03, 0.0)
LEFT_WRIST_CAMERA_ROT = (0.3799281966, -0.3799281966, 0.5963678105, -0.5963678105)
RIGHT_WRIST_CAMERA_ROT = (0.5963678105, 0.5963678105, -0.3799281966, -0.3799281966)
