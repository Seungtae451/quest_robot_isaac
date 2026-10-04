"""Fixed metric XR world mapping; no simulator, headset or IK runtime imports."""
import numpy as np

# Columns are Isaac +X forward, +Y left, +Z up in WebXR's Y-up frame.
XR_FROM_ISAAC = np.array([[0., -1., 0.], [0., 0., 1.], [-1., 0., 0.]])


def rigid_matrix(values):
    matrix = np.asarray(values, dtype=float)
    if matrix.shape == (16,):
        matrix = matrix.reshape(4, 4, order="F")
    if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
        raise ValueError("Expected a finite 4x4 rigid transform")
    r = matrix[:3, :3]
    if (not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-5)
            or not np.allclose(r.T @ r, np.eye(3), atol=1e-4)
            or not np.isclose(np.linalg.det(r), 1., atol=1e-4)):
        raise ValueError("Transform must be rigid with unit scale")
    return matrix.copy()


def world_matrix(origin, yaw=0.):
    c, s = np.cos(yaw), np.sin(yaw)
    result = np.eye(4)
    result[:3, :3] = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]]) @ XR_FROM_ISAAC
    result[:3, 3] = origin
    return rigid_matrix(result)


def controller_in_robot(controller, xr_from_robot):
    """Use gripSpace and a world-fixed transform, never the viewer/head pose."""
    world = rigid_matrix(xr_from_robot)
    return np.linalg.inv(world) @ rigid_matrix(controller)


def attachment_distances(poses, measured_wrists):
    positions = np.asarray(measured_wrists, dtype=float)
    poses = np.asarray(poses, dtype=float)
    if (positions.shape != (2, 3) or poses.shape != (2, 4, 4)
            or not np.isfinite(positions).all() or not np.isfinite(poses).all()):
        raise ValueError("Expected left/right grip poses and measured wrist positions")
    return np.linalg.norm(poses[:, :3, 3] - positions, axis=1)
