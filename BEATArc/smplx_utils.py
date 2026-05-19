"""Small SMPL-X math helpers used by the active BEAT2 to NAO path."""

import numpy as np


def axis_angle_to_matrix(axis_angle: np.ndarray) -> np.ndarray:
    """Convert axis-angle rotation parameters shaped (3,) to a 3x3 matrix."""
    angle = np.linalg.norm(axis_angle)
    if angle < 1e-8:
        return np.eye(3, dtype=axis_angle.dtype)
    axis = axis_angle / angle
    cross_matrix = np.array([
        [0, -axis[2], axis[1]],
        [axis[2], 0, -axis[0]],
        [-axis[1], axis[0], 0],
    ], dtype=axis_angle.dtype)
    return (
        np.eye(3, dtype=axis_angle.dtype)
        + np.sin(angle) * cross_matrix
        + (1 - np.cos(angle)) * (cross_matrix @ cross_matrix)
    )
