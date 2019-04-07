import numpy as np
import math

# Checks if r is a valid rotation matrix
def is_rotation_matrix(r):
    return np.linalg.norm(np.identity(3, dtype=r.dtype) - np.dot(np.transpose(r), r)) < 1e-6


def rot_to_euler(r):
    assert (is_rotation_matrix(r))
    sy = math.sqrt(r[0, 0] * r[0, 0] + r[1, 0] * r[1, 0])
    singular = sy < 1e-6
    if not singular:
        x = math.atan2(r[2, 1], r[2, 2])
        y = math.atan2(-r[2, 0], sy)
        z = math.atan2(r[1, 0], r[0, 0])
    else:
        x = math.atan2(-r[1, 2], r[1, 1])
        y = math.atan2(-r[2, 0], sy)
        z = 0
    return np.array([x, y, z])
