import numpy as np

def rotation_matrix_to_euler_angles(R: np.ndarray) -> np.ndarray:
    """
    Converts a rotation matrix to Euler angles.
    Assuming the angles are in the following order: Roll, Pitch, Yaw (X, Y, Z rotations)
    """
    sy = np.sqrt(R[0, 0]**2 + R[1, 0]**2)

    singular = sy < 1e-6

    if not singular:
        x = np.arctan2(R[2, 1], R[2, 2])  # Roll
        y = np.arctan2(-R[2, 0], sy)      # Pitch
        z = np.arctan2(R[1, 0], R[0, 0])  # Yaw
    else:
        x = np.arctan2(-R[1, 2], R[1, 1])  # Roll
        y = np.arctan2(-R[2, 0], sy)       # Pitch
        z = 0                              # Yaw

    return np.array([x, y, z])