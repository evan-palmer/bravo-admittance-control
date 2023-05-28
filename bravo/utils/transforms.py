from scipy.spatial.transform import Rotation as R
import numpy as np


def transform_forces(
    forces: np.ndarray, translation: np.ndarray, rotation: R
) -> np.ndarray:
    """Apply a transformation to a vector of forces and moments.

    Args:
        forces: The forces and moments to transform provided as the 1x6 vector
            [fx, fy, fz, mx, my, mz].
        translation: The translation vector from the current frame to the desired frame.
        rotation: The rotation from the current frame to the desired frame.

    Returns:
        The transformed forces.
    """
    # Convert the translation into a skew-symmetric matrix
    skew = np.array(
        [
            [0, -translation[2], translation[1]],
            [translation[2], 0, -translation[0]],
            [-translation[1], translation[0], 0],
        ]
    )

    # Get the rotation matrix
    rot_mat = rotation.as_matrix()

    # Create the transformation matrix
    transform = np.zeros((6, 6))
    transform[:3, :3] = rot_mat
    transform[3:, :3] = skew @ rot_mat
    transform[3:, 3:] = rot_mat

    # Apply the transformation and return
    return transform @ forces
