import numpy as np
import transformations as tf


# Point operations

def distance(point1, point2):
    """Calculates the distance between two points.

    >>> point1 = np.array([0.8, 0.9, 1.0])
    >>> point2 = np.array([1.0, 0.9, 0.8])
    >>> distance(point1, point2)
    0.28284271247461895
    """
    vec12 = point2 - point1
    return np.sqrt(np.sum(vec12 * vec12))


def distance2(point1, point2):
    """Calculates the square of the distance between two points.

    >>> point1 = np.array([0.8, 0.9, 1.0])
    >>> point2 = np.array([1.0, 0.9, 0.8])
    >>> distance2(point1, point2)
    0.07999999999999996
    """
    mag12 = point2 - point1
    return np.sum(mag12 * mag12)


def angle(point1, point2, point3):
    pnt21, pnt23 = point1 - point2, point3 - point2
    dist21 = np.sqrt(np.sum(pnt21 * pnt21))
    dist23 = np.sqrt(np.sum(pnt23 * pnt23))
    val = np.sum(pnt21 * pnt23) / (dist21 * dist23)
    if val < -1:
        val = -1.0
    elif val > 1:
        val = 1.0
    return np.arccos(val)


def normal_vector(point1, point2, point3):
    vector1 = vector(point1, point2)
    vector2 = vector(point1, point3)
    vector3 = cross(vector1, vector2)
    return normalize(vector3)


def line2d(point1, point2):
    (x1, y1), (x2, y2) = point1, point2
    m = (y2 - y1) / (x2 - x1)
    b = y1 - m * x1
    return m, b


# Vector operations

def vector(point1, point2):
    """Calculates the vector defined by two points."""
    return point2 - point1


def magnitude2(vector):
    """Calculates the square of the magnitude of the vector.

    >>> magnitude2(np.float32([1.0, 0.9, 1.0]))
    2.8099999
    """
    return np.sum(vector * vector)


def magnitude(vector):
    """Calculates the magnitude of the vector.

    >>> magnitude(np.float32([1.0, 0.9, 1.0]))
    1.6763054
    """
    return np.sqrt(magnitude2(vector))


def normalize(vector):
    """Scales each component of the vector so that it has a magnitude of 1.

    >>> normalize(np.float32([1.0, 0.9, 1.0]))
    array([ 0.59654999,  0.53689498,  0.59654999], dtype=float32)
    """
    m = magnitude(vector)
    if m:
        vector = vector / m
    return vector


def dot(vector1, vector2):
    """Calculates the dot product of two vectors.

    >>> vec = np.array([0.8, 0.9, 1.0])
    >>> dot(vec, vec)
    2.4500000000000002
    """
    return np.sum(vector1 * vector2)


def cross(vector1, vector2):
    """Calculates the cross product of two vectors.

    >>> vec1 = np.array([0.8, 0.9, 1.0])
    >>> vec2 = np.array([1.0, 0.9, 0.8])
    >>> cross(vec1, vec2)
    array([-0.18,  0.36, -0.18])
    """
    (u1, v1, w1), (u2, v2, w2) = vector1, vector2
    return np.array([v1 * w2 - w1 * v2, w1 * u2 - u1 * w2, u1 * v2 - v1 * u2])


# Frame pose transformation functions

def pose_to_matrix(pose):
    """Returns the homogeneous transformation matrix from the pose (R, t)."""
    mat = np.eye(4)
    mat[:3, :3] = pose[0]  # R
    mat[:3, 3] = pose[1]  # t
    return mat


def rpypose_to_matrix(trans, rpy):
    """Returns the homogeneous matrix from a roll, pitch, yaw pose."""
    mat = tf.euler_matrix(*rpy, axes='sxyz')
    mat[:3, 3] = trans
    return mat


def rpypose_to_pose(trans, rpy):
    """Returns the (R,t) pose from a rpy pose."""
    mat = tf.euler_matrix(*rpy, axes='sxyz')
    return mat[:3, :3], np.array(trans)


def rpypose_to_quatpose(trans, rpy):
    """Returns the rpy pose from a quaternion pose."""
    quat = tf.quaternion_from_euler(*rpy, axes='sxyz')
    return np.array(trans), quat


def quatpose_to_matrix(trans, quat):
    """Returns the homogeneous transformation matrix from a quaternion pose."""
    mat = tf.quaternion_matrix(quat)
    mat[:3, 3] = trans
    return mat


def quatpose_to_pose(trans, quat):
    """Returns the (R,t) pose from a quaternion pose."""
    mat = tf.quaternion_matrix(quat)
    return mat[:3, :3], np.array(trans)


def quatpose_to_rpypose(trans, quat):
    """Returns the rpy pose from a quaternion pose."""
    rpy = tf.euler_from_quaternion(quat, axes='sxyz')
    return np.array(trans), rpy


def matrix_to_pose(mat):
    """Returns the (R,t) pose from a homogeneous transformation matrix."""
    R = mat[:3, :3]
    t = mat[:3, 3]
    return R, t


def matrix_to_rpypose(mat):
    """Returns the rpy pose from a homogeneous transformation matrix."""
    rpy = tf.euler_from_matrix(mat, axes='sxyz')
    trans = mat[:3, 3]
    return trans, rpy


def matrix_to_quatpose(mat):
    """Returns the quaternion pose from a homogeneous transformation matrix."""
    quat = tf.quaternion_from_matrix(mat)
    trans = mat[:3, 3]
    return trans, quat


def matrix_invert(matrix):
    """Returns the inverted transformation matrix."""
    return np.linalg.inv(matrix)


def matrix_compose(matrices):
    """Returns the concatenation of the transformations one by one."""
    transformation = np.eye(4)
    for matrix in matrices:
        transformation = np.dot(transformation, matrix)
    return transformation


def matrix_transformation(matrix1, matrix2):
    """Returns the transformation matrix between matrix1 and matrix2."""
    imatrix1 = matrix_invert(matrix1)
    transformation = matrix_compose((imatrix1, matrix2))
    return transformation


def points_transformation(matrix, points):
    """Transforms points with the especified transformation matrix."""
    pnts = np.empty(points.shape)
    for k, point in enumerate(points):
        pnts[k] = np.dot(matrix, np.hstack((point, 1)))[:3]
    return pnts

# ---------------------------------------------------------------------------


def transform_points2d(points, pose):
    """Transforms 2D points to world coordinates."""
    pnts = np.float32([np.dot(pose[0], np.float32([x, y, 0])) + pose[1]
                       for x, y in points])
    return pnts


def transform_points(pose, points):
    """Transfrom 3D points to world coordinates."""
    pnts = np.float32([np.dot(pose[0], point) + pose[1] for point in points])
    return pnts


if __name__ == '__main__':
    import doctest
    doctest.testmod()
