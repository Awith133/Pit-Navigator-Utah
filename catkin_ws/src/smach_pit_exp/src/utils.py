import tf


def get_transformation_matrix(tvec, quat):
    euler_angles = tf.transformations.euler_from_quaternion(quat, 'rxyz')
    T = tf.transformations.compose_matrix(angles=euler_angles, translate=tvec)
    return T
