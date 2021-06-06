import numpy as np
from scipy.constants import g


def accelerometer_to_attitude(accelerometer_x, accelerometer_y, accelerometer_z):
    """Calculate roll and pitch angles from normalized (calibrated, filtered) accelerometer readings. (Measurement for Kalman) """

    roll = np.arcsin(accelerometer_x / g)
    pitch = -np.arcsin(accelerometer_y / (g * np.cos(roll)))
    yaw = 0

    return roll, pitch, yaw


def euler_to_quaternion(roll, pitch, yaw):
    """Convert Euler angles to Quaternion"""

    q_1 = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
    q_2 = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
    q_3 = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2)
    q_4 = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2)

    return q_1, q_2, q_3, q_4

def quoternion_to_euler_angles(q_1, q_2, q_3, q_4):
    """Convert Quaternion to Euler angles"""

    phi = np.degrees(np.arctan2(2 * (q_1 * q_2 + q_3 * q_4), 1 - 2 * (q_2 ** 2 + q_3 ** 2)))
    theta = np.degrees(np.arcsin(2 * (q_1 * q_3 - q_4 * q_2)))
    omega = np.degrees(np.arctan2(2 * (q_1 * q_4 + q_2 * q_3), 1 - 2 * (q_3 ** 2 + q_4 ** 2)))

    return phi, theta, omega

def gyro_transition_matrix(gyro_phi, gyro_theta, gyro_omega, delta_t):
    """Calculate state transition matrix from gyro readings. (Estimate for Kalman) """

    A = np.array(
        np.identity(4)
        + (delta_t / 2)
        * np.array(
            [
                [0, -gyro_phi, -gyro_theta, -gyro_omega],
                [gyro_phi, 0, gyro_omega, -gyro_theta],
                [gyro_theta, -gyro_omega, 0, gyro_phi],
                [gyro_omega, gyro_theta, -gyro_phi, 0],
            ]
        )
    )
    return A


def normalize_quaternion(q_1, q_2, q_3, q_4):

    norm = np.sqrt(q_1 ** 2 + q_2 ** 2 + q_3 ** 2 +  q_4 ** 2)

    return np.array([q/norm for q in [q_1, q_2, q_3, q_4]])
