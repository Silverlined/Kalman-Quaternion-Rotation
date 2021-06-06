import numpy as np
import scipy.constants
from math import sqrt
import pandas as pd
from numpy.linalg import inv as inverse
import matplotlib.pyplot as plt

# roll -> phi
# pitch -> theta
# yaw -> omega

# Constants
g = scipy.constants.g  # Standard acceleration of gravity
delta_t = 0.01  # Time step [s], 100Hz

# 4x4 transformation matrices are needed because we are working with quaternions
H = np.identity(4)
C = np.identity(4)

# Initialize covariance matrices
Q_process_noise_matrix = np.array([[10 ** -4, 0, 0, 0], [0, 10 ** -4, 0, 0], [0, 0, 10 ** -4, 0], [0, 0, 0, 10 ** -4]])
R_measurement_noise_matrix = np.array([[10, 0, 0, 0], [0, 10, 0, 0], [0, 0, 10, 0], [0, 0, 0, 10]])

def accelerometer_to_attitude(accelerometer_x, accelerometer_y, accelerometer_z):
    """Calculate roll and pitch angles from normalized (calibrated, filtered) accelerometer readings. (Measurement for Kalman) """

    pitch = np.arcsin(accelerometer_x / g)
    roll = np.arcsin(accelerometer_y / (g * np.cos(pitch)))
    yaw = 0 # 

    # roll = np.arctan2(accelerometer_y, accelerometer_z)
    # pitch = np.arctan2(-accelerometer_x, sqrt(accelerometer_y * accelerometer_y + accelerometer_z * accelerometer_z))
    # return np.array([roll, pitch, yaw], ndmin=2).transpose()
    return roll, pitch, yaw


def euler_angles_to_quaternions(roll, pitch, yaw):
    q_1 = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
    q_2 = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
    q_3 = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2)
    q_4 = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) + np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2)

    # return np.array([q_1, q_2, q_3, q_4], ndmin=2).transpose()
    return q_1, q_2, q_3, q_4


def quoternions_to_euler_angles(q_1, q_2, q_3, q_4):
    phi = np.degrees(np.arctan2(2 * (q_1 * q_2 + q_3 * q_4), 1 - 2 * (q_2 ** 2 + q_3 ** 2)))
    theta = np.degrees(np.arcsin(2 * (q_1 * q_3 - q_4 * q_2)))
    omega = np.degrees(np.arctan2(2 * (q_1 * q_4 + q_2 * q_3), 1 - 2 * (q_3 ** 2 + q_4 ** 2)))

    # return np.array([phi, theta, omega], ndmin=2).transpose()
    return phi, theta, omega


def calculate_transormation_martix_A(gyro_phi, gyro_theta, gyro_omega):
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

def main():
    # Initial Conditions
    q_1, q_2, q_3, q_4 = euler_angles_to_quaternions(0, 0, 0)
    prev_state = np.array([q_1, q_2, q_3, q_4], ndmin=2).transpose()
    prev_process_covariance = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

    # Read Excel Sheet
    data_frame = pd.read_excel("KF_data_2.xlsx", engine="openpyxl")
    # gyro_roll = data_frame["Gyro Phi"].to_numpy()
    # gyro_pitch = data_frame["Gyro Pitch"].to_numpy()

    # Collect Data
    accelerometer_data = np.array([data_frame["Accel X"], data_frame["Accel Y"], data_frame["Accel Z"]], ndmin=2).transpose()
    gyro_data = np.array([(data_frame["Gyro Phi"]), (data_frame["Gyro Theta"]), (data_frame["Gyro Omega"])], ndmin=2).transpose()

    time = np.linspace(0, 200.1, num=20001)
    kalman_corrected_phi = []
    kalman_corrected_theta = []
    kalman_corrected_omega = []
    i = 0
    break_point = 50000

    # Read gyro and accelerometer data
    for accelerometer_measurement, gyro_measurement in zip(accelerometer_data, gyro_data):
        if i == break_point:
            # print(accelerometer_measurement, gyro_measurement)
            break

        # Create a new matrix 'A' with angular velocities from gyro data
        A = calculate_transormation_martix_A(gyro_measurement[0], gyro_measurement[1], gyro_measurement[2])

        # Calculate the state estimate and process covariance estimate
        state_estimate = A @ prev_state
        process_covariance_estimate = A @ prev_process_covariance @ A.transpose() + Q_process_noise_matrix
        # process_covariance_estimate = np.diag(np.diag(process_covariance_estimate))

        # Compute the Kalman gain
        kalman_gain = (
            process_covariance_estimate
            @ H
            @ inverse(H @ process_covariance_estimate @ H.transpose() + R_measurement_noise_matrix)
        )

        # Convert measured acceleration to Euler angles
        accelerometer_roll, accelerometer_pitch, accelerometer_yaw = accelerometer_to_attitude(
            accelerometer_measurement[0], accelerometer_measurement[1], accelerometer_measurement[2]
        )

        # Convert then to Quaternions
        q_1, q_2, q_3, q_4 = euler_angles_to_quaternions(accelerometer_roll, accelerometer_pitch, accelerometer_yaw)
        Y_measurement_matrix = np.array([q_1, q_2, q_3, q_4], ndmin=2).transpose()

        # Compute the measurement
        state_measurement = C @ Y_measurement_matrix + 0  # Assume noise in calculating the measurement is 0

        # Correct the state estimate and process covariance estimate
        new_state = state_estimate + kalman_gain @ (state_measurement - H @ state_estimate)
        new_process_covariance = process_covariance_estimate - kalman_gain @ H @ process_covariance_estimate

        # Update previous state
        prev_state = new_state
        prev_process_covariance = new_process_covariance

        # Append data to list and repeat
        new_phi, new_theta, new_omega = quoternions_to_euler_angles(
            new_state.take(0), new_state.take(1), new_state.take(2), new_state.take(3)
        )

        # Collect data for comparison
        measured_phi, measured_theta, measured_omega = quoternions_to_euler_angles(q_1, q_2, q_3, q_4)

        kalman_corrected_phi.append(new_phi)
        kalman_corrected_theta.append(new_theta)
        kalman_corrected_omega.append(new_omega)
        i += 1

    dictionary = {
        "Time": time[:break_point],
        "Phi": kalman_corrected_phi,
        "Theta": kalman_corrected_theta,
        "Omega": kalman_corrected_omega,
    }
    kalman_data = pd.DataFrame(data=dictionary)
    print(kalman_data.tail())
    kalman_data.plot(x="Time", y=["Phi"])
    plt.show()


if __name__ == "__main__":
    main()
