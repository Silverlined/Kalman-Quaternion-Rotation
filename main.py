import numpy as np
import pandas as pd
from Kalman import KalmanFilter
from conversion import accelerometer_to_attitude, euler_to_quaternion, quoternion_to_euler_angles, gyro_transition_matrix, normalize_quaternion
from matplotlib import pyplot as plt
from time import sleep
from serial import Serial
from filter import applyLowPass

import socket
UDP_IP = "127.0.0.1"
UDP_PORT = 5005
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Config
PORT = "/dev/ttyACM0"
SAMPLE_SIZE = 600
BAUD_RATE = 115200
TIME_STEP = 0.01

delta_t = 0.01  # Time step [s], 100Hz

# Initialize covariance matrices
Q = np.array([[10 ** -4, 0, 0, 0],
            [0, 10 ** -4, 0, 0], 
            [0, 0, 10 ** -4, 0], 
            [0, 0, 0, 10 ** -4]])

R = np.array([[10, 0, 0, 0],
            [0, 10, 0, 0],
            [0, 0, 10, 0],
            [0, 0, 0, 10]])

    
arduino = None

def getData():
    global arduino
    
    if not arduino:
        arduino = Serial(PORT, BAUD_RATE, timeout=0.1)
        print("Opened", arduino.name)
        sleep(3)
        arduino.readline() # Flush input

    ser_bytes = arduino.readline()
    decoded_bytes = ser_bytes[0:len(ser_bytes)-2].decode("utf-8", errors='ignore') # remove trailing characters (\r\n)
    
    if not "Data:" in decoded_bytes:
        return None

    vals = decoded_bytes.replace("Data:", "").strip().split(',')
    if len(vals) != 6:
        return None
    vals = [float(i) for i in vals]
    return vals


def main():
    x0 = np.array(euler_to_quaternion(0,0,0))
    df = pd.read_excel("data/KF_data_2.xlsx", engine="openpyxl")
    F = np.identity(4)
    H = np.identity(4)
    P = np.eye(4)

    kalman = KalmanFilter(x0, F, H, P, Q, R)

    # Collect Data
    accelerometer_data = np.array([df["Accel X"], df["Accel Y"], df["Accel Z"]], ndmin=2).transpose()
    gyro_data = np.array([(df["Gyro Phi"]), (df["Gyro Theta"]), (df["Gyro Omega"])], ndmin=2).transpose()

    time = np.linspace(0, 200.1, num=20001)
    kalman_corrected_phi = []
    kalman_corrected_theta = []
    kalman_corrected_omega = []
    i = 0
    buffers_accel = np.zeros((3,6))
    buffers_gyros = np.zeros((3,6))
    buffers = np.zeros((6,6))

    while(1):
    # for accelerometer_measurement, gyro_measurement in zip(accelerometer_data, gyro_data):
        data = getData()
        if data is not None:
        
            # accelerometer_measurement = data[0:3] 
            # gyro_measurement = data[3:6]
            filtered_data = np.zeros((1, 6))

            for index, val in enumerate(data):
                buffers[index], filtered_data[0, index] = applyLowPass(buffers[index], val)

            accelerometer_measurement = filtered_data[0, 0:3] 
            gyro_measurement = filtered_data[0, 3:6]
            
            F = gyro_transition_matrix(gyro_measurement[0], gyro_measurement[1], gyro_measurement[2], delta_t)
            kalman.update_state_transition(F)

            kalman.predict()

            z = euler_to_quaternion(*accelerometer_to_attitude(accelerometer_measurement[0], accelerometer_measurement[1], accelerometer_measurement[2]))

            x = kalman.correct(z)
            x = normalize_quaternion(*x)
            kalman.normalize_x(x)

            phi, theta, omega = quoternion_to_euler_angles(*x)

            kalman_corrected_phi.append(phi)
            kalman_corrected_theta.append(theta)
            kalman_corrected_omega.append(omega)

            message = b"y%.4fyp%.4fpr%.4fr" % (omega, theta, phi)
            # Quaternions for the win.
            q_message = b"w%.4fwa%.4fab%.4fbc%.4fc" % tuple(x[:])
            sock.sendto(q_message, (UDP_IP, UDP_PORT))
            sleep(0.003)

    def _plotData():
        dictionary = {
            "Time": time,
            "Phi": kalman_corrected_phi,
            "Theta": kalman_corrected_theta,
            "Omega": kalman_corrected_omega,
        }
        kalman_data = pd.DataFrame(data=dictionary)
        kalman_data.plot(x="Time", y=["Phi"])
        kalman_data.plot(x="Time", y=["Theta"])
        kalman_data.plot(x="Time", y=["Omega"])
        plt.show()
    
    # _plotData()
    

        

if __name__ == "__main__":
    main()
