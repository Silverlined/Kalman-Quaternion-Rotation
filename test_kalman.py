# https://github.com/zziz/kalman-filter
import numpy as np
from Kalman import KalmanFilter

def example():
    dt = 1.0/60
    F = np.array([[1, dt, 0], [0, 1, dt], [0, 0, 1]])
    P = np.eye(F.shape[1])
    H = np.array([1, 0, 0]).reshape(1, 3)
    Q = np.array([[0.05, 0.05, 0.0], [0.05, 0.05, 0.0], [0.0, 0.0, 0.0]])
    R = np.array([0.5]).reshape(1, 1)
    x0 = np.zeros((F.shape[1], 1))

    x = np.linspace(-10, 10, 100)
    measurements = - (x**2 + 2*x - 2)  + np.random.normal(0, 2, 100)

    kf = KalmanFilter(F = F, H = H, P = P, Q = Q, R = R, x0 = x0)
    predictions = []

    for z in measurements:
        kf.predict()
        predictions.append(np.dot(H,  kf.correct(z))[0])

    import matplotlib.pyplot as plt
    plt.plot(range(len(measurements)), measurements, label = 'Measurements')
    plt.plot(range(len(predictions)), np.array(predictions), label = 'Kalman Filter Prediction')
    plt.legend()
    plt.show()

if __name__ == '__main__':
    example()
