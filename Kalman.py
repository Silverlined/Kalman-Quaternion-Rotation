import numpy as np
from numpy.linalg import inv as inverse

"""
# x -> state estimate;
# z -> state measurement;
# F -> state-transition model;
# H -> observation model;
# P -> process covariance;
# Q -> covariance of the process noise;
# R -> covariance of the observation noise;
# K -> kalman gain;
"""

class KalmanFilter:
    def __init__(self, x0, F, H, P, Q, R):
        self.n = F.shape[1]
        self.m = H.shape[1]

        self.F = F
        self.H = H
        self.P = P
        self.Q = Q
        self.R = R
        self.x = x0

    def predict(self):
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.transpose() + self.Q
        
    def correct(self, z):
        self.K = self.P @ self.H.transpose() @ inverse(self.H @ self.P @ self.H.transpose() + self.R)
        self.x = self.x + self.K @ (z - self.H @ self.x)

        I = np.eye(self.n)
        self.P = (I - self.K @ self.H) @ self.P

        return self.x

    def update_state_transition(self, F):
        self.F = F

    def normalize_x(self, x):
        self.x = x

    
