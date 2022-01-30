"""Implementation of Kalman Filter"""

import numpy as np

class KalmanFilter:
    def __init__(self, A: np.array, B: np.array, H: np.array):
        self.A = A
        self.B = B
        self.H = H

        self.Q = None
        self.R = None

        self.x0 = None
        self.p0 = None

        # Kalman gain
        self.K = None

    def covariance_matrices(self, Q: np.array, R: np.array):
        self.Q = Q
        self.R = R

    def initialise(self, x0: np.array, p0:np.array):
        self.x0 = x0
        self.p0 = p0

    def update(self, uk: np.array):
        self.x0 = self.A @ self.x0 + self.B @ uk
        self.p0 = self.A @ self.p0 @ self.A.T + self.Q

    def correct(self, zk: np.array):
        S = self.H @ self.p0 @ self.H.T + self.R
        self.K = self.p0 @ self.H.T @ S.T
        self.x0 = self.x0 + self.K @ (zk - self.H @ self.x0)
        self.p0 = (np.eye(self.H.shape[0]) - self.K * self.H) @ self.p0
