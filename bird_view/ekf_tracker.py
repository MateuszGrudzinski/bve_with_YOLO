import numpy as np
import time

class TrackedObject:
    def __init__(self, obj_id, initial_pos):
        self.id = obj_id
        self.state = np.array([*initial_pos, 0.0, 0.0, 0.0])  # x, y, z, vx, vy, vz
        self.P = np.eye(6) * 0.1  # initial uncertainty
        self.last_update = time.time()
        self.missed = 0

    def predict(self, dt):
        F = np.eye(6)
        for i in range(3):
            F[i, i+3] = dt

        Q = np.eye(6) * 0.01  # process noise
        self.state = F @ self.state
        self.P = F @ self.P @ F.T + Q

    def update(self, z):
        H = np.eye(3, 6)  # measurement matrix
        R = np.eye(3) * 0.05  # measurement noise

        y = z - H @ self.state
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)

        self.state += K @ y
        self.P = (np.eye(6) - K @ H) @ self.P
        self.last_update = time.time()
        self.missed = 0

    def get_position(self):
        return self.state[:3]