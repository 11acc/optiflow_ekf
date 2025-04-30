
# ekf.py
#
# Script implementation of an Extended Kalman Filter (EKF) for position and velocity estimation
# with sensor validation for PMW3901 sensors
#
# External Code Attribution:
# - The EKF algorithm structure (prediction and update steps) is based on:
#   - "The Extended Kalman Filter: An Interactive Tutorial for Non-Experts" by Kalman Levy
#     (https://home.wlu.edu/~levys/kalman_tutorial/)
#   - "Kalman Filtering – A Practical Implementation Guide (with code!)" by Robots for Roboticists
#     (https://www.robotsforroboticists.com/kalman-filtering/)
# - These sources provide the standard EKF equations for nonlinear systems, including linearization and covariance updates
#
# Original Contributions:
# - Added sensor measurement validation to handle outliers from PMW3901 sensors
# - Enhanced visualization with uncertainty ellipses for position estimate
# - Tailored the measurement model and state transition for PMW3901 sensor data.
#

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse


class EKF:
    def __init__(self, dt: float, initial_state: np.array, initial_covariance: np.array, process_noise: np.array, measurement_noise: np.array) -> None:
        """
        :param: dt:                  Time step between measurements in seconds
        :param: initial_state:       Initial state [x, y, z, vx, vy, vz]
        :param: initial_covariance:  Initial state covariance matrix (6x6)
        :param: process_noise:       Process noise covariance matrix Q (6x6)
        :param: measurement_noise:   Measurement noise covariance matrix R (8x8)
        """
        # State vector: [x, y, z, vx, vy, vz]
        self.x = initial_state
        self.P = initial_covariance
        self.Q = process_noise
        self.R = measurement_noise
        self.dt = dt

        # State transition matrix F (constant velocity model)
        self.F = np.array([
            [1, 0, 0, dt, 0,  0],
            [0, 1, 0, 0,  dt, 0],
            [0, 0, 1, 0,  0,  dt],
            [0, 0, 0, 1,  0,  0],
            [0, 0, 0, 0,  1,  0],
            [0, 0, 0, 0,  0,  1]
        ])

        # Measurement matrix H, state to measurements
        # For each sensor: dx = vx * dt, dy = vy * dt
        # z = [dx1, dy1, dx2, dy2, dx3, dy3, dx4, dy4]
        self.H = np.zeros((8, 6))
        for i in range(4):  # Four sensors
            self.H[2*i, 3] = dt  # dx_i relates to vx
            self.H[2*i + 1, 4] = dt  # dy_i relates to vy

        # For measurement validation
        self.measurement_history = []

    def predict(self) -> None:
        """
        EKF prediction step: Predict the next state and covariance
        """
        # Predict state: x = F * x
        self.x = self.F @ self.x

        # Predict covariance: P = F * P * F^T + Q
        self.P = self.F @ self.P @ self.F.T + self.Q

    def update(self, z: np.array) -> None:
        """
        EKF update step: Update state and covariance with new measurements

        :param: z: Measurement vector [dx1, dy1, dx2, dy2, dx3, dy3, dx4, dy4]
        """
        # Validate measurement
        if not self.validate_measurement(z):
            print("Measurement outlier detected, skipping update.")
            return

        # Predicted measurement: z_pred = H * x
        z_pred = self.H @ self.x

        # Measurement residual: y = z - z_pred
        y = z - z_pred

        # Innovation covariance: S = H * P * H^T + R
        S = self.H @ self.P @ self.H.T + self.R

        # Kalman gain: K = P * H^T * S^-1
        K = self.P @ self.H.T @ np.linalg.inv(S)

        # Update state: x = x + K * y
        self.x = self.x + K @ y

        # Update covariance: P = (I - K * H) * P
        I = np.eye(self.P.shape[0])
        self.P = (I - K @ self.H) @ self.P

    def validate_measurement(self, z: np.array) -> bool:
        """
        Validate sensor measurements by checking for outliers

        :param: z:  Measurement vector
        :return:    True if measurement is valid, False if outlier
        """

        if len(self.measurement_history) < 10:
            self.measurement_history.append(z)
            return True

        # Compute mean and std dev of recent measurements
        history = np.array(self.measurement_history[-10:])
        mean = np.mean(history, axis=0)
        std = np.std(history, axis=0)

        # Check if measurement is within 3 standard deviations
        if np.any(np.abs(z - mean) > 3 * std):
            return False

        self.measurement_history.append(z)
        return True

    def get_state(self) -> np.array:
        # Return the current state
        return self.x

    def get_position_covariance(self) -> np.array:
        # Return the position covariance (top-left 3x3 of P)
        return self.P[:3, :3]


# /// Load and processing of data for EKF
def run_ekf(data_file: str, dt: float) -> pd.DataFrame:
    """
    Run the EKF on raw data CSV file and plot results with uncertainty ellipses

    :param: data_file:  Path to serial monitor raw output CSV file
    :param: dt:         Time step between measurements in seconds
    """

    df = pd.read_csv(data_file)

    # Initialise EKF
    initial_state = np.zeros(6)
    initial_covariance = np.eye(6) * 10
    process_noise = np.diag([0.1, 0.1, 0.1, 1.0, 1.0, 1.0])
    measurement_noise = np.eye(8) * 5

    ekf = EKF(dt, initial_state, initial_covariance, process_noise, measurement_noise)

    # State estimates
    states = []
    position_covariances = []

    # For each time step...
    for _, row in df.iterrows():
        # extract measurement vector [dx_down, dy_down, ..., dx_right, dy_right]
        z = np.array([
            row['dx_down'], row['dy_down'],
            row['dx_forward'], row['dy_forward'],
            row['dx_left'], row['dy_left'],
            row['dx_right'], row['dy_right']
        ])

        # Now run the EKF steps
        ekf.predict()
        ekf.update(z)

        # And store the state and position covariance
        states.append(ekf.get_state().copy())
        position_covariances.append(ekf.get_position_covariance().copy())

    # Save in df
    states_df = pd.DataFrame(states, columns=['x', 'y', 'z', 'vx', 'vy', 'vz'])
    states_df['timestamp'] = df['timestamp']

    # // Plotting
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10))

    # Position with uncertainty ellipses (x-y plane)
    ax1.plot(states_df['x'], states_df['y'], label='Estimated Path', color='blue')
    for i in range(0, len(states_df), 10):  # Plot every 10th point for clarity
        cov = position_covariances[i][:2, :2]  # x-y covariance
        eigvals, eigvecs = np.linalg.eigh(cov)
        angle = np.degrees(np.arctan2(eigvecs[1, 0], eigvecs[0, 0]))
        width, height = 2 * np.sqrt(eigvals)  # 2-sigma ellipse
        ellipse = Ellipse((states_df['x'].iloc[i], states_df['y'].iloc[i]), width, height, angle=angle, alpha=0.2, color='red')
        ax1.add_patch(ellipse)
    ax1.set_title('Estimated Path with Uncertainty Ellipses (x-y Plane)')
    ax1.set_xlabel('X Position (pixels)')
    ax1.set_ylabel('Y Position (pixels)')
    ax1.legend()
    ax1.grid(True)
    ax1.axis('equal')

    # Velocity
    ax2.plot(states_df['timestamp'], states_df['vx'], label='vx')
    ax2.plot(states_df['timestamp'], states_df['vy'], label='vy')
    ax2.plot(states_df['timestamp'], states_df['vz'], label='vz')
    ax2.set_title('Estimated Velocity Over Time')
    ax2.set_xlabel('Timestamp (s)')
    ax2.set_ylabel('Velocity (pixels/s)')
    ax2.legend()
    ax2.grid(True)

    plt.tight_layout()
    plt.savefig(f"{data_file.split('.')[0]}_results.png")
    plt.close()

    return states_df


# Run EKF on experiment files
dt = 0.1  # 10 Hz sampling rate
exp1_states = run_ekf('exp1_ekf.csv', dt)
exp2_states = run_ekf('exp2_ekf.csv', dt)

# Save estimated states to CSV
exp1_states.to_csv('exp1_ekf_states.csv', index=False)
exp2_states.to_csv('exp2_ekf_states.csv', index=False)

