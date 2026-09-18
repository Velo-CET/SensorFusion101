import numpy as np
from helper import *

class MadgwickAHRS:
    def __init__(self, sample_rate=100.0, beta=0.3):
        self.sample_rate = sample_rate
        self.dt = 1.0 / sample_rate
        self.beta = beta  # Filter gain representing algorithm guess rate
        self.q = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)  # Initial unit quaternion

    def update_imu(self, gx, gy, gz, ax, ay, az):
        """
        Updates orientation using 6-DOF sensor fusion (Gyroscope + Accelerometer)
        Gyro units: radians/sec
        Accel units: m/s^2 or g
        """
        # Step 1: Compute rate of change of quaternion from gyroscope data
        q_gyro_dot = 0.5 * quat_mul(self.q, [0.0, gx, gy, gz])
        
        # Step 2: Calculate gradient descent direction from accelerometer data
        step = compute_gradient_descent_step(self.q, ax, ay, az)
        
        # Step 3: Apply filter gain and fuse rate measurements
        q_dot = q_gyro_dot - self.beta * step
        
        # Step 4: Integrate to yield quaternion
        self.q += q_dot * self.dt
        
        # Step 5: Normalize quaternion
        self.q /= np.linalg.norm(self.q)

        
        return self.q
