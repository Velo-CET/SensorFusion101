import numpy as np

def quat_mul(q1, q2):
    """
    Helper function for quaternion multiplication: q1 * q2
    Quaternions formatted as [w, x, y, z]
    """
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    
    w = w1*w2 - x1*x2 - y1*y2 - z1*z2
    x = w1*x2 + x1*w2 + y1*z2 - z1*y2
    y = w1*y2 - x1*z2 + y1*w2 + z1*x2
    z = w1*z2 + x1*y2 - y1*x2 + z1*w2
    
    return np.array([w, x, y, z], dtype=np.float64)


def compute_gradient_descent_step(q, ax, ay, az):
    """
    Helper function isolating the gradient descent algorithm.
    Calculates the direction of the error gradient based on accelerometer readings.
    """
    w, x, y, z = q
    
    # Normalize accelerometer measurement
    norm_a = np.sqrt(ax**2 + ay**2 + az**2)
    if norm_a == 0:
        return np.zeros(4)
    ax, ay, az = ax / norm_a, ay / norm_a, az / norm_a
    
    # Objective function f(q, a) = Estimated Gravity - Measured Gravity
    f1 = 2.0 * (x * z - w * y) - ax
    f2 = 2.0 * (w * x + y * z) - ay
    f3 = 2.0 * (0.5 - x**2 - y**2) - az
    
    # Jacobian matrix J(q) transposed multiplied by f(q, a)
    # J = [ -2y   2z  -2w   2x ]
    #     [  2x   2w   2z   2y ]
    #     [   0  -4x  -4y    0 ]
    g_w = -2.0 * y * f1 + 2.0 * x * f2
    g_x =  2.0 * z * f1 + 2.0 * w * f2 - 4.0 * x * f3
    g_y = -2.0 * w * f1 + 2.0 * z * f2 - 4.0 * y * f3
    g_z =  2.0 * x * f1 + 2.0 * y * f2
    
    step = np.array([g_w, g_x, g_y, g_z], dtype=np.float64)
    
    # Normalize the gradient descent step
    norm_step = np.linalg.norm(step)
    if norm_step > 0:
        step /= norm_step
        
    return step
