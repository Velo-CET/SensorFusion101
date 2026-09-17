import time
import serial
import numpy as np
import matplotlib.pyplot as plt
from madgwick import MadgwickAHRS

# Configuration
SERIAL_PORT = input("Enter PORT name >> ")
BAUD_RATE = 115200
SAMPLE_RATE = 50.0   # Hz (match this with loop frequency)
BETA = 0.1            # Madgwick gain parameter

def quat_to_rotation_matrix(q):
    """Converts unit quaternion [w, x, y, z] into a 3x3 rotation matrix."""
    w, x, y, z = q
    return np.array([
        [1 - 2*(y**2 + z**2), 2*(x*y - w*z),     2*(x*z + w*y)],
        [2*(x*y + w*z),     1 - 2*(x**2 + z**2), 2*(y*z - w*x)],
        [2*(x*z - w*y),     2*(y*z + w*x),     1 - 2*(x**2 + y**2)]
    ])

def parse_serial_data(line):
    """
    Parses a comma-separated string from MPU6050: "ax,ay,az,gx,gy,gz"
    - Gyro values in degrees/second are converted to radians/second.
    """
    data = line.decode('utf-8', errors='ignore').strip().split(',')
    if len(data) == 6:
        raw = [float(val) for val in data]
        ax, ay, az = raw[0], raw[1], raw[2]
        # Convert deg/s to rad/s for Gyroscope inputs
        gx, gy, gz = np.radians(raw[3]), np.radians(raw[4]), np.radians(raw[5])
        return ax, ay, az, gx, gy, gz
    return None

def calibrate_gyro(ser, samples=200):
    """Call this at startup while the IMU is completely stationary."""
    gx_off, gy_off, gz_off = 0.0, 0.0, 0.0
    for _ in range(samples):
        line = ser.readline()
        parsed = parse_serial_data(line)
        if parsed:
            gx_off += parsed[3]
            gy_off += parsed[4]
            gz_off += parsed[5]
    return gx_off / samples, gy_off / samples, gz_off / samples

def main():
    # Initialize Madgwick Filter
    filter_madgwick = MadgwickAHRS(sample_rate=SAMPLE_RATE, beta=BETA)

    
    # Initialize Serial Port
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
        time.sleep(2.0)  # Wait for Arduino/ESP to reset
        print(f"Connected to {SERIAL_PORT} at {BAUD_RATE} baud.")
    except Exception as e:
        print(f"Failed to open serial port {SERIAL_PORT}: {e}")
        return
    
    a, b, c = calibrate_gyro(ser=ser, samples=10)

    # Setup Matplotlib 3D Canvas
    plt.ion()
    fig = plt.figure(figsize=(8, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    ax.set_xlim([-1.5, 1.5])
    ax.set_ylim([-1.5, 1.5])
    ax.set_zlim([-1.5, 1.5])
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Real-Time MPU6050 Orientation (Madgwick)')

    # Unit axis lines in local IMU frame
    origin = np.array([0, 0, 0])
    unit_x = np.array([1, 0, 0])
    unit_y = np.array([0, 1, 0])
    unit_z = np.array([0, 0, 1])

    # Plot initial lines
    line_x, = ax.plot([], [], [], color='red', lw=3, label='X Axis')
    line_y, = ax.plot([], [], [], color='green', lw=3, label='Y Axis')
    line_z, = ax.plot([], [], [], color='blue', lw=3, label='Z Axis')
    ax.legend()

    # Real-time Loop
    while plt.fignum_exists(fig.number):
        if ser.in_waiting:
            line = ser.readline()
            parsed = parse_serial_data(line)
            
            if parsed is not None:
                ax_val, ay_val, az_val, gx_val, gy_val, gz_val = parsed

                gx_val -= a
                gy_val -= b
                gz_val -= c
                
                # Update Madgwick orientation estimation
                q = filter_madgwick.update_imu(gx_val, gy_val, gz_val, ax_val, ay_val, az_val)
                
                # Transform local axes into global orientation space
                R = quat_to_rotation_matrix(q)
                x_axis = R @ unit_x
                y_axis = R @ unit_y
                z_axis = R @ unit_z

                # Update plot 3D line coordinates
                line_x.set_data([origin[0], x_axis[0]], [origin[1], x_axis[1]])
                line_x.set_3d_properties([origin[2], x_axis[2]])

                line_y.set_data([origin[0], y_axis[0]], [origin[1], y_axis[1]])
                line_y.set_3d_properties([origin[2], y_axis[2]])

                line_z.set_data([origin[0], z_axis[0]], [origin[1], z_axis[1]])
                line_z.set_3d_properties([origin[2], z_axis[2]])

                # Redraw canvas
                fig.canvas.draw_idle()
                fig.canvas.flush_events()

    ser.close()

if __name__ == "__main__":
    main()