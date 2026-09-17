import matplotlib.pyplot as plt
import serial
import time

PORT = input("Enter PORT >> ")
BAUD_RATE = 115200

print(f"Connecting to {PORT}...")

ax, ay, az = [], [], []
angx, angy, angz = [], [], []

data_counter = 0
MAXCOUNT = 50
prev_time = None

try:
    ser = serial.Serial(PORT, BAUD_RATE, timeout=1)
    time.sleep(2)

    ser.reset_input_buffer()
    ser.readline()

    print(f"Connected to {PORT} at {BAUD_RATE} baud.\n")

    while data_counter < MAXCOUNT:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8', errors='ignore').rstrip()
            if line:
                try:
                    val_ax, val_ay, val_az, gx, gy, gz = map(float, line.split(","))
                except ValueError:
                    continue 

                current_time = time.time()

                if prev_time is None:
                    angx.append(0.0)
                    angy.append(0.0)
                    angz.append(0.0)
                else:
                    dt = current_time - prev_time
                    angx.append(angx[-1] + gx * dt)
                    angy.append(angy[-1] + gy * dt)
                    angz.append(angz[-1] + gz * dt)

                prev_time = current_time

                ax.append(val_ax)
                ay.append(val_ay)
                az.append(val_az)

                data_counter += 1
                filled = int(20 * data_counter / MAXCOUNT)
                print(f"Reading data:  [{'-' * filled}{' ' * (20 - filled)}] {data_counter / MAXCOUNT * 100:.1f}%", end="\r")

except serial.SerialException as e:
    print(f"Serial port error: {e}")
except KeyboardInterrupt:
    print("\nProgram interrupted by user.")
finally:
    if 'ser' in locals() and ser.is_open:
        ser.close()
        print("\nSerial connection closed.")

fig, (accAx, angAx) = plt.subplots(2, 1, figsize=(10, 4))

accAx.plot(ax, label="Acc. X")
accAx.plot(ay, label="Acc. Y")
accAx.plot(az, label="Acc. Z")
accAx.set_title("Accelerometer data")
accAx.set_ylabel("Acceleration (g)")
accAx.legend(loc="upper right")

angAx.plot(angx, label="Ang. X")
angAx.plot(angy, label="Ang. Y")
angAx.plot(angz, label="Ang. Z")
angAx.set_title("Gyro data")
angAx.set_ylabel("Angle (deg)")
angAx.legend(loc="upper right")

plt.tight_layout()
plt.show()