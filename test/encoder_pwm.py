import serial
import matplotlib.pyplot as plt

ser = serial.Serial('COM6', 115200)  # Thay COM3 bằng cổng của bạn

timestamps = []
rpm_left = []
rpm_right = []

while True:
    line = ser.readline().decode().strip()
    try:
        t, r_l, r_r = map(float, line.split(','))
        timestamps.append(t / 1000)  # ms → s
        rpm_left.append(r_l)
        rpm_right.append(r_r)

        if len(timestamps) > 200:
            break
    except:
        continue

plt.plot(timestamps, rpm_left, label='Left RPM')
plt.plot(timestamps, rpm_right, label='Right RPM')
plt.xlabel('Time (s)')
plt.ylabel('RPM')
plt.title('Motor Speed vs Time')
plt.legend()
plt.grid(True)
plt.show()
