import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R

# LOAD THE .csv FILE
file_path = '/home/priyanshu/NEU/Lab3_imu/src/analysis_scripts/imu_data.csv' # PATH TO FILE HERE. NEED TO CHANGE ACCORDINGLY 
df = pd.read_csv(file_path)

# TAKING THE COLUMNS
time = df['Time (Seconds)'].to_numpy()
accel_x = df['Linear Acceleration X'].to_numpy()
accel_y = df['Linear Acceleration Y'].to_numpy()
accel_z = df['Linear Acceleration Z'].to_numpy()
gyro_x = df['Angular Velocity X'].to_numpy()
gyro_y = df['Angular Velocity Y'].to_numpy()
gyro_z = df['Angular Velocity Z'].to_numpy()
mag_field_x = df['Magnetic Field X'].to_numpy()
mag_field_y = df['Magnetic Field Y'].to_numpy()
mag_field_z = df['Magnetic Field Z'].to_numpy()
orientation_quat = df[['Orientation X', 'Orientation Y', 'Orientation Z', 'Orientation W']].to_numpy()

# TO PLOT FROM ZERO INSTEAD OF 4700 (ACTUAL TIME)
time = time - time[0]   

# CONVERSION TO EULER ANGLES FROM QUANTERNIONS
def quat_to_euler(quat):
    x, y, z, w = quat
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if np.abs(sinp) >= 1:
        pitch = np.sign(sinp) * np.pi / 2 
    else:
        pitch = np.arctan2(sinp, np.sqrt(1 - sinp**2))

    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return np.degrees(roll), np.degrees(pitch), np.degrees(yaw)

# CALLING FUNC TO CONVERT TO EULER ANGLE
euler_angles = np.array([quat_to_euler(quat) for quat in orientation_quat])
roll = euler_angles[:, 0]
pitch = euler_angles[:, 1]
yaw = euler_angles[:, 2]

# COMMON FUNCTION TO PLOT ALL THE TIME SERIES GRAPHS
def plot_time_series(time, data, title, xlabel, ylabel):
    plt.figure(figsize=(10, 6))
    plt.plot(time, data, label=title)
    plt.title(title)
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.legend()
    plt.grid()
    plt.show()  

# COMMON FUNCTION TO PLOT ALL THE FREQ-DISTRIBUTION GRAPHS
def plot_frequency_distribution(data, title, xlabel):
    plt.figure(figsize=(10, 6))
    plt.hist(data, bins=30, alpha=0.7)
    plt.title(title)
    plt.xlabel(xlabel)
    plt.ylabel('Frequency')
    plt.grid()
    plt.show()  

# NOISE CHAR - MEAN AND STD_DEVIATION
def calculate_noise_characteristics(data, sensor_name):
    mean_value = np.mean(data)
    std_deviation = np.std(data)
    print(f"{sensor_name} - Mean: {mean_value:.6f}, Standard Deviation: {std_deviation:.6f}")

# PLOTTING Euler Angles Data
for data, label in zip([roll, pitch, yaw], ['Roll', 'Pitch', 'Yaw']):
    plot_time_series(time, data, f'Time vs {label}', 'Time (seconds)', f'{label} (degrees)')
    plot_frequency_distribution(data, f'Frequency Distribution of {label}', f'{label} (degrees)')
    calculate_noise_characteristics(data, label)

# PLOTTING Accelerometer Data
for data, label in zip([accel_x, accel_y, accel_z], ['Accel X', 'Accel Y', 'Accel Z']):
    plot_time_series(time, data, f'Time vs {label}', 'Time (seconds)', f'{label} (m/s^2)')
    plot_frequency_distribution(data, f'Frequency Distribution of {label}', f'{label} (m/s^2)')
    calculate_noise_characteristics(data, label)

# PLOTTING Gyroscope Data
for data, label in zip([gyro_x, gyro_y, gyro_z], ['Gyro X', 'Gyro Y', 'Gyro Z']):
    plot_time_series(time, data, f'Time vs {label}', 'Time (seconds)', f'{label} (rad/s)')
    plot_frequency_distribution(data, f'Frequency Distribution of {label}', f'{label} (rad/s)')
    calculate_noise_characteristics(data, label)

# PLOTTING Magnetic Field Data
for data, label in zip([mag_field_x, mag_field_y, mag_field_z], ['Magnetic Field X', 'Magnetic Field Y', 'Magnetic Field Z']):
    plot_time_series(time, data, f'Time vs {label}', 'Time (seconds)', f'{label} (T)')
    plot_frequency_distribution(data, f'Frequency Distribution of {label}', f'{label} (T)')
    calculate_noise_characteristics(data, label)