import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import os

# === Matplotlib Global Styling ===
plt.rcParams.update({
    'font.size': 14,            # Base font size for most text
    'axes.titlesize': 18,       # Axes title font size
    'axes.labelsize': 16,       # X and Y axis label font size
    'xtick.labelsize': 14,      # X tick label font size
    'ytick.labelsize': 14,      # Y tick label font size
    'legend.fontsize': 14,      # Legend font size
    'figure.titlesize': 18      # Figure-wide title font size
})

# === Constants and File Lists ===
kinematic_files = [f'kin_{i}.csv' for i in range(1, 6)]
pid_files = [f'PID_{i}.csv' for i in range(1, 6)]
INTERP_POINTS = 200
TARGET_DIST = 115  # mm
DIST_TOLERANCE = 5  # mm

# === Load and Interpolate CSV Data ===
def load_and_interpolate(files, metric):
    interpolated = []

    for file in files:
        if os.path.exists(file):
            df = pd.read_csv(file)
            time = df['Time (s)'].values
            y = df[metric].values

            # Interpolate to a common time vector
            interp_time = np.linspace(0, time[-1], INTERP_POINTS)
            interp_y = np.interp(interp_time, time, y)

            interpolated.append(interp_y)
        else:
            print(f"File not found: {file}")

    interpolated = np.array(interpolated)
    return interp_time, interpolated

# === Plot Averages with Shaded Variance ===
def plot_average_with_shading(metric, ylabel, title, save_as=None):
    plt.figure(figsize=(10, 6))

    t_kin, kin_data = load_and_interpolate(kinematic_files, metric)
    t_pid, pid_data = load_and_interpolate(pid_files, metric)

    kin_mean = np.mean(kin_data, axis=0)
    kin_std = np.std(kin_data, axis=0)
    pid_mean = np.mean(pid_data, axis=0)
    pid_std = np.std(pid_data, axis=0)

    # Plot lines and shaded areas
    plt.plot(t_kin, kin_mean, label='Kinematic (avg)', linestyle='--', color='blue')
    plt.fill_between(t_kin, kin_mean - kin_std, kin_mean + kin_std, alpha=0.2, color='blue')

    plt.plot(t_pid, pid_mean, label='PID (avg)', linestyle='-', color='green')
    plt.fill_between(t_pid, pid_mean - pid_std, pid_mean + pid_std, alpha=0.2, color='green')

    # Styling
    plt.xlabel('Time (s)')
    plt.ylabel(ylabel)
    plt.title(title, fontweight='bold')
    plt.grid(True)
    plt.legend()
    plt.tight_layout()

    if save_as:
        plt.savefig(save_as, dpi=300)
        print(f"Saved plot: {save_as}")

    plt.show()

# === Compute Reach Times (Target Hit) ===
def compute_reach_times(files, metric='Distance (mm)', threshold=TARGET_DIST, tolerance=DIST_TOLERANCE):
    reach_times = []

    for file in files:
        if os.path.exists(file):
            df = pd.read_csv(file)
            time = df['Time (s)'].values
            distance = df[metric].values

            for t, d in zip(time, distance):
                if d <= threshold + tolerance:
                    reach_times.append(t)
                    break
            else:
                reach_times.append(np.nan)  # Target not reached
        else:
            print(f"File not found: {file}")
            reach_times.append(np.nan)

    return reach_times

# === Plot Reach Time Comparison Bar Chart ===
def plot_reach_time_comparison(kin_times, pid_times):
    labels = [f'Trial {i+1}' for i in range(len(kin_times))]
    x = np.arange(len(labels))
    width = 0.35

    plt.figure(figsize=(10, 6))
    plt.bar(x - width/2, kin_times, width, label='Kinematic', color='blue')
    plt.bar(x + width/2, pid_times, width, label='PID', color='green')

    # Styling
    plt.ylabel('Time to Reach Target (s)')
    plt.title('Time to Reach Target (Distance ≤ 105 mm)', fontweight='bold')
    plt.xticks(x, labels)
    plt.grid(True, axis='y')
    plt.legend()
    plt.tight_layout()

    plt.savefig('reach_time_comparison.png', dpi=300)
    print("Saved plot: reach_time_comparison.png")
    plt.show()

# === RUN ALL PLOTS ===

# 1. Average with Shading Plots
plot_average_with_shading('Distance (mm)', 'Distance to Target (mm)', 'Average Distance vs Time (PID vs Kinematic)', 'avg_distance_vs_time.png')
plot_average_with_shading('Angle (deg)', 'Angle (deg)', 'Average Angle vs Time (PID vs Kinematic)', 'avg_angle_vs_time.png')
plot_average_with_shading('Left Speed (mm/s)', 'Left Wheel Speed', 'Left Speed vs Time (PID vs Kinematic)', 'avg_left_speed.png')
plot_average_with_shading('Right Speed (mm/s)', 'Right Wheel Speed', 'Right Speed vs Time (PID vs Kinematic)', 'avg_right_speed.png')

# 2. Reach Time Comparison Plot
kin_reach_times = compute_reach_times(kinematic_files)
pid_reach_times = compute_reach_times(pid_files)
plot_reach_time_comparison(kin_reach_times, pid_reach_times)
