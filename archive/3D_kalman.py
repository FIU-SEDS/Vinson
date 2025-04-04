import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D

# Load data from the CSV file
def load_data(filename):
    """Load data from the CSV file created by the rocket."""
    df = pd.read_csv(filename)
    return df

# Plot acceleration data
def plot_acceleration(df):
    """Plot raw vs filtered acceleration data."""
    plt.figure(figsize=(15, 10))
    
    # X acceleration
    plt.subplot(3, 1, 1)
    plt.plot(df['time'] / 1000, df['ax_raw'], 'b.', alpha=0.3, label='Raw')
    plt.plot(df['time'] / 1000, df['ax_filtered'], 'r-', linewidth=2, label='Filtered')
    plt.title('X-Axis Acceleration')
    plt.ylabel('Acceleration (m/s²)')
    plt.legend()
    plt.grid(True, alpha=0.3)
    
    # Y acceleration
    plt.subplot(3, 1, 2)
    plt.plot(df['time'] / 1000, df['ay_raw'], 'b.', alpha=0.3, label='Raw')
    plt.plot(df['time'] / 1000, df['ay_filtered'], 'r-', linewidth=2, label='Filtered')
    plt.title('Y-Axis Acceleration')
    plt.ylabel('Acceleration (m/s²)')
    plt.legend()
    plt.grid(True, alpha=0.3)
    
    # Z acceleration
    plt.subplot(3, 1, 3)
    plt.plot(df['time'] / 1000, df['az_raw'], 'b.', alpha=0.3, label='Raw')
    plt.plot(df['time'] / 1000, df['az_filtered'], 'r-', linewidth=2, label='Filtered')
    plt.title('Z-Axis Acceleration')
    plt.xlabel('Time (seconds)')
    plt.ylabel('Acceleration (m/s²)')
    plt.legend()
    plt.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('acceleration_plot.png', dpi=150)
    print("Acceleration plot saved as 'acceleration_plot.png'")

# Plot gyroscope data
def plot_gyroscope(df):
    """Plot raw vs filtered gyroscope data."""
    plt.figure(figsize=(15, 10))
    
    # X gyro
    plt.subplot(3, 1, 1)
    plt.plot(df['time'] / 1000, df['gx_raw'], 'b.', alpha=0.3, label='Raw')
    plt.plot(df['time'] / 1000, df['gx_filtered'], 'r-', linewidth=2, label='Filtered')
    plt.title('X-Axis Angular Velocity')
    plt.ylabel('Angular Velocity (rad/s)')
    plt.legend()
    plt.grid(True, alpha=0.3)
    
    # Y gyro
    plt.subplot(3, 1, 2)
    plt.plot(df['time'] / 1000, df['gy_raw'], 'b.', alpha=0.3, label='Raw')
    plt.plot(df['time'] / 1000, df['gy_filtered'], 'r-', linewidth=2, label='Filtered')
    plt.title('Y-Axis Angular Velocity')
    plt.ylabel('Angular Velocity (rad/s)')
    plt.legend()
    plt.grid(True, alpha=0.3)
    
    # Z gyro
    plt.subplot(3, 1, 3)
    plt.plot(df['time'] / 1000, df['gz_raw'], 'b.', alpha=0.3, label='Raw')
    plt.plot(df['time'] / 1000, df['gz_filtered'], 'r-', linewidth=2, label='Filtered')
    plt.title('Z-Axis Angular Velocity')
    plt.xlabel('Time (seconds)')
    plt.ylabel('Angular Velocity (rad/s)')
    plt.legend()
    plt.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('gyroscope_plot.png', dpi=150)
    print("Gyroscope plot saved as 'gyroscope_plot.png'")

# Plot 3D trajectory (estimated)
def plot_trajectory(df):
    """Estimate and plot 3D trajectory by integrating acceleration twice."""
    # Convert time to seconds and get time step
    df['time_sec'] = df['time'] / 1000
    dt = np.diff(df['time_sec']).mean()
    
    # Initialize velocity and position arrays
    vx = np.zeros(len(df))
    vy = np.zeros(len(df))
    vz = np.zeros(len(df))
    x = np.zeros(len(df))
    y = np.zeros(len(df))
    z = np.zeros(len(df))
    
    # Simple integration (Euler method)
    for i in range(1, len(df)):
        # Update velocity by integrating acceleration
        vx[i] = vx[i-1] + df['ax_filtered'].iloc[i] * dt
        vy[i] = vy[i-1] + df['ay_filtered'].iloc[i] * dt
        vz[i] = vz[i-1] + (df['az_filtered'].iloc[i] - 9.81) * dt  # Subtract gravity
        
        # Update position by integrating velocity
        x[i] = x[i-1] + vx[i] * dt
        y[i] = y[i-1] + vy[i] * dt
        z[i] = z[i-1] + vz[i] * dt
    
    # Create 3D plot
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    # Plot the trajectory
    ax.plot(x, y, z, 'r-', linewidth=2)
    
    # Mark start and end points
    ax.scatter(x[0], y[0], z[0], color='green', s=100, label='Launch')
    ax.scatter(x[-1], y[-1], z[-1], color='blue', s=100, label='End')
    
    # Find apogee (highest point)
    apogee_idx = np.argmax(z)
    ax.scatter(x[apogee_idx], y[apogee_idx], z[apogee_idx], color='red', s=100, label='Apogee')
    
    # Set labels and title
    ax.set_xlabel('X Position (m)')
    ax.set_ylabel('Y Position (m)')
    ax.set_zlabel('Z Position (m)')
    ax.set_title('Estimated Rocket Trajectory')
    ax.legend()
    
    # Make axes equal for better visualization
    max_range = np.array([x.max()-x.min(), y.max()-y.min(), z.max()-z.min()]).max() / 2.0
    mid_x = (x.max()+x.min()) / 2
    mid_y = (y.max()+y.min()) / 2
    mid_z = (z.max()+z.min()) / 2
    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)
    
    plt.tight_layout()
    plt.savefig('trajectory_plot.png', dpi=150)
    print("Trajectory plot saved as 'trajectory_plot.png'")
    
    # Return estimated apogee altitude
    return z[apogee_idx]

# Plot noise reduction effectiveness
def plot_noise_reduction(df):
    """Plot histogram of noise reduction effectiveness."""
    # Calculate the difference between raw and filtered data
    df['ax_diff'] = abs(df['ax_raw'] - df['ax_filtered'])
    df['ay_diff'] = abs(df['ay_raw'] - df['ay_filtered'])
    df['az_diff'] = abs(df['az_raw'] - df['az_filtered'])
    df['gx_diff'] = abs(df['gx_raw'] - df['gx_filtered'])
    df['gy_diff'] = abs(df['gy_raw'] - df['gy_filtered'])
    df['gz_diff'] = abs(df['gz_raw'] - df['gz_filtered'])
    
    # Calculate noise reduction percentage
    accel_reduction = (df[['ax_diff', 'ay_diff', 'az_diff']].mean().mean() / 
                      df[['ax_raw', 'ay_raw', 'az_raw']].std().mean()) * 100
    gyro_reduction = (df[['gx_diff', 'gy_diff', 'gz_diff']].mean().mean() / 
                     df[['gx_raw', 'gy_raw', 'gz_raw']].std().mean()) * 100
    
    plt.figure(figsize=(10, 6))
    plt.bar(['Accelerometer', 'Gyroscope'], [accel_reduction, gyro_reduction])
    plt.ylabel('Noise Reduction (%)')
    plt.title('Kalman Filter Noise Reduction Effectiveness')
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig('noise_reduction.png', dpi=150)
    print("Noise reduction plot saved as 'noise_reduction.png'")
    
    return accel_reduction, gyro_reduction

# Main function
def analyze_flight_data(filename):
    """Analyze and visualize flight data."""
    print(f"Analyzing flight data from: {filename}")
    
    # Load data
    df = load_data(filename)
    print(f"Loaded {len(df)} data points")
    
    # Plot data
    plot_acceleration(df)
    plot_gyroscope(df)
    
    # Estimate trajectory and apogee
    apogee = plot_trajectory(df)
    print(f"Estimated apogee: {apogee:.2f} meters")
    
    # Calculate noise reduction
    accel_reduction, gyro_reduction = plot_noise_reduction(df)
    print(f"Accelerometer noise reduction: {accel_reduction:.2f}%")
    print(f"Gyroscope noise reduction: {gyro_reduction:.2f}%")
    
    # Calculate flight statistics
    flight_duration = df['time'].max() / 1000  # seconds
    max_accel = df[['ax_filtered', 'ay_filtered', 'az_filtered']].abs().max().max()
    max_rotation = df[['gx_filtered', 'gy_filtered', 'gz_filtered']].abs().max().max()
    
    print(f"Flight duration: {flight_duration:.2f} seconds")
    print(f"Maximum acceleration: {max_accel:.2f} m/s²")
    print(f"Maximum rotation rate: {max_rotation:.2f} rad/s")
    
    plt.show()

# If this script is run directly
if __name__ == "__main__":
    import sys
    
    # Use command line argument or default filename
    if len(sys.argv) > 1:
        filename = sys.argv[1]
    else:
        filename = "FLIGHT_000.CSV"  # Default filename
    
    analyze_flight_data(filename)