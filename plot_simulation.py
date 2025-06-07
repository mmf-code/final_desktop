import pandas as pd
import matplotlib.pyplot as plt
import os
import glob

def plot_zn_analysis(df, drone_id=0):
    """
    Plots the error of a specific drone's axis over time, which is key for
    Ziegler-Nichols analysis.
    """
    error_col = f'ErrorX{drone_id}'
    time_col = 'Time'

    if error_col not in df.columns:
        print(f"Error: Column '{error_col}' not found in the CSV file.")
        return

    plt.figure(figsize=(12, 6))
    plt.plot(df[time_col], df[error_col], label=f'Drone {drone_id} X-Axis Error')
    plt.axhline(0, color='r', linestyle='--', linewidth=0.8, label='Zero Error')
    
    plt.title(f'Ziegler-Nichols Analysis: Drone {drone_id} X-Axis Error vs. Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Error (meters)')
    plt.grid(True, which='both', linestyle='--', linewidth=0.5)
    plt.legend()
    print(f"Plotting Z-N Analysis for Drone {drone_id}...")
    
def plot_position_tracking(df, drone_id=0):
    """
    Plots the target position vs the actual position for both X and Y axes.
    """
    target_x_col = f'TargetX{drone_id}'
    current_x_col = f'CurrentX{drone_id}'
    target_y_col = f'TargetY{drone_id}'
    current_y_col = f'CurrentY{drone_id}'
    time_col = 'Time'

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10), sharex=True)
    fig.suptitle(f'Position Tracking for Drone {drone_id}', fontsize=16)

    # X-Axis Plot
    ax1.plot(df[time_col], df[target_x_col], 'r--', label='Target X')
    ax1.plot(df[time_col], df[current_x_col], 'b-', label='Current X')
    ax1.set_ylabel('X Position (m)')
    ax1.grid(True, which='both', linestyle='--', linewidth=0.5)
    ax1.legend()

    # Y-Axis Plot
    ax2.plot(df[time_col], df[target_y_col], 'g--', label='Target Y')
    ax2.plot(df[time_col], df[current_y_col], 'k-', label='Current Y')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Y Position (m)')
    ax2.grid(True, which='both', linestyle='--', linewidth=0.5)
    ax2.legend()
    print(f"Plotting Position Tracking for Drone {drone_id}...")

def plot_2d_trajectory(df):
    """
    Plots the 2D (X vs Y) trajectory for all drones found in the CSV.
    """
    plt.figure(figsize=(10, 10))
    
    # Find how many drones are in the file
    num_drones = 0
    while f'CurrentX{num_drones}' in df.columns:
        num_drones += 1

    if num_drones == 0:
        print("No drone data found to plot 2D trajectory.")
        return

    for i in range(num_drones):
        x_col = f'CurrentX{i}'
        y_col = f'CurrentY{i}'
        
        plt.plot(df[x_col], df[y_col], label=f'Drone {i} Trajectory')
        # Mark start and end points
        plt.plot(df[x_col].iloc[0], df[y_col].iloc[0], 'go', markersize=8, label=f'Drone {i} Start')
        plt.plot(df[x_col].iloc[-1], df[y_col].iloc[-1], 'ro', markersize=8, label=f'Drone {i} End')

    plt.title('2D Drone Trajectories')
    plt.xlabel('X Position (m)')
    plt.ylabel('Y Position (m)')
    plt.grid(True, which='both', linestyle='--', linewidth=0.5)
    plt.legend()
    plt.axis('equal') # Ensure X and Y scales are the same
    print("Plotting 2D Trajectory...")


if __name__ == "__main__":
    # --- Configuration ---
    # The script will look inside this folder.
    # It assumes you run it from the root of your project folder.
    search_path = os.path.join('build', 'Debug', 'simulation_outputs', 'zn_tuning', '*.csv')

    # Find all CSV files in the directory
    list_of_files = glob.glob(search_path)
    
    if not list_of_files:
        print(f"Error: No CSV files found in '{os.path.dirname(search_path)}'")
        print("Please check if the simulation ran and created an output file.")
        exit()

    # Get the most recent file
    latest_file = max(list_of_files, key=os.path.getctime)
    print(f"Loading latest simulation data from: {latest_file}\n")

    try:
        # Load the data using pandas
        data = pd.read_csv(latest_file)
        
        # --- Call the plotting functions ---
        
        # This is the most important plot for ZN tuning
        plot_zn_analysis(data, drone_id=0)
        
        # This plot is useful for visualizing the step response
        plot_position_tracking(data, drone_id=0)
        
        # This plot shows the overall movement of the formation
        plot_2d_trajectory(data)
        
        # Display all the plots
        plt.tight_layout(rect=[0, 0.03, 1, 0.95]) # Adjust layout to prevent title overlap
        plt.show()

    except FileNotFoundError:
        print(f"Error: The file '{latest_file}' was not found.")
    except Exception as e:
        print(f"An error occurred: {e}")