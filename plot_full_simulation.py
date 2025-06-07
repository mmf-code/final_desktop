import pandas as pd
import matplotlib.pyplot as plt
import os
import glob
import numpy as np

def find_phase_changes(df):
    """Identifies the start times of each new phase."""
    phases = []
    # Compare the target of the first drone to detect changes
    target_x_col = 'TargetX0'
    # Use .shift() to compare a row to the one before it
    phase_change_indices = df.index[df[target_x_col] != df[target_x_col].shift()]
    
    for idx in phase_change_indices:
        time = df.loc[idx, 'Time']
        target_x = df.loc[idx, target_x_col]
        target_y = df.loc[idx, 'TargetY0']
        phases.append({'time': time, 'target_x': target_x, 'target_y': target_y})
        
    return phases

def plot_position_tracking_with_phases(df, num_drones, phases):
    """
    Plots the target position vs the actual position for both X and Y axes for all drones,
    with vertical lines indicating phase changes.
    """
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(15, 12), sharex=True)
    fig.suptitle('Drone Position Tracking (All Drones)', fontsize=16)
    
    colors = plt.cm.jet(np.linspace(0, 1, num_drones))

    for i in range(num_drones):
        # X-Axis Plot
        ax1.plot(df['Time'], df[f'CurrentX{i}'], color=colors[i], label=f'Drone {i} Current X')
        # Y-Axis Plot
        ax2.plot(df['Time'], df[f'CurrentY{i}'], color=colors[i], label=f'Drone {i} Current Y')

    # Plot target for the first drone (assuming formation moves together)
    ax1.plot(df['Time'], df['TargetX0'], 'k--', label='Target X (Formation)')
    ax2.plot(df['Time'], df['TargetY0'], 'k--', label='Target Y (Formation)')

    # Add phase change indicators
    for i, phase in enumerate(phases):
        ax1.axvline(x=phase['time'], color='gray', linestyle=':', linewidth=1.5)
        ax2.axvline(x=phase['time'], color='gray', linestyle=':', linewidth=1.5)
        ax1.text(phase['time'] + 0.5, ax1.get_ylim()[1] * 0.9, f'Phase {i+1}', rotation=90, verticalalignment='top')
    
    ax1.set_ylabel('X Position (m)')
    ax1.grid(True, which='both', linestyle='--', linewidth=0.5)
    ax1.legend(loc='upper right')

    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Y Position (m)')
    ax2.grid(True, which='both', linestyle='--', linewidth=0.5)
    ax2.legend(loc='upper right')
    
    print("Plotting Position Tracking...")

def plot_disturbance_vs_correction(df):
    """
    Plots the external wind disturbance and the FLS correction signal to show
    how the FLS is responding.
    """
    if 'SimWindX' not in df.columns or 'FLSCorrX0' not in df.columns:
        print("Skipping Disturbance vs. Correction plot (Wind or FLS data not found).")
        return
        
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(15, 10), sharex=True)
    fig.suptitle('Disturbance vs. FLS Correction (Drone 0)', fontsize=16)

    # X-Axis Disturbance and Correction
    ax1.plot(df['Time'], df['SimWindX'], 'r-', label='Wind Force X')
    ax1.plot(df['Time'], df['FLSCorrX0'], 'b--', label='FLS Correction X')
    ax1.set_ylabel('Force / Correction')
    ax1.grid(True, which='both', linestyle='--', linewidth=0.5)
    ax1.legend()
    ax1.set_title('X-Axis')

    # Y-Axis Disturbance and Correction
    ax2.plot(df['Time'], df['SimWindY'], 'r-', label='Wind Force Y')
    ax2.plot(df['Time'], df['FLSCorrY0'], 'b--', label='FLS Correction Y')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Force / Correction')
    ax2.grid(True, which='both', linestyle='--', linewidth=0.5)
    ax2.legend()
    ax2.set_title('Y-Axis')
    
    print("Plotting Disturbance vs. FLS Correction...")

def plot_2d_trajectory(df, num_drones, phases):
    """
    Plots the 2D (X vs Y) trajectory for all drones, marking phase targets.
    """
    plt.figure(figsize=(12, 12))
    colors = plt.cm.jet(np.linspace(0, 1, num_drones))
    
    for i in range(num_drones):
        x_col, y_col = f'CurrentX{i}', f'CurrentY{i}'
        plt.plot(df[x_col], df[y_col], color=colors[i], alpha=0.8, label=f'Drone {i} Trajectory')
        plt.plot(df[x_col].iloc[0], df[y_col].iloc[0], 'o', color=colors[i], markersize=10, markeredgecolor='k', label=f'Drone {i} Start')

    # Plot phase target centers
    for i, phase in enumerate(phases):
        # Note: The target in the CSV is for each drone. We infer the center from drone 0's target.
        # This is a simplification; a more robust way would be to save center points in the CSV.
        formation_offset_x = df['TargetX0'].iloc[0] - df['TargetX0'].iloc[0] # Assuming first target is center + offset
        formation_offset_y = df['TargetY0'].iloc[0] - df['TargetY0'].iloc[0] 
        center_x = phase['target_x'] - (df['TargetX0'][phase['time'] == df['Time']].iloc[0] - df['CurrentX0'][phase['time'] == df['Time']].iloc[0] if not df['CurrentX0'][phase['time'] == df['Time']].empty else 0)
        center_y = phase['target_y'] - (df['TargetY0'][phase['time'] == df['Time']].iloc[0] - df['CurrentY0'][phase['time'] == df['Time']].iloc[0] if not df['CurrentY0'][phase['time'] == df['Time']].empty else 0)


        plt.plot(phase['target_x'], phase['target_y'], 'X', color='black', markersize=15, label=f'Phase {i+1} Target')
        plt.text(phase['target_x'], phase['target_y'] + 0.5, f'Phase {i+1}', fontsize=12)

    plt.title('2D Drone Trajectories & Phase Targets')
    plt.xlabel('X Position (m)')
    plt.ylabel('Y Position (m)')
    plt.grid(True, which='both', linestyle='--', linewidth=0.5)
    plt.legend()
    plt.axis('equal')
    print("Plotting 2D Trajectory...")


if __name__ == "__main__":
    # --- Configuration ---
    # Search in the main output folder for normal runs
    search_path = os.path.join('build', 'Debug', 'simulation_outputs', '*.csv')

    # Find all CSV files that are NOT from ZN tuning
    list_of_files = [f for f in glob.glob(search_path) if 'zn_test' not in os.path.basename(f)]
    
    if not list_of_files:
        print(f"Error: No simulation CSV files found in '{os.path.dirname(search_path)}'")
        print("Please check if the simulation ran and created an output file.")
        exit()

    latest_file = max(list_of_files, key=os.path.getctime)
    print(f"Loading latest simulation data from: {os.path.basename(latest_file)}\n")

    try:
        data = pd.read_csv(latest_file)
        
        num_drones = 0
        while f'CurrentX{num_drones}' in data.columns:
            num_drones += 1

        if num_drones == 0:
            raise ValueError("No drone data columns (e.g., 'CurrentX0') found in the CSV.")
            
        print(f"Detected data for {num_drones} drones.")
        
        phases = find_phase_changes(data)
        print(f"Detected {len(phases)} phase changes.\n")
        
        # --- Call the plotting functions ---
        plot_position_tracking_with_phases(data, num_drones, phases)
        plot_disturbance_vs_correction(data)
        plot_2d_trajectory(data, num_drones, phases)
        
        plt.tight_layout(rect=[0, 0, 1, 0.96])
        plt.show()

    except Exception as e:
        print(f"An error occurred: {e}")