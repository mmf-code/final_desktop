import pandas as pd
import matplotlib.pyplot as plt
import os
import sys
import numpy as np

# --- Configuration ---
# Use the new CSV file name from the C++ FLS code
csv_file_name = "multi_drone_pid_fls_sim_data.csv"
script_dir = os.path.dirname(os.path.abspath(__file__))
NUM_DRONES = 3 # Should match the simulation
colors = ['blue', 'green', 'red', 'purple', 'orange', 'brown'] # Colors for plotting

# --- CSV Path Finding ---
potential_csv_path = ""
# Try to find the CSV relative to this script's location,
# common when script is in 'scripts' and CSV in 'build/Debug'
# Adjust these paths if your project structure is different.

# Scenario 1: Script is in a 'scripts' or project root, CSV is in a typical build path
build_path_rel_to_script = os.path.join("agent_control_pkg", "build", "Debug", csv_file_name) # Common ROS2 structure
build_path_alt_dev = os.path.join("build", "Debug", csv_file_name) # Simpler non-ROS dev structure

# Path if script is in project root (e.g., 'final') and CSV is in build/Debug
path_from_final_root = os.path.join(script_dir, "agent_control_pkg", "build", "Debug", csv_file_name)
# Path if script is in build/Debug directly
path_in_build_debug = os.path.join(script_dir, csv_file_name)
# Path if script is in project root and build is directly under it
path_from_project_root_simple_build = os.path.join(script_dir, "build", "Debug", csv_file_name)


# Check common locations
if os.path.exists(path_in_build_debug):
    csv_path = path_in_build_debug
elif os.path.exists(path_from_final_root):
     csv_path = path_from_final_root
elif os.path.exists(path_from_project_root_simple_build):
    csv_path = path_from_project_root_simple_build
elif os.path.exists(os.path.join(script_dir, csv_file_name)): # Script and CSV in same dir
    csv_path = os.path.join(script_dir, csv_file_name)
else:
    # Fallback for more complex structures or if script is elsewhere
    # Assuming script might be in a 'scripts' folder, one level up from where 'agent_control_pkg' might be
    project_root_guess = os.path.dirname(script_dir)
    path_from_project_root_guess = os.path.join(project_root_guess, "agent_control_pkg", "build", "Debug", csv_file_name)
    if os.path.exists(path_from_project_root_guess):
        csv_path = path_from_project_root_guess
    else:
        print(f"Error: CSV file '{csv_file_name}' not found in common locations.")
        print(f"Checked: {path_in_build_debug}")
        print(f"Checked: {path_from_final_root}")
        print(f"Checked: {path_from_project_root_simple_build}")
        print(f"Checked: {os.path.join(script_dir, csv_file_name)}")
        print(f"Checked: {path_from_project_root_guess}")
        sys.exit()

print("Attempting to load CSV from:", csv_path)
# --- End CSV Path Finding ---

try:
    df = pd.read_csv(csv_path, on_bad_lines='skip')
    # Clean up any non-numeric data that might have slipped in, e.g., "Changing FORMATION..."
    data_cols = [col for col in df.columns if col != 'Time' and 'Changing' not in col]
    for col in data_cols:
        if col in df.columns: # Ensure column exists before trying to convert
            df[col] = pd.to_numeric(df[col], errors='coerce')
    df.dropna(subset=['Time'], inplace=True) # Ensure Time column is valid
except FileNotFoundError:
    print(f"Error: {csv_path} not found.")
    sys.exit()
except Exception as e:
    print(f"An error occurred while loading/processing CSV: {e}")
    sys.exit()

if df.empty:
    print("DataFrame is empty after loading and cleaning. Exiting.")
    sys.exit()

# Determine appropriate tick step for time axis
max_time = df['Time'].max() if 'Time' in df.columns and not df['Time'].empty else 10.0
min_time = df['Time'].min() if 'Time' in df.columns and not df['Time'].empty else 0.0

if pd.isna(max_time) or pd.isna(min_time) or max_time == min_time: # Handle cases where time data might be faulty
    integer_ticks = np.arange(0, 11, 1) # Default ticks
else:
    tick_range = max_time - min_time
    if tick_range <= 10:
        tick_step = 1
    elif tick_range <= 30:
        tick_step = 2
    elif tick_range <= 60:
        tick_step = 5
    else:
        tick_step = 10
    integer_ticks = np.arange(np.floor(min_time / tick_step) * tick_step, np.ceil(max_time) + 1, tick_step)


# --- Figure 1: Overall Formation Analysis ---
# This figure remains 2x2 as per original structure, wind will be added to individual drone plots
fig1, axs1 = plt.subplots(2, 2, figsize=(18, 10))
fig1.suptitle('Overall Formation PID (+FLS) Control Analysis', fontsize=16)

# Plot 1.1: X Positions
for i in range(NUM_DRONES):
    if f'TargetX{i}' in df.columns:
        axs1[0, 0].plot(df['Time'], df[f'TargetX{i}'], label=f'D{i} Target X', linestyle='--', color=colors[i % len(colors)])
    if f'CurrentX{i}' in df.columns:
        axs1[0, 0].plot(df['Time'], df[f'CurrentX{i}'], label=f'D{i} Current X', color=colors[i % len(colors)])
axs1[0, 0].set_xlabel('Time (s)')
axs1[0, 0].set_ylabel('X Position')
axs1[0, 0].set_title('X Position Control (All Drones)')
axs1[0, 0].legend(fontsize='small')
axs1[0, 0].grid(True)
axs1[0, 0].set_xticks(integer_ticks)

# Plot 1.2: Y Positions
for i in range(NUM_DRONES):
    if f'TargetY{i}' in df.columns:
        axs1[0, 1].plot(df['Time'], df[f'TargetY{i}'], label=f'D{i} Target Y', linestyle='--', color=colors[i % len(colors)])
    if f'CurrentY{i}' in df.columns:
        axs1[0, 1].plot(df['Time'], df[f'CurrentY{i}'], label=f'D{i} Current Y', color=colors[i % len(colors)])
axs1[0, 1].set_xlabel('Time (s)')
axs1[0, 1].set_ylabel('Y Position')
axs1[0, 1].set_title('Y Position Control (All Drones)')
axs1[0, 1].legend(fontsize='small')
axs1[0, 1].grid(True)
axs1[0, 1].set_xticks(integer_ticks)

# Plot 1.3: Formation Path (X vs Y)
for i in range(NUM_DRONES):
    if f'CurrentX{i}' in df.columns and f'CurrentY{i}' in df.columns:
        axs1[1, 0].plot(df[f'CurrentX{i}'], df[f'CurrentY{i}'], label=f'D{i} Path', color=colors[i % len(colors)])
    if f'TargetX{i}' in df.columns and f'TargetY{i}' in df.columns and not df[f'TargetX{i}'].empty:
        axs1[1, 0].plot(df[f'TargetX{i}'].iloc[0], df[f'TargetY{i}'].iloc[0], marker='x', color=colors[i % len(colors)], markersize=10, label=f'D{i} Start Tgt' if i == 0 else "_nolegend_")
        if f'CurrentX{i}' in df.columns and not df[f'CurrentX{i}'].empty:
             axs1[1, 0].plot(df[f'CurrentX{i}'].iloc[0], df[f'CurrentY{i}'].iloc[0], marker='o', color=colors[i % len(colors)], markersize=6, label=f'D{i} Start Pos' if i == 0 else "_nolegend_")
        if df[f'TargetX{i}'].nunique() > 1: # Check if target actually changed
             axs1[1, 0].plot(df[f'TargetX{i}'].iloc[-1], df[f'TargetY{i}'].iloc[-1], marker='s', color=colors[i % len(colors)], markersize=10, label=f'D{i} End Tgt' if i == 0 else "_nolegend_")
axs1[1, 0].set_xlabel('X Position')
axs1[1, 0].set_ylabel('Y Position')
axs1[1, 0].set_title('Drone Formation Paths (X vs Y)')
axs1[1, 0].legend(fontsize='small')
axs1[1, 0].grid(True)
axs1[1, 0].axis('equal')

# Plot 1.4: Total Final Commands for Drone 0 (changed from PIDOut to FinalCmd)
final_cmd_x0_col = 'FinalCmdX0'
final_cmd_y0_col = 'FinalCmdY0'
if final_cmd_x0_col in df.columns and final_cmd_y0_col in df.columns:
    axs1[1, 1].plot(df['Time'], df[final_cmd_x0_col], label='D0 Final Command X', color='green')
    axs1[1, 1].plot(df['Time'], df[final_cmd_y0_col], label='D0 Final Command Y', color='purple', linestyle='-.')
    axs1[1, 1].set_xlabel('Time (s)')
    axs1[1, 1].set_ylabel('Final Command (Accel)')
    axs1[1, 1].set_title('Total Final Commands for Drone 0')
    axs1[1, 1].legend(fontsize='small')
    axs1[1, 1].grid(True)
    axs1[1, 1].set_xticks(integer_ticks)
else:
    axs1[1, 1].text(0.5, 0.5, f'{final_cmd_x0_col}/{final_cmd_y0_col}\ndata not found.', ha='center', va='center', transform=axs1[1, 1].transAxes)

fig1.tight_layout(rect=[0, 0.03, 1, 0.95])
fig1_filename = "overall_formation_analysis_fls.png" # Updated filename
fig1.savefig(fig1_filename, dpi=150)
print(f"Saved Figure 1 to {fig1_filename}")


# --- Figures for Individual Drone Analysis (Expanded Layout) ---
for drone_idx in range(NUM_DRONES):
    # Increased to 4 rows to accommodate FLS and Wind plots per drone
    fig_drone, axs_drone = plt.subplots(4, 2, figsize=(18, 16)) # Adjusted figsize for more rows
    fig_drone.suptitle(f'PID+FLS Detailed Analysis for Drone {drone_idx}', fontsize=16)

    # Plot D.1 (axs_drone[0, 0]): X-Axis Error
    error_x_col = f'ErrorX{drone_idx}'
    if error_x_col in df.columns:
        axs_drone[0, 0].plot(df['Time'], df[error_x_col], label=f'D{drone_idx} Error X', color=colors[drone_idx % len(colors)])
        axs_drone[0, 0].axhline(0, color='black', linewidth=0.5, linestyle='--')
        axs_drone[0, 0].set_xlabel('Time (s)')
        axs_drone[0, 0].set_ylabel('Position Error X')
        axs_drone[0, 0].set_title(f'X-Axis Position Error (D{drone_idx})')
        axs_drone[0, 0].legend(fontsize='small')
        axs_drone[0, 0].grid(True)
        axs_drone[0, 0].set_xticks(integer_ticks)
    else:
        axs_drone[0, 0].text(0.5, 0.5, f'{error_x_col} data not found.', ha='center', va='center', transform=axs_drone[0, 0].transAxes)

    # Plot D.2 (axs_drone[0, 1]): Y-Axis Error
    error_y_col = f'ErrorY{drone_idx}'
    if error_y_col in df.columns:
        axs_drone[0, 1].plot(df['Time'], df[error_y_col], label=f'D{drone_idx} Error Y', color=colors[drone_idx % len(colors)], linestyle='--')
        axs_drone[0, 1].axhline(0, color='black', linewidth=0.5, linestyle='--')
        axs_drone[0, 1].set_xlabel('Time (s)')
        axs_drone[0, 1].set_ylabel('Position Error Y')
        axs_drone[0, 1].set_title(f'Y-Axis Position Error (D{drone_idx})')
        axs_drone[0, 1].legend(fontsize='small')
        axs_drone[0, 1].grid(True)
        axs_drone[0, 1].set_xticks(integer_ticks)
    else:
        axs_drone[0, 1].text(0.5, 0.5, f'{error_y_col} data not found.', ha='center', va='center', transform=axs_drone[0, 1].transAxes)

    # Plot D.3 (axs_drone[1, 0]): PID Terms for X-axis
    p_term_x_col = f'PTermX{drone_idx}'
    i_term_x_col = f'ITermX{drone_idx}'
    d_term_x_col = f'DTermX{drone_idx}'
    pid_out_x_col = f'PIDOutX{drone_idx}'
    if all(col in df.columns for col in [p_term_x_col, i_term_x_col, d_term_x_col, pid_out_x_col]):
        axs_drone[1, 0].plot(df['Time'], df[p_term_x_col], label='P-Term X', color='cyan')
        axs_drone[1, 0].plot(df['Time'], df[i_term_x_col], label='I-Term X', color='magenta')
        axs_drone[1, 0].plot(df['Time'], df[d_term_x_col], label='D-Term X', color='yellow')
        axs_drone[1, 0].plot(df['Time'], df[pid_out_x_col], label='PID Out X', color=colors[drone_idx % len(colors)], linestyle=':')
        axs_drone[1, 0].set_xlabel('Time (s)')
        axs_drone[1, 0].set_ylabel('PID Term Contribution')
        axs_drone[1, 0].set_title(f'PID Term Breakdown for D{drone_idx} (X-axis)')
        axs_drone[1, 0].legend(fontsize='small')
        axs_drone[1, 0].grid(True)
        axs_drone[1, 0].set_xticks(integer_ticks)
    else:
        axs_drone[1, 0].text(0.5, 0.5, f'PID terms X data not found\nfor D{drone_idx}.', ha='center', va='center', transform=axs_drone[1, 0].transAxes)

    # Plot D.4 (axs_drone[1, 1]): PID Terms for Y-axis
    p_term_y_col = f'PTermY{drone_idx}'
    i_term_y_col = f'ITermY{drone_idx}'
    d_term_y_col = f'DTermY{drone_idx}'
    pid_out_y_col = f'PIDOutY{drone_idx}'
    if all(col in df.columns for col in [p_term_y_col, i_term_y_col, d_term_y_col, pid_out_y_col]):
        axs_drone[1, 1].plot(df['Time'], df[p_term_y_col], label='P-Term Y', color='lime')
        axs_drone[1, 1].plot(df['Time'], df[i_term_y_col], label='I-Term Y', color='coral')
        axs_drone[1, 1].plot(df['Time'], df[d_term_y_col], label='D-Term Y', color='gold')
        axs_drone[1, 1].plot(df['Time'], df[pid_out_y_col], label='PID Out Y', color=colors[drone_idx % len(colors)], linestyle=':')
        axs_drone[1, 1].set_xlabel('Time (s)')
        axs_drone[1, 1].set_ylabel('PID Term Contribution')
        axs_drone[1, 1].set_title(f'PID Term Breakdown for D{drone_idx} (Y-axis)')
        axs_drone[1, 1].legend(fontsize='small')
        axs_drone[1, 1].grid(True)
        axs_drone[1, 1].set_xticks(integer_ticks)
    else:
        axs_drone[1, 1].text(0.5, 0.5, f'PID terms Y data not found\nfor D{drone_idx}.', ha='center', va='center', transform=axs_drone[1, 1].transAxes)

    # --- NEW PLOTS FOR FLS ---
    # Plot D.5 (axs_drone[2, 0]): FLS Correction and Final Command for X-axis
    fls_corr_x_col = f'FLSCorrX{drone_idx}'
    final_cmd_x_col = f'FinalCmdX{drone_idx}'
    # pid_out_x_col is already defined
    if all(col in df.columns for col in [pid_out_x_col, fls_corr_x_col, final_cmd_x_col]):
        axs_drone[2, 0].plot(df['Time'], df[pid_out_x_col], label=f'PID Out X', linestyle=':', color='gray')
        axs_drone[2, 0].plot(df['Time'], df[fls_corr_x_col], label=f'FLS Corr X', linestyle='--', color='orange')
        axs_drone[2, 0].plot(df['Time'], df[final_cmd_x_col], label=f'Final Cmd X', color=colors[drone_idx % len(colors)])
        axs_drone[2, 0].set_xlabel('Time (s)')
        axs_drone[2, 0].set_ylabel('Command Value')
        axs_drone[2, 0].set_title(f'PID, FLS, Final Cmd X (D{drone_idx})')
        axs_drone[2, 0].legend(fontsize='small')
        axs_drone[2, 0].grid(True)
        axs_drone[2, 0].set_xticks(integer_ticks)
    else:
        axs_drone[2, 0].text(0.5, 0.5, f'FLS/Final Cmd X data not found\nfor D{drone_idx}.', ha='center', va='center', transform=axs_drone[2, 0].transAxes)

    # Plot D.6 (axs_drone[2, 1]): FLS Correction and Final Command for Y-axis
    fls_corr_y_col = f'FLSCorrY{drone_idx}'
    final_cmd_y_col = f'FinalCmdY{drone_idx}'
    # pid_out_y_col is already defined
    if all(col in df.columns for col in [pid_out_y_col, fls_corr_y_col, final_cmd_y_col]):
        axs_drone[2, 1].plot(df['Time'], df[pid_out_y_col], label=f'PID Out Y', linestyle=':', color='gray')
        axs_drone[2, 1].plot(df['Time'], df[fls_corr_y_col], label=f'FLS Corr Y', linestyle='--', color='teal')
        axs_drone[2, 1].plot(df['Time'], df[final_cmd_y_col], label=f'Final Cmd Y', color=colors[drone_idx % len(colors)], linestyle='-') # Keep solid for main command
        axs_drone[2, 1].set_xlabel('Time (s)')
        axs_drone[2, 1].set_ylabel('Command Value')
        axs_drone[2, 1].set_title(f'PID, FLS, Final Cmd Y (D{drone_idx})')
        axs_drone[2, 1].legend(fontsize='small')
        axs_drone[2, 1].grid(True)
        axs_drone[2, 1].set_xticks(integer_ticks)
    else:
        axs_drone[2, 1].text(0.5, 0.5, f'FLS/Final Cmd Y data not found\nfor D{drone_idx}.', ha='center', va='center', transform=axs_drone[2, 1].transAxes)

    # Plot D.7 (axs_drone[3, 0]): Simulated Wind (plotted per drone figure, though it's global)
    sim_wind_x_col = 'SimWindX'
    sim_wind_y_col = 'SimWindY'
    if sim_wind_x_col in df.columns and sim_wind_y_col in df.columns:
        axs_drone[3, 0].plot(df['Time'], df[sim_wind_x_col], label='Simulated Wind X')
        axs_drone[3, 0].plot(df['Time'], df[sim_wind_y_col], label='Simulated Wind Y', linestyle='--')
        axs_drone[3, 0].set_xlabel('Time (s)')
        axs_drone[3, 0].set_ylabel('Wind Strength')
        axs_drone[3, 0].set_title('Simulated Wind Conditions')
        axs_drone[3, 0].legend(fontsize='small')
        axs_drone[3, 0].grid(True)
        axs_drone[3, 0].set_xticks(integer_ticks)
    else:
        axs_drone[3, 0].text(0.5, 0.5, 'Simulated Wind data not found.', ha='center', va='center', transform=axs_drone[3, 0].transAxes)

    # Hide the empty subplot D.8 (axs_drone[3, 1])
    axs_drone[3, 1].axis('off')


    fig_drone.tight_layout(rect=[0, 0.03, 1, 0.95])
    fig_drone_filename = f"drone_{drone_idx}_detailed_analysis_fls.png" # Updated filename
    fig_drone.savefig(fig_drone_filename, dpi=150)
    print(f"Saved Figure for Drone {drone_idx} to {fig_drone_filename}")

plt.show()