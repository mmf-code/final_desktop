import pandas as pd
import matplotlib.pyplot as plt
import os
import sys
import numpy as np

csv_file_name = "multi_drone_sim_data.csv"
script_dir = os.path.dirname(os.path.abspath(__file__))

# --- CSV Path Finding (same as before, ensure it's correct for your setup) ---
potential_csv_path = ""
if os.path.basename(script_dir) == "final":
    potential_csv_path = os.path.join(script_dir, "agent_control_pkg", "build", "Debug", csv_file_name)
elif os.path.basename(script_dir) == "Debug" and \
     os.path.basename(os.path.dirname(script_dir)) == "build" and \
     os.path.basename(os.path.dirname(os.path.dirname(script_dir))) == "agent_control_pkg":
    potential_csv_path = os.path.join(script_dir, csv_file_name)
else:
    potential_csv_path = os.path.join(script_dir, csv_file_name)

if not os.path.exists(potential_csv_path):
    alt_csv_path_from_final_root = os.path.join(script_dir, "agent_control_pkg", "build", "Debug", csv_file_name)
    if os.path.exists(alt_csv_path_from_final_root):
        csv_path = alt_csv_path_from_final_root
    else:
        print(f"Error: CSV not found at primary ({potential_csv_path}) or alt path ({alt_csv_path_from_final_root}).")
        sys.exit()
else:
    csv_path = potential_csv_path
print("Attempting to load CSV from:", csv_path)
# --- End CSV Path Finding ---

try:
    df = pd.read_csv(csv_path, on_bad_lines='skip')
    data_cols = [col for col in df.columns if col != 'Time' and 'Changing' not in col]
    for col in data_cols:
        if col in df.columns:
            df[col] = pd.to_numeric(df[col], errors='coerce')
    df.dropna(subset=['Time'], inplace=True)
except FileNotFoundError:
    print(f"Error: {csv_path} not found.")
    sys.exit()
except Exception as e:
    print(f"An error occurred while loading/processing CSV: {e}")
    sys.exit()

if df.empty:
    print("DataFrame is empty. Exiting.")
    sys.exit()

NUM_DRONES = 3
colors = ['blue', 'green', 'red', 'purple', 'orange', 'brown']

max_time = df['Time'].max()
tick_step = 1 if max_time <= 10 else 2
integer_ticks = np.arange(0, np.ceil(max_time) + 1, tick_step)

# --- Figure 1: Overall Formation Analysis ---
fig1, axs1 = plt.subplots(2, 2, figsize=(18, 10))
fig1.suptitle('Overall Formation PID Control Analysis', fontsize=16)

# Plot 1.1: X Positions
# ... (plotting code for axs1[0,0] as before) ...
for i in range(NUM_DRONES):
    axs1[0, 0].plot(df['Time'], df.get(f'TargetX{i}'), label=f'D{i} Target X', linestyle='--', color=colors[i])
    axs1[0, 0].plot(df['Time'], df.get(f'CurrentX{i}'), label=f'D{i} Current X', color=colors[i])
axs1[0, 0].set_xlabel('Time (s)')
axs1[0, 0].set_ylabel('X Position')
axs1[0, 0].set_title('X Position Control (All Drones)')
axs1[0, 0].legend(fontsize='small')
axs1[0, 0].grid(True)
axs1[0, 0].set_xticks(integer_ticks)

# Plot 1.2: Y Positions
# ... (plotting code for axs1[0,1] as before) ...
for i in range(NUM_DRONES):
    axs1[0, 1].plot(df['Time'], df.get(f'TargetY{i}'), label=f'D{i} Target Y', linestyle='--', color=colors[i])
    axs1[0, 1].plot(df['Time'], df.get(f'CurrentY{i}'), label=f'D{i} Current Y', color=colors[i])
axs1[0, 1].set_xlabel('Time (s)')
axs1[0, 1].set_ylabel('Y Position')
axs1[0, 1].set_title('Y Position Control (All Drones)')
axs1[0, 1].legend(fontsize='small')
axs1[0, 1].grid(True)
axs1[0, 1].set_xticks(integer_ticks)


# Plot 1.3: Formation Path (X vs Y)
# ... (plotting code for axs1[1,0] as before) ...
for i in range(NUM_DRONES):
    axs1[1, 0].plot(df.get(f'CurrentX{i}'), df.get(f'CurrentY{i}'), label=f'D{i} Path', color=colors[i])
    if f'TargetX{i}' in df.columns and f'TargetY{i}' in df.columns: # Check if target columns exist
        axs1[1, 0].plot(df[f'TargetX{i}'].iloc[0], df[f'TargetY{i}'].iloc[0], marker='x', color=colors[i], markersize=10, label=f'D{i} Start Tgt' if i == 0 else "_nolegend_")
        axs1[1, 0].plot(df[f'CurrentX{i}'].iloc[0], df[f'CurrentY{i}'].iloc[0], marker='o', color=colors[i], markersize=6, label=f'D{i} Start Pos' if i == 0 else "_nolegend_")
        if df[f'TargetX{i}'].nunique() > 1: # Check if target actually changed
             axs1[1, 0].plot(df[f'TargetX{i}'].iloc[-1], df[f'TargetY{i}'].iloc[-1], marker='s', color=colors[i], markersize=10, label=f'D{i} End Tgt' if i == 0 else "_nolegend_")
axs1[1, 0].set_xlabel('X Position')
axs1[1, 0].set_ylabel('Y Position')
axs1[1, 0].set_title('Drone Formation Paths (X vs Y)')
axs1[1, 0].legend(fontsize='small')
axs1[1, 0].grid(True)
axs1[1, 0].axis('equal')

# Plot 1.4: Total PID Outputs for Drone 0
# ... (plotting code for axs1[1,1] as before) ...
if f'PIDOutX0' in df.columns and f'PIDOutY0' in df.columns:
    axs1[1, 1].plot(df['Time'], df['PIDOutX0'], label='D0 PID Output X', color='green')
    axs1[1, 1].plot(df['Time'], df['PIDOutY0'], label='D0 PID Output Y', color='purple', linestyle='-.')
    axs1[1, 1].set_xlabel('Time (s)')
    axs1[1, 1].set_ylabel('PID Output (Total Accel Cmd)')
    axs1[1, 1].set_title('Total PID Outputs for Drone 0')
    axs1[1, 1].legend(fontsize='small')
    axs1[1, 1].grid(True)
    axs1[1, 1].set_xticks(integer_ticks)
else:
    axs1[1, 1].text(0.5, 0.5, 'PIDOutX0/Y0 data not found.', ha='center', va='center', transform=axs1[1, 1].transAxes)

fig1.tight_layout(rect=[0, 0.03, 1, 0.95])
# --- SAVE FIGURE 1 ---
fig1_filename = "overall_formation_analysis.png"
fig1.savefig(fig1_filename, dpi=150) # dpi for resolution
print(f"Saved Figure 1 to {fig1_filename}")
# --- END SAVE FIGURE 1 ---


# --- Figures for Individual Drone Analysis ---
for drone_idx in range(NUM_DRONES):
    fig_drone, axs_drone = plt.subplots(2, 2, figsize=(18, 10))
    fig_drone.suptitle(f'PID Detailed Analysis for Drone {drone_idx}', fontsize=16)

    # Plot D.1: X-Axis Error
    # ... (plotting code for axs_drone[0,0] as before) ...
    if f'ErrorX{drone_idx}' in df.columns:
        axs_drone[0, 0].plot(df['Time'], df[f'ErrorX{drone_idx}'], label=f'D{drone_idx} Error X', color=colors[drone_idx])
        axs_drone[0, 0].axhline(0, color='black', linewidth=0.5, linestyle='--')
        axs_drone[0, 0].set_xlabel('Time (s)')
        axs_drone[0, 0].set_ylabel('Position Error X')
        axs_drone[0, 0].set_title(f'X-Axis Position Error (D{drone_idx})')
        axs_drone[0, 0].legend(fontsize='small')
        axs_drone[0, 0].grid(True)
        axs_drone[0, 0].set_xticks(integer_ticks)
    else:
        axs_drone[0, 0].text(0.5, 0.5, f'ErrorX{drone_idx} data not found.', ha='center', va='center', transform=axs_drone[0, 0].transAxes)

    # Plot D.2: Y-Axis Error
    # ... (plotting code for axs_drone[0,1] as before) ...
    if f'ErrorY{drone_idx}' in df.columns:
        axs_drone[0, 1].plot(df['Time'], df[f'ErrorY{drone_idx}'], label=f'D{drone_idx} Error Y', color=colors[drone_idx], linestyle='--') # Different linestyle
        axs_drone[0, 1].axhline(0, color='black', linewidth=0.5, linestyle='--')
        axs_drone[0, 1].set_xlabel('Time (s)')
        axs_drone[0, 1].set_ylabel('Position Error Y')
        axs_drone[0, 1].set_title(f'Y-Axis Position Error (D{drone_idx})')
        axs_drone[0, 1].legend(fontsize='small')
        axs_drone[0, 1].grid(True)
        axs_drone[0, 1].set_xticks(integer_ticks)
    else:
        axs_drone[0, 1].text(0.5, 0.5, f'ErrorY{drone_idx} data not found.', ha='center', va='center', transform=axs_drone[0, 1].transAxes)

    # Plot D.3: PID Terms for X-axis
    # ... (plotting code for axs_drone[1,0] as before) ...
    pid_terms_x_cols = [f'PTermX{drone_idx}', f'ITermX{drone_idx}', f'DTermX{drone_idx}']
    if all(col in df.columns for col in pid_terms_x_cols) and f'PIDOutX{drone_idx}' in df.columns:
        axs_drone[1, 0].plot(df['Time'], df[f'PTermX{drone_idx}'], label='P-Term X', color='cyan')
        axs_drone[1, 0].plot(df['Time'], df[f'ITermX{drone_idx}'], label='I-Term X', color='magenta')
        axs_drone[1, 0].plot(df['Time'], df[f'DTermX{drone_idx}'], label='D-Term X', color='yellow')
        axs_drone[1, 0].plot(df['Time'], df[f'PIDOutX{drone_idx}'], label='Total Out X', color=colors[drone_idx], linestyle=':')
        axs_drone[1, 0].set_xlabel('Time (s)')
        axs_drone[1, 0].set_ylabel('PID Term Contribution')
        axs_drone[1, 0].set_title(f'PID Term Breakdown for D{drone_idx} (X-axis)')
        axs_drone[1, 0].legend(fontsize='small')
        axs_drone[1, 0].grid(True)
        axs_drone[1, 0].set_xticks(integer_ticks)
    else:
        axs_drone[1, 0].text(0.5, 0.5, f'P,I,D term data not found\nfor D{drone_idx} X-axis.', ha='center', va='center', transform=axs_drone[1, 0].transAxes)

    # Plot D.4: PID Terms for Y-axis
    # ... (plotting code for axs_drone[1,1] as before) ...
    pid_terms_y_cols = [f'PTermY{drone_idx}', f'ITermY{drone_idx}', f'DTermY{drone_idx}']
    if all(col in df.columns for col in pid_terms_y_cols) and f'PIDOutY{drone_idx}' in df.columns:
        axs_drone[1, 1].plot(df['Time'], df[f'PTermY{drone_idx}'], label='P-Term Y', color='lime')
        axs_drone[1, 1].plot(df['Time'], df[f'ITermY{drone_idx}'], label='I-Term Y', color='coral')
        axs_drone[1, 1].plot(df['Time'], df[f'DTermY{drone_idx}'], label='D-Term Y', color='gold')
        axs_drone[1, 1].plot(df['Time'], df[f'PIDOutY{drone_idx}'], label='Total Out Y', color=colors[drone_idx], linestyle=':')
        axs_drone[1, 1].set_xlabel('Time (s)')
        axs_drone[1, 1].set_ylabel('PID Term Contribution')
        axs_drone[1, 1].set_title(f'PID Term Breakdown for D{drone_idx} (Y-axis)')
        axs_drone[1, 1].legend(fontsize='small')
        axs_drone[1, 1].grid(True)
        axs_drone[1, 1].set_xticks(integer_ticks)
    else:
        axs_drone[1, 1].text(0.5, 0.5, f'P,I,D term data not found\nfor D{drone_idx} Y-axis.', ha='center', va='center', transform=axs_drone[1, 1].transAxes)


    fig_drone.tight_layout(rect=[0, 0.03, 1, 0.95])
    # --- SAVE INDIVIDUAL DRONE FIGURE ---
    fig_drone_filename = f"drone_{drone_idx}_detailed_analysis.png"
    fig_drone.savefig(fig_drone_filename, dpi=150)
    print(f"Saved Figure for Drone {drone_idx} to {fig_drone_filename}")
    # --- END SAVE INDIVIDUAL DRONE FIGURE ---

plt.show() # Show all figures at the end