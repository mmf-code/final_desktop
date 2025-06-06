#!/usr/bin/env python3
import os
import subprocess
import glob
import yaml
import re
import time
import pandas as pd # For CSV analysis
import numpy as np  # For numerical operations
from scipy.signal import find_peaks # For peak detection in oscillation analysis

# Get the absolute path to the workspace root directory
# Assuming this script is in a 'scripts' folder at the root of your project
SCRIPTS_DIR = os.path.dirname(os.path.abspath(__file__))
WORKSPACE_ROOT = os.path.dirname(SCRIPTS_DIR) # Go one level up from 'scripts'

# Use absolute paths for all file references
CONFIG_PATH = os.path.join(WORKSPACE_ROOT, 'agent_control_pkg', 'config', 'simulation_params.yaml')
BUILD_BIN = os.path.join(WORKSPACE_ROOT, 'build', 'Debug', 'multi_drone_pid_tester.exe')
# The C++ code will create subdirectories like 'zn_tuning' inside this
BASE_OUTPUT_DIR = os.path.join(WORKSPACE_ROOT, 'simulation_outputs') # C++ uses this from config
CSV_OUTPUT_SUBDIR_ZN = "zn_tuning" # Matches C++ code


def load_config():
    print(f"Attempting to load config from: {CONFIG_PATH}")
    if not os.path.exists(CONFIG_PATH):
        print(f"ERROR: Config file not found at {CONFIG_PATH}")
        exit(1)
    with open(CONFIG_PATH, 'r') as f:
        return yaml.safe_load(f)

def save_config(cfg):
    print(f"Saving config to: {CONFIG_PATH}")
    with open(CONFIG_PATH, 'w') as f:
        yaml.dump(cfg, f, sort_keys=False) # sort_keys=False to maintain order

def run_sim():
    # The C++ executable is expected to be run from its build directory
    # or have its paths configured correctly internally if run from elsewhere.
    # For simplicity, let's assume it can be run directly if it's in PATH,
    # or specify its full path.
    # The C++ code uses findConfigFilePath, so relative paths for config should work.
    
    # The C++ executable should create its output files based on its internal logic
    # and the output_directory specified in simulation_params.yaml.
    
    # Running from the directory where the exe is located is often safest for DLLs etc.
    exe_dir = os.path.dirname(BUILD_BIN)
    exe_name = os.path.basename(BUILD_BIN)

    print(f"Running simulation: {BUILD_BIN}")
    try:
        # subprocess.run([BUILD_BIN], check=True, capture_output=True, text=True) # To see C++ output
        process = subprocess.Popen([BUILD_BIN], cwd=exe_dir, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        stdout, stderr = process.communicate()
        print("C++ stdout:\n", stdout)
        if process.returncode != 0:
            print(f"C++ stderr (Return Code {process.returncode}):\n", stderr)
            # raise subprocess.CalledProcessError(process.returncode, BUILD_BIN, output=stdout, stderr=stderr)
        elif stderr: # Print stderr even on success if there's anything
             print(f"C++ stderr (Return Code 0):\n", stderr)

    except subprocess.CalledProcessError as e:
        print(f"ERROR: Simulation run failed for Kp_test.")
        print(f"Stdout: {e.stdout}")
        print(f"Stderr: {e.stderr}")
        return False
    except FileNotFoundError:
        print(f"ERROR: Executable not found at {BUILD_BIN}")
        return False
    return True


def find_latest_zn_csv(kp_test_val):
    # The C++ code creates filenames like: zn_test_KpX.XXX_FLS_OFF_WIND_OFF_TIMESTAMP.csv
    # inside 'simulation_outputs/zn_tuning/'
    # We need to get BASE_OUTPUT_DIR from the config for robustness
    cfg_temp = load_config() # Load fresh config to get output_directory
    output_dir_from_cfg = cfg_temp.get('output_settings', {}).get('output_directory', 'simulation_outputs')
    
    zn_csv_dir = os.path.join(WORKSPACE_ROOT, output_dir_from_cfg, CSV_OUTPUT_SUBDIR_ZN)
    
    # Glob pattern needs to match the Kp format from C++ (e.g., 3 decimal places)
    # The timestamp makes finding the "latest" tricky if runs are very fast.
    # A better way is to have the C++ code output a fixed name or return the name.
    # For now, let's try to find *a* file matching the Kp.
    
    # Construct a precise pattern based on how C++ names files for ZN
    # Example: zn_test_Kp2.500_FLS_OFF_WIND_OFF_*.csv
    pattern_str = f"zn_test_Kp{kp_test_val:.3f}_FLS_OFF_WIND_OFF_*.csv"
    
    search_pattern = os.path.join(zn_csv_dir, pattern_str)
    print(f"Searching for ZN CSV with pattern: {search_pattern}")
    
    files = glob.glob(search_pattern)
    if not files:
        print(f"No ZN CSV files found for Kp_test={kp_test_val:.3f} in {zn_csv_dir}")
        return None
    
    # Get the most recently modified file among the matches
    latest_file = max(files, key=os.path.getctime)
    print(f"Found ZN CSV: {latest_file}")
    return latest_file

def analyze_csv_for_oscillations(csv_path, target_x_value, phase_start_time, phase_end_time):
    """
    Analyzes CSV data for sustained oscillations.
    Returns (is_sustained, period_Pu)
    'is_sustained' can be: True, False, 'growing', 'damped'
    'period_Pu' is the estimated period if sustained or somewhat oscillatory.
    """
    if not csv_path or not os.path.exists(csv_path):
        return 'error_no_csv', None

    try:
        df = pd.read_csv(csv_path)
    except Exception as e:
        print(f"Error reading CSV {csv_path}: {e}")
        return 'error_csv_read', None

    # Filter data for the first relevant phase (where Z-N step response occurs)
    # Assuming phase_start_time and phase_end_time define this window
    df_phase = df[(df['Time'] >= phase_start_time) & (df['Time'] < phase_end_time)].copy()
    
    if df_phase.empty:
        print("No data in the specified phase window for oscillation analysis.")
        return 'no_data_in_phase', None

    # Focus on Drone 0, X-axis position ('CurrentX0') and its error
    # Error relative to the target_x_value for this phase
    current_x_col = 'CurrentX0' # Make sure this matches your CSV header
    if current_x_col not in df_phase.columns:
        print(f"Column '{current_x_col}' not found in CSV.")
        return 'error_missing_column', None
        
    df_phase['ErrorFromTarget'] = df_phase[current_x_col] - target_x_value
    
    signal = df_phase['ErrorFromTarget'].values
    time_signal = df_phase['Time'].values

    if len(signal) < 10: # Need enough data points
        return 'insufficient_data', None

    # Simple Oscillation Detection:
    # 1. Find peaks (local maxima) and troughs (local minima, i.e., peaks of -signal)
    # More robust peak finding might be needed (e.g. prominence, distance)
    peaks_indices, _ = find_peaks(signal, prominence=0.05) # Prominence helps filter noise
    troughs_indices, _ = find_peaks(-signal, prominence=0.05)

    if len(peaks_indices) < 3 or len(troughs_indices) < 3:
        print("Not enough peaks/troughs found for robust oscillation analysis.")
        return 'damped_or_no_oscillation', None

    # Combine and sort all extremas by time
    all_extrema_indices = sorted(np.concatenate((peaks_indices, troughs_indices)))
    
    if len(all_extrema_indices) < 5: # Need at least a few full cycles
         print("Not enough alternating extrema for oscillation analysis.")
         return 'damped_or_no_oscillation', None

    extrema_values = signal[all_extrema_indices]
    extrema_times = time_signal[all_extrema_indices]

    # Check amplitude stability (very basic)
    # Look at the amplitudes of the last few oscillations
    # A more sophisticated check would look at the envelope of the signal
    amplitudes = []
    periods = []
    for i in range(len(all_extrema_indices) - 1):
        # Amplitude can be taken as difference from zero crossing, or peak-to-trough/2
        # For simplicity, let's look at magnitude of extrema
        amplitudes.append(abs(extrema_values[i]))
        if i < len(all_extrema_indices) - 2: # Need three points for two periods
            # Period: time between two similar extrema (e.g. peak-to-peak)
            # Or, roughly, time between peak and next peak, or trough and next trough
            # More simply: time between any two consecutive extrema is half a period.
            # Let's try average time between consecutive extrema * 2
            if i < len(extrema_times) -1:
                 # Time between successive extrema (peaks or troughs)
                if (signal[all_extrema_indices[i]] > 0 and signal[all_extrema_indices[i+1]] < 0) or \
                   (signal[all_extrema_indices[i]] < 0 and signal[all_extrema_indices[i+1]] > 0):
                    # this is approx half period
                    periods.append(2 * (extrema_times[i+1] - extrema_times[i]))


    if not periods or not amplitudes:
        print("Could not extract periods or amplitudes.")
        return 'damped_or_no_oscillation', None
        
    avg_period = np.mean(periods[-3:]) if len(periods) >=3 else np.mean(periods) if periods else None # Avg of last few periods
    
    # Check amplitude trend in the latter half of extrema
    num_to_check = len(amplitudes) // 2
    if num_to_check < 3:
        print("Not enough amplitudes to check trend reliably.")
        return 'damped_or_no_oscillation', avg_period

    first_half_amps_mean = np.mean(amplitudes[num_to_check//2 : num_to_check + num_to_check//2])
    second_half_amps_mean = np.mean(amplitudes[-num_to_check:])
    
    print(f"  Extrema Amplitudes (latter half): {amplitudes[-num_to_check:]}")
    print(f"  Periods (last 3 estimates): {periods[-3:] if len(periods)>=3 else periods}")
    print(f"  Avg Period Pu_est: {avg_period}")
    print(f"  Mean amp 1st half considered: {first_half_amps_mean}, 2nd half: {second_half_amps_mean}")


    # Decision logic (can be refined)
    # This is a heuristic. A more robust way is to fit an envelope or analyze FFT.
    if avg_period is None or avg_period < 0.1 : # Unreliable period
        return 'no_clear_oscillation', None

    amplitude_change_ratio = second_half_amps_mean / first_half_amps_mean if first_half_amps_mean > 1e-3 else 1.0

    if 0.85 < amplitude_change_ratio < 1.15: # Amplitudes roughly constant (+-15%)
        return 'sustained', avg_period
    elif amplitude_change_ratio >= 1.15:
        return 'growing', avg_period
    else: # ratio < 0.85
        return 'damped', avg_period

def main():
    os.makedirs(os.path.join(WORKSPACE_ROOT, "simulation_outputs", CSV_OUTPUT_SUBDIR_ZN), exist_ok=True)
    
    print(f"Workspace root: {WORKSPACE_ROOT}")
    print(f"Config path: {CONFIG_PATH}")
    print(f"Build binary: {BUILD_BIN}")
    
    cfg_original = load_config() # Keep original to restore later

    # --- Settings for Z-N Ku/Pu Search ---
    cfg = load_config() # Work on a copy
    cfg['simulation_settings']['ziegler_nichols_tuning']['enable'] = True
    # The simulation_time for Z-N should be read from the ZN section
    # And ensure the C++ code uses it when ZN is active
    zn_sim_time = cfg['simulation_settings']['ziegler_nichols_tuning'].get('simulation_time', 5.0)
    cfg['simulation_settings']['total_time'] = zn_sim_time # Ensure C++ respects this if ZN section doesn't override total_time

    # PID gains for ZN mode (Ki=0, Kd=0) are set by C++ when zn_tuning.enable is true.
    # We only need to iterate kp_test_value.

    # Define the Kp values to test for Z-N
    # You might want to make this range and step configurable
    kp_test_min = cfg_original.get('auto_tuner_settings', {}).get('kp_test_min', 0.5)
    kp_test_max = cfg_original.get('auto_tuner_settings', {}).get('kp_test_max', 5.0)
    kp_test_step = cfg_original.get('auto_tuner_settings', {}).get('kp_test_step', 0.2)
    
    kp_values_to_test = [round(k,3) for k in np.arange(kp_test_min, kp_test_max + kp_test_step/2, kp_test_step)]
    print(f"Will test Kp_test values for Z-N: {kp_values_to_test}")

    results_zn = []
    found_ku = None
    found_pu = None

    # Determine the target and time window for the first phase from config for analysis
    # This assumes ZN focuses on the first configured phase.
    first_phase_target_x = 0.0
    first_phase_start_time = 0.0
    first_phase_end_time = zn_sim_time # Analyze up to the end of ZN sim time

    if cfg_original.get('scenario_settings', {}).get('phases'):
        first_phase_config = cfg_original['scenario_settings']['phases'][0]
        if 'center' in first_phase_config and len(first_phase_config['center']) > 0:
            first_phase_target_x = first_phase_config['center'][0] # Assuming X is first
        # first_phase_start_time is implicitly 0 due to C++ ZN logic
        if len(cfg_original['scenario_settings']['phases']) > 1:
            # If there's a second phase, analyze up to its start time or zn_sim_time, whichever is shorter
            first_phase_end_time = min(zn_sim_time, cfg_original['scenario_settings']['phases'][1].get('start_time', zn_sim_time))
        print(f"ZN Analysis: Target X={first_phase_target_x}, StartT={first_phase_start_time}, EndT={first_phase_end_time}")
    else:
        print("Warning: No phases defined in config for ZN analysis reference. Assuming target 5.0 for X.")
        first_phase_target_x = 5.0 # Default if no phases defined

    for kp_test in kp_values_to_test:
        print(f"\n--- Testing Z-N with Kp_test = {kp_test:.3f} ---")
        cfg['simulation_settings']['ziegler_nichols_tuning']['kp_test_value'] = kp_test
        save_config(cfg)

        if not run_sim():
            print(f"Simulation failed for Kp_test = {kp_test:.3f}. Skipping.")
            results_zn.append({'kp_test': kp_test, 'status': 'sim_failed', 'Ku': None, 'Pu': None})
            continue
        
        time.sleep(1) # Allow filesystem to catch up

        latest_csv = find_latest_zn_csv(kp_test)
        if not latest_csv:
            print(f"Could not find CSV for Kp_test = {kp_test:.3f}. Check output paths and filenames.")
            results_zn.append({'kp_test': kp_test, 'status': 'no_csv', 'Ku': None, 'Pu': None})
            continue

        oscillation_status, Pu_estimate = analyze_csv_for_oscillations(latest_csv, 
                                                                       first_phase_target_x, 
                                                                       first_phase_start_time, 
                                                                       first_phase_end_time)
        
        print(f"Kp_test={kp_test:.3f} -> Oscillation: {oscillation_status}, Estimated Pu: {Pu_estimate if Pu_estimate else 'N/A'}")
        results_zn.append({'kp_test': kp_test, 'status': oscillation_status, 'Ku': None, 'Pu': Pu_estimate})

        if oscillation_status == 'sustained':
            found_ku = kp_test
            found_pu = Pu_estimate
            print(f"*** Found potential Ku = {found_ku:.3f} with Pu = {found_pu:.3f} ***")
            # Decide if you want to stop here or continue searching for a "better" sustained oscillation
            # break # Uncomment to stop at first sign of sustained
        elif oscillation_status == 'growing':
            print(f"Oscillations growing at Kp_test = {kp_test:.3f}. Ku is likely lower.")
            # If you have a previous 'sustained' or 'damped' this might be too high
            # Or if this is the first 'growing' after 'damped', Ku is between this and previous.
            # break # Often good to stop if it's clearly growing

    print("\n--- Z-N Ku/Pu Search Summary ---")
    for res in results_zn:
        print(f"Kp_test: {res['kp_test']:.3f}, Status: {res['status']}, Pu_est: {res['Pu'] if res['Pu'] else 'N/A'}")

    if found_ku and found_pu:
        print(f"\nRecommended: Ku = {found_ku:.3f}, Pu = {found_pu:.3f}")
        # Calculate Z-N PID gains
        kp_zn_pid = 0.6 * found_ku
        ti_zn = found_pu / 2.0
        td_zn = found_pu / 8.0
        ki_zn_pid = kp_zn_pid / ti_zn if ti_zn > 1e-6 else 0
        kd_zn_pid = kp_zn_pid * td_zn
        print(f"Calculated Z-N PID gains: Kp={kp_zn_pid:.3f}, Ki={ki_zn_pid:.3f}, Kd={kd_zn_pid:.3f}")
        
        # Optional: Automatically update config with these ZN-derived PID gains for a normal run
        # print("\nUpdating config with ZN-derived PID gains for a normal test run...")
        # cfg_test = load_config()
        # cfg_test['simulation_settings']['ziegler_nichols_tuning']['enable'] = False
        # cfg_test['controller_settings']['pid']['kp'] = round(kp_zn_pid,3)
        # cfg_test['controller_settings']['pid']['ki'] = round(ki_zn_pid,3)
        # cfg_test['controller_settings']['pid']['kd'] = round(kd_zn_pid,3)
        # cfg_test['scenario_settings']['enable_wind'] = True # Or your desired test setting
        # cfg_test['simulation_settings']['total_time'] = cfg_original.get('simulation_settings',{}).get('total_time', 60.0) # Restore original total time
        # save_config(cfg_test)
        # print("Running simulation with ZN-derived PID gains...")
        # run_sim()
        # print("Analyze the output from this run (metrics_sim_...txt and its CSV).")

    else:
        print("\nCould not definitively determine Ku and Pu automatically. Manual CSV analysis required.")

    # Restore original config if needed, or leave it at the last ZN test setting
    # save_config(cfg_original) 
    # print("Original config restored.")

if __name__ == '__main__':
    main()