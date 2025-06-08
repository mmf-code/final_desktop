#!/usr/bin/env python3
"""
Multi-Drone Formation Control Analysis Plotter
==============================================
This script analyzes and plots results from 4 simulation scenarios:
1. PID only (FLS_OFF_WIND_OFF)
2. PID + Wind (FLS_OFF_WIND_ON)
3. PID + FLS (FLS_ON_WIND_OFF)
4. PID + FLS + Wind (FLS_ON_WIND_ON)

Run this script in the same directory as your CSV files.
"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import glob
import os
from pathlib import Path

# Configuration
plt.style.use('seaborn-v0_8')  # Modern clean style
plt.rcParams['figure.figsize'] = (12, 8)
plt.rcParams['font.size'] = 10
plt.rcParams['lines.linewidth'] = 2

# Disable interactive plotting - only save plots
plt.ioff()


def find_csv_files():
    """Find the 4 simulation CSV files automatically"""
    files = {}

    # Specific CSV files from latest runs
    specific_files = {
        'pid_only': 'multi_agent_sim_Kp3.100_Ki0.400_Kd2.200_FLS_OFF_WIND_OFF_20250609_012947.csv',
        'pid_wind': 'multi_agent_sim_Kp3.100_Ki0.400_Kd2.200_FLS_OFF_WIND_ON_20250609_013004.csv',
        'pid_fls': 'multi_agent_sim_Kp3.100_Ki0.400_Kd2.200_FLS_ON_WIND_OFF_20250609_013024.csv',
        'pid_fls_wind': 'multi_agent_sim_Kp3.100_Ki0.400_Kd2.200_FLS_ON_WIND_ON_20250609_013041.csv'
    }

    for scenario, filename in specific_files.items():
        # Check multiple possible directories
        possible_paths = [
            f"build/Debug/simulation_outputs/{filename}",
            f"simulation_outputs/{filename}",
            filename  # Current directory
        ]

        found = False
        for filepath in possible_paths:
            if os.path.exists(filepath):
                files[scenario] = filepath
                print(f"Found {scenario}: {filepath}")
                found = True
                break

        if not found:
            print(f"WARNING: CSV file not found: {filename}")
            print(f"  Searched in: {', '.join(possible_paths)}")

    return files


def load_simulation_data(csv_files):
    """Load and organize simulation data"""
    data = {}

    for scenario, filepath in csv_files.items():
        try:
            df = pd.read_csv(filepath)
            data[scenario] = df
            print(f"Loaded {scenario}: {len(df)} rows, {len(df.columns)} columns")
        except Exception as e:
            print(f"Error loading {scenario}: {e}")

    return data

def plot_drone_trajectories(data):
    """Plot 2D trajectories for all drones in all scenarios"""
    fig, axes = plt.subplots(2, 2, figsize=(15, 12))
    fig.suptitle('Drone Trajectories - 2D Movement Paths', fontsize=16, fontweight='bold')

    scenarios = ['pid_only', 'pid_wind', 'pid_fls', 'pid_fls_wind']
    titles = ['PID Only', 'PID + Wind', 'PID + FLS', 'PID + FLS + Wind']

    for idx, (scenario, title) in enumerate(zip(scenarios, titles)):
        if scenario not in data:
            continue

        ax = axes[idx//2, idx%2]
        df = data[scenario]

        # Plot trajectories for each drone
        colors = ['blue', 'red', 'green']
        for drone_id in range(3):  # 3 drones
            if f'CurrentX{drone_id}' in df.columns:
                ax.plot(df[f'CurrentX{drone_id}'], df[f'CurrentY{drone_id}'],
                       color=colors[drone_id], label=f'Drone {drone_id}', alpha=0.7)

                # Mark start and end points
                ax.scatter(df[f'CurrentX{drone_id}'].iloc[0], df[f'CurrentY{drone_id}'].iloc[0],
                          color=colors[drone_id], marker='o', s=100, edgecolor='black', label=f'Start {drone_id}')
                ax.scatter(df[f'CurrentX{drone_id}'].iloc[-1], df[f'CurrentY{drone_id}'].iloc[-1],
                          color=colors[drone_id], marker='s', s=100, edgecolor='black', label=f'End {drone_id}')

        # Plot target points (formation centers)
        target_points = [(5, 5), (-5, 0), (0, -5), (5, 0)]  # Based on console output
        for i, (tx, ty) in enumerate(target_points):
            ax.scatter(tx, ty, marker='*', s=200, color='gold', edgecolor='black',
                      alpha=0.8, label='Target' if i == 0 else "")

        ax.set_xlabel('X Position (m)')
        ax.set_ylabel('Y Position (m)')
        ax.set_title(title)
        ax.grid(True, alpha=0.3)
        ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
        ax.axis('equal')

    plt.tight_layout()
    plt.savefig('01_drone_trajectories.png', dpi=300, bbox_inches='tight')
    plt.close()

def plot_position_tracking(data):
    """Plot position tracking for Drone 0 - X and Y separately"""

    # X-axis tracking
    fig, axes = plt.subplots(2, 2, figsize=(15, 10))
    fig.suptitle('Drone 0 - X Position Tracking', fontsize=16, fontweight='bold')

    scenarios = ['pid_only', 'pid_wind', 'pid_fls', 'pid_fls_wind']
    titles = ['PID Only', 'PID + Wind', 'PID + FLS', 'PID + FLS + Wind']

    for idx, (scenario, title) in enumerate(zip(scenarios, titles)):
        if scenario not in data:
            continue

        ax = axes[idx//2, idx%2]
        df = data[scenario]

        ax.plot(df['Time'], df['TargetX0'], 'r--', linewidth=2, label='Target X')
        ax.plot(df['Time'], df['CurrentX0'], 'b-', linewidth=2, label='Actual X')

        ax.set_xlabel('Time (s)')
        ax.set_ylabel('X Position (m)')
        ax.set_title(title)
        ax.grid(True, alpha=0.3)
        ax.legend()

        plt.tight_layout()
    plt.savefig('02_x_position_tracking.png', dpi=300, bbox_inches='tight')
    plt.close()

    # Y-axis tracking
    fig, axes = plt.subplots(2, 2, figsize=(15, 10))
    fig.suptitle('Drone 0 - Y Position Tracking', fontsize=16, fontweight='bold')

    for idx, (scenario, title) in enumerate(zip(scenarios, titles)):
        if scenario not in data:
            continue

        ax = axes[idx // 2, idx % 2]
        df = data[scenario]

        ax.plot(df['Time'], df['TargetY0'], 'r--', linewidth=2, label='Target Y')
        ax.plot(df['Time'], df['CurrentY0'], 'g-', linewidth=2, label='Actual Y')

        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Y Position (m)')
        ax.set_title(title)
        ax.grid(True, alpha=0.3)
        ax.legend()

    plt.tight_layout()
    plt.savefig('03_y_position_tracking.png', dpi=300, bbox_inches='tight')
    plt.close()

def plot_error_analysis(data):
    """Plot tracking errors for comparison"""

    # X-axis errors
    fig, axes = plt.subplots(2, 2, figsize=(15, 10))
    fig.suptitle('Drone 0 - X Position Errors', fontsize=16, fontweight='bold')

    scenarios = ['pid_only', 'pid_wind', 'pid_fls', 'pid_fls_wind']
    titles = ['PID Only', 'PID + Wind', 'PID + FLS', 'PID + FLS + Wind']

    for idx, (scenario, title) in enumerate(zip(scenarios, titles)):
        if scenario not in data:
            continue

        ax = axes[idx//2, idx%2]
        df = data[scenario]

        ax.plot(df['Time'], df['ErrorX0'], 'r-', linewidth=2, label='X Error')
        ax.axhline(y=0, color='k', linestyle='--', alpha=0.5)

        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Error (m)')
        ax.set_title(title)
        ax.grid(True, alpha=0.3)

        # Calculate and display RMS error
        rms_error = np.sqrt(np.mean(df['ErrorX0']**2))
        ax.text(0.02, 0.98, f'RMS Error: {rms_error:.3f}m', transform=ax.transAxes,
                verticalalignment='top', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))

    plt.tight_layout()
    plt.savefig('04_x_error_analysis.png', dpi=300, bbox_inches='tight')
    plt.close()

    # Y-axis errors
    fig, axes = plt.subplots(2, 2, figsize=(15, 10))
    fig.suptitle('Drone 0 - Y Position Errors', fontsize=16, fontweight='bold')

    for idx, (scenario, title) in enumerate(zip(scenarios, titles)):
        if scenario not in data:
            continue

        ax = axes[idx//2, idx%2]
        df = data[scenario]

        ax.plot(df['Time'], df['ErrorY0'], 'g-', linewidth=2, label='Y Error')
        ax.axhline(y=0, color='k', linestyle='--', alpha=0.5)

        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Error (m)')
        ax.set_title(title)
        ax.grid(True, alpha=0.3)

        # Calculate and display RMS error
        rms_error = np.sqrt(np.mean(df['ErrorY0']**2))
        ax.text(0.02, 0.98, f'RMS Error: {rms_error:.3f}m', transform=ax.transAxes,
                verticalalignment='top', bbox=dict(boxstyle='round', facecolor='lightgreen', alpha=0.8))

    plt.tight_layout()
    plt.savefig('05_y_error_analysis.png', dpi=300, bbox_inches='tight')
    plt.close()

def plot_fls_vs_wind_forces(data):
    """Plot FLS correction forces vs Wind forces"""

    # Only plot scenarios with FLS and/or Wind
    scenarios_to_plot = []
    if 'pid_wind' in data:
        scenarios_to_plot.append(('pid_wind', 'Wind Only (No FLS)', False, True))
    if 'pid_fls' in data:
        scenarios_to_plot.append(('pid_fls', 'FLS Only (No Wind)', True, False))
    if 'pid_fls_wind' in data:
        scenarios_to_plot.append(('pid_fls_wind', 'FLS + Wind', True, True))

    if not scenarios_to_plot:
        print("No scenarios with FLS or Wind found for force comparison")
        return

    fig, axes = plt.subplots(len(scenarios_to_plot), 2, figsize=(15, 4*len(scenarios_to_plot)))
    if len(scenarios_to_plot) == 1:
        axes = axes.reshape(1, -1)

    fig.suptitle('Force Analysis - FLS Corrections vs Wind Disturbances', fontsize=16, fontweight='bold')

    for idx, (scenario, title, has_fls, has_wind) in enumerate(scenarios_to_plot):
        df = data[scenario]

        # X-axis forces
        ax_x = axes[idx, 0]
        if has_fls:
            ax_x.plot(df['Time'], df['FLSCorrX0'], 'b-', linewidth=2, label='FLS Correction X')
        if has_wind and 'SimWindX' in df.columns and df['SimWindX'].abs().max() > 0.01:
            ax_x.plot(df['Time'], df['SimWindX'], 'r-', linewidth=2, label='Wind Force X')

        ax_x.set_xlabel('Time (s)')
        ax_x.set_ylabel('Force (N)')
        ax_x.set_title(f'{title} - X Forces')
        ax_x.grid(True, alpha=0.3)
        ax_x.legend()

        # Y-axis forces
        ax_y = axes[idx, 1]
        if has_fls:
            ax_y.plot(df['Time'], df['FLSCorrY0'], 'g-', linewidth=2, label='FLS Correction Y')
        if has_wind and 'SimWindY' in df.columns and df['SimWindY'].abs().max() > 0.01:
            ax_y.plot(df['Time'], df['SimWindY'], 'r-', linewidth=2, label='Wind Force Y')

        ax_y.set_xlabel('Time (s)')
        ax_y.set_ylabel('Force (N)')
        ax_y.set_title(f'{title} - Y Forces')
        ax_y.grid(True, alpha=0.3)
        ax_y.legend()

    plt.tight_layout()
    plt.savefig('06_fls_wind_forces.png', dpi=300, bbox_inches='tight')
    plt.close()

def plot_pid_components(data):
    """Plot PID components (P, I, D terms) for analysis"""

    fig, axes = plt.subplots(2, 2, figsize=(15, 12))
    fig.suptitle('PID Components Analysis - Drone 0', fontsize=16, fontweight='bold')

    scenarios = ['pid_only', 'pid_wind', 'pid_fls', 'pid_fls_wind']
    titles = ['PID Only', 'PID + Wind', 'PID + FLS', 'PID + FLS + Wind']

    for idx, (scenario, title) in enumerate(zip(scenarios, titles)):
        if scenario not in data:
            continue

        ax = axes[idx//2, idx%2]
        df = data[scenario]

        # Plot P, I, D terms for X-axis
        ax.plot(df['Time'], df['PTermX0'], 'r-', linewidth=1.5, label='P Term', alpha=0.8)
        ax.plot(df['Time'], df['ITermX0'], 'g-', linewidth=1.5, label='I Term', alpha=0.8)
        ax.plot(df['Time'], df['DTermX0'], 'b-', linewidth=1.5, label='D Term', alpha=0.8)
        ax.plot(df['Time'], df['PIDOutX0'], 'k--', linewidth=2, label='Total PID Output')

        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Control Signal')
        ax.set_title(f'{title} - X-axis PID Components')
        ax.grid(True, alpha=0.3)
        ax.legend()

    plt.tight_layout()
    plt.savefig('07_pid_components.png', dpi=300, bbox_inches='tight')
    plt.close()

def plot_fls_effectiveness(data):
    """Compare FLS effectiveness - Error reduction"""

    if 'pid_only' not in data or 'pid_fls' not in data:
        print("Need both PID-only and PID+FLS data for effectiveness comparison")
        return

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 6))
    fig.suptitle('FLS Effectiveness - Error Reduction Comparison', fontsize=16, fontweight='bold')

    # X-axis comparison
    df_pid = data['pid_only']
    df_fls = data['pid_fls']

    ax1.plot(df_pid['Time'], np.abs(df_pid['ErrorX0']), 'r-', linewidth=2,
             label='PID Only', alpha=0.8)
    ax1.plot(df_fls['Time'], np.abs(df_fls['ErrorX0']), 'b-', linewidth=2,
             label='PID + FLS', alpha=0.8)

    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Absolute Error (m)')
    ax1.set_title('X-axis Error Comparison')
    ax1.grid(True, alpha=0.3)
    ax1.legend()

    # Calculate improvement
    rms_pid_x = np.sqrt(np.mean(df_pid['ErrorX0']**2))
    rms_fls_x = np.sqrt(np.mean(df_fls['ErrorX0']**2))
    improvement_x = ((rms_pid_x - rms_fls_x) / rms_pid_x) * 100

    ax1.text(0.02, 0.98, f'RMS Improvement: {improvement_x:.1f}%',
             transform=ax1.transAxes, verticalalignment='top',
             bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.8))

    # Y-axis comparison
    ax2.plot(df_pid['Time'], np.abs(df_pid['ErrorY0']), 'r-', linewidth=2,
             label='PID Only', alpha=0.8)
    ax2.plot(df_fls['Time'], np.abs(df_fls['ErrorY0']), 'g-', linewidth=2,
             label='PID + FLS', alpha=0.8)

    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Absolute Error (m)')
    ax2.set_title('Y-axis Error Comparison')
    ax2.grid(True, alpha=0.3)
    ax2.legend()

    # Calculate improvement
    rms_pid_y = np.sqrt(np.mean(df_pid['ErrorY0']**2))
    rms_fls_y = np.sqrt(np.mean(df_fls['ErrorY0']**2))
    improvement_y = ((rms_pid_y - rms_fls_y) / rms_pid_y) * 100

    ax2.text(0.02, 0.98, f'RMS Improvement: {improvement_y:.1f}%',
             transform=ax2.transAxes, verticalalignment='top',
             bbox=dict(boxstyle='round', facecolor='lightgreen', alpha=0.8))

    plt.tight_layout()
    plt.savefig('07_fls_effectiveness.png', dpi=300, bbox_inches='tight')
    plt.close()

def plot_wind_disturbance_effect(data):
    """Compare wind disturbance effects"""

    comparisons = []
    if 'pid_only' in data and 'pid_wind' in data:
        comparisons.append(('pid_only', 'pid_wind', 'PID Only vs PID + Wind'))
    if 'pid_fls' in data and 'pid_fls_wind' in data:
        comparisons.append(('pid_fls', 'pid_fls_wind', 'PID + FLS vs PID + FLS + Wind'))

    if not comparisons:
        print("No suitable data for wind disturbance comparison")
        return

    for no_wind, with_wind, title in comparisons:
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 6))
        fig.suptitle(f'Wind Disturbance Effect: {title}', fontsize=16, fontweight='bold')

        df_no_wind = data[no_wind]
        df_with_wind = data[with_wind]

        # X-axis comparison
        ax1.plot(df_no_wind['Time'], np.abs(df_no_wind['ErrorX0']), 'b-', linewidth=2,
                 label='No Wind', alpha=0.8)
        ax1.plot(df_with_wind['Time'], np.abs(df_with_wind['ErrorX0']), 'r-', linewidth=2,
                 label='With Wind', alpha=0.8)

        ax1.set_xlabel('Time (s)')
        ax1.set_ylabel('Absolute Error (m)')
        ax1.set_title('X-axis Error Comparison')
        ax1.grid(True, alpha=0.3)
        ax1.legend()

        # Y-axis comparison
        ax2.plot(df_no_wind['Time'], np.abs(df_no_wind['ErrorY0']), 'b-', linewidth=2,
                 label='No Wind', alpha=0.8)
        ax2.plot(df_with_wind['Time'], np.abs(df_with_wind['ErrorY0']), 'g-', linewidth=2,
                 label='With Wind', alpha=0.8)

        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Absolute Error (m)')
        ax2.set_title('Y-axis Error Comparison')
        ax2.grid(True, alpha=0.3)
        ax2.legend()

        plt.tight_layout()
        filename = f"08_wind_effect_{no_wind}_vs_{with_wind}.png"
        plt.savefig(filename, dpi=300, bbox_inches='tight')
        plt.close()

def plot_performance_summary(data):
    """Create a performance summary table/chart"""

    metrics = {}
    scenarios = ['pid_only', 'pid_wind', 'pid_fls', 'pid_fls_wind']
    scenario_names = ['PID Only', 'PID + Wind', 'PID + FLS', 'PID + FLS + Wind']

    for scenario, name in zip(scenarios, scenario_names):
        if scenario not in data:
            continue

        df = data[scenario]

        # Calculate performance metrics
        rms_x = np.sqrt(np.mean(df['ErrorX0']**2))
        rms_y = np.sqrt(np.mean(df['ErrorY0']**2))
        max_error_x = np.abs(df['ErrorX0']).max()
        max_error_y = np.abs(df['ErrorY0']).max()

        metrics[name] = {
            'RMS Error X': rms_x,
            'RMS Error Y': rms_y,
            'Max Error X': max_error_x,
            'Max Error Y': max_error_y,
            'Total RMS': np.sqrt(rms_x**2 + rms_y**2)
        }

    # Create bar chart comparison
    fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(15, 10))
    fig.suptitle('Performance Metrics Comparison', fontsize=16, fontweight='bold')

    names = list(metrics.keys())
    rms_x_vals = [metrics[name]['RMS Error X'] for name in names]
    rms_y_vals = [metrics[name]['RMS Error Y'] for name in names]
    max_x_vals = [metrics[name]['Max Error X'] for name in names]
    max_y_vals = [metrics[name]['Max Error Y'] for name in names]

    # RMS X errors
    ax1.bar(names, rms_x_vals, color='skyblue', alpha=0.7)
    ax1.set_ylabel('RMS Error (m)')
    ax1.set_title('RMS Error - X Axis')
    ax1.tick_params(axis='x', rotation=45)

    # RMS Y errors
    ax2.bar(names, rms_y_vals, color='lightgreen', alpha=0.7)
    ax2.set_ylabel('RMS Error (m)')
    ax2.set_title('RMS Error - Y Axis')
    ax2.tick_params(axis='x', rotation=45)

    # Max X errors
    ax3.bar(names, max_x_vals, color='lightcoral', alpha=0.7)
    ax3.set_ylabel('Max Error (m)')
    ax3.set_title('Maximum Error - X Axis')
    ax3.tick_params(axis='x', rotation=45)

    # Max Y errors
    ax4.bar(names, max_y_vals, color='lightsalmon', alpha=0.7)
    ax4.set_ylabel('Max Error (m)')
    ax4.set_title('Maximum Error - Y Axis')
    ax4.tick_params(axis='x', rotation=45)

    plt.tight_layout()
    plt.savefig('08_performance_summary.png', dpi=300, bbox_inches='tight')
    plt.show()

    # Print numerical summary
    print("\n" + "="*60)
    print("PERFORMANCE SUMMARY")
    print("="*60)
    for name, metric in metrics.items():
        print(f"\n{name}:")
        print(f"  RMS Error X: {metric['RMS Error X']:.4f} m")
        print(f"  RMS Error Y: {metric['RMS Error Y']:.4f} m")
        print(f"  Max Error X: {metric['Max Error X']:.4f} m")
        print(f"  Max Error Y: {metric['Max Error Y']:.4f} m")
        print(f"  Total RMS:   {metric['Total RMS']:.4f} m")

def main():
    """Main execution function"""
    print("Multi-Drone Formation Control Analysis")
    print("=" * 50)

    # Find CSV files
    csv_files = find_csv_files()

    if not csv_files:
        print("No CSV files found! Make sure they are in 'simulation_outputs/' directory or current directory.")
        return

    # Load data
    data = load_simulation_data(csv_files)

    if not data:
        print("No data loaded successfully!")
        return

    print(f"\nGenerating plots for {len(data)} scenarios...")

    # Generate all plots
    plot_drone_trajectories(data)
    plot_position_tracking(data)
    plot_error_analysis(data)
    plot_fls_vs_wind_forces(data)
    plot_pid_components(data)
    plot_fls_effectiveness(data)
    plot_wind_disturbance_effect(data)
    plot_performance_summary(data)

    print("\n" + "="*50)
    print("All plots generated successfully!")
    print("Generated files:")
    for i in range(1, 9):
        filename = f"{i:02d}_*.png"
        print(f"  {filename}")

if __name__ == "__main__":
    main()
