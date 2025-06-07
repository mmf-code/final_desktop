#!/usr/bin/env python3
"""
Simple PID vs FLS Comparison Script

This script compares 4 CSV simulation files with the same PID parameters:
- Kp=3.1, Ki=0.4, Kd=2.2
But different FLS and Wind settings to show the effects.
"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path
import glob


def find_matching_files():
    """Find CSV files with Kp=3.1, Ki=0.4, Kd=2.2"""
    pattern = "multi_agent_sim_Kp3.100_Ki0.400_Kd2.200_*.csv"

    # Get the directory where this script is located
    script_dir = Path(__file__).parent
    simulation_outputs_dir = script_dir / "simulation_outputs"

    # Look for files in the simulation_outputs directory
    files = glob.glob(str(simulation_outputs_dir / pattern))

    file_dict = {}
    for file in files:
        if "FLS_OFF_WIND_ON" in file:
            file_dict["PID_only_Wind_ON"] = file
        elif "FLS_ON_WIND_ON" in file:
            file_dict["PID_FLS_Wind_ON"] = file
        elif "FLS_OFF_WIND_OFF" in file:
            file_dict["PID_only_Wind_OFF"] = file
        elif "FLS_ON_WIND_OFF" in file:
            file_dict["PID_FLS_Wind_OFF"] = file

    return file_dict


def calculate_errors(df):
    """Calculate position errors for all drones"""
    errors = {}
    num_drones = 3  # Based on your system

    for i in range(num_drones):
        if f'ErrorX{i}' in df.columns and f'ErrorY{i}' in df.columns:
            # Total position error
            errors[f'drone_{i}_total_error'] = np.sqrt(
                df[f'ErrorX{i}']**2 + df[f'ErrorY{i}']**2
            )
            # Final error (last 10% of simulation)
            final_10_percent = int(len(df) * 0.9)
            errors[f'drone_{i}_final_error'] = (
                errors[f'drone_{i}_total_error'][final_10_percent:].mean()
            )

    # Overall system error
    total_error = np.zeros(len(df))
    for i in range(num_drones):
        if f'drone_{i}_total_error' in errors:
            total_error += errors[f'drone_{i}_total_error']
    errors['system_average'] = total_error / num_drones
    errors['system_final_error'] = (
        errors['system_average'][final_10_percent:].mean()
    )

    return errors


def calculate_formation_metrics(df):
    """Calculate formation-specific metrics"""
    metrics = {}
    num_drones = 3

    # Formation centroid
    centroid_x = np.zeros(len(df))
    centroid_y = np.zeros(len(df))

    for i in range(num_drones):
        if f'CurrentX{i}' in df.columns and f'CurrentY{i}' in df.columns:
            centroid_x += df[f'CurrentX{i}']
            centroid_y += df[f'CurrentY{i}']

    metrics['centroid_x'] = centroid_x / num_drones
    metrics['centroid_y'] = centroid_y / num_drones

    # Target centroid (if available)
    target_centroid_x = np.zeros(len(df))
    target_centroid_y = np.zeros(len(df))

    for i in range(num_drones):
        if f'TargetX{i}' in df.columns and f'TargetY{i}' in df.columns:
            target_centroid_x += df[f'TargetX{i}']
            target_centroid_y += df[f'TargetY{i}']

    metrics['target_centroid_x'] = target_centroid_x / num_drones
    metrics['target_centroid_y'] = target_centroid_y / num_drones

    # Formation area (using triangle area for 3 drones)
    if all(f'CurrentX{i}' in df.columns for i in range(3)):
        x0, y0 = df['CurrentX0'], df['CurrentY0']
        x1, y1 = df['CurrentX1'], df['CurrentY1']
        x2, y2 = df['CurrentX2'], df['CurrentY2']

        # Triangle area formula: 0.5 * |x0(y1-y2) + x1(y2-y0) + x2(y0-y1)|
        metrics['formation_area'] = 0.5 * np.abs(
            x0*(y1-y2) + x1*(y2-y0) + x2*(y0-y1)
        )

        # Formation perimeter
        side1 = np.sqrt((x1-x0)**2 + (y1-y0)**2)
        side2 = np.sqrt((x2-x1)**2 + (y2-y1)**2)
        side3 = np.sqrt((x0-x2)**2 + (y0-y2)**2)
        metrics['formation_perimeter'] = side1 + side2 + side3

    # Control command magnitudes
    for i in range(num_drones):
        if f'FinalCmdX{i}' in df.columns and f'FinalCmdY{i}' in df.columns:
            metrics[f'cmd_magnitude_{i}'] = np.sqrt(
                df[f'FinalCmdX{i}']**2 + df[f'FinalCmdY{i}']**2
            )

    return metrics


def create_simple_comparison_plot(file_dict):
    """Create comprehensive comparison plots in 3 separate figures"""
    colors = ['blue', 'red', 'green', 'orange']
    scenarios = list(file_dict.keys())

    # Load data for all scenarios
    data = {}
    errors = {}
    formation_metrics = {}
    for scenario, file_path in file_dict.items():
        print(f"Loading: {scenario}...")
        df = pd.read_csv(file_path)
        data[scenario] = df
        errors[scenario] = calculate_errors(df)
        formation_metrics[scenario] = calculate_formation_metrics(df)

    # ========================================
    # FIGURE 1: BASIC PERFORMANCE ANALYSIS
    # ========================================
    fig1, axes1 = plt.subplots(2, 2, figsize=(15, 10))
    fig1.suptitle('Basic Performance Analysis (Kp=3.1, Ki=0.4, Kd=2.2)',
                  fontsize=16, fontweight='bold')

    # Plot 1: System Average Error Over Time
    ax1 = axes1[0, 0]
    for i, (scenario, df) in enumerate(data.items()):
        ax1.plot(df['Time'], errors[scenario]['system_average'],
                 color=colors[i], label=scenario, linewidth=2)
    ax1.set_xlabel('Time [s]')
    ax1.set_ylabel('Average Position Error [m]')
    ax1.set_title('System Position Error Over Time')
    ax1.grid(True, alpha=0.3)
    ax1.legend()

    # Plot 2: Final Error Comparison
    ax2 = axes1[0, 1]
    final_errors = [errors[scenario]['system_final_error']
                    for scenario in scenarios]
    bars = ax2.bar(range(len(scenarios)), final_errors, color=colors)
    ax2.set_xticks(range(len(scenarios)))
    ax2.set_xticklabels([s.replace('_', '\n') for s in scenarios],
                        rotation=0, ha='center')
    ax2.set_ylabel('Final Average Error [m]')
    ax2.set_title('Final Tracking Accuracy Comparison')
    ax2.grid(True, alpha=0.3, axis='y')

    # Add value labels on bars
    for bar, value in zip(bars, final_errors):
        ax2.text(bar.get_x() + bar.get_width()/2, bar.get_height(),
                 f'{value:.3f}', ha='center', va='bottom')

    # Plot 3: Drone 0 Position Tracking (X-axis)
    ax3 = axes1[1, 0]
    for i, (scenario, df) in enumerate(data.items()):
        ax3.plot(df['Time'], df['CurrentX0'], color=colors[i],
                 label=f'{scenario} Actual', linewidth=2)
    if 'TargetX0' in list(data.values())[0].columns:
        ax3.plot(list(data.values())[0]['Time'],
                 list(data.values())[0]['TargetX0'],
                 'k--', label='Target', linewidth=2)
    ax3.set_xlabel('Time [s]')
    ax3.set_ylabel('X Position [m]')
    ax3.set_title('Drone 0 X-axis Tracking')
    ax3.grid(True, alpha=0.3)
    ax3.legend()

    # Plot 4: Wind Effect Analysis
    ax4 = axes1[1, 1]
    wind_scenarios = [s for s in scenarios if 'Wind_ON' in s]
    no_wind_scenarios = [s for s in scenarios if 'Wind_OFF' in s]

    if wind_scenarios and no_wind_scenarios:
        pid_wind_on = next((s for s in wind_scenarios
                            if 'PID_only' in s), None)
        pid_wind_off = next((s for s in no_wind_scenarios
                             if 'PID_only' in s), None)

        if pid_wind_on and pid_wind_off:
            ax4.plot(data[pid_wind_on]['Time'],
                     errors[pid_wind_on]['system_average'],
                     'blue', label='PID + Wind ON', linewidth=2)
            ax4.plot(data[pid_wind_off]['Time'],
                     errors[pid_wind_off]['system_average'],
                     'green', label='PID + Wind OFF', linewidth=2)

        fls_wind_on = next((s for s in wind_scenarios
                            if 'PID_FLS' in s), None)
        fls_wind_off = next((s for s in no_wind_scenarios
                             if 'PID_FLS' in s), None)

        if fls_wind_on and fls_wind_off:
            ax4.plot(data[fls_wind_on]['Time'],
                     errors[fls_wind_on]['system_average'],
                     'red', label='PID+FLS + Wind ON', linewidth=2)
            ax4.plot(data[fls_wind_off]['Time'],
                     errors[fls_wind_off]['system_average'],
                     'orange', label='PID+FLS + Wind OFF', linewidth=2)

        ax4.set_xlabel('Time [s]')
        ax4.set_ylabel('Average Position Error [m]')
        ax4.set_title('Wind Effect on Control Performance')
        ax4.grid(True, alpha=0.3)
        ax4.legend()
    else:
        ax4.text(0.5, 0.5, 'Wind effect analysis\nrequires both '
                 'Wind ON/OFF data',
                 transform=ax4.transAxes, ha='center', va='center',
                 fontsize=12)
        ax4.set_title('Wind Effect Analysis')

    plt.tight_layout()
    plt.savefig('figure1_basic_performance.png', dpi=300,
                bbox_inches='tight')
    plt.show()

    # ========================================
    # FIGURE 2: ERROR ANALYSIS
    # ========================================
    fig2, axes2 = plt.subplots(2, 2, figsize=(15, 10))
    fig2.suptitle('Error Analysis (Kp=3.1, Ki=0.4, Kd=2.2)',
                  fontsize=16, fontweight='bold')

    # Plot 5: X-Axis Error Breakdown
    ax5 = axes2[0, 0]
    for i, scenario in enumerate(scenarios):
        df = data[scenario]
        if 'ErrorX0' in df.columns:
            avg_error_x = ((df['ErrorX0'] + df['ErrorX1'] +
                            df['ErrorX2']) / 3)
            ax5.plot(df['Time'], avg_error_x, color=colors[i],
                     label=scenario, linewidth=2)
    ax5.set_xlabel('Time [s]')
    ax5.set_ylabel('Average X Error [m]')
    ax5.set_title('X-Axis Error Breakdown')
    ax5.grid(True, alpha=0.3)
    ax5.legend()

    # Plot 6: Y-Axis Error Breakdown
    ax6 = axes2[0, 1]
    for i, scenario in enumerate(scenarios):
        df = data[scenario]
        if 'ErrorY0' in df.columns:
            avg_error_y = ((df['ErrorY0'] + df['ErrorY1'] +
                            df['ErrorY2']) / 3)
            ax6.plot(df['Time'], avg_error_y, color=colors[i],
                     label=scenario, linewidth=2)
    ax6.set_xlabel('Time [s]')
    ax6.set_ylabel('Average Y Error [m]')
    ax6.set_title('Y-Axis Error Breakdown')
    ax6.grid(True, alpha=0.3)
    ax6.legend()

    # Plot 7: Error Convergence Rate Analysis
    ax7 = axes2[1, 0]
    for i, scenario in enumerate(scenarios):
        error_signal = errors[scenario]['system_average']
        # Calculate error rate (smoothed derivative)
        error_rate = np.gradient(error_signal)
        # Smooth the rate signal
        window = min(50, len(error_rate) // 10)
        if window > 1:
            error_rate_smooth = np.convolve(
                error_rate, np.ones(window)/window, mode='same')
        else:
            error_rate_smooth = error_rate
        ax7.plot(data[scenario]['Time'], error_rate_smooth,
                 color=colors[i], label=scenario, linewidth=2)
    ax7.set_xlabel('Time [s]')
    ax7.set_ylabel('Error Rate [m/s]')
    ax7.set_title('Error Convergence Rate')
    ax7.grid(True, alpha=0.3)
    ax7.legend()

    # Plot 8: Formation Centroid Tracking
    ax8 = axes2[1, 1]
    for i, scenario in enumerate(scenarios):
        metrics = formation_metrics[scenario]
        if 'centroid_x' in metrics and 'target_centroid_x' in metrics:
            centroid_error = np.sqrt(
                (metrics['centroid_x'] - metrics['target_centroid_x'])**2 +
                (metrics['centroid_y'] - metrics['target_centroid_y'])**2)
            ax8.plot(data[scenario]['Time'], centroid_error,
                     color=colors[i], label=scenario, linewidth=2)
    ax8.set_xlabel('Time [s]')
    ax8.set_ylabel('Centroid Tracking Error [m]')
    ax8.set_title('Formation Centroid Tracking')
    ax8.grid(True, alpha=0.3)
    ax8.legend()

    plt.tight_layout()
    plt.savefig('figure2_error_analysis.png', dpi=300, bbox_inches='tight')
    plt.show()

    # ========================================
    # FIGURE 3: FORMATION & CONTROL ANALYSIS
    # ========================================
    fig3, axes3 = plt.subplots(2, 2, figsize=(15, 10))
    fig3.suptitle('Formation & Control Analysis (Kp=3.1, Ki=0.4, Kd=2.2)',
                  fontsize=16, fontweight='bold')

    # Plot 9: Formation Shape Stability (Area)
    ax9 = axes3[0, 0]
    for i, scenario in enumerate(scenarios):
        if 'formation_area' in formation_metrics[scenario]:
            ax9.plot(data[scenario]['Time'],
                     formation_metrics[scenario]['formation_area'],
                     color=colors[i], label=scenario, linewidth=2)
    ax9.set_xlabel('Time [s]')
    ax9.set_ylabel('Formation Area [m²]')
    ax9.set_title('Formation Shape Stability')
    ax9.grid(True, alpha=0.3)
    ax9.legend()

    # Plot 10: Control Command Magnitude Comparison
    ax10 = axes3[0, 1]
    for i, scenario in enumerate(scenarios):
        if 'cmd_magnitude_0' in formation_metrics[scenario]:
            avg_cmd = ((formation_metrics[scenario]['cmd_magnitude_0'] +
                        formation_metrics[scenario]['cmd_magnitude_1'] +
                        formation_metrics[scenario]['cmd_magnitude_2']) / 3)
            ax10.plot(data[scenario]['Time'], avg_cmd, color=colors[i],
                      label=scenario, linewidth=2)
    ax10.set_xlabel('Time [s]')
    ax10.set_ylabel('Average Command Magnitude')
    ax10.set_title('Control Command Magnitude')
    ax10.grid(True, alpha=0.3)
    ax10.legend()

    # Plot 11: Drone Trajectory Paths (2D)
    ax11 = axes3[1, 0]
    for i, scenario in enumerate(scenarios):
        df = data[scenario]
        # Plot all 3 drone paths
        for drone in range(3):
            if f'CurrentX{drone}' in df.columns:
                alpha = 0.7 if scenario.endswith('_ON') else 0.9
                label = (f'{scenario} D{drone}' if drone == 0 else "")
                ax11.plot(df[f'CurrentX{drone}'], df[f'CurrentY{drone}'],
                          color=colors[i], alpha=alpha, linewidth=1,
                          label=label)
    ax11.set_xlabel('X Position [m]')
    ax11.set_ylabel('Y Position [m]')
    ax11.set_title('Drone Flight Trajectories')
    ax11.grid(True, alpha=0.3)
    ax11.legend()
    ax11.axis('equal')

    # Plot 12: Formation Perimeter Stability
    ax12 = axes3[1, 1]
    for i, scenario in enumerate(scenarios):
        if 'formation_perimeter' in formation_metrics[scenario]:
            ax12.plot(data[scenario]['Time'],
                      formation_metrics[scenario]['formation_perimeter'],
                      color=colors[i], label=scenario, linewidth=2)
    ax12.set_xlabel('Time [s]')
    ax12.set_ylabel('Formation Perimeter [m]')
    ax12.set_title('Formation Perimeter Stability')
    ax12.grid(True, alpha=0.3)
    ax12.legend()

    plt.tight_layout()
    plt.savefig('figure3_formation_control.png', dpi=300,
                bbox_inches='tight')
    plt.show()

    return errors


def print_summary(file_dict, errors):
    """Print simple text summary"""
    print("\n" + "="*60)
    print("PID vs FLS COMPARISON SUMMARY")
    print("="*60)
    print("PID Parameters: Kp=3.1, Ki=0.4, Kd=2.2")
    print("Files analyzed: {}".format(len(file_dict)))
    print()

    print("FINAL TRACKING ACCURACY (lower is better):")
    print("-" * 40)
    for scenario in file_dict.keys():
        error = errors[scenario]['system_final_error']
        print(f"{scenario:25}: {error:.4f} m")

    print("\nFLS EFFECTIVENESS:")
    print("-" * 20)

    # Compare FLS effect with wind
    if 'PID_only_Wind_ON' in errors and 'PID_FLS_Wind_ON' in errors:
        pid_error = errors['PID_only_Wind_ON']['system_final_error']
        fls_error = errors['PID_FLS_Wind_ON']['system_final_error']
        improvement = ((pid_error - fls_error) / pid_error) * 100
        print("With Wind: FLS improves accuracy by {:.1f}%".format(
            improvement))

    # Compare FLS effect without wind
    if 'PID_only_Wind_OFF' in errors and 'PID_FLS_Wind_OFF' in errors:
        pid_error = errors['PID_only_Wind_OFF']['system_final_error']
        fls_error = errors['PID_FLS_Wind_OFF']['system_final_error']
        improvement = ((pid_error - fls_error) / pid_error) * 100
        print("Without Wind: FLS improves accuracy by {:.1f}%".format(
            improvement))


def main():
    """Main function"""
    print("🎯 Simple PID vs FLS Comparison")
    print("=" * 40)

    # Find matching files
    file_dict = find_matching_files()

    if len(file_dict) < 2:
        print("❌ Not enough matching files found! Need at least 2, "
              "found {}".format(len(file_dict)))
        print("Available files should have pattern: "
              "Kp3.100_Ki0.400_Kd2.200")
        return

    print("✅ Found {} matching simulation files:".format(len(file_dict)))
    for scenario, file_path in file_dict.items():
        print("   {}: {}".format(scenario, Path(file_path).name))

    print("\n🔍 Analyzing data...")

    # Create plots and calculate errors
    errors = create_simple_comparison_plot(file_dict)

    # Print summary
    print_summary(file_dict, errors)

    print("\n✅ Analysis complete!")
    print("📊 Comprehensive plots saved as:")
    print("  - figure1_basic_performance.png")
    print("  - figure2_error_analysis.png")
    print("  - figure3_formation_control.png")


if __name__ == "__main__":
    main() 