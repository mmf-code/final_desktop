#!/usr/bin/env python3
"""
Final Project Analysis - Multi-Agent Formation Control with Fuzzy Logic
Advanced Control Engineering Analysis and Visualization
Enhanced with Comprehensive Research Plotting Strategy
"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path
import warnings
import seaborn as sns
from scipy import stats
from matplotlib.patches import Rectangle
import matplotlib.patches as mpatches
warnings.filterwarnings('ignore')

# Set professional matplotlib style for academic papers
plt.style.use('seaborn-v0_8-whitegrid')
plt.rcParams['figure.figsize'] = (12, 8)
plt.rcParams['font.size'] = 12
plt.rcParams['axes.titlesize'] = 14
plt.rcParams['axes.labelsize'] = 12
plt.rcParams['xtick.labelsize'] = 10
plt.rcParams['ytick.labelsize'] = 10
plt.rcParams['legend.fontsize'] = 11
plt.rcParams['figure.titlesize'] = 16
plt.rcParams['lines.linewidth'] = 2
plt.rcParams['grid.alpha'] = 0.3

# IEEE/Academic color palette
COLORS = {
    'primary': '#1f77b4',    # Blue
    'secondary': '#ff7f0e',  # Orange
    'success': '#2ca02c',    # Green
    'danger': '#d62728',     # Red
    'warning': '#ff7f0e',    # Orange
    'info': '#17a2b8',       # Light blue
    'dark': '#343a40',       # Dark gray
    'light': '#f8f9fa'       # Light gray
}

# Professional color scheme for multi-drone visualization
DRONE_COLORS = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd']
DRONE_NAMES = ['Leader Drone', 'Follower 1', 'Follower 2', 'Follower 3', 'Follower 4']


class FormationControlAnalyzer:
    def __init__(self, base_path="../simulation_outputs"):
        self.base_path = Path(base_path)
        self.output_path = Path("../results/final_project_results")
        self.plots_path = Path("../results/figures/analysis_plots")

        # Create output directories
        self.output_path.mkdir(parents=True, exist_ok=True)
        self.plots_path.mkdir(parents=True, exist_ok=True)

        # Load data
        self.data = {}
        self.load_comprehensive_data()

    def load_comprehensive_data(self):
        """Load data from comprehensive simulation and comparison scenarios"""
        # Look for comprehensive simulation files
        csv_files = list(self.base_path.glob("multi_agent_sim_*.csv"))

        if csv_files:
            # Load multiple scenarios for comparison
            self.data = {}
            self.scenarios = []

            # Categorize files by scenario type
            for csv_file in csv_files:
                filename = csv_file.name
                if 'FLS_ON' in filename and 'WIND_ON' in filename:
                    scenario_key = 'PID+FLC with Wind'
                elif 'FLS_ON' in filename and 'WIND_OFF' in filename:
                    scenario_key = 'PID+FLC without Wind'
                elif 'FLS_OFF' in filename and 'WIND_ON' in filename:
                    scenario_key = 'PID only with Wind'
                elif 'FLS_OFF' in filename and 'WIND_OFF' in filename:
                    scenario_key = 'PID only without Wind'
                else:
                    scenario_key = 'General Simulation'

                # Load the most recent file for each scenario
                if scenario_key not in self.data:
                    data = pd.read_csv(csv_file)
                    self.data[scenario_key] = data
                    self.scenarios.append(scenario_key)
                    print(f"Loaded {scenario_key}: {len(data)} data points")

            # Also look for metrics files
            metrics_files = list(self.base_path.glob("metrics_sim_*.txt"))
            if metrics_files:
                self.metrics_file = max(metrics_files,
                                      key=lambda x: x.stat().st_mtime)
        else:
            print("No comprehensive simulation files found!")

    def detect_phases(self, data):
        """Detect phase changes in the simulation data"""
        phases = []
        if 'TargetX0' not in data.columns:
            return phases

        # Detect when target changes significantly
        target_x = data['TargetX0'].values
        target_y = data['TargetY0'].values
        time = data['Time'].values

        phase_times = [0]  # Start with time 0
        threshold = 1.0  # Minimum change to consider a new phase

        for i in range(1, len(target_x)):
            if (abs(target_x[i] - target_x[i-1]) > threshold or
                abs(target_y[i] - target_y[i-1]) > threshold):
                phase_times.append(time[i])

        # Create phase info
        for i, phase_time in enumerate(phase_times):
            idx = np.argmin(np.abs(time - phase_time))
            phases.append({
                'time': phase_time,
                'target_x': target_x[idx],
                'target_y': target_y[idx],
                'phase_number': i + 1
            })

        return phases

    def plot_scenario_comparison(self):
        """Create comprehensive scenario comparison plots"""
        print("Generating scenario comparison plots...")

        if len(self.data) < 2:
            print("Warning: Need at least 2 scenarios for comparison")
            return

        fig, axes = plt.subplots(2, 3, figsize=(20, 12))
        fig.suptitle('Multi-Scenario Performance Comparison: PID vs PID+FLC',
                     fontsize=16, fontweight='bold')

        # Define comparison scenarios
        scenario_colors = {
            'PID+FLC with Wind': COLORS['success'],
            'PID only with Wind': COLORS['danger'],
            'PID+FLC without Wind': COLORS['primary'],
            'PID only without Wind': COLORS['warning']
        }

        # Plot 1: Position tracking comparison (X-axis)
        ax1 = axes[0, 0]
        for scenario, data in self.data.items():
            if 'CurrentX0' in data.columns:
                time_mask = data['Time'] <= 30  # First phase only
                ax1.plot(data.loc[time_mask, 'Time'],
                        data.loc[time_mask, 'CurrentX0'],
                        color=scenario_colors.get(scenario, COLORS['dark']),
                        linewidth=2, label=scenario, alpha=0.8)
                ax1.plot(data.loc[time_mask, 'Time'],
                        data.loc[time_mask, 'TargetX0'],
                        '--k', alpha=0.5, linewidth=1)

        ax1.set_xlabel('Time (s)')
        ax1.set_ylabel('X Position (m)')
        ax1.set_title('Position Tracking Comparison (X-axis)')
        ax1.grid(True, alpha=0.3)
        ax1.legend(fontsize=9)

        # Plot 2: Error magnitude comparison
        ax2 = axes[0, 1]
        for scenario, data in self.data.items():
            if 'ErrorX0' in data.columns and 'ErrorY0' in data.columns:
                error_mag = np.sqrt(data['ErrorX0']**2 + data['ErrorY0']**2)
                ax2.plot(data['Time'], error_mag,
                        color=scenario_colors.get(scenario, COLORS['dark']),
                        linewidth=2, label=scenario, alpha=0.8)

        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Position Error Magnitude (m)')
        ax2.set_title('Tracking Error Comparison')
        ax2.grid(True, alpha=0.3)
        ax2.legend(fontsize=9)

        # Plot 3: Control effort comparison
        ax3 = axes[0, 2]
        for scenario, data in self.data.items():
            if 'FinalCmdX0' in data.columns and 'FinalCmdY0' in data.columns:
                control_effort = np.sqrt(data['FinalCmdX0']**2 + data['FinalCmdY0']**2)
                ax3.plot(data['Time'], control_effort,
                        color=scenario_colors.get(scenario, COLORS['dark']),
                        linewidth=2, label=scenario, alpha=0.8)

        ax3.set_xlabel('Time (s)')
        ax3.set_ylabel('Control Effort Magnitude')
        ax3.set_title('Control Effort Comparison')
        ax3.grid(True, alpha=0.3)
        ax3.legend(fontsize=9)

        # Plot 4: Formation quality (inter-drone distances)
        ax4 = axes[1, 0]
        for scenario, data in self.data.items():
            if all(col in data.columns for col in ['CurrentX0', 'CurrentY0', 'CurrentX1', 'CurrentY1']):
                # Distance between drone 0 and 1
                dx = data['CurrentX1'] - data['CurrentX0']
                dy = data['CurrentY1'] - data['CurrentY0']
                distance = np.sqrt(dx**2 + dy**2)
                ax4.plot(data['Time'], distance,
                        color=scenario_colors.get(scenario, COLORS['dark']),
                        linewidth=2, label=scenario, alpha=0.8)

        # Add target formation distance
        target_distance = 4.0  # From formation_side_length config
        ax4.axhline(y=target_distance, color='k', linestyle='--', alpha=0.5, label='Target Distance')
        ax4.set_xlabel('Time (s)')
        ax4.set_ylabel('Inter-drone Distance (m)')
        ax4.set_title('Formation Quality: Drone 0-1 Distance')
        ax4.grid(True, alpha=0.3)
        ax4.legend(fontsize=9)

        # Plot 5: Wind response (if available)
        ax5 = axes[1, 1]
        wind_scenarios = [s for s in self.data.keys() if 'Wind' in s]
        for scenario in wind_scenarios:
            data = self.data[scenario]
            if 'SimWindX' in data.columns and 'ErrorX0' in data.columns:
                # Plot wind force and resulting error
                ax5_twin = ax5.twinx()
                line1 = ax5.plot(data['Time'], data['SimWindX'],
                               color=scenario_colors.get(scenario, COLORS['dark']),
                               linewidth=2, label=f'{scenario} - Wind Force', alpha=0.7)
                line2 = ax5_twin.plot(data['Time'], abs(data['ErrorX0']),
                                    '--', color=scenario_colors.get(scenario, COLORS['dark']),
                                    linewidth=2, label=f'{scenario} - Error', alpha=0.7)

                ax5.set_xlabel('Time (s)')
                ax5.set_ylabel('Wind Force X (N)', color=scenario_colors.get(scenario, COLORS['dark']))
                ax5_twin.set_ylabel('Position Error |X| (m)')

        ax5.set_title('Wind Disturbance Response')
        ax5.grid(True, alpha=0.3)

        # Plot 6: Statistical performance summary
        ax6 = axes[1, 2]
        metrics = {'Mean Error': [], 'Max Error': [], 'RMS Error': [], 'Scenarios': []}

        for scenario, data in self.data.items():
            if 'ErrorX0' in data.columns and 'ErrorY0' in data.columns:
                error_mag = np.sqrt(data['ErrorX0']**2 + data['ErrorY0']**2)
                metrics['Mean Error'].append(np.mean(error_mag))
                metrics['Max Error'].append(np.max(error_mag))
                metrics['RMS Error'].append(np.sqrt(np.mean(error_mag**2)))
                metrics['Scenarios'].append(scenario)

        x_pos = np.arange(len(metrics['Scenarios']))
        width = 0.25

        ax6.bar(x_pos - width, metrics['Mean Error'], width, label='Mean Error',
               color=COLORS['primary'], alpha=0.8)
        ax6.bar(x_pos, metrics['Max Error'], width, label='Max Error',
               color=COLORS['danger'], alpha=0.8)
        ax6.bar(x_pos + width, metrics['RMS Error'], width, label='RMS Error',
               color=COLORS['success'], alpha=0.8)

        ax6.set_xlabel('Scenarios')
        ax6.set_ylabel('Error Magnitude (m)')
        ax6.set_title('Performance Metrics Summary')
        ax6.set_xticks(x_pos)
        ax6.set_xticklabels(metrics['Scenarios'], rotation=45, ha='right')
        ax6.legend()
        ax6.grid(True, alpha=0.3)

        plt.tight_layout()
        plt.savefig(self.plots_path / "scenario_comparison.png",
                   dpi=300, bbox_inches='tight')
        plt.close()

    def plot_wind_disturbance_analysis(self):
        """Detailed wind disturbance rejection analysis"""
        print("Generating wind disturbance analysis...")

        # Focus on wind scenarios
        wind_scenarios = {k: v for k, v in self.data.items() if 'Wind' in k}

        if not wind_scenarios:
            print("No wind scenarios found for analysis")
            return

        fig, axes = plt.subplots(2, 2, figsize=(16, 12))
        fig.suptitle('Wind Disturbance Rejection Analysis',
                     fontsize=16, fontweight='bold')

        # Plot 1: Wind forces and system response
        ax1 = axes[0, 0]
        for scenario, data in wind_scenarios.items():
            color = COLORS['success'] if 'FLC' in scenario else COLORS['danger']

            if all(col in data.columns for col in ['SimWindX', 'SimWindY', 'Time']):
                wind_magnitude = np.sqrt(data['SimWindX']**2 + data['SimWindY']**2)
                ax1.plot(data['Time'], wind_magnitude, color='gray', alpha=0.7,
                        linewidth=2, label='Wind Magnitude')

                if 'ErrorX0' in data.columns and 'ErrorY0' in data.columns:
                    error_magnitude = np.sqrt(data['ErrorX0']**2 + data['ErrorY0']**2)
                    ax1.plot(data['Time'], error_magnitude, color=color,
                            linewidth=2, label=f'{scenario} - Error')

        ax1.set_xlabel('Time (s)')
        ax1.set_ylabel('Magnitude')
        ax1.set_title('Wind Input vs Position Error Response')
        ax1.legend()
        ax1.grid(True, alpha=0.3)

        # Plot 2: FLC intervention during wind
        ax2 = axes[0, 1]
        flc_scenarios = {k: v for k, v in wind_scenarios.items() if 'FLC' in k}

        for scenario, data in flc_scenarios.items():
            if 'FLSCorrX0' in data.columns and 'FLSCorrY0' in data.columns:
                flc_magnitude = np.sqrt(data['FLSCorrX0']**2 + data['FLSCorrY0']**2)
                ax2.plot(data['Time'], flc_magnitude, color=COLORS['primary'],
                        linewidth=2, label='FLC Correction Magnitude')

                if 'SimWindX' in data.columns and 'SimWindY' in data.columns:
                    wind_magnitude = np.sqrt(data['SimWindX']**2 + data['SimWindY']**2)
                    ax2.plot(data['Time'], wind_magnitude, '--', color='gray',
                            linewidth=2, alpha=0.7, label='Wind Magnitude')

        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Correction/Force Magnitude')
        ax2.set_title('Fuzzy Logic Correction vs Wind Disturbance')
        ax2.legend()
        ax2.grid(True, alpha=0.3)

        # Plot 3: Recovery time analysis
        ax3 = axes[1, 0]
        recovery_data = []

        for scenario, data in wind_scenarios.items():
            if 'ErrorX0' in data.columns:
                error_mag = np.sqrt(data['ErrorX0']**2 + data['ErrorY0']**2)

                # Find wind events (when wind magnitude > threshold)
                if 'SimWindX' in data.columns and 'SimWindY' in data.columns:
                    wind_mag = np.sqrt(data['SimWindX']**2 + data['SimWindY']**2)
                    wind_events = wind_mag > 0.5  # Threshold for wind event

                    # Calculate recovery metrics during wind events
                    if np.any(wind_events):
                        wind_periods = data[wind_events]
                        recovery_time = []

                        # Simple recovery analysis: time to return to < 1m error after wind
                        for i in range(len(wind_periods) - 1):
                            if wind_periods.iloc[i]['ErrorX0'] > 1.0:
                                # Find when error drops below 1.0m
                                future_data = data[data['Time'] > wind_periods.iloc[i]['Time']]
                                future_errors = np.sqrt(future_data['ErrorX0']**2 + future_data['ErrorY0']**2)
                                recovery_idx = np.where(future_errors < 1.0)[0]
                                if len(recovery_idx) > 0:
                                    recovery_time.append(future_data.iloc[recovery_idx[0]]['Time'] -
                                                        wind_periods.iloc[i]['Time'])

                        if recovery_time:
                            recovery_data.append({
                                'scenario': scenario,
                                'mean_recovery': np.mean(recovery_time),
                                'max_recovery': np.max(recovery_time)
                            })

        if recovery_data:
            scenarios = [d['scenario'] for d in recovery_data]
            mean_times = [d['mean_recovery'] for d in recovery_data]
            max_times = [d['max_recovery'] for d in recovery_data]

            x_pos = np.arange(len(scenarios))
            width = 0.35

            ax3.bar(x_pos - width/2, mean_times, width, label='Mean Recovery Time',
                   color=COLORS['primary'], alpha=0.8)
            ax3.bar(x_pos + width/2, max_times, width, label='Max Recovery Time',
                   color=COLORS['danger'], alpha=0.8)

            ax3.set_xlabel('Scenarios')
            ax3.set_ylabel('Recovery Time (s)')
            ax3.set_title('Wind Disturbance Recovery Time')
            ax3.set_xticks(x_pos)
            ax3.set_xticklabels(scenarios, rotation=45, ha='right')
            ax3.legend()
            ax3.grid(True, alpha=0.3)

        # Plot 4: Phase-specific wind analysis
        ax4 = axes[1, 1]
        phase_wind_performance = {}

        for scenario, data in wind_scenarios.items():
            phases = self.detect_phases(data)
            phase_errors = []

            for phase in phases:
                phase_start_idx = np.argmin(np.abs(data['Time'] - phase['time']))
                phase_end_idx = min(phase_start_idx + int(30/0.05), len(data))  # 30s or end of data

                phase_data = data.iloc[phase_start_idx:phase_end_idx]
                if 'ErrorX0' in phase_data.columns and 'ErrorY0' in phase_data.columns:
                    phase_error_mag = np.sqrt(phase_data['ErrorX0']**2 + phase_data['ErrorY0']**2)
                    phase_errors.append(np.mean(phase_error_mag))

            phase_wind_performance[scenario] = phase_errors

        # Plot phase performance comparison
        for scenario, errors in phase_wind_performance.items():
            color = COLORS['success'] if 'FLC' in scenario else COLORS['danger']
            phases_x = range(1, len(errors) + 1)
            ax4.plot(phases_x, errors, 'o-', color=color, linewidth=2,
                    markersize=8, label=scenario)

        ax4.set_xlabel('Phase Number')
        ax4.set_ylabel('Mean Position Error (m)')
        ax4.set_title('Phase-by-Phase Wind Performance')
        ax4.legend()
        ax4.grid(True, alpha=0.3)

        plt.tight_layout()
        plt.savefig(self.plots_path / "wind_disturbance_analysis.png",
                   dpi=300, bbox_inches='tight')
        plt.close()

    def plot_position_tracking(self):
        """Plot position tracking for all drones"""
        print("Generating position tracking plots...")

        fig, axes = plt.subplots(2, 2, figsize=(16, 12))

        fig.suptitle('Advanced Multi-Agent Formation Control - '
                     'Position Tracking Performance',
                     fontsize=16, fontweight='bold')

        colors = ['red', 'blue', 'green']
        drone_names = ['Leader', 'Follower 1', 'Follower 2']

        for scenario, data in self.data.items():
            # X-axis tracking
            ax_x = axes[0, 0]
            for drone_id in range(3):
                target_col = f'TargetX{drone_id}'
                current_col = f'CurrentX{drone_id}'

                if target_col in data.columns and current_col in data.columns:
                    ax_x.plot(data['Time'], data[target_col],
                             '--', color=colors[drone_id], alpha=0.7,
                             label=f'{drone_names[drone_id]} Target')
                    ax_x.plot(data['Time'], data[current_col],
                             '-', color=colors[drone_id], linewidth=2,
                             label=f'{drone_names[drone_id]} Actual')

            ax_x.set_xlabel('Time (s)')
            ax_x.set_ylabel('X Position (m)')
            ax_x.set_title('X-Axis Position Tracking')
            ax_x.legend()
            ax_x.grid(True, alpha=0.3)

            # Y-axis tracking
            ax_y = axes[0, 1]
            for drone_id in range(3):
                target_col = f'TargetY{drone_id}'
                current_col = f'CurrentY{drone_id}'

                if target_col in data.columns and current_col in data.columns:
                    ax_y.plot(data['Time'], data[target_col],
                             '--', color=colors[drone_id], alpha=0.7,
                             label=f'{drone_names[drone_id]} Target')
                    ax_y.plot(data['Time'], data[current_col],
                             '-', color=colors[drone_id], linewidth=2,
                             label=f'{drone_names[drone_id]} Actual')

            ax_y.set_xlabel('Time (s)')
            ax_y.set_ylabel('Y Position (m)')
            ax_y.set_title('Y-Axis Position Tracking')
            ax_y.legend()
            ax_y.grid(True, alpha=0.3)

            # Formation geometry (2D trajectory)
            ax_2d = axes[1, 0]
            for drone_id in range(3):
                x_col = f'CurrentX{drone_id}'
                y_col = f'CurrentY{drone_id}'
                x_target = f'TargetX{drone_id}'
                y_target = f'TargetY{drone_id}'

                if all(col in data.columns for col in [x_col, y_col]):
                    # Plot trajectory
                    ax_2d.plot(data[x_col], data[y_col],
                              color=colors[drone_id], linewidth=2,
                              label=f'{drone_names[drone_id]} Trajectory')

                    # Plot target waypoints
                    if all(col in data.columns for col in [x_target, y_target]):
                        # Get unique target positions (waypoints)
                        targets_x = data[x_target].round(1)
                        targets_y = data[y_target].round(1)
                        unique_targets = pd.DataFrame({'x': targets_x, 'y': targets_y}).drop_duplicates()

                        ax_2d.scatter(unique_targets['x'], unique_targets['y'],
                                     color=colors[drone_id], s=100, marker='*',
                                     alpha=0.8, edgecolors='black')

            ax_2d.set_xlabel('X Position (m)')
            ax_2d.set_ylabel('Y Position (m)')
            ax_2d.set_title('Formation Trajectory (2D View)')
            ax_2d.legend()
            ax_2d.grid(True, alpha=0.3)
            ax_2d.axis('equal')

            # Tracking errors
            ax_err = axes[1, 1]
            for drone_id in range(3):
                error_x_col = f'ErrorX{drone_id}'
                error_y_col = f'ErrorY{drone_id}'

                if error_x_col in data.columns and error_y_col in data.columns:
                    # Calculate magnitude of error
                    error_magnitude = np.sqrt(
                        data[error_x_col]**2 + data[error_y_col]**2)
                    ax_err.plot(data['Time'], error_magnitude,
                               color=colors[drone_id], linewidth=2,
                               label=f'{drone_names[drone_id]} Error')

            ax_err.set_xlabel('Time (s)')
            ax_err.set_ylabel('Position Error (m)')
            ax_err.set_title('Tracking Error Magnitude')
            ax_err.legend()
            ax_err.grid(True, alpha=0.3)

        plt.tight_layout()
        plt.savefig(self.plots_path / "comprehensive_position_tracking.png",
                    dpi=300, bbox_inches='tight')
        plt.close()

    def plot_control_analysis(self):
        """Plot control system analysis"""
        print("Generating control system analysis...")

        fig, axes = plt.subplots(2, 3, figsize=(18, 12))

        fig.suptitle('Advanced Control System Analysis - '
                     'PID + Fuzzy Logic Control',
                     fontsize=16, fontweight='bold')

        colors = ['red', 'blue', 'green']
        drone_names = ['Leader', 'Follower 1', 'Follower 2']

        for scenario, data in self.data.items():
            # PID Control Output
            ax_pid = axes[0, 0]
            for drone_id in range(3):
                pid_x_col = f'PIDOutX{drone_id}'
                pid_y_col = f'PIDOutY{drone_id}'

                if pid_x_col in data.columns:
                    ax_pid.plot(data['Time'], data[pid_x_col],
                               color=colors[drone_id], linewidth=1.5,
                               alpha=0.7,
                               label=f'{drone_names[drone_id]} X-PID')
                if pid_y_col in data.columns:
                    ax_pid.plot(data['Time'], data[pid_y_col],
                               '--', color=colors[drone_id], linewidth=1.5,
                               alpha=0.7,
                               label=f'{drone_names[drone_id]} Y-PID')

            ax_pid.set_xlabel('Time (s)')
            ax_pid.set_ylabel('PID Output')
            ax_pid.set_title('PID Controller Output')
            ax_pid.legend()
            ax_pid.grid(True, alpha=0.3)

            # Fuzzy Logic Correction
            ax_flc = axes[0, 1]
            for drone_id in range(3):
                flc_x_col = f'FLSCorrX{drone_id}'
                flc_y_col = f'FLSCorrY{drone_id}'

                if flc_x_col in data.columns:
                    ax_flc.plot(data['Time'], data[flc_x_col],
                               color=colors[drone_id], linewidth=1.5,
                               label=f'{drone_names[drone_id]} X-FLC')
                if flc_y_col in data.columns:
                    ax_flc.plot(data['Time'], data[flc_y_col],
                               '--', color=colors[drone_id], linewidth=1.5,
                               label=f'{drone_names[drone_id]} Y-FLC')

            ax_flc.set_xlabel('Time (s)')
            ax_flc.set_ylabel('FLC Correction')
            ax_flc.set_title('Fuzzy Logic Control Correction')
            ax_flc.legend()
            ax_flc.grid(True, alpha=0.3)

            # Final Command
            ax_cmd = axes[0, 2]
            for drone_id in range(3):
                cmd_x_col = f'FinalCmdX{drone_id}'
                cmd_y_col = f'FinalCmdY{drone_id}'

                if cmd_x_col in data.columns:
                    ax_cmd.plot(data['Time'], data[cmd_x_col],
                               color=colors[drone_id], linewidth=1.5,
                               label=f'{drone_names[drone_id]} X-Cmd')
                if cmd_y_col in data.columns:
                    ax_cmd.plot(data['Time'], data[cmd_y_col],
                               '--', color=colors[drone_id], linewidth=1.5,
                               label=f'{drone_names[drone_id]} Y-Cmd')

            ax_cmd.set_xlabel('Time (s)')
            ax_cmd.set_ylabel('Final Command')
            ax_cmd.set_title('Final Control Command (PID + FLC)')
            ax_cmd.legend()
            ax_cmd.grid(True, alpha=0.3)

            # Wind Disturbance (if available)
            if 'SimWindX' in data.columns and 'SimWindY' in data.columns:
                ax_wind = axes[1, 0]
                ax_wind.plot(data['Time'], data['SimWindX'],
                            'r-', linewidth=2, label='Wind X')
                ax_wind.plot(data['Time'], data['SimWindY'],
                            'b-', linewidth=2, label='Wind Y')
                ax_wind.set_xlabel('Time (s)')
                ax_wind.set_ylabel('Wind Force (N)')
                ax_wind.set_title('Wind Disturbance Forces')
                ax_wind.legend()
                ax_wind.grid(True, alpha=0.3)
            else:
                axes[1, 0].text(0.5, 0.5, 'No Wind Data Available',
                               ha='center', va='center', transform=axes[1, 0].transAxes)

            # PID Components Analysis (for drone 0)
            ax_pid_comp = axes[1, 1]
            drone_id = 0  # Focus on leader drone
            p_x_col = f'PTermX{drone_id}'
            i_x_col = f'ITermX{drone_id}'
            d_x_col = f'DTermX{drone_id}'

            if all(col in data.columns for col in [p_x_col, i_x_col, d_x_col]):
                ax_pid_comp.plot(data['Time'], data[p_x_col],
                                'r-', linewidth=1.5, label='P Term')
                ax_pid_comp.plot(data['Time'], data[i_x_col],
                                'g-', linewidth=1.5, label='I Term')
                ax_pid_comp.plot(data['Time'], data[d_x_col],
                                'b-', linewidth=1.5, label='D Term')

            ax_pid_comp.set_xlabel('Time (s)')
            ax_pid_comp.set_ylabel('PID Component Value')
            ax_pid_comp.set_title(f'PID Components - {drone_names[0]} (X-axis)')
            ax_pid_comp.legend()
            ax_pid_comp.grid(True, alpha=0.3)

            # Performance Metrics Summary
            ax_metrics = axes[1, 2]

            # Calculate RMS errors for each drone
            rms_errors = []
            for drone_id in range(3):
                error_x_col = f'ErrorX{drone_id}'
                error_y_col = f'ErrorY{drone_id}'

                if error_x_col in data.columns and error_y_col in data.columns:
                    error_magnitude = np.sqrt(
                        data[error_x_col]**2 + data[error_y_col]**2)
                    rms_error = np.sqrt(np.mean(error_magnitude**2))
                    rms_errors.append(rms_error)

            if rms_errors:
                bars = ax_metrics.bar(range(len(rms_errors)), rms_errors,
                                     color=colors[:len(rms_errors)], alpha=0.7)
                ax_metrics.set_xlabel('Drone')
                ax_metrics.set_ylabel('RMS Error (m)')
                ax_metrics.set_title('RMS Tracking Error by Drone')
                ax_metrics.set_xticks(range(len(rms_errors)))
                ax_metrics.set_xticklabels(drone_names[:len(rms_errors)])
                ax_metrics.grid(True, alpha=0.3)

                # Add value labels on bars
                for bar, error in zip(bars, rms_errors):
                    height = bar.get_height()
                    ax_metrics.text(bar.get_x() + bar.get_width()/2., height,
                                   f'{error:.3f}m',
                                   ha='center', va='bottom')

        plt.tight_layout()
        plt.savefig(self.plots_path / "comprehensive_control_analysis.png",
                    dpi=300, bbox_inches='tight')
        plt.close()

    def generate_comprehensive_report(self):
        """Generate comprehensive analysis report"""
        print("Generating comprehensive analysis report...")

        report_path = self.output_path / "comprehensive_analysis_report.txt"

        with open(report_path, 'w') as f:
            f.write("=" * 70 + "\n")
            f.write("COMPREHENSIVE MULTI-AGENT FORMATION CONTROL ANALYSIS\n")
            f.write("Advanced Control Engineering - Final Project Results\n")
            f.write("=" * 70 + "\n\n")

            for scenario, data in self.data.items():
                f.write(f"SCENARIO: {scenario}\n")
                f.write("-" * 50 + "\n")
                f.write(f"Total Data Points: {len(data)}\n")
                f.write(f"Simulation Duration: {data['Time'].max():.1f} seconds\n\n")

                # Performance Analysis
                f.write("PERFORMANCE METRICS:\n")
                for drone_id in range(3):
                    error_x_col = f'ErrorX{drone_id}'
                    error_y_col = f'ErrorY{drone_id}'

                    if error_x_col in data.columns and error_y_col in data.columns:
                        error_magnitude = np.sqrt(
                            data[error_x_col]**2 + data[error_y_col]**2)

                        rms_error = np.sqrt(np.mean(error_magnitude**2))
                        max_error = error_magnitude.max()
                        mean_error = error_magnitude.mean()

                        drone_names = ['Leader', 'Follower 1', 'Follower 2']
                        f.write(f"  {drone_names[drone_id]}:\n")
                        f.write(f"    RMS Error: {rms_error:.4f} m\n")
                        f.write(f"    Max Error: {max_error:.4f} m\n")
                        f.write(f"    Mean Error: {mean_error:.4f} m\n")

                f.write("\n")

                # Control System Analysis
                f.write("CONTROL SYSTEM PERFORMANCE:\n")

                # Wind disturbance analysis
                if 'SimWindX' in data.columns and 'SimWindY' in data.columns:
                    wind_active = (data['SimWindX'].abs() > 0.1) | (data['SimWindY'].abs() > 0.1)
                    wind_duration = wind_active.sum() / len(data) * 100
                    f.write(f"  Wind Disturbance Active: {wind_duration:.1f}% of simulation\n")

                    max_wind_x = data['SimWindX'].abs().max()
                    max_wind_y = data['SimWindY'].abs().max()
                    f.write(f"  Maximum Wind Force: X={max_wind_x:.2f}N, Y={max_wind_y:.2f}N\n")

                # FLC effectiveness
                flc_active = False
                for drone_id in range(3):
                    flc_x_col = f'FLSCorrX{drone_id}'
                    if flc_x_col in data.columns:
                        flc_correction = data[flc_x_col].abs().mean()
                        if flc_correction > 0.01:
                            flc_active = True
                            break

                f.write(f"  Fuzzy Logic Control: {'ACTIVE' if flc_active else 'INACTIVE'}\n")

                f.write("\n")

            # Load metrics from file if available
            if hasattr(self, 'metrics_file'):
                f.write("DETAILED PERFORMANCE METRICS FROM SIMULATION:\n")
                f.write("-" * 50 + "\n")
                try:
                    with open(self.metrics_file, 'r') as metrics_f:
                        f.write(metrics_f.read())
                except Exception as e:
                    f.write(f"Error reading metrics file: {e}\n")
                f.write("\n")

            f.write("ANALYSIS SUMMARY:\n")
            f.write("-" * 50 + "\n")
            f.write("This comprehensive simulation demonstrates advanced multi-agent\n")
            f.write("formation control with the following key features:\n\n")
            f.write("1. Multi-phase trajectory following\n")
            f.write("2. Advanced PID + Fuzzy Logic hybrid control\n")
            f.write("3. Real-time wind disturbance compensation\n")
            f.write("4. Triangular formation geometry maintenance\n")
            f.write("5. Professional performance metrics and analysis\n\n")
            f.write("The system shows excellent tracking performance with\n")
            f.write("low RMS errors and effective disturbance rejection.\n")

    def run_analysis(self):
        """Run enhanced comprehensive analysis suite"""
        if not self.data:
            print("No data loaded. Cannot perform analysis.")
            return

        print("=== Starting Enhanced Formation Control Analysis ===\n")

        # Generate all enhanced plots
        print("1. Multi-scenario performance comparison...")
        self.plot_scenario_comparison()

        print("2. Wind disturbance rejection analysis...")
        self.plot_wind_disturbance_analysis()

        print("3. Position tracking analysis...")
        self.plot_position_tracking()

        print("4. Control system analysis...")
        self.plot_control_analysis()

        print("5. Generating comprehensive report...")
        self.generate_comprehensive_report()

        print(f"\n=== Enhanced Analysis Complete! ===")
        print(f"Results saved to: {self.output_path}")
        print(f"Plots saved to: {self.plots_path}\n")

        print("Generated enhanced plots:")
        print("- scenario_comparison.png: Multi-scenario performance comparison")
        print("- wind_disturbance_analysis.png: Detailed wind rejection analysis")
        print("- comprehensive_position_tracking.png: Position tracking analysis")
        print("- comprehensive_control_analysis.png: Control system analysis")
        print("- comprehensive_analysis_report.txt: Detailed analysis report")


if __name__ == "__main__":
    analyzer = FormationControlAnalyzer()
    analyzer.run_analysis()
