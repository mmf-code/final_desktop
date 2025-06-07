import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path

# Create output directories
output_path = Path("final_project_results")
plots_path = output_path / "plots"
plots_path.mkdir(parents=True, exist_ok=True)

# Load data
base_path = Path("build/simulation_outputs")
data_files = {
    'PID Only': 'pid_only_formation_test_data.csv',
    'PID + Wind + FLC': 'pid_wind_flc_formation_test_data.csv'
}

print("Loading data...")
datasets = {}
for scenario, filename in data_files.items():
    filepath = base_path / filename
    if filepath.exists():
        datasets[scenario] = pd.read_csv(filepath)
        print(f"Loaded {scenario}: {len(datasets[scenario])} points")

# Plot position tracking
if datasets:
    fig, axes = plt.subplots(2, len(datasets),
                             figsize=(8*len(datasets), 12))
    if len(datasets) == 1:
        axes = axes.reshape(-1, 1)

    fig.suptitle('Multi-Agent Formation Control - Position Tracking',
                 fontsize=16, fontweight='bold')
    colors = ['red', 'blue', 'green']

    for i, (scenario, data) in enumerate(datasets.items()):
        # X positions
        ax_x = axes[0, i]
        for drone_id in range(3):
            target_col = f'drone{drone_id}_target_x'
            pos_col = f'drone{drone_id}_pos_x'
            if target_col in data.columns and pos_col in data.columns:
                ax_x.plot(data['time'], data[pos_col],
                          color=colors[drone_id],
                          label=f'Drone {drone_id} Actual', linewidth=2)
                ax_x.plot(data['time'], data[target_col],
                          color=colors[drone_id],
                          label=f'Drone {drone_id} Target',
                          linestyle='--', alpha=0.6)

        ax_x.set_title(f'{scenario} - X Position')
        ax_x.set_xlabel('Time (s)')
        ax_x.set_ylabel('X Position (m)')
        ax_x.grid(True)
        ax_x.legend()

        # Y positions
        ax_y = axes[1, i]
        for drone_id in range(3):
            target_col = f'drone{drone_id}_target_y'
            pos_col = f'drone{drone_id}_pos_y'
            if target_col in data.columns and pos_col in data.columns:
                ax_y.plot(data['time'], data[pos_col],
                          color=colors[drone_id],
                          label=f'Drone {drone_id} Actual', linewidth=2)
                ax_y.plot(data['time'], data[target_col],
                          color=colors[drone_id],
                          label=f'Drone {drone_id} Target',
                          linestyle='--', alpha=0.6)

        ax_y.set_title(f'{scenario} - Y Position')
        ax_y.set_xlabel('Time (s)')
        ax_y.set_ylabel('Y Position (m)')
        ax_y.grid(True)
        ax_y.legend()

    plt.tight_layout()
    plt.savefig(plots_path / 'position_tracking.png', dpi=300,
                bbox_inches='tight')
    plt.close()
    print("✓ Position tracking plot saved")

# Plot tracking errors
if datasets:
    fig, axes = plt.subplots(2, len(datasets),
                             figsize=(8*len(datasets), 12))
    if len(datasets) == 1:
        axes = axes.reshape(-1, 1)

    fig.suptitle('Tracking Error Analysis', fontsize=16, fontweight='bold')

    for i, (scenario, data) in enumerate(datasets.items()):
        # X errors
        ax_x = axes[0, i]
        for drone_id in range(3):
            error_col = f'drone{drone_id}_error_x'
            if error_col in data.columns:
                ax_x.plot(data['time'], data[error_col],
                          color=colors[drone_id],
                          label=f'Drone {drone_id}', linewidth=2)

        ax_x.set_title(f'{scenario} - X Error')
        ax_x.set_xlabel('Time (s)')
        ax_x.set_ylabel('X Error (m)')
        ax_x.grid(True)
        ax_x.axhline(y=0, color='black', linestyle=':', alpha=0.5)
        ax_x.legend()

        # Y errors
        ax_y = axes[1, i]
        for drone_id in range(3):
            error_col = f'drone{drone_id}_error_y'
            if error_col in data.columns:
                ax_y.plot(data['time'], data[error_col],
                          color=colors[drone_id],
                          label=f'Drone {drone_id}', linewidth=2)

        ax_y.set_title(f'{scenario} - Y Error')
        ax_y.set_xlabel('Time (s)')
        ax_y.set_ylabel('Y Error (m)')
        ax_y.grid(True)
        ax_y.axhline(y=0, color='black', linestyle=':', alpha=0.5)
        ax_y.legend()

    plt.tight_layout()
    plt.savefig(plots_path / 'tracking_errors.png', dpi=300,
                bbox_inches='tight')
    plt.close()
    print("✓ Tracking error plot saved")

# FLC-specific analysis
if 'PID + Wind + FLC' in datasets:
    data = datasets['PID + Wind + FLC']
    fig, axes = plt.subplots(2, 2, figsize=(16, 12))
    fig.suptitle('Fuzzy Logic Controller Analysis', fontsize=16,
                 fontweight='bold')

    # Wind forces
    ax1 = axes[0, 0]
    if 'wind_x' in data.columns and 'wind_y' in data.columns:
        ax1.plot(data['time'], data['wind_x'], label='Wind X', color='red')
        ax1.plot(data['time'], data['wind_y'], label='Wind Y', color='blue')
    ax1.set_title('Wind Disturbances')
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Wind Force (N/kg)')
    ax1.grid(True)
    ax1.legend()

    # FLC corrections
    ax2 = axes[0, 1]
    if 'drone0_flc_corr_x' in data.columns:
        ax2.plot(data['time'], data['drone0_flc_corr_x'], label='FLC X',
                 color='green')
        ax2.plot(data['time'], data['drone0_flc_corr_y'], label='FLC Y',
                 color='orange')
    ax2.set_title('FLC Corrections (Drone 0)')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('FLC Correction')
    ax2.grid(True)
    ax2.legend()

    # Control comparison
    ax3 = axes[1, 0]
    if 'drone0_pid_cmd_x' in data.columns:
        ax3.plot(data['time'], data['drone0_pid_cmd_x'], label='PID Only',
                 color='blue')
        ax3.plot(data['time'], data['drone0_total_cmd_x'],
                 label='PID + FLC', color='red')
    ax3.set_title('Control Commands (Drone 0 - X)')
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Control Command')
    ax3.grid(True)
    ax3.legend()

    # Formation at different times
    ax4 = axes[1, 1]
    times = [10, 25, 40]
    for j, t in enumerate(times):
        idx = (data['time'] - t).abs().idxmin()
        x_pos = [data.loc[idx, f'drone{i}_pos_x'] for i in range(3)]
        y_pos = [data.loc[idx, f'drone{i}_pos_y'] for i in range(3)]
        x_pos.append(x_pos[0])  # Close triangle
        y_pos.append(y_pos[0])
        ax4.plot(x_pos, y_pos, 'o-', label=f't={t}s', markersize=8)

    ax4.set_title('Formation Geometry')
    ax4.set_xlabel('X Position (m)')
    ax4.set_ylabel('Y Position (m)')
    ax4.grid(True)
    ax4.legend()
    ax4.axis('equal')

    plt.tight_layout()
    plt.savefig(plots_path / 'flc_analysis.png', dpi=300,
                bbox_inches='tight')
    plt.close()
    print("✓ FLC analysis plot saved")

# Performance metrics
if datasets:
    metrics = {}
    for scenario, data in datasets.items():
        scenario_metrics = {}
        for drone_id in range(3):
            error_x = data[f'drone{drone_id}_error_x']
            error_y = data[f'drone{drone_id}_error_y']
            error_mag = np.sqrt(error_x**2 + error_y**2)

            scenario_metrics[f'drone_{drone_id}'] = {
                'rms_total': np.sqrt(np.mean(error_mag**2)),
                'max_error': np.max(error_mag),
                'mean_error': np.mean(np.abs(error_mag))
            }
        metrics[scenario] = scenario_metrics

    # Plot metrics
    fig, axes = plt.subplots(1, 2, figsize=(16, 6))
    fig.suptitle('Performance Metrics Comparison', fontsize=16,
                 fontweight='bold')

    # RMS errors
    scenarios = list(metrics.keys())
    x = np.arange(len(scenarios))
    width = 0.25

    ax1 = axes[0]
    for i in range(3):
        rms_values = [metrics[s][f'drone_{i}']['rms_total']
                      for s in scenarios]
        ax1.bar(x + i*width, rms_values, width, label=f'Drone {i}',
                color=colors[i])

    ax1.set_title('RMS Tracking Error')
    ax1.set_xlabel('Scenario')
    ax1.set_ylabel('RMS Error (m)')
    ax1.set_xticks(x + width)
    ax1.set_xticklabels(scenarios)
    ax1.legend()
    ax1.grid(True)

    # Error distributions
    ax2 = axes[1]
    for scenario, data in datasets.items():
        error_mag = np.sqrt(data['drone0_error_x']**2 +
                            data['drone0_error_y']**2)
        ax2.hist(error_mag, bins=20, alpha=0.6, label=scenario)

    ax2.set_title('Error Distribution (Drone 0)')
    ax2.set_xlabel('Tracking Error (m)')
    ax2.set_ylabel('Frequency')
    ax2.legend()
    ax2.grid(True)

    plt.tight_layout()
    plt.savefig(plots_path / 'performance_metrics.png', dpi=300,
                bbox_inches='tight')
    plt.close()
    print("✓ Performance metrics plot saved")

# Generate report
report_path = output_path / "analysis_report.txt"
with open(report_path, 'w') as f:
    f.write("MULTI-AGENT FORMATION CONTROL - "
            "FINAL PROJECT ANALYSIS\n")
    f.write("="*60 + "\n\n")

    f.write("SCENARIOS ANALYZED:\n")
    for scenario in datasets.keys():
        f.write(f"- {scenario}\n")

    f.write("\nPERFORMANCE METRICS:\n")
    for scenario, scenario_metrics in metrics.items():
        f.write(f"\n{scenario}:\n")
        for drone, metrics_dict in scenario_metrics.items():
            f.write(f"  {drone}: RMS={metrics_dict['rms_total']:.3f}m, "
                    f"Max={metrics_dict['max_error']:.3f}m\n")

    f.write("\nGENERATED PLOTS:\n")
    f.write("- position_tracking.png\n")
    f.write("- tracking_errors.png\n")
    f.write("- flc_analysis.png\n")
    f.write("- performance_metrics.png\n")

print("\n" + "="*50)
print("✅ FINAL PROJECT ANALYSIS COMPLETE!")
print(f"📊 Plots saved to: {plots_path}")
print(f"📋 Report saved to: {report_path}")
print("="*50) 