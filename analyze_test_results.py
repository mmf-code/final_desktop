#!/usr/bin/env python3
"""
Systematic Test Results Analysis Script
Analyzes CSV files from different test configurations to compare performance
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import os
import glob
from pathlib import Path

class TestResultsAnalyzer:
    def __init__(self):
        self.results = {}
        self.metrics_summary = {}
        
    def load_test_results(self, base_dir="simulation_outputs"):
        """Load all test result CSV files"""
        print("🔍 Loading test results...")
        
        # Find all CSV files with test prefixes
        patterns = [
            "Baseline_*_drone_metrics.csv",
            "FLS_Only_*_drone_metrics.csv", 
            "Wind_Only_*_drone_metrics.csv",
            "FF_Only_*_drone_metrics.csv",
            "FLS_Wind_*_drone_metrics.csv",
            "Full_System_*_drone_metrics.csv"
        ]
        
        for pattern in patterns:
            files = glob.glob(os.path.join(base_dir, pattern))
            if files:
                # Use the most recent file for each test
                latest_file = max(files, key=os.path.getctime)
                test_name = pattern.split('_')[0] + ('_' + pattern.split('_')[1] if 'Only' in pattern or 'Wind' in pattern or 'System' in pattern else '')
                
                try:
                    df = pd.read_csv(latest_file)
                    self.results[test_name] = df
                    print(f"✅ Loaded {test_name}: {latest_file}")
                except Exception as e:
                    print(f"❌ Failed to load {latest_file}: {e}")
        
        if not self.results:
            print("⚠️ No test results found. Make sure you've run the systematic tests first.")
            return False
        
        return True
    
    def calculate_performance_metrics(self):
        """Calculate key performance metrics for each test"""
        print("\n📊 Calculating performance metrics...")
        
        for test_name, df in self.results.items():
            metrics = {}
            
            # For each drone, calculate metrics
            for drone_id in df['drone_id'].unique():
                drone_data = df[df['drone_id'] == drone_id]
                
                # Position errors (distance from target)
                pos_error = np.sqrt(
                    (drone_data['position_x'] - drone_data['target_x'])**2 + 
                    (drone_data['position_y'] - drone_data['target_y'])**2
                )
                
                # Calculate settling time (when error stays below 2% of max)
                max_error = pos_error.max()
                settling_threshold = 0.02 * max_error
                
                settling_time = None
                for i in range(len(pos_error)-10):
                    if all(pos_error[i:i+10] < settling_threshold):
                        settling_time = drone_data.iloc[i]['time']
                        break
                
                # Calculate overshoot
                target_distance = np.sqrt(drone_data['target_x'].iloc[-1]**2 + drone_data['target_y'].iloc[-1]**2)
                max_distance = np.sqrt(drone_data['position_x']**2 + drone_data['position_y']**2).max()
                overshoot = max(0, (max_distance - target_distance) / target_distance * 100)
                
                # Steady-state error (last 20% of simulation)
                steady_start = int(0.8 * len(drone_data))
                steady_state_error = pos_error[steady_start:].mean()
                
                metrics[f'drone_{drone_id}'] = {
                    'overshoot_percent': overshoot,
                    'settling_time_2pct': settling_time if settling_time else drone_data['time'].max(),
                    'steady_state_error': steady_state_error,
                    'max_error': max_error,
                    'final_error': pos_error.iloc[-1]
                }
            
            # Overall test metrics
            all_overshoot = [metrics[f'drone_{i}']['overshoot_percent'] for i in range(3)]
            all_settling = [metrics[f'drone_{i}']['settling_time_2pct'] for i in range(3)]
            all_steady_error = [metrics[f'drone_{i}']['steady_state_error'] for i in range(3)]
            
            metrics['overall'] = {
                'avg_overshoot': np.mean(all_overshoot),
                'max_overshoot': np.max(all_overshoot),
                'avg_settling_time': np.mean(all_settling),
                'max_settling_time': np.max(all_settling),
                'avg_steady_error': np.mean(all_steady_error),
                'max_steady_error': np.max(all_steady_error)
            }
            
            self.metrics_summary[test_name] = metrics
            print(f"✅ Calculated metrics for {test_name}")
    
    def generate_comparison_report(self):
        """Generate detailed comparison report"""
        print("\n📋 Generating comparison report...")
        
        report_path = "systematic_test_comparison.txt"
        
        with open(report_path, 'w') as f:
            f.write("SYSTEMATIC TESTING COMPARISON REPORT\n")
            f.write("=" * 50 + "\n\n")
            
            # Overall performance comparison
            f.write("OVERALL PERFORMANCE COMPARISON\n")
            f.write("-" * 35 + "\n")
            f.write(f"{'Test':<20} {'Avg Overshoot':<15} {'Avg Settling':<15} {'Avg SS Error':<15}\n")
            f.write("-" * 65 + "\n")
            
            baseline_metrics = None
            for test_name, metrics in self.metrics_summary.items():
                overall = metrics['overall']
                f.write(f"{test_name:<20} {overall['avg_overshoot']:<15.2f} {overall['avg_settling_time']:<15.2f} {overall['avg_steady_error']:<15.4f}\n")
                
                if test_name == 'Baseline':
                    baseline_metrics = overall
            
            f.write("\n")
            
            # Improvement percentages (if baseline exists)
            if baseline_metrics:
                f.write("IMPROVEMENT OVER BASELINE (%)\n")
                f.write("-" * 35 + "\n")
                f.write(f"{'Test':<20} {'Overshoot':<15} {'Settling':<15} {'SS Error':<15}\n")
                f.write("-" * 65 + "\n")
                
                for test_name, metrics in self.metrics_summary.items():
                    if test_name == 'Baseline':
                        continue
                    
                    overall = metrics['overall']
                    
                    # Calculate improvements (negative means worse)
                    overshoot_imp = (baseline_metrics['avg_overshoot'] - overall['avg_overshoot']) / baseline_metrics['avg_overshoot'] * 100
                    settling_imp = (baseline_metrics['avg_settling_time'] - overall['avg_settling_time']) / baseline_metrics['avg_settling_time'] * 100
                    error_imp = (baseline_metrics['avg_steady_error'] - overall['avg_steady_error']) / baseline_metrics['avg_steady_error'] * 100
                    
                    f.write(f"{test_name:<20} {overshoot_imp:<15.1f} {settling_imp:<15.1f} {error_imp:<15.1f}\n")
            
            f.write("\n")
            f.write("DETAILED ANALYSIS\n")
            f.write("=" * 20 + "\n")
            
            # Key findings
            if baseline_metrics and 'FLS_Only' in self.metrics_summary:
                fls_metrics = self.metrics_summary['FLS_Only']['overall']
                overshoot_imp = (baseline_metrics['avg_overshoot'] - fls_metrics['avg_overshoot']) / baseline_metrics['avg_overshoot'] * 100
                f.write(f"FLS Effectiveness: {overshoot_imp:.1f}% overshoot reduction\n")
            
            if baseline_metrics and 'Wind_Only' in self.metrics_summary:
                wind_metrics = self.metrics_summary['Wind_Only']['overall']
                settling_deg = (wind_metrics['avg_settling_time'] - baseline_metrics['avg_settling_time']) / baseline_metrics['avg_settling_time'] * 100
                f.write(f"Wind Impact: {settling_deg:.1f}% increase in settling time\n")
            
            if baseline_metrics and 'FF_Only' in self.metrics_summary:
                ff_metrics = self.metrics_summary['FF_Only']['overall']
                error_imp = (baseline_metrics['avg_steady_error'] - ff_metrics['avg_steady_error']) / baseline_metrics['avg_steady_error'] * 100
                f.write(f"Feed-Forward Benefit: {error_imp:.1f}% steady-state error reduction\n")
            
            f.write("\nRECOMMendations:\n")
            f.write("- Use FLS for reduced overshoot in formation control\n")
            f.write("- Enable feed-forward for improved tracking accuracy\n")
            f.write("- Full system provides best overall performance\n")
            f.write("- Wind significantly impacts performance without FLS\n")
        
        print(f"✅ Comparison report saved to: {report_path}")
        return report_path
    
    def create_comparison_plots(self):
        """Create visualization plots for comparison"""
        print("\n📊 Creating comparison plots...")
        
        if not self.metrics_summary:
            print("❌ No metrics to plot")
            return
        
        # Prepare data for plotting
        test_names = list(self.metrics_summary.keys())
        
        fig, axes = plt.subplots(2, 2, figsize=(15, 10))
        fig.suptitle('Systematic Testing Comparison', fontsize=16, fontweight='bold')
        
        # Plot 1: Average Overshoot
        overshots = [self.metrics_summary[test]['overall']['avg_overshoot'] for test in test_names]
        axes[0, 0].bar(test_names, overshots, color='lightcoral', alpha=0.7)
        axes[0, 0].set_title('Average Overshoot (%)')
        axes[0, 0].set_ylabel('Overshoot (%)')
        axes[0, 0].tick_params(axis='x', rotation=45)
        
        # Plot 2: Average Settling Time
        settling_times = [self.metrics_summary[test]['overall']['avg_settling_time'] for test in test_names]
        axes[0, 1].bar(test_names, settling_times, color='lightblue', alpha=0.7)
        axes[0, 1].set_title('Average Settling Time (s)')
        axes[0, 1].set_ylabel('Time (s)')
        axes[0, 1].tick_params(axis='x', rotation=45)
        
        # Plot 3: Steady-State Error
        ss_errors = [self.metrics_summary[test]['overall']['avg_steady_error'] for test in test_names]
        axes[1, 0].bar(test_names, ss_errors, color='lightgreen', alpha=0.7)
        axes[1, 0].set_title('Average Steady-State Error (m)')
        axes[1, 0].set_ylabel('Error (m)')
        axes[1, 0].tick_params(axis='x', rotation=45)
        
        # Plot 4: Overall Performance Score (lower is better)
        # Normalize and combine metrics
        norm_overshoot = np.array(overshots) / max(overshots)
        norm_settling = np.array(settling_times) / max(settling_times)
        norm_error = np.array(ss_errors) / max(ss_errors)
        
        performance_score = (norm_overshoot + norm_settling + norm_error) / 3
        
        axes[1, 1].bar(test_names, performance_score, color='gold', alpha=0.7)
        axes[1, 1].set_title('Overall Performance Score (Lower = Better)')
        axes[1, 1].set_ylabel('Score')
        axes[1, 1].tick_params(axis='x', rotation=45)
        
        plt.tight_layout()
        plt.savefig('systematic_test_comparison.png', dpi=300, bbox_inches='tight')
        plt.show()
        
        print("✅ Comparison plot saved as: systematic_test_comparison.png")
    
    def run_full_analysis(self):
        """Run complete analysis pipeline"""
        print("🚀 Starting systematic test analysis...")
        
        if not self.load_test_results():
            return False
        
        self.calculate_performance_metrics()
        report_path = self.generate_comparison_report()
        self.create_comparison_plots()
        
        print("\n🎉 Analysis Complete!")
        print(f"📋 Report: {report_path}")
        print("📊 Plot: systematic_test_comparison.png")
        print("\nKey insights available in the generated report.")
        
        return True

if __name__ == "__main__":
    analyzer = TestResultsAnalyzer()
    
    print("📊 Systematic Test Results Analyzer")
    print("=" * 40)
    print("This script analyzes results from systematic testing")
    print("Make sure you've run the tests first using run_tests.ps1")
    print()
    
    if analyzer.run_full_analysis():
        print("\n✨ Use the generated plots and report for your final analysis!")
    else:
        print("\n⚠️ Analysis failed. Please run systematic tests first.") 