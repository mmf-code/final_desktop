#!/usr/bin/env python3
"""
Systematic Testing Script for Multi-Agent Formation Control
Tests combinations of PID improvements, FLS, wind, and feed-forward control
"""

import shutil
import subprocess
import time
from datetime import datetime
from pathlib import Path
import yaml  # Using PyYAML for robust config management


class SystematicTester:
    """Systematic tester for multi-agent formation control system."""

    def __init__(self):
        # NOTE: Using a hardcoded absolute path is not ideal.
        # Consider making this a relative path or a script argument.
        self.external_config_path = (
            "C:/Users/ataka/OneDrive/Masaüstü/config/"
            "simulation_params.yaml"
        )
        self.executable_path = "../build/Release/multi_drone_pid_tester.exe"
        self.results_dir = Path("../results/simulation_outputs")
        self.timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

        # Create results directory
        self.results_dir.mkdir(parents=True, exist_ok=True)

    def update_config(self, config_path, modifications):
        """
        Reads a YAML config, applies modifications, and writes it back.

        Args:
            config_path (str): The path to the YAML configuration file.
            modifications (dict): A dictionary where keys are dot-separated
                                  paths (e.g., 'a.b.c') and values are the
                                  new values to set.
        """
        try:
            with open(config_path, 'r', encoding='utf-8') as f:
                config_data = yaml.safe_load(f)

            for key_path, value in modifications.items():
                keys = key_path.split('.')
                current_level = config_data
                # Navigate to the parent dictionary
                for key in keys[:-1]:
                    current_level = current_level[key]
                # Set the final value
                current_level[keys[-1]] = value

            with open(config_path, 'w', encoding='utf-8') as f:
                # dump preserves comments and structure better with ruamel.yaml,
                # but PyYAML is simpler and sufficient here.
                yaml.dump(config_data, f, default_flow_style=False,
                          sort_keys=False)

        except (FileNotFoundError, KeyError) as e:
            print(f"Error updating config file: {e}")
            raise

    def run_test(self, test_name, config_modifications):
        """Run a single test with specific configuration"""
        print(f"\n🔬 Running Test: {test_name}")
        print("=" * 50)

        try:
            # Update the YAML configuration file using the robust Python method
            print(f"Applying configuration: {config_modifications}")
            self.update_config(self.external_config_path,
                               config_modifications)
        except Exception as e:
            print(f"Failed to update configuration. Aborting test. Error: {e}")
            return False

        # Brief pause to ensure the file system has registered the change
        time.sleep(1)

        # Run the test executable
        try:
            print(f"Executing: {self.executable_path}")
            result = subprocess.run(
                [self.executable_path],
                capture_output=True,
                text=True,
                timeout=180,  # 3-minute timeout
                check=False,  # Don't raise exception on non-zero exit codes
                encoding='utf-8',
                errors='ignore'
            )

            if result.returncode == 0:
                print("✅ Test completed successfully.")
                self.organize_results(test_name)
                return True
            else:
                print(f"❌ Test failed with return code: {result.returncode}")
                print("--- STDOUT ---")
                print(result.stdout)
                print("--- STDERR ---")
                print(result.stderr)
                return False

        except subprocess.TimeoutExpired:
            print("❌ Test timed out after 3 minutes.")
            return False
        except FileNotFoundError:
            print(f"❌ Executable not found at: {self.executable_path}")
            return False
        except Exception as e:
            print(f"❌ Test failed with an unexpected exception: {e}")
            return False

    def organize_results(self, test_name):
        """Organize test results into structured directories"""
        test_dir = self.results_dir / f"{self.timestamp}_{test_name}"
        test_dir.mkdir(exist_ok=True)

        # Copy the config file used for this test for traceability
        shutil.copy2(self.external_config_path, test_dir / "used_config.yaml")

        # Move CSV and metrics files from the output directory
        output_dir = Path("../simulation_outputs")
        if output_dir.exists():
            # Using a time window to find recent files is brittle.
            # A better approach is for the C++ executable to generate uniquely
            # named files, but this works as a fallback.
            cutoff_time = time.time() - 180  # Files modified in last 3 mins

            files_moved = 0
            for file_pattern in ["*.csv", "*.txt"]:
                for file in output_dir.glob(file_pattern):
                    if file.stat().st_mtime > cutoff_time:
                        shutil.copy2(file,
                                     test_dir / f"{test_name}_{file.name}")
                        files_moved += 1

            print(f"Results ({files_moved} files) saved to: {test_dir}")
        else:
            print(f"Warning: Output directory '{output_dir}' not found.")

    def run_feature_comparison_suite(self):
        """Run systematic feature comparison tests"""

        test_scenarios = [
            # Core Comparison Tests
            {
                "name": "01_Baseline_Clean",
                "config": {
                    "controller_settings.fls.enable": False,
                    "scenario_settings.enable_wind": False,
                    "controller_settings.pid.enable_feedforward": False,
                    "controller_settings.pid.enable_derivative_filter": True,
                }
            },
            {
                "name": "02_FLS_Effect",
                "config": {
                    "controller_settings.fls.enable": True,
                    "scenario_settings.enable_wind": False,
                    "controller_settings.pid.enable_feedforward": False,
                    "controller_settings.pid.enable_derivative_filter": True,
                }
            },
            {
                "name": "03_Wind_Challenge",
                "config": {
                    "controller_settings.fls.enable": False,
                    "scenario_settings.enable_wind": True,
                    "controller_settings.pid.enable_feedforward": False,
                    "controller_settings.pid.enable_derivative_filter": True,
                }
            },
            {
                "name": "04_FeedForward_Benefit",
                "config": {
                    "controller_settings.fls.enable": False,
                    "scenario_settings.enable_wind": False,
                    "controller_settings.pid.enable_feedforward": True,
                    "controller_settings.pid.enable_derivative_filter": True,
                }
            },
            {
                "name": "05_FLS_vs_Wind",
                "config": {
                    "controller_settings.fls.enable": True,
                    "scenario_settings.enable_wind": True,
                    "controller_settings.pid.enable_feedforward": False,
                    "controller_settings.pid.enable_derivative_filter": True,
                }
            },
            {
                "name": "06_Full_System_Optimal",
                "config": {
                    "controller_settings.fls.enable": True,
                    "scenario_settings.enable_wind": True,
                    "controller_settings.pid.enable_feedforward": True,
                    "controller_settings.pid.enable_derivative_filter": True,
                }
            }
        ]

        print(f"Starting Feature Comparison Suite ({len(test_scenarios)} tests)")
        print(f"Timestamp: {self.timestamp}")
        print("=" * 60)

        test_results = {}
        for i, scenario in enumerate(test_scenarios, 1):
            print(f"\nProgress: {i}/{len(test_scenarios)}")
            success = self.run_test(scenario["name"], scenario["config"])
            test_results[scenario["name"]] = success
            time.sleep(2)  # Brief pause between tests

        self.generate_comparison_report(test_results)
        return test_results

    def generate_comparison_report(self, test_results):
        """Generate comparison analysis report in Markdown format"""
        report_path = self.results_dir / f"comparison_analysis_{self.timestamp}.md"

        with open(report_path, 'w', encoding='utf-8') as f:
            f.write("# Multi-Agent Formation Control - Feature Comparison Analysis\n\n")
            f.write(f"**Test Date:** {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f"**Test ID:** {self.timestamp}\n\n")

            f.write("## Test Results Summary\n\n")
            f.write("| Test Name | Status | Description |\n")
            f.write("|-----------|--------|-------------|\n")

            descriptions = {
                "01_Baseline_Clean": "Basic PID with improvements, no FLS/Wind/FF",
                "02_FLS_Effect": "Baseline + Fuzzy Logic System",
                "03_Wind_Challenge": "Baseline + Wind Disturbances",
                "04_FeedForward_Benefit": "Baseline + Feed-Forward Control",
                "05_FLS_vs_Wind": "FLS handling wind disturbances",
                "06_Full_System_Optimal": "All features enabled"
            }

            for test_name, success in test_results.items():
                status = "PASS ✅" if success else "FAIL ❌"
                desc = descriptions.get(test_name, "Custom test")
                f.write(f"| {test_name} | {status} | {desc} |\n")

            f.write("\n## Analysis Instructions\n\n")
            f.write("### Performance Metrics to Compare:\n")
            f.write("1. **Overshoot Percentage** - Lower is better.\n")
            f.write("2. **Settling Time (2% and 5%)** - Faster is better.\n")
            f.write("3. **Steady-State Error** - Lower is better.\n")
            f.write("4. **Wind Disturbance Rejection** - Compare error metrics between wind/no-wind tests.\n\n")

            f.write("### Key Comparisons:\n")
            f.write("- **FLS Effectiveness**: Compare Test 01 vs Test 02.\n")
            f.write("- **Wind Impact**: Compare Test 01 vs Test 03.\n")
            f.write("- **Feed-Forward Benefit**: Compare Test 01 vs Test 04.\n")
            f.write("- **FLS vs Wind**: Compare Test 03 vs Test 05 to see if FLS mitigates wind effects.\n")
            f.write("- **Complete System**: Test 06 shows combined performance.\n\n")

            f.write("### Next Steps:\n")
            f.write("1. Analyze the generated CSV data in each test subfolder.\n")
            f.write("2. Generate comparative plots (e.g., position error vs. time).\n")
            f.write("3. Quantify improvements (e.g., % reduction in overshoot).\n")
            f.write("4. Document findings for the final report.\n")

        print(f"\nComparison analysis report generated: {report_path}")


if __name__ == "__main__":
    tester = SystematicTester()

    print("Multi-Agent Formation Control - Systematic Testing")
    print("=======================================================")
    print("This will run systematic tests to compare:")
    print("• Baseline PID vs FLS enhancement")
    print("• System performance with/without wind")
    print("• Feed-forward control benefits")
    print("• Combined system effectiveness")
    print()

    TOTAL_TESTS = 6

    try:
        confirm = input("Start systematic testing? (y/n): ").strip().lower()

        if confirm in ('y', 'yes'):
            print("\nStarting systematic test suite...")
            results = tester.run_feature_comparison_suite()

            # Generate summary
            successful_tests = sum(1 for status in results.values() if status)

            print("\n====================")
            print("Testing Complete!")
            print(f"Successful: {successful_tests}/{TOTAL_TESTS}")
            print(f"Results saved in: {tester.results_dir.resolve()}")
            print("Use the generated markdown report and CSV files for detailed analysis.")
            print("====================")

        else:
            print("Testing cancelled.")

    except KeyboardInterrupt:
        print("\nTesting interrupted by user.")
    except Exception as e:
        print(f"\nAn unexpected error occurred during testing: {e}")
