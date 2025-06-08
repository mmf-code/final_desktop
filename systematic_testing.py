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


class SystematicTester:
    """Systematic tester for multi-agent formation control system."""

    def __init__(self):
        self.base_config_path = "agent_control_pkg/config/simulation_params.yaml"
        self.external_config_path = (
            "C:/Users/ataka/OneDrive/Masaüstü/config/"
            "simulation_params.yaml"
        )
        self.executable_path = (
            "./build/Release/multi_drone_pid_tester.exe"
        )
        self.results_dir = "systematic_test_results"
        self.timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

        # Create results directory
        Path(self.results_dir).mkdir(exist_ok=True)

    def update_config_value(self, config_path, key_path, value):
        """Update a specific configuration value using PowerShell"""
        ps_command = f"""
        $config = Get-Content "{config_path}" | ConvertFrom-Yaml
        $keys = "{key_path}".Split('.')
        $current = $config
        for ($i = 0; $i -lt $keys.Length - 1; $i++) {{
            $current = $current[$keys[$i]]
        }}
        $current[$keys[-1]] = {str(value).lower() if isinstance(value, bool) else value}
        $config | ConvertTo-Yaml | Set-Content "{config_path}"
        """
        subprocess.run(["powershell", "-Command", ps_command], check=False)

    def run_test(self, test_name, config_modifications):
        """Run a single test with specific configuration"""
        print(f"\n🔬 Running Test: {test_name}")
        print("=" * 50)

        # Apply configuration modifications using PowerShell
        for key_path, value in config_modifications.items():
            if key_path == "controller_settings.fls.enable":
                ps_cmd = (
                    f'$config = Get-Content "{self.external_config_path}"; '
                    f'$flsLine = $config | Select-String -Pattern "enable:" | '
                    f'Select-Object -Index 1; if ($flsLine) {{ '
                    f'$lineNum = $flsLine.LineNumber - 1; '
                    f'$config[$lineNum] = "    enable: {str(value).lower()}"; '
                    f'$config | Set-Content "{self.external_config_path}" }}'
                )
            elif key_path == "scenario_settings.enable_wind":
                ps_cmd = (
                    f'$config = Get-Content "{self.external_config_path}"; '
                    f'$windLine = $config | Select-String -Pattern '
                    f'"enable_wind:" | Select-Object -First 1; if ($windLine) {{ '
                    f'$lineNum = $windLine.LineNumber - 1; '
                    f'$config[$lineNum] = "  enable_wind: {str(value).lower()}"; '
                    f'$config | Set-Content "{self.external_config_path}" }}'
                )
            elif key_path == "controller_settings.pid.enable_feedforward":
                ps_cmd = (
                    f'$config = Get-Content "{self.external_config_path}"; '
                    f'$ffLine = $config | Select-String -Pattern '
                    f'"enable_feedforward:" | Select-Object -First 1; '
                    f'if ($ffLine) {{ $lineNum = $ffLine.LineNumber - 1; '
                    f'$config[$lineNum] = "    enable_feedforward: '
                    f'{str(value).lower()}"; '
                    f'$config | Set-Content "{self.external_config_path}" }}'
                )
            elif key_path == "controller_settings.pid.enable_derivative_filter":
                ps_cmd = (
                    f'$config = Get-Content "{self.external_config_path}"; '
                    f'$dfLine = $config | Select-String -Pattern '
                    f'"enable_derivative_filter:" | Select-Object -First 1; '
                    f'if ($dfLine) {{ $lineNum = $dfLine.LineNumber - 1; '
                    f'$config[$lineNum] = "    enable_derivative_filter: '
                    f'{str(value).lower()}"; '
                    f'$config | Set-Content "{self.external_config_path}" }}'
                )
            else:
                continue  # Skip unsupported modifications for now

            subprocess.run(["powershell", "-Command", ps_cmd], check=False)

        print(f"Configuration applied: {config_modifications}")

        # Brief pause to ensure config is written
        time.sleep(1)

        # Run the test
        try:
            result = subprocess.run([self.executable_path],
                                    capture_output=True, text=True, timeout=180,
                                    check=False)

            if result.returncode == 0:
                print("✅ Test completed successfully")
                self.organize_results(test_name)
                return True
            else:
                print(f"❌ Test failed with return code: {result.returncode}")
                return False

        except subprocess.TimeoutExpired:
            print("⏰ Test timed out after 3 minutes")
            return False
        except Exception as e:
            print(f"💥 Test failed with exception: {e}")
            return False

    def organize_results(self, test_name):
        """Organize test results into structured directories"""
        test_dir = Path(self.results_dir) / f"{self.timestamp}_{test_name}"
        test_dir.mkdir(exist_ok=True)

        # Move CSV and metrics files
        output_dir = Path("simulation_outputs")
        if output_dir.exists():
            for file in output_dir.glob("*.csv"):
                # Modified in last 3 minutes
                if file.stat().st_mtime > time.time() - 180:
                    shutil.copy2(file, test_dir / f"{test_name}_{file.name}")

            for file in output_dir.glob("*.txt"):
                # Modified in last 3 minutes
                if file.stat().st_mtime > time.time() - 180:
                    shutil.copy2(file, test_dir / f"{test_name}_{file.name}")

        print(f"📁 Results saved to: {test_dir}")

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
                    # Keep critical fixes
                    "controller_settings.pid.enable_derivative_filter": True
                }
            },
            {
                "name": "02_FLS_Effect",
                "config": {
                    "controller_settings.fls.enable": True,
                    "scenario_settings.enable_wind": False,
                    "controller_settings.pid.enable_feedforward": False,
                    "controller_settings.pid.enable_derivative_filter": True
                }
            },
            {
                "name": "03_Wind_Challenge",
                "config": {
                    "controller_settings.fls.enable": False,
                    "scenario_settings.enable_wind": True,
                    "controller_settings.pid.enable_feedforward": False,
                    "controller_settings.pid.enable_derivative_filter": True
                }
            },
            {
                "name": "04_FeedForward_Benefit",
                "config": {
                    "controller_settings.fls.enable": False,
                    "scenario_settings.enable_wind": False,
                    "controller_settings.pid.enable_feedforward": True,
                    "controller_settings.pid.enable_derivative_filter": True
                }
            },
            {
                "name": "05_FLS_vs_Wind",
                "config": {
                    "controller_settings.fls.enable": True,
                    "scenario_settings.enable_wind": True,
                    "controller_settings.pid.enable_feedforward": False,
                    "controller_settings.pid.enable_derivative_filter": True
                }
            },
            {
                "name": "06_Full_System_Optimal",
                "config": {
                    "controller_settings.fls.enable": True,
                    "scenario_settings.enable_wind": True,
                    "controller_settings.pid.enable_feedforward": True,
                    "controller_settings.pid.enable_derivative_filter": True
                }
            }
        ]

        print(f"🚀 Starting Feature Comparison Suite "
              f"({len(test_scenarios)} tests)")
        print(f"📅 Timestamp: {self.timestamp}")
        print("=" * 60)

        test_results = {}
        for i, scenario in enumerate(test_scenarios, 1):
            print(f"\n📊 Progress: {i}/{len(test_scenarios)}")
            success = self.run_test(scenario["name"], scenario["config"])
            test_results[scenario["name"]] = success
            time.sleep(2)  # Brief pause between tests

        self.generate_comparison_report(test_results)
        return test_results

    def generate_comparison_report(self, test_results):
        """Generate comparison analysis report"""
        report_path = (Path(self.results_dir) /
                       f"comparison_analysis_{self.timestamp}.md")

        with open(report_path, 'w', encoding='utf-8') as f:
            f.write("# Multi-Agent Formation Control - "
                    "Feature Comparison Analysis\n\n")
            f.write(f"**Test Date:** "
                    f"{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f"**Test ID:** {self.timestamp}\n\n")

            f.write("## Test Results Summary\n\n")
            f.write("| Test Name | Status | Description |\n")
            f.write("|-----------|--------|-------------|\n")

            descriptions = {
                "01_Baseline_Clean": "Basic PID with improvements, "
                                     "no FLS/Wind/FF",
                "02_FLS_Effect": "Baseline + Fuzzy Logic System",
                "03_Wind_Challenge": "Baseline + Wind Disturbances",
                "04_FeedForward_Benefit": "Baseline + Feed-Forward Control",
                "05_FLS_vs_Wind": "FLS handling wind disturbances",
                "06_Full_System_Optimal": "All features enabled"
            }

            for test_name, success in test_results.items():
                status = "✅ PASS" if success else "❌ FAIL"
                desc = descriptions.get(test_name, "Custom test")
                f.write(f"| {test_name} | {status} | {desc} |\n")

            f.write("\n## Analysis Instructions\n\n")
            f.write("### Performance Metrics to Compare:\n")
            f.write("1. **Overshoot Percentage** - Lower is better\n")
            f.write("2. **Settling Time (2% and 5%)** - Faster is better\n")
            f.write("3. **Steady-State Error** - Lower is better\n")
            f.write("4. **Wind Disturbance Rejection** - "
                    "Compare wind vs no-wind tests\n\n")

            f.write("### Key Comparisons:\n")
            f.write("- **FLS Effectiveness**: Compare Test 01 vs Test 02\n")
            f.write("- **Wind Impact**: Compare Test 01 vs Test 03\n")
            f.write("- **Feed-Forward Benefit**: Compare Test 01 vs Test 04\n")
            f.write("- **FLS vs Wind**: Test 05 shows FLS handling "
                    "disturbances\n")
            f.write("- **Complete System**: Test 06 shows optimal "
                    "performance\n\n")

            f.write("### Next Steps:\n")
            f.write("1. Run analysis scripts on the generated CSV data\n")
            f.write("2. Generate comparative plots\n")
            f.write("3. Calculate improvement percentages\n")
            f.write("4. Document findings for final report\n")

        print(f"\n📋 Comparison analysis generated: {report_path}")


if __name__ == "__main__":
    tester = SystematicTester()

    print("🎯 Multi-Agent Formation Control - Systematic Testing")
    print("=" * 55)
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
            print("\n🚀 Starting systematic test suite...")
            results = tester.run_feature_comparison_suite()

            # Generate summary
            successful_tests = sum(results.values())

            print("\n🎉 Testing Complete!")
            print(f"✅ Successful: {successful_tests}/{TOTAL_TESTS}")
            print(f"📁 Results saved in: {tester.results_dir}")
            print("📊 Use the generated CSV files for detailed analysis")

        else:
            print("Testing cancelled.")

    except KeyboardInterrupt:
        print("\n⚠️ Testing interrupted by user")
    except Exception as e:
        print(f"\n💥 Testing failed: {e}") 