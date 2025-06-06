#!/usr/bin/env python3
import os
import subprocess
import glob
import yaml
import re
import time

CONFIG_PATH = os.path.join('agent_control_pkg', 'config', 'simulation_params.yaml')
BUILD_BIN = os.path.join('agent_control_pkg', 'build', 'multi_drone_pid_tester')
OUTPUT_DIR = os.path.join('agent_control_pkg', 'build', 'outputs')

# Regex to extract overshoot and settling time from metrics file
METRIC_RE = re.compile(r"OS=(?P<os>[0-9\.]+)%.*, ST\(2%\)=(?P<st>[0-9\.]+)s")


def load_config():
    with open(CONFIG_PATH, 'r') as f:
        return yaml.safe_load(f)


def save_config(cfg):
    with open(CONFIG_PATH, 'w') as f:
        yaml.dump(cfg, f)


def run_sim():
    cwd = os.path.dirname(BUILD_BIN)
    exe = os.path.basename(BUILD_BIN)
    subprocess.run([f"./{exe}"], check=True, cwd=cwd)


def find_latest_metrics(kp):
    pattern = os.path.join(OUTPUT_DIR, f"metrics_sim_Kp{kp:.3f}_Ki0.000_Kd0.000_*txt")
    files = glob.glob(pattern)
    if not files:
        return None
    latest = max(files, key=os.path.getmtime)
    return latest


def parse_metrics(path):
    if not path or not os.path.exists(path):
        return None, None
    with open(path, 'r') as f:
        for line in f:
            if line.strip().startswith('X-axis:'):
                m = METRIC_RE.search(line)
                if m:
                    return float(m.group('os')), float(m.group('st'))
    return None, None


def main():
    cfg = load_config()
    # Ensure ZN tuning disabled and wind off for consistent results
    cfg['simulation_settings']['ziegler_nichols_tuning']['enable'] = False
    cfg['scenario_settings']['enable_wind'] = False
    cfg['controller_settings']['pid']['ki'] = 0.0
    cfg['controller_settings']['pid']['kd'] = 0.0

    limit = int(os.environ.get('LIMIT_ITERS', '30'))
    kp_values = [round(x * 0.1, 2) for x in range(1, limit + 1)]  # 0.1 step
    results = []

    for kp in kp_values:
        cfg['controller_settings']['pid']['kp'] = kp
        save_config(cfg)

        run_sim()
        time.sleep(0.5)  # ensure files flushed
        metrics_file = find_latest_metrics(kp)
        os_val, st_val = parse_metrics(metrics_file)
        results.append((kp, os_val, st_val))
        print(f"Kp={kp:.2f} -> Overshoot={os_val}%, ST={st_val}s")

    print("\nSummary:")
    for kp, os_val, st_val in results:
        print(f"{kp:.2f}, OS={os_val}%, ST={st_val}s")

if __name__ == '__main__':
    main()
