# 🚁 Multi-Agent Formation Control Simulation Guide

This comprehensive guide covers all ways to run simulations with your multi-agent formation control system based on `multi_drone_pid_test_main.cpp`.

## 📋 Table of Contents
1. [Quick Start](#quick-start)
2. [Simulation Modes](#simulation-modes)
3. [Configuration Options](#configuration-options)
4. [Running Different Test Scenarios](#running-different-test-scenarios)
5. [Parameter Tuning Modes](#parameter-tuning-modes)
6. [Advanced Features](#advanced-features)
7. [Output and Analysis](#output-and-analysis)
8. [Troubleshooting](#troubleshooting)

---

## 🚀 Quick Start

### Prerequisites
1. ROS2 workspace built and sourced
2. All dependencies installed
3. Configuration files in `agent_control_pkg/config/`

### Basic Simulation Run
```bash
# Navigate to your workspace
cd /path/to/your/workspace

# Build the package
colcon build --packages-select agent_control_pkg

# Source the workspace
source install/setup.bash

# Run the basic simulation
./build/agent_control_pkg/multi_drone_pid_test_main
```

---

## 🎮 Simulation Modes

Your system supports 6 different simulation modes:

### 1. **Normal Simulation Mode** (Default)
- Uses PID + FLS controllers as configured
- Follows formation phases with wind disturbances
- **Usage**: Just run the executable without special flags

### 2. **Ziegler-Nichols Auto-Search Mode**
- Automatically finds Ultimate Gain (Ku) and Period (Pu)
- Tests multiple Kp values to find oscillation point
- **Configuration**: Set in `simulation_params.yaml`:
```yaml
ziegler_nichols_tuning:
  enable_auto_search: true
  auto_search_kp_start: 10.0
  auto_search_kp_step: 5.0
  auto_search_kp_max: 200.0
  simulation_time: 20.0
```

### 3. **Ziegler-Nichols Manual Testing Mode**
- Tests a specific Kp value with Ki=Kd=0
- Useful for manual oscillation analysis
- **Configuration**:
```yaml
ziegler_nichols_tuning:
  enable: true
  kp_test_value: 3.5
  simulation_time: 15.0
```

### 4. **Quick ZN Method Testing Mode**
- Applies pre-calculated ZN formulas
- Tests different ZN variations quickly
- **Configuration**: Set in `simulation_params.yaml`:
```yaml
ziegler_nichols_tuning:
  quick_method_test: 'Classic'  # Options: Classic, Conservative, LessOvershoot, NoOvershoot, Aggressive, ModernZN
  manual_ku: 150.0
  manual_pu: 12.0
```

### 5. **Systematic Testing Mode**
- Run predefined test scenarios via PowerShell scripts
- **Usage**:
```powershell
# Run all 6 systematic tests
.\run_systematic_tests.ps1

# Or run specific tests
.\quick_test.ps1
```

### 6. **Custom Parameter Testing**
- Test specific parameter combinations
- Modify `simulation_params.yaml` directly

---

## ⚙️ Configuration Options

### Main Configuration File: `agent_control_pkg/config/simulation_params.yaml`

#### **Controller Settings**
```yaml
controller_settings:
  pid:
    kp: 7.0          # Proportional gain
    ki: 1.0          # Integral gain  
    kd: 4.5          # Derivative gain
    
    # Output limits
    output_min_x: -10.0
    output_max_x: 10.0
    output_min_y: -8.0
    output_max_y: 12.0
    
    # Advanced features
    enable_derivative_filter: true
    derivative_filter_alpha: 0.1
    enable_feedforward: true
    velocity_feedforward_gain: 0.8
    acceleration_feedforward_gain: 0.1
    
  fls:
    enable: true
    params_file: fuzzy_params.yaml
```

#### **Scenario Settings**
```yaml
scenario_settings:
  enable_wind: true                    # Enable/disable wind disturbances
  formation_side_length: 4.0           # Formation size
  
  formation:
    initial_positions:                 # Starting positions for each drone
      drone_0: [0.0, 1.15]
      drone_1: [-2.0, -2.0]
      drone_2: [2.0, -2.0]
      
  phases:                              # Formation movement phases
    - center: [5.0, 5.0]
      start_time: 0.0
    - center: [-5.0, 0.0]
      start_time: 30.0
    # ... more phases
```

#### **Wind Configuration**
```yaml
wind:
  phases:
    - phase: 1
      time_windows:
        - start_time: 5.0
          end_time: 15.0
          force: [3.0, 0.0]           # [x_force, y_force] in Newtons
          is_sine_wave: false
        - start_time: 10.0
          end_time: 20.0
          force: [0.0, -1.0]
          is_sine_wave: false
```

---

## 🧪 Running Different Test Scenarios

### **Scenario 1: Baseline Test (PID Only)**
```yaml
# In simulation_params.yaml
controller_settings:
  fls:
    enable: false
scenario_settings:
  enable_wind: false
```

### **Scenario 2: FLS Effectiveness Test**
```yaml
controller_settings:
  fls:
    enable: true
scenario_settings:
  enable_wind: false
```

### **Scenario 3: Wind Disturbance Test**
```yaml
controller_settings:
  fls:
    enable: false
scenario_settings:
  enable_wind: true
```

### **Scenario 4: Full System Test**
```yaml
controller_settings:
  fls:
    enable: true
scenario_settings:
  enable_wind: true
```

### **Scenario 5: Feed-Forward Only Test**
```yaml
controller_settings:
  pid:
    enable_feedforward: true
  fls:
    enable: false
scenario_settings:
  enable_wind: false
```

### **Scenario 6: Custom PID Parameters**
Use configurations from `test_faster_settling.yaml`:

**Test 1 - Moderate:**
```yaml
controller_settings:
  pid:
    kp: 4.5
    ki: 0.6
    kd: 3.0
```

**Test 2 - Balanced (Recommended):**
```yaml
controller_settings:
  pid:
    kp: 5.5
    ki: 0.8
    kd: 3.5
```

**Test 3 - Aggressive:**
```yaml
controller_settings:
  pid:
    kp: 7.0
    ki: 1.0
    kd: 4.5
```

---

## 🔧 Parameter Tuning Modes

### **1. Auto-Discovery of Ku and Pu**
Best for finding optimal parameters for your system:

```yaml
ziegler_nichols_tuning:
  enable_auto_search: true
  auto_search_kp_start: 10.0      # Start testing from this Kp
  auto_search_kp_step: 5.0        # Increment step
  auto_search_kp_max: 200.0       # Maximum Kp to test
  simulation_time: 20.0           # Test duration for each Kp
```

**Expected Output:**
- Console output showing Kp values being tested
- Oscillation detection results
- Recommended parameter sets for 6 different ZN methods

### **2. Manual Ku/Pu Testing**
When you know approximate values:

```yaml
ziegler_nichols_tuning:
  enable: true
  kp_test_value: 3.5             # Test this specific Kp
  simulation_time: 15.0
```

### **3. Quick Method Comparison**
Test different ZN variations:

```yaml
ziegler_nichols_tuning:
  quick_method_test: 'Conservative'   # Options below
  manual_ku: 150.0                    # Your known Ku value
  manual_pu: 12.0                     # Your known Pu value
```

**Available Methods:**
- `Classic`: Balanced response
- `Conservative`: Less overshoot, slower
- `LessOvershoot`: Minimal overshoot
- `NoOvershoot`: Guaranteed no overshoot
- `Aggressive`: Fast response, higher overshoot
- `ModernZN`: Updated ZN approach

---

## 🎯 Advanced Features

### **1. Derivative Filtering**
Reduces noise in the derivative term:
```yaml
enable_derivative_filter: true
derivative_filter_alpha: 0.1       # 0.05-0.2 typical range
```

### **2. Feed-Forward Control**
Improves tracking performance:
```yaml
enable_feedforward: true
velocity_feedforward_gain: 0.8      # Velocity compensation
acceleration_feedforward_gain: 0.1  # Acceleration compensation
```

### **3. Anti-Windup Protection**
Built-in output saturation limits prevent integral windup:
```yaml
output_min_x: -10.0
output_max_x: 10.0
output_min_y: -8.0
output_max_y: 12.0
```

### **4. Realistic Drone Physics**
The simulation includes:
- **Mass**: 1.5 kg (typical quadcopter)
- **Drag coefficient**: 0.1 (air resistance)
- **Max acceleration**: 15.0 m/s²
- **Response delay**: 0.1s (motor/ESC lag)
- **Max velocity**: 20.0 m/s

---

## 📊 Output and Analysis

### **Simulation Outputs**
All results are saved to `simulation_outputs/` directory:

1. **CSV Files**: `multi_agent_sim_TIMESTAMP.csv`
   - Time series data for all drones
   - Position, velocity, errors, control outputs
   - Target positions and wind forces

2. **Metrics Files**: `metrics_sim_TIMESTAMP.csv`
   - Performance metrics per drone per phase
   - Overshoot, settling time, steady-state error
   - Calculated automatically

3. **ZN Analysis Reports**: `zn_analysis_report_TIMESTAMP.txt`
   - Ultimate gain and period results
   - Recommended parameter sets
   - Oscillation analysis

### **Key Performance Metrics**
- **Overshoot**: Peak deviation from target (%)
- **Settling Time**: Time to reach ±2% of target
- **Steady-State Error**: Final error after settling
- **Peak Time**: Time to reach maximum/minimum
- **Formation Error**: Distance from desired formation

### **Analysis Tools**
Use provided Python scripts for detailed analysis:

```bash
# Run automatic analysis
python analyze_test_results.py

# Generate comparison plots
python final_project_analysis.py

# Create systematic test summary
python systematic_testing.py
```

---

## 🛠️ Troubleshooting

### **Common Issues**

1. **Build Errors**
   ```bash
   # Clean and rebuild
   colcon build --packages-select agent_control_pkg --cmake-clean-cache
   ```

2. **Config File Not Found**
   - Ensure `simulation_params.yaml` is in `agent_control_pkg/config/`
   - Check file permissions

3. **Simulation Unstable**
   - Reduce PID gains
   - Enable derivative filtering
   - Check output limits

4. **No Oscillations in ZN Mode**
   - Increase `auto_search_kp_max`
   - Reduce `auto_search_kp_step` for finer search
   - Check that integral and derivative gains are zero

5. **FLS Not Working**
   - Verify `fuzzy_params.yaml` exists
   - Check FLS enable flag: `fls.enable: true`
   - Ensure fuzzy rules are properly formatted

### **Performance Optimization**

1. **For Faster Settling**:
   - Increase Kp gradually
   - Use feed-forward control
   - Enable derivative filtering

2. **For Less Overshoot**:
   - Reduce Kp slightly
   - Increase Kd
   - Use Conservative ZN method

3. **For Better Disturbance Rejection**:
   - Enable FLS
   - Increase Ki slightly
   - Use wind-resistant formation

---

## 📝 Quick Reference Commands

```bash
# Basic simulation
./build/agent_control_pkg/multi_drone_pid_test_main

# Run systematic tests (PowerShell)
.\run_systematic_tests.ps1

# Quick single test
.\quick_test.ps1

# Analyze results
python analyze_test_results.py

# Generate comprehensive analysis
python final_project_analysis.py
```

---

## 🎓 Recommended Testing Workflow

1. **Start with Auto ZN Search** to find baseline parameters
2. **Test different ZN methods** to compare performance
3. **Run systematic scenarios** (Baseline, FLS, Wind, Full)
4. **Fine-tune parameters** based on results
5. **Generate final analysis** with plotting tools
6. **Document best configuration** for your application

This guide should help you run comprehensive simulations and analyze the effectiveness of your FLS system against wind disturbances! 