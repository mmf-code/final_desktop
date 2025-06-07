# Build Instructions for Modular PID System

## Quick Start - ZN Tuning Test

### Option 1: Using the Build Script (Recommended)

```powershell
# Navigate to the modular system directory
cd agent_control_pkg/src/modular_pid_system

# Build and run ZN tuning test directly
.\build_and_test.ps1 zn_tuning

# Or build everything and run all tests
.\build_and_test.ps1 all

# Clean build if needed
.\build_and_test.ps1 zn_tuning -Clean -Verbose
```

### Option 2: Manual Build

```bash
# Navigate to project root
cd agent_control_pkg

# Create build directory
mkdir build
cd build

# Configure with CMake
cmake .. -DCMAKE_BUILD_TYPE=Release

# Build the project
cmake --build . --config Release

# Run ZN tuning test
./src/modular_pid_system/zn_tuning_test.exe
```

## Available Tests

- **`pid_only`** - Pure PID control testing
- **`pid_wind`** - PID with wind disturbance testing
- **`pid_wind_flc`** - PID + Fuzzy Logic Controller testing
- **`zn_tuning`** - Ziegler-Nichols auto-tuning

## Test-Specific Commands

```powershell
# Run individual tests
.\build_and_test.ps1 pid_only
.\build_and_test.ps1 pid_wind
.\build_and_test.ps1 pid_wind_flc
.\build_and_test.ps1 zn_tuning

# Run all tests
.\build_and_test.ps1 all
```

## Output Locations

- **CSV Data**: `simulation_outputs/`
- **Metrics**: `simulation_outputs/`
- **ZN Tuning Results**: `simulation_outputs/zn_tuning_results/`

## Troubleshooting

### Build Errors

1. **yaml-cpp not found**: Ensure vcpkg is properly installed
2. **CMake errors**: Make sure you're in the correct directory
3. **Linker errors**: Try a clean build with `-Clean` flag

### Example Commands

```powershell
# Clean build with verbose output
.\build_and_test.ps1 zn_tuning -Clean -Verbose

# Quick test run
.\build_and_test.ps1 zn_tuning

# Test all scenarios
.\build_and_test.ps1 all
```

## Expected ZN Tuning Output

The ZN tuning test should produce:

1. **Console output** showing Kp values being tested
2. **Results summary** with ultimate gain (Ku) and period (Tu)
3. **Recommended PID gains** using different ZN methods
4. **CSV files** with detailed tuning data
5. **Report files** with analysis results

## Configuration

The ZN tuning uses these default settings:
- Kp range: 0.5 to 15.0 (step 0.5)
- Test duration: 30 seconds per Kp
- Target: Single drone, X-axis tuning
- Test setpoint: (5.0, 0.0)

These can be modified in the `zn_tuning_test.cpp` file. 

# Build everything and run all tests
.\build_and_test.ps1 all

# Or build everything then run ZN tuning specifically
.\build_and_test.ps1 all
.\build_and_test.ps1 zn_tuning 