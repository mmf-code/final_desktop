# Modular PID System for Multi-Drone Formation Control

This modular system provides a clean, separated architecture for testing different PID control scenarios with multi-drone formations. It's designed to be more maintainable and extensible than the original monolithic `multi_drone_pid_test_main.cpp`.

## Architecture Overview

The system is organized into several key components:

### Core Components

1. **`drone_system.hpp/cpp`** - Main system definitions and implementations
   - `SystemConfig` - Configuration structure for different test scenarios
   - `DroneState` - Individual drone state management
   - `DroneController` - Single drone PID and FLC control
   - `MultiDroneSystem` - Multi-drone coordination and simulation
   - `PerformanceMetrics` - Performance analysis and metrics collection
   - `WindModel` - Wind disturbance simulation

2. **`zn_tuning.hpp`** - Ziegler-Nichols auto-tuning functionality
   - `ZNTuningEngine` - Automated PID tuning
   - `ZNAnalysisResult` - Tuning results and recommendations
   - `BatchZNTuning` - Multi-scenario tuning comparison

### Test Scenarios

The system supports three main test scenarios:

1. **PID Only** (`pid_only_test.cpp`)
   - Pure PID control without disturbances
   - Baseline performance measurement
   - Formation control testing

2. **PID + Wind** (`pid_wind_test.cpp`)
   - PID control with wind disturbances
   - Disturbance rejection analysis
   - Variable wind intensity testing

3. **PID + Wind + FLC** (`pid_wind_flc_test.cpp`)
   - PID with Fuzzy Logic Controller enhancement
   - Wind compensation using FLC
   - Performance comparison with PID-only

4. **ZN Tuning** (`zn_tuning_test.cpp`)
   - Automated Ziegler-Nichols tuning
   - Ultimate gain and period detection
   - Recommended PID gains calculation

## Key Features

### Modular Design
- **Separation of Concerns**: Each component has a specific responsibility
- **Configurable Scenarios**: Easy switching between test scenarios
- **Reusable Components**: Common functionality shared across tests
- **Extensible Architecture**: Easy to add new test scenarios or features

### Configuration System
```cpp
SystemConfig config = createPIDOnlyConfig();
config.num_drones = 3;
config.simulation_time = 60.0;
config.enable_csv_output = true;
config.test_name = "my_test";
```

### Scenario Management
```cpp
// Switch between scenarios dynamically
system.setScenario(SystemConfig::TestScenario::PID_WIND_FLC);

// Or create specific configurations
SystemConfig pid_config = createPIDOnlyConfig();
SystemConfig wind_config = createPIDWindConfig();
SystemConfig flc_config = createPIDWindFLCConfig();
```

### Performance Analysis
- Automatic metrics collection (overshoot, settling time, peak values)
- Phase-based analysis for multi-stage tests
- CSV data export for detailed analysis
- Comparison between different scenarios

## Usage Examples

### Basic PID Testing
```bash
# Build the system
cd agent_control_pkg/src/modular_pid_system
mkdir build && cd build
cmake ..
make

# Run PID-only test
./pid_only_test

# Run PID with wind disturbance
./pid_wind_test

# Run PID with wind and fuzzy logic
./pid_wind_flc_test
```

### Custom Configuration
```cpp
// Create custom configuration
SystemConfig config;
config.current_scenario = SystemConfig::TestScenario::PID_ONLY;
config.num_drones = 5;
config.simulation_time = 120.0;
config.dt = 0.005;  // Higher resolution
config.test_name = "custom_formation_test";

// Initialize system
MultiDroneSystem system(config);
system.initialize();

// Set custom formation
auto formation = createLineFormation(3.0);  // Line formation with 3m spacing
system.setFormationOffsets(formation);
```

### ZN Auto-Tuning
```cpp
// Configure tuning parameters
ZNTuningConfig zn_config = createDetailedTuningConfig();
zn_config.kp_start = 0.1;
zn_config.kp_end = 20.0;
zn_config.kp_step = 0.2;

// Run auto-tuning
ZNTuningEngine tuning(&system, zn_config);
ZNAnalysisResult result = tuning.performAutoTuning();

// Get recommended gains
for (const auto& gains : result.recommended_gains) {
    std::cout << gains.method_name << ": Kp=" << gains.kp 
              << ", Ki=" << gains.ki << ", Kd=" << gains.kd << std::endl;
}
```

## Configuration Files

The system uses the same YAML configuration files as the original system:

- **`simulation_params.yaml`** - Simulation parameters and phases
- **`pid_params.yaml`** - PID controller parameters
- **`fuzzy_params.yaml`** - Fuzzy logic controller parameters

## Output Structure

Each test generates organized output:

```
simulation_outputs/
├── pid_only_formation_test_data.csv
├── pid_only_formation_test_metrics.txt
├── pid_wind_formation_test_data.csv
├── pid_wind_formation_test_metrics.txt
└── zn_tuning_results/
    ├── zn_tuning_results.txt
    ├── batch_comparison_report.txt
    └── detailed_analysis/
```

## Comparison with Original System

### Advantages of Modular System

1. **Maintainability**
   - Clear separation of functionality
   - Easier to debug and modify
   - Reduced code duplication

2. **Flexibility**
   - Easy to add new test scenarios
   - Configurable without code changes
   - Modular component testing

3. **Reusability**
   - Components can be used independently
   - Common functionality shared
   - Easy integration with other systems

4. **Testing**
   - Individual component testing
   - Scenario-specific testing
   - Automated tuning capabilities

### Migration from Original

The modular system maintains compatibility with existing configurations while providing enhanced functionality:

- Same YAML configuration files
- Similar performance metrics
- Enhanced CSV output format
- Additional analysis capabilities

## Development Guidelines

### Adding New Test Scenarios

1. Create new test scenario enum in `SystemConfig::TestScenario`
2. Implement scenario-specific logic in `MultiDroneSystem::setScenario()`
3. Create new test executable (e.g., `my_new_test.cpp`)
4. Add to CMakeLists.txt

### Extending Functionality

1. **New Controllers**: Extend `DroneController` class
2. **New Metrics**: Add to `PerformanceMetrics` structure
3. **New Formations**: Add utility functions for formation patterns
4. **New Analysis**: Extend `ZNTuningUtils` for additional analysis

### Best Practices

- Use configuration objects instead of hardcoded values
- Implement proper error handling and validation
- Document new functionality and parameters
- Maintain backward compatibility when possible
- Write unit tests for new components

## Future Enhancements

- [ ] Complete ZN tuning implementation
- [ ] Add more formation patterns
- [ ] Implement adaptive control strategies
- [ ] Add real-time visualization
- [ ] Support for different drone dynamics models
- [ ] Integration with ROS topics and services
- [ ] Machine learning-based tuning
- [ ] Multi-objective optimization

## Troubleshooting

### Common Issues

1. **Build Errors**: Ensure yaml-cpp is properly installed
2. **Configuration Errors**: Check YAML file syntax and paths
3. **Performance Issues**: Adjust simulation time step and duration
4. **Tuning Problems**: Verify Kp range and system stability

### Debug Mode

Enable detailed logging by setting:
```cpp
config.enable_console_output = true;
// Reduce console_interval for more frequent updates
```

## Contributing

When contributing to this modular system:

1. Follow the existing code structure and naming conventions
2. Add appropriate documentation and comments
3. Test new functionality with multiple scenarios
4. Update this README with new features or changes
5. Maintain compatibility with existing configurations 