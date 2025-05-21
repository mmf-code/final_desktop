# Multi-Drone Control Simulation

This project started as a C++ simulation environment for developing and testing drone control algorithms, primarily focusing on PID controllers. It is currently being refactored and expanded with the goal of becoming a ROS 2 based system for simulating and controlling multi-drone formations.

## Current Capabilities (Standalone C++ Simulation)

The `agent_control_pkg` contains the current standalone C++ simulation:

*   **`pid_logic_lib`**: A C++ library providing a robust PID controller implementation. Key files:
    *   `agent_control_pkg/include/agent_control_pkg/pid_controller.hpp`
    *   `agent_control_pkg/src/pid_controller.cpp`
*   **`pid_standalone_tester`**: A C++ executable that simulates multiple "fake" drones. It uses the `pid_logic_lib` to guide these drones to specific target coordinates, simulating formation flying. Key file:
    *   `agent_control_pkg/src/agent_control_main.cpp`
*   **Simulation Output**:
    *   `multi_drone_sim_data.csv`: Generated in the execution directory, this file contains detailed time-series data of the simulation, including drone positions, velocities, errors, and PID controller term values.
    *   `pid_performance_metrics.txt`: Also generated in the execution directory, this file provides performance metrics for each drone's PID controller, such as peak overshoot and settling time.
*   **Plotting Utility**:
    *   `agent_control_pkg/build/Debug/plot_pid_data.py`: A Python script (may require adjustment based on actual build path) intended for visualizing the data from `multi_drone_sim_data.csv`.

## Planned ROS 2 Integration (Future Work)

The project is evolving towards a ROS 2 architecture with the following packages:

*   **`agent_control_pkg`**: This package will be refactored into a ROS 2 node responsible for the control of individual drones.
    *   It will continue to use the PID controller.
    *   It plans to incorporate a General Type-2 Fuzzy Logic System (GT2FLS) for more advanced control strategies (see placeholder `agent_control_pkg/include/agent_control_pkg/gt2_fuzzy_logic_system.hpp`).
    *   Configuration parameters for PID and Fuzzy Logic controllers will likely reside in `agent_control_pkg/config/`.
*   **`formation_coordinator_pkg`**: This new ROS 2 package will manage and coordinate the formation of multiple drones.
    *   It may utilize optimization algorithms like Particle Swarm Optimization (PSO) (see placeholder `formation_coordinator_pkg/include/formation_coordinator_pkg/particle_swarm_optimizer.hpp`).
    *   Formation configurations will likely be defined in `formation_coordinator_pkg/config/formation_config.yaml`.
*   **`my_custom_interfaces_pkg`**: This package will define custom ROS 2 messages and services for communication between the `agent_control_pkg` and `formation_coordinator_pkg`. Examples:
    *   `my_custom_interfaces_pkg/msg/FormationState.msg`
    *   `my_custom_interfaces_pkg/srv/UpdateFormation.srv`

## Building and Running the Standalone Simulation

Currently, only the C++ standalone simulation within `agent_control_pkg` is buildable.

**Prerequisites:**

*   A C++ compiler supporting C++17 (e.g., GCC, Clang, MSVC)
*   CMake (version 3.16 or higher)

**Build Instructions:**

The `agent_control_pkg` is set up for a local build.

1.  Navigate to the `agent_control_pkg` directory:
    ```bash
    cd agent_control_pkg
    ```
2.  Create and navigate to a build directory:
    ```bash
    mkdir build
    cd build
    ```
3.  Configure the project with CMake:
    ```bash
    cmake ..
    ```
    *Note: If you are using Visual Studio, you might need to specify a generator, e.g., `cmake .. -G "Visual Studio 17 2022"`.*
4.  Build the project:
    ```bash
    cmake --build .
    ```
    *Note: For Visual Studio, you might build by opening the generated `.sln` file or using `cmake --build . --config Debug` (or `Release`).*

**Running the Simulation:**

1.  The executable `pid_standalone_tester` (or `pid_standalone_tester.exe` on Windows) will be located in a subdirectory within `agent_control_pkg/build/`, typically `Debug` or directly in `agent_control_pkg/build/Debug/` depending on your CMake setup and build type.
2.  Run the executable from its location. For example:
    ```bash
    ./Debug/pid_standalone_tester
    ```
    or on Windows:
    ```bash
    Debug\pid_standalone_tester.exe
    ```
3.  The simulation will run, and `multi_drone_sim_data.csv` and `pid_performance_metrics.txt` will be created in the same directory where you ran the executable.

## Project Structure

```
.
├── agent_control_pkg/                # Main package with current C++ PID simulation
│   ├── src/                          # Source files (.cpp)
│   ├── include/agent_control_pkg/    # Header files (.hpp)
│   ├── CMakeLists.txt                # Build script for the standalone simulation
│   ├── package.xml                   # ROS 2 package manifest (aspirational)
│   ├── config/                       # Placeholder for future ROS 2 params (pid_params.yaml, fuzzy_params.yaml)
│   └── launch/                       # Placeholder for future ROS 2 launch files
├── formation_coordinator_pkg/        # Placeholder for future ROS 2 formation coordination
│   ├── include/formation_coordinator_pkg/
│   ├── src/
│   ├── CMakeLists.txt                # Placeholder
│   ├── package.xml                   # Placeholder
│   ├── config/                       # Placeholder for (formation_config.yaml, pso_params.yaml)
│   └── launch/                       # Placeholder
├── my_custom_interfaces_pkg/         # Placeholder for future ROS 2 custom messages/services
│   ├── msg/FormationState.msg
│   ├── srv/UpdateFormation.srv
│   ├── CMakeLists.txt                # Placeholder
│   └── package.xml                   # Placeholder
├── .vscode/                          # VS Code specific settings
└── README.md                         # This file
```

## Dependencies (for current C++ build)

*   **CMake** (>=3.16)
*   **C++17 compatible compiler**

## Contributing

Contributions to enhance the simulation, implement ROS 2 features, or improve documentation are welcome. Please consider opening an issue to discuss changes or submitting a pull request.

## License

The `agent_control_pkg/package.xml` mentions the Apache License 2.0. Please add a `LICENSE` file to the root of the project if this is the chosen license.
