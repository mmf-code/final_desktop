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
*   `multi_drone_sim_data_<timestamp>.csv`: Generated in the execution directory with the current date and time appended to the filename. It contains detailed time-series data of the simulation, including drone positions, velocities, errors, and PID controller term values.
*   `performance_metrics_<timestamp>_pid.txt` (or `_fls.txt`): Also generated in the execution directory with a timestamp. This file provides performance metrics for each drone's controller, such as peak overshoot and settling time.
*   **Plotting Utility**:
*   `agent_control_pkg/build/Debug/plot_pid_data.py`: A Python script (may require adjustment based on actual build path) intended for visualizing the data from the generated CSV file.

## Planned ROS 2 Integration (Future Work)

The project is evolving towards a ROS 2 architecture with the following packages:

*   **`agent_control_pkg`**: This package is being refactored into a ROS 2 node responsible for the control of individual drones.
    *   It will continue to use the PID controller (implemented in `pid_controller.hpp`/`.cpp`).
    *   It now includes an implementation of a General Type-2 Fuzzy Logic System (GT2FLS) for more advanced control strategies.
        *   **Implementation:** `agent_control_pkg/include/agent_control_pkg/gt2_fuzzy_logic_system.hpp` and `agent_control_pkg/src/gt2_fuzzy_logic_system.cpp`.
        *   **Functionality:** This system takes crisp inputs for `error`, `dError` (derivative of error), and `wind`. It uses Interval Type-2 Triangular Fuzzy Sets for inputs and fuzzifies them. Rules are defined by specifying antecedents (conditions on input variables) and a consequent (output fuzzy set). The current implementation uses a simplified weighted average for rule aggregation and placeholder methods for Type-2 type reduction and defuzzification (averaging the resulting interval).
        *   **Configuration:** The FLS is configured programmatically by adding input/output variables, defining their fuzzy sets, and adding rules. The `agent_control_pkg/config/fuzzy_params.yaml` is available for future parameterization.
        *   **Testing:** A basic test executable `fuzzy_test_main.cpp` exists in `agent_control_pkg/src/` which demonstrates how to set up and run the FLS.
        *   **ROS 2 Node Integration:** The intended ROS 2 node `agent_controller_node.cpp` that would utilize this FLS is currently a placeholder and does not yet integrate the FLS.
    *   Configuration parameters for PID and Fuzzy Logic controllers will reside in `agent_control_pkg/config/`.
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
    Use the `--use-fls` flag to enable the fuzzy logic system at runtime.
3.  The simulation will run, creating timestamped output files (e.g., `multi_drone_sim_data_20240101_120000.csv` and `performance_metrics_20240101_120000_pid.txt`) in the same directory where you ran the executable.

## Project Structure

This overview highlights the key code, configuration, and documentation files you'll typically interact with.

```
.
├── .gitattributes
├── .gitignore
├── README.md                         # This file
├── agent_control_pkg/                # For individual agent control logic
│   ├── config/                       # Parameters and settings
│   │   ├── fuzzy_params.yaml         # For the Fuzzy Logic System
│   │   └── pid_params.yaml           # For the PID Controller
│   ├── docs/                         # Agent-specific documentation & diagrams
│   │   ├── drone_0_detailed_analysis.png
│   │   ├── drone_1_detailed_analysis.png
│   │   ├── drone_2_detailed_analysis.png
│   │   └── overall_formation_analysis.png
│   ├── include/agent_control_pkg/    # C++ Header files (.hpp)
│   │   ├── agent_controller_node.hpp
│   │   ├── gt2_fuzzy_logic_system.hpp
│   │   └── pid_controller.hpp
│   ├── launch/                       # ROS 2 Launch files
│   │   └── agent_controller.launch.py
│   └── src/                          # C++ Source files (.cpp)
│       ├── agent_control_main.cpp    # Legacy standalone main
│       ├── agent_controller_node.cpp # (Currently placeholder)
│       ├── fuzzy_test_main.cpp       # Test for GT2FLS
│       ├── gt2_fuzzy_logic_system.cpp
│       └── pid_controller.cpp
├── docs/                             # General project documentation & diagrams
│   ├── drone_0_detailed_analysis_fls.png
│   ├── drone_1_detailed_analysis_fls.png
│   ├── drone_2_detailed_analysis_fls.png
│   ├── fls_control_surface.png
│   ├── fls_membership_functions.png
│   └── overall_formation_analysis_fls.png
├── formation_coordinator_pkg/        # For multi-agent formation coordination
│   ├── config/                       # Parameters and settings
│   │   ├── formation_config.yaml     # Formation definitions
│   │   └── pso_params.yaml           # For Particle Swarm Optimization
│   ├── include/formation_coordinator_pkg/ # C++ Header files (.hpp)
│   │   ├── formation_coordinator_node.hpp
│   │   └── particle_swarm_optimizer.hpp
│   ├── launch/                       # ROS 2 Launch files
│   │   └── formation_coordinator.launch.py
│   └── src/                          # C++ Source files (.cpp)
│       ├── formation_coordinator_main.cpp
│       ├── formation_coordinator_node.cpp
│       └── particle_swarm_optimizer.cpp
└── my_custom_interfaces_pkg/         # Custom message and service definitions
    ├── msg/                          # Message definitions (.msg)
    │   └── FormationState.msg
    └── srv/                          # Service definitions (.srv)
        └── UpdateFormation.srv
```

## Dependencies (for current C++ build)

*   **CMake** (>=3.16)
*   **C++17 compatible compiler**

## Contributing

Contributions to enhance the simulation, implement ROS 2 features, or improve documentation are welcome. Please consider opening an issue to discuss changes or submitting a pull request.

## License

This project is licensed under the [Apache License 2.0](LICENSE).
