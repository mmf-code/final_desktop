# Modular PID System CMakeLists.txt
cmake_minimum_required(VERSION 3.16)

# Include directories
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_SOURCE_DIR}/include)

# Create the modular drone system library
add_library(modular_drone_system
    drone_system.cpp
)

target_link_libraries(modular_drone_system
    pid_logic_lib      # Link against PIDController
    fuzzy_logic_lib    # Link against GT2FuzzyLogicSystem
    config_reader_lib  # Link against ConfigReader
)

# Create the ZN tuning library
add_library(zn_tuning
    zn_tuning.cpp
)

target_link_libraries(zn_tuning
    modular_drone_system
    pid_logic_lib      # Direct link for PID access
)

# PID Only Test Executable
add_executable(pid_only_test
    pid_only_test.cpp
)

target_link_libraries(pid_only_test
    modular_drone_system
    pid_logic_lib
    fuzzy_logic_lib
    config_reader_lib
)

# PID + Wind Test Executable
add_executable(pid_wind_test
    pid_wind_test.cpp
)

target_link_libraries(pid_wind_test
    modular_drone_system
    pid_logic_lib
    fuzzy_logic_lib
    config_reader_lib
)

# PID + Wind + FLC Test Executable
add_executable(pid_wind_flc_test
    pid_wind_flc_test.cpp
)

target_link_libraries(pid_wind_flc_test
    modular_drone_system
    pid_logic_lib
    fuzzy_logic_lib
    config_reader_lib
)

# ZN Tuning Test Executable
add_executable(zn_tuning_test
    zn_tuning_test.cpp
)

target_link_libraries(zn_tuning_test
    modular_drone_system
    zn_tuning
    pid_logic_lib
    fuzzy_logic_lib
    config_reader_lib
)

# Set C++ standard
set_property(TARGET modular_drone_system PROPERTY CXX_STANDARD 17)
set_property(TARGET zn_tuning PROPERTY CXX_STANDARD 17)
set_property(TARGET pid_only_test PROPERTY CXX_STANDARD 17)
set_property(TARGET pid_wind_test PROPERTY CXX_STANDARD 17)
set_property(TARGET pid_wind_flc_test PROPERTY CXX_STANDARD 17)
set_property(TARGET zn_tuning_test PROPERTY CXX_STANDARD 17)

# Install targets
install(TARGETS 
    modular_drone_system
    zn_tuning
    pid_only_test
    pid_wind_test
    pid_wind_flc_test
    zn_tuning_test
    DESTINATION bin
) 