cmake_minimum_required(VERSION 3.16)
project(agent_control_standalone_test CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF) # Good practice for portability

# Define an INTERFACE library for our PID controller logic
# This makes it easy to link against if we had multiple executables
# or if other local projects needed to use it.
add_library(pid_logic_lib
    src/pid_controller.cpp
)
target_include_directories(pid_logic_lib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Define the executable for our standalone test
add_executable(pid_standalone_tester
    src/agent_control_main.cpp
)

# Link our tester against the PID logic library
target_link_libraries(pid_standalone_tester PRIVATE pid_logic_lib)

# --- Later, when this becomes a ROS2 package, this file will look very different ---
# --- and will use ament_cmake functions. For now, this is for local Windows build. ---

# Optional: Add install rules if want to practice that
# install(TARGETS pid_standalone_tester
#     DESTINATION bin)
# install(TARGETS pid_logic_lib
#     ARCHIVE DESTINATION lib
#     LIBRARY DESTINATION lib
#     RUNTIME DESTINATION bin)
# install(DIRECTORY include/
#     DESTINATION include)