cmake_minimum_required(VERSION 3.16)

# Set vcpkg toolchain file - ADD THIS BEFORE project()
set(CMAKE_TOOLCHAIN_FILE "C:/vcpkg/scripts/buildsystems/vcpkg.cmake" CACHE STRING "Vcpkg toolchain file")

project(agent_control_standalone_test CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Enable testing with CTest
enable_testing()
include(CTest)
include(GoogleTest)

# Find GTest package
find_package(GTest CONFIG REQUIRED)

# --- PID Logic Library ---
add_library(pid_logic_lib
    src/pid_controller.cpp
)
target_include_directories(pid_logic_lib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# --- Fuzzy Logic Library (will contain FLS implementation) ---
add_library(fuzzy_logic_lib
    src/gt2_fuzzy_logic_system.cpp
)
target_include_directories(fuzzy_logic_lib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# --- Executable for PID standalone test (NOW USES FLS) ---
add_executable(pid_standalone_tester
    src/agent_control_main.cpp
)
target_link_libraries(pid_standalone_tester PRIVATE
    pid_logic_lib
    fuzzy_logic_lib
)

# --- Executable for Fuzzy Logic standalone test ---
add_executable(fuzzy_standalone_tester
    src/fuzzy_test_main.cpp
)
target_link_libraries(fuzzy_standalone_tester PRIVATE fuzzy_logic_lib)

# --- Unit Tests ---
add_executable(test_pid_controller
    test/test_pid_controller.cpp
)
target_include_directories(test_pid_controller PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(test_pid_controller PRIVATE
    pid_logic_lib
    GTest::gtest
    GTest::gtest_main
)

# Discover tests automatically
gtest_discover_tests(test_pid_controller)

# Optional: Install rules
# install(TARGETS pid_standalone_tester fuzzy_standalone_tester DESTINATION bin)
# install(TARGETS pid_logic_lib fuzzy_logic_lib
#     ARCHIVE DESTINATION lib
#     LIBRARY DESTINATION lib
#     RUNTIME DESTINATION bin)
# install(DIRECTORY include/ DESTINATION include)