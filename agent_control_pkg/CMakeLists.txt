cmake_minimum_required(VERSION 3.16)

# Optionally use vcpkg toolchain if available
if(NOT DEFINED CMAKE_TOOLCHAIN_FILE)
  if(DEFINED ENV{VCPKG_ROOT})
    set(_VCPKG_TOOLCHAIN "$ENV{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake")
    if(EXISTS "${_VCPKG_TOOLCHAIN}")
      set(CMAKE_TOOLCHAIN_FILE "${_VCPKG_TOOLCHAIN}" CACHE STRING "Vcpkg toolchain file")
    endif()
  elseif(WIN32)
    # Fallback to a common Windows installation path
    set(_VCPKG_TOOLCHAIN "C:/vcpkg/scripts/buildsystems/vcpkg.cmake")
    if(EXISTS "${_VCPKG_TOOLCHAIN}")
      set(CMAKE_TOOLCHAIN_FILE "${_VCPKG_TOOLCHAIN}" CACHE STRING "Vcpkg toolchain file")
    endif()
  endif()
endif()

# Enable compile commands export
set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE INTERNAL "")

project(agent_control_standalone_test CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Add filesystem-related definitions
if(MSVC)
    add_definitions(-D_SILENCE_EXPERIMENTAL_FILESYSTEM_DEPRECATION_WARNING)
    add_definitions(-DFILESYSTEM_EXPERIMENTAL=0)
endif()

# Add include directories for all targets
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_BINARY_DIR}
)

# Enable testing with CTest
enable_testing()
include(CTest)
include(GoogleTest)

# Find required packages
find_package(GTest CONFIG REQUIRED)
find_package(yaml-cpp CONFIG REQUIRED)

# --- Config Reader Library ---
add_library(config_reader_lib
    src/config_reader.cpp
    src/fuzzy_params_loader.cpp
)
target_include_directories(config_reader_lib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(config_reader_lib PUBLIC yaml-cpp)

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
target_include_directories(pid_standalone_tester PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(pid_standalone_tester PRIVATE
    pid_logic_lib
    fuzzy_logic_lib
    config_reader_lib
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

# --- FLS Unit Tests ---
add_executable(test_gt2_fls
    test/test_gt2_fls.cpp
)
target_include_directories(test_gt2_fls PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(test_gt2_fls PRIVATE
    fuzzy_logic_lib
    GTest::gtest
    GTest::gtest_main
)

gtest_discover_tests(test_gt2_fls)

# Optional: Install rules
# install(TARGETS pid_standalone_tester fuzzy_standalone_tester DESTINATION bin)
# install(TARGETS pid_logic_lib fuzzy_logic_lib
#     ARCHIVE DESTINATION lib
#     LIBRARY DESTINATION lib
#     RUNTIME DESTINATION bin)
# install(DIRECTORY include/ DESTINATION include)

# --- Executable for Simple PID Tuning ---
add_executable(simple_pid_tuner
    src/simple_pid_tuning_main.cpp
)
target_include_directories(simple_pid_tuner PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(simple_pid_tuner PRIVATE
    pid_logic_lib  # Link against your PID library
)

# --- Executable for Multi-Drone PID Test ---
add_executable(multi_drone_pid_tester
    src/multi_drone_pid_test_main.cpp
)
target_include_directories(multi_drone_pid_tester PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(multi_drone_pid_tester PRIVATE
    pid_logic_lib  # Link against your PID library
    fuzzy_logic_lib  # Link against fuzzy logic library
    config_reader_lib  # Link against config reader library
    fuzzy_logic_lib
)

# Copy compile_commands.json to source directory
if(CMAKE_EXPORT_COMPILE_COMMANDS)
  execute_process(
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            ${CMAKE_BINARY_DIR}/compile_commands.json
            ${CMAKE_SOURCE_DIR}/compile_commands.json)
endif()