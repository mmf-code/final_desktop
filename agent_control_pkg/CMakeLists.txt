cmake_minimum_required(VERSION 3.16)

# This CMakeLists.txt is for the agent_control_pkg
# It assumes yaml-cpp might be provided by a parent CMakeLists.txt (FetchContent)
# or vcpkg (via CMAKE_TOOLCHAIN_FILE).

# Enable compile commands export for IDEs like VS Code
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

project(agent_control_pkg_standalone CXX) # Project name specific to this package

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# For std::filesystem with MSVC
if(MSVC)
    add_compile_definitions(_SILENCE_EXPERIMENTAL_FILESYSTEM_DEPRECATION_WARNING)
    # For C++17, std::filesystem should be standard, but some older MSVC versions might need hints
    # add_compile_definitions(FILESYSTEM_EXPERIMENTAL=0) # Usually not needed with C++17 target
endif()

# --- Find Dependencies ---
# Google Test (for unit tests)
find_package(GTest CONFIG REQUIRED)

# YAML-CPP
# Try to find an installed version first (e.g., from vcpkg if toolchain file is set, or system install)
find_package(yaml-cpp QUIET CONFIG) 
if (NOT yaml-cpp_FOUND)
    # If not found via CONFIG mode, check if a target 'yaml-cpp' already exists globally
    # (this would be true if FetchContent was used in a parent CMakeLists.txt)
    if (NOT TARGET yaml-cpp)
        message(STATUS "yaml-cpp not found via find_package(CONFIG) and no global 'yaml-cpp' target exists. Assuming it might be declared via FetchContent in a parent scope or through vcpkg toolchain implicitly.")
        # If this CMakeLists.txt is the TOP LEVEL, you would put FetchContent for yaml-cpp here.
        # Since it's a package, we assume the root CMakeLists.txt handles fetching it.
        # If it's truly missing, linking will fail later.
    else()
         message(STATUS "Found global target 'yaml-cpp' (likely from FetchContent in parent or vcpkg).")
    endif()
endif()


# --- Define Libraries ---

# Config Reader Library
add_library(config_reader_lib
    src/config_reader.cpp
    # src/fuzzy_params_loader.cpp # REMOVE THIS LINE if fuzzy_params_loader.cpp is not used or doesn't exist
                                  # Your config_reader.cpp now has loadFuzzyParamsYAML
)
target_include_directories(config_reader_lib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>  # For other targets in this build
    $<INSTALL_INTERFACE:include>                           # For installed version
)
# Link yaml-cpp to config_reader_lib
if(TARGET yaml-cpp) # If FetchContent/vcpkg provided a direct target 'yaml-cpp'
    target_link_libraries(config_reader_lib PUBLIC yaml-cpp)
    message(STATUS "config_reader_lib linked against TARGET yaml-cpp.")
elseif(yaml-cpp_FOUND AND TARGET yaml-cpp::yaml-cpp) # If find_package(CONFIG) found it
    target_link_libraries(config_reader_lib PUBLIC yaml-cpp::yaml-cpp)
    message(STATUS "config_reader_lib linked against IMPORTED TARGET yaml-cpp::yaml-cpp.")
else()
    message(WARNING "yaml-cpp target not found for config_reader_lib. Linking might fail or be incomplete. Ensure yaml-cpp is available (e.g., via FetchContent in root CMakeLists.txt or vcpkg).")
    # As a last resort, if you know the library name but not target (less ideal)
    # target_link_libraries(config_reader_lib PUBLIC yaml-cpp) 
endif()

# PID Logic Library
add_library(pid_logic_lib
    src/pid_controller.cpp
)
target_include_directories(pid_logic_lib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Fuzzy Logic Library
add_library(fuzzy_logic_lib
    src/gt2_fuzzy_logic_system.cpp
)
target_include_directories(fuzzy_logic_lib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# --- Define Executables ---

# Main simulation executable (formerly pid_standalone_tester)
add_executable(agent_control_main_executable # Renamed for clarity
    src/agent_control_main.cpp
)
target_link_libraries(agent_control_main_executable PRIVATE
    pid_logic_lib
    fuzzy_logic_lib
    config_reader_lib
)

# Fuzzy Logic specific test executable
add_executable(fuzzy_standalone_tester
    src/fuzzy_test_main.cpp
)
target_link_libraries(fuzzy_standalone_tester PRIVATE fuzzy_logic_lib)

# Multi-Drone PID Test / Auto-Tuner executable
add_executable(multi_drone_pid_tester
    src/multi_drone_pid_test_main.cpp
)
target_link_libraries(multi_drone_pid_tester PRIVATE
    pid_logic_lib
    fuzzy_logic_lib     # Link FLS library
    config_reader_lib
)

# Simple PID Tuner (if still used)
# add_executable(simple_pid_tuner
#     src/simple_pid_tuning_main.cpp
# )
# target_link_libraries(simple_pid_tuner PRIVATE pid_logic_lib)


# --- Unit Tests ---
if(TARGET GTest::gtest_main) # More robust check for GTest availability
    include(GoogleTest) # Defines gtest_discover_tests

    add_executable(test_pid_controller test/test_pid_controller.cpp)
    target_link_libraries(test_pid_controller PRIVATE
        pid_logic_lib
        GTest::gtest_main # Preferred way to link Google Test
    )
    gtest_discover_tests(test_pid_controller)

    add_executable(test_gt2_fls test/test_gt2_fls.cpp)
    target_link_libraries(test_gt2_fls PRIVATE
        fuzzy_logic_lib
        GTest::gtest_main
    )
    gtest_discover_tests(test_gt2_fls)
else()
    message(WARNING "GTest::gtest_main target not found. Skipping unit tests that depend on it.")
endif()


# --- Copy compile_commands.json ---
# This helps IDEs like VS Code find include paths correctly.
# It's copied to the root of THIS package's source directory.
# CMAKE_BINARY_DIR refers to the top-level build directory (e.g., project_root/build)
# CMAKE_CURRENT_SOURCE_DIR refers to agent_control_pkg
if(CMAKE_EXPORT_COMPILE_COMMANDS)
  add_custom_command(TARGET multi_drone_pid_tester POST_BUILD # Run after a key executable is built
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${CMAKE_BINARY_DIR}/compile_commands.json"
            "${CMAKE_CURRENT_SOURCE_DIR}/compile_commands.json"
    COMMENT "Copying compile_commands.json to ${CMAKE_CURRENT_SOURCE_DIR}"
    VERBATIM
  )
endif()
# --- Optional: Installation rules (uncomment and adapt if needed) ---
# install(TARGETS agent_control_main_executable fuzzy_standalone_tester multi_drone_pid_tester
#     RUNTIME DESTINATION bin)
# install(TARGETS pid_logic_lib fuzzy_logic_lib config_reader_lib
#     ARCHIVE DESTINATION lib
#     LIBRARY DESTINATION lib
#     RUNTIME DESTINATION bin) # For DLLs on Windows
# install(DIRECTORY include/agent_control_pkg DESTINATION include)
