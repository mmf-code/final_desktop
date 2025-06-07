cmake_minimum_required(VERSION 3.8)
project(formation_coordinator_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(my_custom_interfaces_pkg REQUIRED)

# Include directories
include_directories(include)

# Add executable (when source files are added)
# add_executable(formation_coordinator src/formation_coordinator.cpp)
# ament_target_dependencies(formation_coordinator
#   rclcpp
#   std_msgs
#   geometry_msgs
#   my_custom_interfaces_pkg
# )

# Install targets
# install(TARGETS
#   formation_coordinator
#   DESTINATION lib/${PROJECT_NAME}
# )

# Install config files
install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config/
)

# Install launch files
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
