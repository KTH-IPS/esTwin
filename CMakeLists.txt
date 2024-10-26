cmake_minimum_required(VERSION 3.5)
project(dtir_solution_manager)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Compiler options for GCC and Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)

# Include directories
include_directories(
  include
)

# Define libraries
add_library(dtir_solution_manager SHARED
  src/ros2_base_node.cpp
  src/ros2_base_lc_node.cpp
  src/parameter_handler.cpp
  src/controller_handler.cpp
)
ament_target_dependencies(dtir_solution_manager rclcpp rclcpp_lifecycle lifecycle_msgs)

# Define another library for communication helpers
add_library(communication_helpers SHARED
  include/communication_helpers/ros2_control_tools.hpp
  include/communication_helpers/service_tools.hpp)
ament_target_dependencies(communication_helpers rclcpp)
set_target_properties(communication_helpers PROPERTIES LINKER_LANGUAGE CXX)
ament_export_targets(communication_helpers HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp)

# Define executables
add_executable(robot_manager_node
  src/robot_manager_node.cpp
  src/ros2_base_lc_node.cpp
  src/parameter_handler.cpp
  src/controller_handler.cpp)
ament_target_dependencies(robot_manager_node rclcpp rclcpp_lifecycle std_msgs lifecycle_msgs controller_manager_msgs)
add_executable(ur_robot_manager_node
  src/ur_robot_manager_node.cpp
  src/ros2_base_lc_node.cpp
  src/parameter_handler.cpp
  src/controller_handler.cpp)
ament_target_dependencies(ur_robot_manager_node rclcpp rclcpp_lifecycle std_msgs lifecycle_msgs controller_manager_msgs)
add_executable(abb_robot_manager_node
  src/abb_robot_manager_node.cpp
  src/ros2_base_lc_node.cpp
  src/parameter_handler.cpp
  src/controller_handler.cpp)
ament_target_dependencies(abb_robot_manager_node rclcpp rclcpp_lifecycle std_msgs lifecycle_msgs controller_manager_msgs)
add_executable(solution_manager
  src/solution_manager.cpp)
ament_target_dependencies(solution_manager rclcpp rclcpp_lifecycle std_msgs lifecycle_msgs controller_manager_msgs)

# Export libraries and targets
ament_export_targets(export_dtir_solution_manager HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp rclcpp_lifecycle lifecycle_msgs)
ament_export_libraries(${PROJECT_NAME})

# Install libraries and executables
install(DIRECTORY include/solution_manager_core/
  DESTINATION include/solution_manager_core/
)
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})
  
install(TARGETS ${PROJECT_NAME}
  EXPORT export_dtir_solution_manager
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)

install(DIRECTORY include/communication_helpers DESTINATION include)
install(
  TARGETS communication_helpers
  EXPORT communication_helpers
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include
)

install(TARGETS ${PROJECT_NAME} robot_manager_node
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME} ur_robot_manager_node
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME} abb_robot_manager_node
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME} solution_manager
  DESTINATION lib/${PROJECT_NAME})
  
# Export include directories
ament_export_include_directories(include)


ament_package()
set(SONARQUBE_PACKAGES_FILE "none" CACHE STRING "Path to the file that contains the package source directories for SonarQube.")
option(TEST_COVERAGE "Generate test coverage reports upon testing." OFF)

if(NOT ${SONARQUBE_PACKAGES_FILE} MATCHES "none")
  file(APPEND ${SONARQUBE_PACKAGES_FILE} "${PROJECT_NAME};${PROJECT_SOURCE_DIR}\n")
  message(${SONARQUBE_PACKAGES_FILE})
endif()

if(TEST_COVERAGE)
  set(CMAKE_BUILD_TYPE Debug)
  list(APPEND CMAKE_MODULE_PATH "/usr/lib/cmake/CodeCoverage")
  include(CodeCoverage)
  append_coverage_compiler_flags()
  set_coverage_output_dir()
endif()

if(TEST_COVERAGE)
endif()
