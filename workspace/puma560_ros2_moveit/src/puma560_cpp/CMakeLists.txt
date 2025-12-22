cmake_minimum_required(VERSION 3.8)
project(puma560_cpp)

# Use C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

# ============================================================
# Math Library (shared by all nodes)
# ============================================================
add_library(puma560_math SHARED
  src/trapezoidal_profile.cpp
  src/synchronized_profiles.cpp
  src/quaternion_utils.cpp
)
target_include_directories(puma560_math PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(puma560_math Eigen3::Eigen)
ament_target_dependencies(puma560_math
  rclcpp
  geometry_msgs
)

# ============================================================
# Executables
# ============================================================

# Joint Space Motion Node
add_executable(joint_space_motion src/joint_space_motion.cpp)
target_link_libraries(joint_space_motion puma560_math Eigen3::Eigen)
ament_target_dependencies(joint_space_motion
  rclcpp
  rclcpp_action
  sensor_msgs
  trajectory_msgs
  control_msgs
  builtin_interfaces
)

# Cartesian Motion Node
add_executable(cartesian_motion src/cartesian_motion.cpp)
target_link_libraries(cartesian_motion puma560_math Eigen3::Eigen)
ament_target_dependencies(cartesian_motion
  rclcpp
  rclcpp_action
  sensor_msgs
  geometry_msgs
  trajectory_msgs
  control_msgs
  builtin_interfaces
  moveit_msgs
)

# # Compliance Control Node
# add_executable(compliance_control src/compliance_control.cpp)
# target_link_libraries(compliance_control puma560_math Eigen3::Eigen)
# ament_target_dependencies(compliance_control
#   rclcpp
#   rclcpp_action
#   sensor_msgs
#   geometry_msgs
#   trajectory_msgs
#   control_msgs
#   builtin_interfaces
#   moveit_msgs
# )

# ============================================================
# Install
# ============================================================

# Install library
install(TARGETS puma560_math
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install executables
install(TARGETS
  joint_space_motion
  cartesian_motion
  # compliance_control
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install scripts
install(PROGRAMS
  scripts/plot_results.py
  DESTINATION lib/${PROJECT_NAME}
)

# ============================================================
# Testing
# ============================================================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

