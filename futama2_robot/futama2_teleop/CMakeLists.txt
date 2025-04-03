# SPDX-FileCopyrightText: 2024 German Aerospace Center
# SPDX-License-Identifier: MIT

cmake_minimum_required(VERSION 3.22)
project(futama2_teleop)

find_package(ament_cmake REQUIRED)
find_package(moveit_common REQUIRED)
moveit_package()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  control_msgs
  control_toolbox
  controller_manager_msgs
  geometry_msgs
  moveit_core
  moveit_msgs
  moveit_ros_planning
  moveit_ros_planning_interface
  pluginlib
  rclcpp
  rclcpp_components
  realtime_tools
  sensor_msgs
  std_msgs
  tf2_eigen
  tf2_ros
  tf2_geometry_msgs
  trajectory_msgs
  nav_msgs
  graph_msgs
  rviz_visual_tools
  moveit_visual_tools
  visualization_msgs
)

# Find dependencies
find_package(rclpy REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(rviz_visual_tools REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# Include headers
include_directories(
  include
)

######################
## Library Nodes ##
######################

# joystick_teleop
add_library(joystick_teleop SHARED src/joystick_teleop.cpp)
ament_target_dependencies(joystick_teleop PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
rclcpp_components_register_nodes(joystick_teleop "futama2_teleop::JoystickTeleop")

# joystick_teleop_percept_coll_avoidance
add_library(joystick_teleop_percept_coll_avoidance SHARED src/joystick_teleop_percept_coll_avoidance.cpp)
ament_target_dependencies(joystick_teleop_percept_coll_avoidance PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
rclcpp_components_register_nodes(joystick_teleop_percept_coll_avoidance "futama2_teleop::JoystickTeleopPerceptCollAvoidance")

# keyboard_teleop
add_library(keyboard_teleop SHARED src/keyboard_teleop.cpp)
ament_target_dependencies(keyboard_teleop PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
rclcpp_components_register_nodes(keyboard_teleop "futama2_teleop::KeyboardNode")

######################
## Executable Nodes ##
######################

# Define the executable for joystick_node
add_executable(joystick_teleop_node src/joystick_teleop.cpp)
ament_target_dependencies(joystick_teleop_node PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Define the executable for joystick_node
add_executable(joystick_teleop_percept_coll_avoidance_node src/joystick_teleop_percept_coll_avoidance.cpp)
ament_target_dependencies(joystick_teleop_percept_coll_avoidance_node PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Define the executable for keyboard_node
add_executable(keyboard_teleop_node src/keyboard_teleop.cpp)
ament_target_dependencies(keyboard_teleop_node PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

add_executable(octomap_distance_node src/octomap_distance_node.cpp)

ament_target_dependencies(octomap_distance_node
  rclcpp
  std_msgs
  geometry_msgs
  moveit_msgs
  octomap
  octomap_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
)

target_link_libraries(octomap_distance_node
  octomap
  octomath
  tf2_ros::tf2_ros
)

#############
## Install ##
#############

# Install C++ libraries
install(TARGETS
  joystick_teleop
  joystick_teleop_percept_coll_avoidance
  keyboard_teleop
  EXPORT futama2_teleopTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include/${PROJECT_NAME}
)

# Install C++ executables
install(TARGETS
  joystick_teleop_node
  joystick_teleop_percept_coll_avoidance_node
  keyboard_teleop_node
  octomap_distance_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python scripts
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/odometry_node.py
  scripts/foto_capture_node.py
  scripts/auto_insp_oip_node.py
  scripts/minimal_motion_planner_api.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install config and launch files
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

# Export targets and dependencies
ament_export_targets(futama2_teleopTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

# Enable testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
