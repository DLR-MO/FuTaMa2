# SPDX-FileCopyrightText: 2024 German Aerospace Center <adrian.ricardezortigosa@dlr.de>
#
# SPDX-License-Identifier: MIT

cmake_minimum_required(VERSION 3.22)
project(futama2_moveit_config)

# modified by Adrian
#find_package(ament_cmake REQUIRED)
find_package(moveit_common REQUIRED)
moveit_package()
#

set(THIS_PACKAGE_INCLUDE_DEPENDS
  geometry_msgs
  geometric_shapes
  moveit_msgs
  pluginlib
  rclcpp
  rclcpp_components
  std_msgs
  std_srvs
  visualization_msgs

  # added by Adrian
  control_msgs    
  control_toolbox
  moveit_core
  moveit_msgs
  moveit_ros_planning
  realtime_tools
  sensor_msgs
  tf2_eigen
  tf2_ros
  trajectory_msgs
  nav_msgs
  #
)

# added by Adrian
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(generate_parameter_library REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

include_directories(
  include
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(planning_scene_publisher SHARED src/planning_scene_publisher.cpp)
ament_target_dependencies(planning_scene_publisher PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
rclcpp_components_register_nodes(planning_scene_publisher "futama2_moveit_config::PlanningScenePublisher")

# Install Libraries
install(TARGETS
  planning_scene_publisher
  EXPORT futama_moveitTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

ament_export_targets(futama_moveitTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
