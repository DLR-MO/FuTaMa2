# SPDX-FileCopyrightText: 2024 German Aerospace Center <adrian.ricardezortigosa@dlr.de>
#
# SPDX-License-Identifier: MIT

import launch_ros
import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node, LoadComposableNodes, ComposableNodeContainer
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch.conditions import IfCondition

from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EqualsSubstitution, PythonExpression

from launch.actions import DeclareLaunchArgument, LogInfo, RegisterEventHandler, TimerAction, IncludeLaunchDescription
from launch.events.process import ProcessIO
from launch.event_handlers import (OnExecutionComplete, OnProcessExit,
                                   OnProcessIO, OnProcessStart, OnShutdown)

def load_yaml(package_name, file_name):
    package_path = get_package_share_directory(package_name)
    file_path = os.path.join(package_path, file_name)
    with open(file_path, 'r') as file:
        return yaml.safe_load(file)

def generate_launch_description():
    robot_driver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [get_package_share_directory("futama2_description"), "/launch.py"]),
        launch_arguments={
            "mode": 'mock',
        }.items(),
    )
        
    # MoveIt configuration (but don't publish transforms)
    moveit_config = (MoveItConfigsBuilder("robot", package_name="futama2_moveit_config")
                     .robot_description(file_path="config/robot.urdf.xacro")
                     .moveit_cpp(
        file_path=get_package_share_directory("futama2_teleop")
        + "/config/motion_planning_python_api_tutorial.yaml")
    ).to_moveit_configs()

    # Set up MoveIt group node configuration
    move_group_configuration = {
        "publish_robot_description_semantic": True,
        "allow_trajectory_execution": False,
        "capabilities": "",
        "disable_capabilities": "",
        "publish_planning_scene": False,
        "publish_geometry_updates": False,
        "publish_state_updates": False,
        "publish_transforms_updates": False,
        "monitor_dynamics": False,
    }

    move_group_params = [
        moveit_config.to_dict(),
        move_group_configuration,
    ]

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=move_group_params,
    )

    # RViz visualization
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        respawn=False,
        arguments=[
            "-d", get_package_share_directory("futama2_teleop") + "/config/futama2.rviz"],
        parameters=[
            moveit_config.robot_description,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
    )

    servo_params = {"moveit_servo": load_yaml("futama2_teleop", "config/futama2_ur_servo.yaml")}

    # Launch composable nodes to reduce latency and enable real-time control
    load_composable_nodes = LoadComposableNodes(
        target_container="robot_container",
        composable_node_descriptions=[
            # Joystick teleoperation for manual control
            launch_ros.descriptions.ComposableNode(
                package="futama2_teleop",
                plugin="futama2_teleop::JoystickTeleop",
                name="joy_teleop",
                parameters=[{"use_sim_time": 'true'}],
            ),
            # Spacemouse (or Spacenav) integration for 3D input control
            launch_ros.descriptions.ComposableNode(
                package="spacenav",
                plugin="spacenav::Spacenav",
                name="spacenav_node",
                parameters=[{"use_sim_time": 'true'}],
                remappings=[("/spacenav/joy", "/joy")],  # Remap to joystick topic if needed
            ),
            # Servo for MoveIt integration for teleoperation
            launch_ros.descriptions.ComposableNode(
                package="moveit_servo",
                plugin="moveit_servo::ServoNode",
                name="servo_node",
                parameters=[
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    {
                        "moveit_servo": load_yaml("futama2_teleop", "config/futama2_ur_servo.yaml"),
                        "use_sim_time": 'true',
                    }
                ], 
            ),
        ],
    )

    return LaunchDescription(
        [
            robot_driver_cmd,
            rviz_node,
            TimerAction(period=2.0,
                        actions=[
                            load_composable_nodes,
                            move_group_node,
                        ]),
        ]
    )
