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
from moveit_configs_utils.launches import generate_move_group_launch
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
    mode = LaunchConfiguration("mode")
    mode_cmd = DeclareLaunchArgument(
        "mode",
        description="In which mode do you want to start the FUTAMA2 robot?",
        choices=["real", "mock", "sim"],
        default_value="real",
    )
    insp_mode = LaunchConfiguration("insp_mode")
    insp_mode_cmd = DeclareLaunchArgument(
        "insp_mode",
        description="How do you want to perform the inspection?",
        choices=["manual", "auto_oip", "auto_wing"],
        default_value="manual",
    )
    robot_driver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [get_package_share_directory("futama2_description"), "/launch.py"]),
        launch_arguments={
            "mode": mode,
        }.items(),
    )
    camera_mdl = LaunchConfiguration("camera_mdl")
    camera_mdl_cmd = DeclareLaunchArgument(
        "camera_mdl",
        description="Which realsense camera model are you using?",
        choices=["d405", "d435i"],
        default_value="d405",
    )
    multicam = LaunchConfiguration("multicam")
    multicam_cmd = DeclareLaunchArgument(
        "multicam",
        description="Are you using the three cameras?",
        choices=["false", "true"],
        default_value='false',
    )
    spacemouse = LaunchConfiguration("spacemouse")
    spacemouse_cmd = DeclareLaunchArgument(
        "spacemouse",
        description="Is the spacemouse available?",
        choices=["false", "true"],
        default_value='false',
    )
    octomap = LaunchConfiguration("octomap")
    octomap_cmd = DeclareLaunchArgument(
        "octomap",
        description="Do you want to plan with octomap and visualize it?",
        choices=["false", "true"],
        default_value='false',
    )

    octomap_config = {'octomap_frame': 'world',
                      'octomap_resolution': 0.02,
                      'max_range': 1.5}

    octomap_updater_config = load_yaml(
        'futama2_moveit_config', 'config/realsense_pointcloud.yaml')

    # Start the actual move_group node/action server
    moveit_config = (MoveItConfigsBuilder("robot", package_name="futama2_moveit_config")
                     .robot_description(file_path="config/robot.urdf.xacro")
                     .moveit_cpp(
        file_path=get_package_share_directory("futama2_teleop")
        + "/config/motion_planning.yaml")
    ).to_moveit_configs()

    move_group_configuration = {
        # with only this on, the octomap is still visible, but stuck!
        "publish_robot_description_semantic": True,
        "allow_trajectory_execution": True,
        "capabilities": "",
        "disable_capabilities": "",
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "monitor_dynamics": False,
    }

    move_group_params = [
        moveit_config.to_dict(),
        move_group_configuration,
    ]

    move_group_with_octomap_params = [
        moveit_config.to_dict(),
        move_group_configuration,
        octomap_config,
        octomap_updater_config
    ]

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=move_group_params,
        condition=IfCondition(EqualsSubstitution(octomap, "false")),
    )

    move_group_with_octomap_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=move_group_with_octomap_params,
        condition=IfCondition(EqualsSubstitution(octomap, "true")),
    )

    motion_planning_python_api_node = Node(
        name="motion_planning_python_api_node",
        package="futama2_teleop",
        executable="motion_planning_python_api.py",
        output="both",
        parameters=move_group_params,
    )

    auto_insp_wing_node = Node(
        name="auto_insp_wing",
        package="futama2_teleop",
        executable="auto_insp_wing_node.py",
        output="both",
        parameters=move_group_params,
        condition=IfCondition(EqualsSubstitution(insp_mode, "auto_wing")),
    )

    auto_insp_oip_node = Node(
        name="auto_insp_oip",
        package="futama2_teleop",
        executable="auto_insp_oip_node.py",
        output="both",
        parameters=move_group_params,
        condition=IfCondition(EqualsSubstitution(insp_mode, "auto_oip")),
    )

    # rviz
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

    # odometry (for getting moveit current state)
    odometry_node = Node(
        package="futama2_teleop",
        executable="odometry_node.py",
        output="screen"
    )

    # Python node to save images from the front camera's topic (TO DO save all cameras pics)
    foto_capture_node = Node(
        package="futama2_teleop",
        executable="foto_capture_node.py",
        output="screen",
        parameters=[{"insp_mode": insp_mode}],
        condition=IfCondition(EqualsSubstitution(mode, "real")),
    )

    servo_params = {"moveit_servo": load_yaml("futama2_teleop", "config/futama2_ur_servo.yaml")}

    # Launch as much as possible in components to reduce latency
    load_composable_nodes = LoadComposableNodes(
        target_container="robot_container",
        composable_node_descriptions=[
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
                        "use_sim_time": EqualsSubstitution(mode, "sim")
                    }
                ], 
                # todo currently disabled due to bug in moveit https://github.com/ros-planning/moveit2/issues/2632
                # extra_arguments=[{"use_intra_process_comms": True}],
            ),
            # For publishing the wing and the base box
            launch_ros.descriptions.ComposableNode(
                package="futama2_moveit_config",
                plugin="futama2_moveit_config::PlanningScenePublisher",
                name="planning_scene_publisher",
                parameters=[{"mode": mode,
                            #"use_sim_time": EqualsSubstitution(mode, "sim"),
                            }],
                # todo activate when transient local topics are implemented for intra process comms
                # extra_arguments=[{"use_intra_process_comms": True}],
            ),
            launch_ros.descriptions.ComposableNode(
                package="futama2_teleop",
                plugin="futama2_teleop::JoystickTeleop",
                name="joy_teleop",
                parameters=[{"use_sim_time": EqualsSubstitution(mode, "sim")}],
                # extra_arguments=[{"use_intra_process_comms": True}],
                condition=IfCondition(EqualsSubstitution(spacemouse, 'true')),
            ),
            launch_ros.descriptions.ComposableNode(
                package="spacenav",
                plugin="spacenav::Spacenav",
                name="spacenav_node",
                parameters=[{"use_sim_time": EqualsSubstitution(mode, "sim")}],
                remappings=[("/spacenav/joy", "/joy")],
                # extra_arguments=[{"use_intra_process_comms": True}],
                # if you are actually using the spacemouse, otherwise, only keyboard run
                condition=IfCondition(
                            PythonExpression(
                                ["'", spacemouse, "' == 'true' and '", insp_mode, "' == 'manual'"]
                            )
                        ),
            ),
        ],
    )

    # One camera launch
    rs_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution(
                [FindPackageShare("futama2_teleop"), "launch", "rs_launch.py"])]
        ),
        launch_arguments={
            "pointcloud.enable": "true",
            #"align_depth.enable": "true",
            #"depth_module.enable_auto_exposure": "true",
            "device_type": camera_mdl,
            "serial_no": "_128422271521",
            "depth_module.profile": "1280x720x15",
            "rgb_camera.profile": "1280x720x15",
        }.items(),
        condition=IfCondition(
                    PythonExpression(
                        ["'", camera_mdl, "' == 'd405' and '", mode, "' == 'real'"]
                    )
                ),
    )

    # just when using the d435i camera at home (Adrian)
    rs_launch_d435i = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution(
                [FindPackageShare("futama2_teleop"), "launch", "rs_launch.py"])]
        ),
        launch_arguments={
            "pointcloud.enable": "true",
            #"align_depth.enable": "true",
            #"depth_module.enable_auto_exposure": "true",
            "device_type": camera_mdl,
            "depth_module.profile": "1280x720x15",
            "rgb_camera.profile": "1280x720x15",
        }.items(),
        condition=IfCondition(
                    PythonExpression(
                        ["'", camera_mdl, "' == 'd435i' and '", mode, "' == 'real'"]
                    )
                ),
    )

    # Three cameras launch
    rs_multi_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution(
                [FindPackageShare("futama2_teleop"), "launch", "rs_multi_camera_launch.py"])]
        ),
        launch_arguments={
            "pointcloud.enable1": "true",
            "pointcloud.enable2": "true",
            #"align_depth": "true",
            "serial_no1": "_128422272518",
            "serial_no2": "_128422272647",
            "depth_module.profile1": "1280x720x15",
            "rgb_camera.profile1": "1280x720x15",
            "depth_module.profile2": "1280x720x15",
            "rgb_camera.profile2": "1280x720x15",
        }.items(),
        condition=IfCondition(EqualsSubstitution(multicam, "true")),
    )

    tf_static_oip_object = Node( 
        package='tf2_ros', 
        executable='static_transform_publisher', 
        arguments=['-0.645', '0.795', '0.098', '0', '0', '0', 'world', 'object_link'], 
        output='screen',
        condition=IfCondition(EqualsSubstitution(insp_mode, "auto_oip")),
    )

    return LaunchDescription(
        [
            mode_cmd,
            camera_mdl_cmd,
            multicam_cmd,
            insp_mode_cmd,
            spacemouse_cmd,
            octomap_cmd,
            robot_driver_cmd,
            rs_launch,
            rs_launch_d435i,rs_multi_camera_launch,
            foto_capture_node,
            rviz_node,
            load_composable_nodes,
            move_group_node,
            move_group_with_octomap_node,
            odometry_node,
            TimerAction(period=5.0,
                        actions=[
                            tf_static_oip_object,
                            auto_insp_wing_node,
                            auto_insp_oip_node,
                        ]),
        ]
    )