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
        choices=["manual", "auto_oip", "auto_minimal"],
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
        choices=["none" ,"d405", "d435i"],
        default_value="none",
    )
    multicam = LaunchConfiguration("multicam")
    multicam_cmd = DeclareLaunchArgument(
        "multicam",
        description="Are you using the three cameras?",
        choices=["false", "true"],
        default_value='false',
    )
    spacemouse_mdl = LaunchConfiguration("spacemouse_mdl")
    spacemouse_mdl_cmd = DeclareLaunchArgument(
        "spacemouse_mdl",
        description="Which spacemouse model is available?",
        choices=["false", "simple", "pro"],
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
                     .moveit_cpp(file_path=get_package_share_directory("futama2_teleop")+"/config/motion_planning.yaml")
                     ).to_moveit_configs()

    move_group_configuration = {
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

    minimal_motion_planner_api_node = Node(
        name="minimal_motion_planner_api",
        package="futama2_teleop",
        executable="minimal_motion_planner_api.py",
        output="both",
        parameters=move_group_params,
        condition=IfCondition(EqualsSubstitution(insp_mode, "auto_minimal")),
    )

    auto_insp_oip_node = Node(
        name="auto_insp_oip",
        package="futama2_teleop",
        executable="auto_insp_oip_node.py",
        output="both",
        parameters=move_group_params,
        condition=IfCondition(EqualsSubstitution(insp_mode, "auto_oip")),
    )

    # rviz mock hardware 1x D435i (suitable for debugging at home)
    rviz_1x_cam_node = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        respawn=False,
        arguments=[
            "-d", get_package_share_directory("futama2_teleop") + "/rviz/futama2_1x_cam.rviz"],
        parameters=[
            moveit_config.robot_description,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
        condition=IfCondition(EqualsSubstitution(multicam, "false")),
    )

    # rviz real robot 3x D405
    rviz_3x_cam_node = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        respawn=False,
        arguments=[
            "-d", get_package_share_directory("futama2_teleop") + "/rviz/futama2_3x_cam.rviz"],
        parameters=[
            moveit_config.robot_description,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
        condition=IfCondition(EqualsSubstitution(multicam, "true")),
    )

    # odometry (for getting moveit current state)
    odometry_node = Node(
        package="rar_perception",
        executable="odometry_node.py",
        output="screen",
        parameters=[
            {'target_frame': 'base_link'},
            {'child_frame': 'realsense_center_link'},
            {'publish_rate': 100.0},
            # Calibration (TODO calibrate mock and real hardware on FuTaMa2)
            {'offset_position_x': 0.0},
            {'offset_position_y': 0.0},
            {'offset_position_z': 0.0},
            {'offset_orientation_x': 0.0},
            {'offset_orientation_y': 0.0},
            {'offset_orientation_z': 0.0},
            {'offset_orientation_w': 0.0},
        ]
    )

    # Python node to save images from the front camera's topic (TO DO save all cameras pics)
    foto_capture_node = Node(
        package="futama2_teleop",
        executable="foto_capture_node.py",
        output="screen",
        parameters=[{"insp_mode": insp_mode}],
        condition=IfCondition(EqualsSubstitution(mode, "real")),
    )
    
    joystick_teleoperating_modes = Node(
        package="rar_teleop_modes",
        executable="joystick_teleoperating_modes.py",
        output="screen",
        parameters=[{"spacemouse_mdl": spacemouse_mdl},
                    {"robot_variant": "ur_only"},
                    ] + move_group_with_octomap_params,
        condition=IfCondition(
            PythonExpression(
                ["'", spacemouse_mdl, "' != 'false' and '", insp_mode, "' == 'manual'"]
            )
        ),
    )

    spacemouse_filter = Node(
        package="rar_filters",
        executable="spacemouse_filter_ros2.py",
        output="screen",
        condition=IfCondition(
            PythonExpression(
                ["'", spacemouse_mdl, "' != 'false' and '", insp_mode, "' == 'manual'"]
            )
        ),
    )

    # joy from spacemouse, parameters required to avoid strange / non-continues data
    spacemouse = Node(
        package='spacenav', 
        executable='spacenav_node', 
        output='screen',
        name='spacenav_node',
        parameters=[{
            'zero_when_static': False,
            'static_count_threshold': 1000,
            'static_trans_deadband': 0.01,
            'static_rot_deadband': 0.01
        }]
    )

    # node that calculates the distance between the selected frame and the closest octomap voxel in the scene
    octomap_distance_node = Node(
        package='rar_perception', 
        executable='octomap_distance_node', 
        output='screen',
        name='octomap_distance_node',
        parameters=[
            {'transform_from_frame': 'world'},
            {'transform_to_frame': 'realsense_center_link'},
            {'header_frame': 'world'},
        ]
    )

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
                    {**load_yaml("futama2_teleop", "config/futama2_ur_servo_front.yaml")},
                    {"use_sim_time": False},
                ],
                remappings=[
                    ("~/delta_twist_cmds", "/servo/delta_twist_cmds/front"),
                    ("~/status", "/servo/status/front")
                ]
            ),
            launch_ros.descriptions.ComposableNode(
                package="moveit_servo",
                plugin="moveit_servo::ServoNode",
                name="servo_node",
                parameters=[
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    {**load_yaml("futama2_teleop", "config/futama2_ur_servo_fit.yaml")},
                    {"use_sim_time": False},
                ],
                remappings=[
                    ("~/delta_twist_cmds", "/servo/delta_twist_cmds/fit"),
                    ("~/status", "/servo/status/fit")
                ]
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
                plugin="futama2_teleop::JoystickTeleopPerceptCollAvoidance",
                name="joystick_teleop_percept_coll_avoidance",
                parameters=[{"use_sim_time": EqualsSubstitution(mode, "sim")}],
                # extra_arguments=[{"use_intra_process_comms": True}],
                condition=IfCondition(
                    PythonExpression(
                        ["'", spacemouse_mdl, "' != 'false' and '", insp_mode, "' == 'manual'"]
                    )
                ),
            ),
        ],
    )

    realsense_cam_eeloscope_launch = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('rar_perception'),
            'launch',
            'realsense_cam_eeloscope.launch.py'
        ]),
        launch_arguments={
            'camera_mdl': 'd435i',
            'multicam': 'false',
            # use_sim_time not working from the realsense2_camera package unfortunately:
            #'use_sim_time': 'true', # needs to be passed as string, not bool
        }.items(),
    )

    # Include the diagnostics launch file and pass parameters from the parent launch file
    diagnostics_launch = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('rar_diagnostics'),
            'launch',
            'futama2.launch.py'
        ]),
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
            # Arguments
            mode_cmd,
            camera_mdl_cmd,
            multicam_cmd,
            insp_mode_cmd,
            spacemouse_mdl_cmd,
            octomap_cmd,

            # Core stack
            robot_driver_cmd,

            # Delayed logic for safety
            TimerAction(period=3.0, actions=[
                LogInfo(msg="🟢 move_group starting..."),
                move_group_node,
                move_group_with_octomap_node,
            ]),
            TimerAction(period=6.0, actions=[
                LogInfo(msg="🟢 Starting Servo + PlanningScene nodes..."),
                load_composable_nodes,
            ]),
            TimerAction(period=7.0, actions=[
                LogInfo(msg="🟢 Starting RViz, diagnostics, and input devices..."),
                rviz_1x_cam_node, rviz_3x_cam_node,
                diagnostics_launch,
                spacemouse,
                spacemouse_filter,
                joystick_teleoperating_modes,
            ]),

            # Other camera + utility nodes
            realsense_cam_eeloscope_launch,
            foto_capture_node,
            odometry_node,
            octomap_distance_node,

            TimerAction(period=9.0, actions=[
                tf_static_oip_object,
                minimal_motion_planner_api_node,
                auto_insp_oip_node,
            ]),
        ]
    )