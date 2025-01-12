# SPDX-FileCopyrightText: 2024 German Aerospace Center <adrian.ricardezortigosa@dlr.de>
#
# SPDX-License-Identifier: MIT

"""
Launch file that executes the ur10e urdf + eeloscope2 and its necessary components
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, EqualsSubstitution, OrSubstitution
from launch_ros.actions import Node, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    package_dir = get_package_share_directory("futama2_description")

    ur_type = "ur10e"
    robot_ip = "192.168.1.102"

    mode = LaunchConfiguration("mode")

    mode_cmd = DeclareLaunchArgument(
        "mode",
        description="In which mode do you want to start the FUTAMA2 robot?",
        choices=["real", "mock", "sim", "urdf"],
        default_value="mock",
    )
    container = Node(
        name="robot_container",
        package="rclcpp_components",
        executable="component_container",
        output="both",
    )

    # todo adapt the launch so that nodes are composable
    real_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([package_dir, "/ur_control.launch.py"]),
        launch_arguments={
            "ur_type": ur_type,
            "robot_ip": robot_ip,
            "initial_joint_controller": "joint_trajectory_controller",
            "launch_rviz": "False",
        }.items(),
        condition=IfCondition(EqualsSubstitution(mode, "real")),
    )

    # todo adapt the launch so that nodes are composable
    mock_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([package_dir, "/ur_control.launch.py"]),
        launch_arguments={
            "ur_type": ur_type,
            # this is just here because the UR launch unnecessarily requires this argument
            "robot_ip": robot_ip,
            "use_mock_hardware": "True",
            "initial_joint_controller": "joint_trajectory_controller",
            "launch_rviz": "False",
        }.items(),
        condition=IfCondition(EqualsSubstitution(mode, "mock")),
    )

    # simulation
    # todo Marc

    # only urdf
    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                "",
                " ",
                os.path.join(package_dir, "urdf", "robot.urdf.xacro"),
                " ros_control_active:=deactive name:=robot ur_type:=",
                ur_type,
            ]
        ),
        value_type=str,
    )

    load_composable_nodes = LoadComposableNodes(
        target_container="robot_container",
        composable_node_descriptions=[
            ComposableNode(
                package="robot_state_publisher",
                plugin="robot_state_publisher::RobotStatePublisher",
                name="robot_state_publisher",
                parameters=[{"robot_description": robot_description,
                             "use_sim_time": EqualsSubstitution(mode, "sim")}],
                condition=IfCondition(OrSubstitution(EqualsSubstitution(
                    mode, "urdf"), EqualsSubstitution(mode, "sim"))),
                extra_arguments=[{"use_intra_process_comms": False}],
            )
        ],
    )

    return LaunchDescription(
        [
            mode_cmd,
            container,
            real_robot_launch,
            mock_robot_launch,
            load_composable_nodes,
        ]
    )