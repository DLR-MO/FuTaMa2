# SPDX-FileCopyrightText: 2025 German Aerospace Center <adrian.ricardezortigosa@dlr.de>
#
# SPDX-License-Identifier: MIT

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch

def generate_launch_description():
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

    minimal_motion_planner_api_node = Node(
        name="minimal_motion_planner_api",
        package="futama2_teleop",
        executable="minimal_motion_planner_api.py",
        output="both",
        parameters=move_group_params,
    )

    return LaunchDescription(
        [
            minimal_motion_planner_api_node,
        ],
    )