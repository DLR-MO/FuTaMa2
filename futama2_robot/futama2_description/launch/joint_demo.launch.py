# SPDX-FileCopyrightText: 2024 German Aerospace Center <adrian.ricardezortigosa@dlr.de>
#
# SPDX-License-Identifier: MIT

"""
Launch file to visualize the urdf demo of the robot and play with the joint movement
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    package_dir = get_package_share_directory("futama2_description")

    start_robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([package_dir, "/launch.py"]),
        launch_arguments={
            "mode": "urdf",
        }.items(),
    )

    start_joint_state_publisher_cmd = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
    )

    rviz_cmd = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", os.path.join(package_dir, "rviz", "default.rviz")],
        output="screen",
    )

    return LaunchDescription(
        [
            start_joint_state_publisher_cmd,
            start_robot_state_publisher_cmd,
            rviz_cmd,
        ]
    )
