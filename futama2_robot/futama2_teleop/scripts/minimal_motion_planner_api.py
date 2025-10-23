#!/usr/bin/env python3

# SPDX-FileCopyrightText: 2024 German Aerospace Center <adrian.ricardezortigosa@dlr.de>
# SPDX-License-Identifier: MIT

"""
Node that moves the robot to a single predefined pose.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from controller_manager_msgs.srv import SwitchController
from moveit.core.robot_state import RobotState  # DO NOT DELETE! otherwise, error
from moveit.planning import MoveItPy
from rar_moveit import moveit_funcs
import time

# Define Target Pose in front of the wing (just for testing)
POSE_TO_MOVE_TO = [-0.3134135603904724, -1.5278343439102173, 0.07761852443218231,
                             0.29451873898506165, 0.2945699691772461, -0.6427989602088928, 0.6428815722465515]

class SinglePoseMotionPlanner(Node):
    def __init__(self):
        super().__init__('single_pose_motion_planner')

        # Initialize MoveItPy
        self.moveit_py = MoveItPy(node_name="single_pose_motion_planner")
        self.move_group = self.moveit_py.get_planning_component("front")  # Adjust to your planning group

        self.get_logger().info("MoveItPy Initialized!")

        # Ensure trajectory mode is active
        self.switch_to_trajectory_mode()

        # Move the robot to the target pose
        self.move_to_pose(POSE_TO_MOVE_TO)

        # Shutdown node after execution
        self.destroy_node()
        rclpy.shutdown()

    def move_to_pose(self, pose_cmd, retries=3):
        """Plans and executes a move to a target pose."""
        for attempt in range(retries):
            self.get_logger().info(f"Attempt {attempt + 1} to move to pose: {pose_cmd}")

            # Create a PoseStamped message
            pose_goal = PoseStamped()
            pose_goal.header.frame_id = "base_link"
            pose_goal.pose.position.x, pose_goal.pose.position.y, pose_goal.pose.position.z = pose_cmd[:3]
            pose_goal.pose.orientation.x, pose_goal.pose.orientation.y, pose_goal.pose.orientation.z, pose_goal.pose.orientation.w = pose_cmd[3:]

            # Set goal state
            self.move_group.set_goal_state(pose_stamped_msg=pose_goal, pose_link="realsense_center_link")

            # Plan and Execute
            if moveit_funcs.plan_and_execute(self.moveit_py, self.move_group, self.get_logger(), sleep_time=0.5):
                self.get_logger().info("Successfully moved to the target pose!")
                return True

            self.get_logger().warning(f"Planning attempt {attempt + 1} failed")

        self.get_logger().error("All planning attempts failed")
        return False

    def switch_to_trajectory_mode(self):
        """Switches the robot to trajectory mode for motion planning."""
        self.get_logger().info("Switching to trajectory mode...")
        req = SwitchController.Request()
        req.activate_controllers = {"joint_trajectory_controller"}
        req.deactivate_controllers = {"forward_velocity_controller"}

        client = self.create_client(SwitchController, '/controller_manager/switch_controller')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for SwitchController service...')

        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result():
            self.get_logger().info("Controller switched to trajectory mode")
        else:
            self.get_logger().error("Failed to switch controllers")

def main(args=None):
    rclpy.init(args=args)
    node = SinglePoseMotionPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
