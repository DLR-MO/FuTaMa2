#!/usr/bin/env python3

# SPDX-FileCopyrightText: 2024 German Aerospace Center <adrian.ricardezortigosa@dlr.de>
# SPDX-License-Identifier: MIT

"""
Node that executes the automatic robotic inspection process around the wing.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, Image
from cv_bridge import CvBridge
import numpy as np
import time
from controller_manager_msgs.srv import SwitchController
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from futama2_utils import moveit_funcs
from rclpy.logging import get_logger
from moveit.planning import MoveItPy

# Constants
MAX_VELOCITY = 0.35  # [m/s]
MIN_DEPTH_DISTANCE = 350  # [mm]
VELOCITY_SLOPE = MAX_VELOCITY / MIN_DEPTH_DISTANCE
POSE_ORIGIN = [0.17433129251003265, -1.216812252998352, 0.849524974822998, 
               0.033915333449840546, 0.03393150493502617, -0.7062444090843201, 0.7063407301902771]
POSE_CONFINED_SPACE_FRONT = [-0.3134135603904724, -1.5278343439102173, 0.07761852443218231,
                             0.29451873898506165, 0.2945699691772461, -0.6427989602088928, 0.6428815722465515]

class AutoInsp(Node):
    def __init__(self):
        super().__init__('auto_insp_demo')

        self.depth_distance = 0
        self.current_pose_msg = PoseStamped()
        self.previous_pose = [0, 0, 0, 0, 0, 0, 0]

        # Publisher: Joy
        self.joy_pub = self.create_publisher(Joy, 'joy', 10)
        self.joy_cmd = Joy(axes=[0] * 6, buttons=[0] * 2)

        # Subscribers
        self.odom_sub = self.create_subscription(Odometry, '/rtabmap/odom', self.robot_states_callback, 10)

        # Service client: Switch Controller
        self.switch_controller_client = self.create_client(SwitchController, '/controller_manager/switch_controller')
        while not self.switch_controller_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for SwitchController service...')

        # MoveIt Initialization
        self.futama2 = MoveItPy(node_name="auto_insp_demo")
        self.move_group = self.futama2.get_planning_component("front")
        self.logger = get_logger("moveit_py.pose_goal")

        #time.sleep(5.0)  # Allow other modules to initialize (Octomap, etc.)

        self.switch_to_trajectory_mode()
        self.move_to_pose(POSE_ORIGIN)
        time.sleep(1.0)
        self.auto_insp_demo_mode()

    def robot_states_callback(self, msg):
        self.current_pose_msg.header = msg.header
        self.current_pose_msg.pose = msg.pose.pose

    def calculate_linear_velocity(self):
        velocity = MAX_VELOCITY - VELOCITY_SLOPE * self.depth_distance
        return max(min(velocity, MAX_VELOCITY), -MAX_VELOCITY)

    def publish_velocity_command(self):
        self.joy_cmd.header.stamp = self.get_clock().now().to_msg()
        self.joy_pub.publish(self.joy_cmd)

    def move_to_pose(self, pose_cmd, retries=3):
        for attempt in range(retries):
            self.logger.info(f"Attempt {attempt + 1} to plan to pose")
            self.move_group.set_start_state_to_current_state()
            pose_goal = PoseStamped()
            pose_goal.header.frame_id = "base_link"
            pose_goal.pose.position.x, pose_goal.pose.position.y, pose_goal.pose.position.z = pose_cmd[:3]
            pose_goal.pose.orientation.x, pose_goal.pose.orientation.y, pose_goal.pose.orientation.z, pose_goal.pose.orientation.w = pose_cmd[3:]
            self.move_group.set_goal_state(pose_stamped_msg=pose_goal, pose_link="realsense_front_link")
            if moveit_funcs.plan_and_execute(self.futama2, self.move_group, self.logger, sleep_time=0.5):
                return True
            self.logger.warning(f"Planning attempt {attempt + 1} failed")
        self.logger.error("All planning attempts failed")
        return False


    def switch_to_trajectory_mode(self):
        self._switch_controller("joint_trajectory_controller", "forward_position_controller")
        self.logger.info("Switched to trajectory mode")

    def _switch_controller(self, activate, deactivate):
        req = SwitchController.Request()
        req.activate_controllers = {activate}
        req.deactivate_controllers = {deactivate}
        future = self.switch_controller_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result():
            self.logger.info(f"Controller switched: Activated {activate}, Deactivated {deactivate}")
        else:
            self.logger.error("Failed to switch controllers")

    def auto_insp_demo_mode(self):
        self.move_to_pose(POSE_CONFINED_SPACE_FRONT)
        self.move_to_pose(POSE_ORIGIN)
        self.destroy_node()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    try:
        node = AutoInsp()
        rclpy.spin(node)
    except Exception as e:
        node.get_logger().error(f"Exception occurred: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()