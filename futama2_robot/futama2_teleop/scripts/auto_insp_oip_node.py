#!/usr/bin/env python3

# SPDX-FileCopyrightText: 2024 German Aerospace Center <adrian.ricardezortigosa@dlr.de>
#
# SPDX-License-Identifier: MIT

import time
import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger
from rclpy.exceptions import ParameterNotDeclaredException
from std_msgs.msg import Header
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, PoseArray
from controller_manager_msgs.srv import SwitchController
from moveit.planning import MoveItPy
from futama2_utils import moveit_funcs
from inspection_helper_funcs import get_poses, tuple_to_pose, alternative_poses, CAMERA_POSES
from scipy.spatial.transform import Rotation as R
import numpy as np

class AutoInsp(Node):
    def __init__(self):
        super().__init__('auto_insp')

        self.logger = get_logger('AutoInsp')
        self.reached_points = []
        self.arrival_times = []
        self.possible_camera_poses = CAMERA_POSES

        # Initialize publishers and subscribers
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.robot_states_callback, 10)
        self.path_pub = self.create_publisher(Path, 'path', 10)
        self.current_point_pub = self.create_publisher(PoseStamped, 'point', 10)
        self.reached_point_pub = self.create_publisher(PoseStamped, 'reached_point', 10)
        self.cam_pose_pub = self.create_publisher(PoseStamped, 'cam_pose', 10)

        # Initialize MoveItPy
        try:
            self.futama2 = MoveItPy(node_name="auto_insp")
            self.move_group = self.futama2.get_planning_component('front')
            self.planning_scene_monitor = self.futama2.get_planning_scene_monitor()
            self.logger.info("MoveItPy instance successfully created")
        except Exception as e:
            self.logger.error(f"Failed to initialize MoveItPy: {e}")
            raise

        # Initialize the controller service
        self.switchcontroller_client = self.create_client(SwitchController, '/controller_manager/switch_controller')
        while not self.switchcontroller_client.wait_for_service(timeout_sec=1.0):
            self.logger.warn("Waiting for SwitchController service...")

        # Load and publish the path
        poses = get_poses()
        path_msg = self.to_path_msg([*poses])
        self.publish_path(path_msg)

    def robot_states_callback(self, msg):
        self.logger.debug(f"Received robot state update: {msg}")

    def to_path_msg(self, path_poses_tuple: list[tuple]) -> Path:
        path_msg = Path()
        current_time = self.get_clock().now().to_msg()
        path_msg.header = Header(frame_id='object_link', stamp=current_time)
        path_msg.poses = [tuple_to_pose(pose, current_time) for pose in path_poses_tuple]
        return path_msg

    def publish_path(self, path_msg: Path):
        self.logger.info("Publishing path...")
        self.path_pub.publish(path_msg)

        start_time = self.get_clock().now()
        for pose in path_msg.poses:
            self.move_group_planner_and_executer(pose)

        total_time = self.get_clock().now() - start_time
        self.logger.info(f"Finished path execution in {total_time.nanoseconds / 1e9:.2f} seconds")

    def move_group_planner_and_executer(self, pose: PoseStamped):
        self.logger.info(f"Planning and executing for pose: {pose}")
        self.move_group.set_start_state_to_current_state()
        self.cam_pose_pub.publish(pose)

        cameras = self.possible_camera_poses.copy()
        for camera, cam_pose in zip(cameras, alternative_poses(pose, cameras)):
            self.move_group.set_goal_state(pose_stamped_msg=cam_pose, pose_link='realsense_center_link')
            reached = moveit_funcs.plan_and_execute(self.futama2, self.move_group, self.logger)

            if reached:
                self.logger.info("Reached goal pose")
                self.reached_point_pub.publish(cam_pose)
                self.reached_points.append(True)
                break
        else:
            self.logger.warn("Failed to reach any goal pose")
            self.reached_points.append(False)

    def switch_to_trajectory_mode(self):
        self.logger.info("Switching to trajectory mode...")
        req = SwitchController.Request()
        req.activate_controllers = ["joint_trajectory_controller"]
        req.deactivate_controllers = ["forward_velocity_controller"]
        self.switchcontroller_client.call_async(req)
        time.sleep(1)

def main(args=None):
    rclpy.init(args=args)
    try:
        node = AutoInsp()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.logger.info("Shutdown requested by user")
    except Exception as e:
        rclpy.logging.get_logger('AutoInsp').error(f"Unhandled exception: {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
