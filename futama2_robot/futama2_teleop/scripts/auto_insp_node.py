#!/usr/bin/env python3

# SPDX-FileCopyrightText: 2024 German Aerospace Center <adrian.ricardezortigosa@dlr.de>
#
# SPDX-License-Identifier: MIT

"""
Input <- Array [n] of pose arrays [7] in quaternions [[px1,py1,pz1,ox1,oy1,oz1,ow1],
                                            [px2,py2,pz2,ox2,oy2,oz2,ow2]
                                            ...]
being 'px1' e.g. the cartesian position 1 in x and 'ox1' the orientation 1 in x
Process Output -> State of the automatic inspection: movement of the robot, reaching commanded points, finished trajectory
"""

import time

import rclpy
import rclpy.logging
from rclpy.node import Node

from std_msgs.msg import Header
import numpy as np

from nav_msgs.msg import Odometry, Path
from scipy.spatial.transform import Rotation as R
from futama2_utils import moveit_funcs
from rclpy.logging import get_logger
from moveit.planning import MoveItPy

from geometry_msgs.msg import PoseStamped, Quaternion, PoseArray

from inspection_helper_funcs import get_poses, CAMERA_POSES, tuple_to_pose, alternative_poses


class AutoInsp(Node):

    reached_points = []
    possible_camera_poses = CAMERA_POSES

    def __init__(self):
        self.arrival_times = []

        super().__init__('auto_insp')

        # Subscriber: Odometry for robot states, useful to compare msg poses with commanded goals
        self.odom_sub = self.create_subscription(Odometry,
                                                 '/rtabmap/odom',
                                                 self.robot_states_callback,
                                                 10)

        # Publisher
        self.path_pub = self.create_publisher(Path,
                                              'path',
                                              10)
        self.current_point_pub = self.create_publisher(PoseStamped,
                                                        'point',
                                                        100)
        self.cam_pose_pub = self.create_publisher(PoseStamped,
                                                        'cam_pose',
                                                        100)
        self.reached_point_pub = self.create_publisher(PoseStamped,
                                                       'reached_point',
                                                       100)
        self.top_point_pub = self.create_publisher(PoseStamped,
                                                    'top_point',
                                                    100)
        self.possible_poses_pub = self.create_publisher(PoseArray,
                                                        'possible_poses',
                                                        100)

        self.futama2 = MoveItPy(node_name="auto_insp")
        self.move_group = self.futama2.get_planning_component('front')
        self.planning_scene_monitor = self.futama2.get_planning_scene_monitor()
        self.logger = get_logger('moveit_py.pose_goal')
        self.logger.info('MoveItPy instance created')

        poses = get_poses()
        first_pose = (-1.2055292081832886, -0.17412646114826202, 0.7930378317832947,
                      -0.18334272503852844, -1.076518492482137e-05, 0.9830490946769714, 2.7753067115554586e-05)
        path_msg = self.to_path_msg([first_pose, *poses])

        time.sleep(5)

        self.publish_path(path_msg)

    

    def to_path_msg(self,
                    path_poses_tuple: list[tuple]) -> Path:
        path_msg = Path()
        current_time = self.get_clock().now().to_msg()
        path_msg.header = Header(frame_id='base_link', stamp=current_time)

        # change position of the object based on the transformation
        for i, pose in enumerate(path_poses_tuple[1:]):
            new_tuple = list(pose)
            new_tuple[0] += -0.4
            new_tuple[1] += -0.4
            path_poses_tuple[i] = tuple(new_tuple)

        path_msg.poses = [tuple_to_pose(pose_tuple, current_time)
                          for pose_tuple in path_poses_tuple]

        return path_msg

    def robot_states_callback(self, msg):
        pass

    def move_group_planner_and_executer(self, pose: PoseStamped):
        #  set plan start state to current state
        self.logger.info('PLANNING AND EXECUTING TO GIVEN POSE')
        self.move_group.set_start_state_to_current_state()
        
        # plan to goal
        self.cam_pose_pub.publish(pose)
        cameras = self.possible_camera_poses.copy()

        for camera, cam_pose in zip(cameras, alternative_poses(pose, cameras)):
            self.logger.info(f'CAMERA POSE: {camera}')
            
            self.move_group.set_goal_state(
                pose_stamped_msg=cam_pose, pose_link='realsense_front_link')
            self.current_point_pub.publish(cam_pose)

            reached = moveit_funcs.plan_and_execute(
                self.futama2,
                self.move_group,
                self.logger,
                sleep_time=0)

            if reached:
                self.arrival_times.append(self.get_clock().now())
                # resort the cameras, last used as first
                self.possible_camera_poses.remove(camera)
                self.possible_camera_poses.insert(0, camera)
                # publish reached point
                self.reached_point_pub.publish(pose)
                break

        self.reached_points.append(reached)
        self.logger.info('PLANNING AND EXECUTING TO GIVEN POSE')

    def publish_path(self, path_msg: Path):
        self.path_pub.publish(path_msg)

        start_time = self.get_clock().now()
        for i in path_msg.poses:
            self.move_group_planner_and_executer(i)

        total_time = self.get_clock().now() - start_time
        count_reached = self.reached_points.count(True)

        self.logger.info(f'FINISHED PATH, reached {count_reached} \
                         of {len(path_msg.poses)} points in \
                         {total_time}')
        self.logger.info(f'{self.reached_points}')
        self.logger.info(f'{self.arrival_times}')

def main(args=None):
    rclpy.init(args=args)
    node = AutoInsp()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
