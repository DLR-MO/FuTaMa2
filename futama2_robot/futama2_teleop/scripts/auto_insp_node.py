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

import rclpy
import rclpy.logging
from rclpy.node import Node
from sensor_msgs.msg import Joy
from cv_bridge import CvBridge
from std_msgs.msg import Header
from sensor_msgs.msg import Image
import numpy as np
import time
from scipy.spatial.transform import Rotation as R
from controller_manager_msgs.srv import SwitchController
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry, Path
#from pathlib import Path
#from object_inspection_pathfinding import Camera, solve_wmp, PathPlotter

from futama2_utils import moveit_funcs
from rclpy.logging import get_logger
from moveit.planning import MoveItPy

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from geometry_msgs.msg import PoseStamped, Quaternion


class AutoInsp(Node):

    T = 0.7071
    possible_camera_poses = [
        R.from_euler('XYZ', [0,   0, 0], degrees=True),  # Front
        R.from_euler('XYZ', [90,  0, 0], degrees=True),  # 90°
        R.from_euler('XYZ', [180, 0, 0], degrees=True),  # 180°
        R.from_euler('XYZ', [270, 0, 0], degrees=True),  # 270°

        R.from_euler('ZYX', [0,        90,  0], degrees=True),  # Top
        R.from_euler('ZYX', [0,       270,  180], degrees=True),  # 180° (wrong)
        R.from_euler('ZYX', [0,         0,  90], degrees=True),  # 90°
        R.from_euler('ZYX', [-90,       0, -90], degrees=True),  # 270°

        R.from_euler('ZYX', [0,       -90, 0], degrees=True),  # Bottom
        R.from_euler('ZYX', [0,      90,   180], degrees=True),  # 180°
        R.from_euler('ZYX', [-90,     0,   90], degrees=True),  # 90°
        R.from_euler('ZYX', [90,      0,  -90], degrees=True),  # 270°
    ]
    reached_points = []

    def __init__(self):

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
        self.reached_point_pub = self.create_publisher(PoseStamped,
                                                       'reached_point',
                                                       100)
        self.top_point_pub = self.create_publisher(PoseStamped,
                                                    'top_point',
                                                    100)

        self.futama2 = MoveItPy(node_name="auto_insp")
        self.move_group = self.futama2.get_planning_component('front')
        self.planning_scene_monitor = self.futama2.get_planning_scene_monitor()
        self.logger = get_logger('moveit_py.pose_goal')
        self.logger.info('MoveItPy instance created')

        poses = [(0.06282123862824529, -0.3559787706555339, 0.10522103126413154, 0.5000000000000001, 0.5000000000000001, -0.5, -0.5), (0.055098049972206506, -0.35067563901584103, 0.10235027383687303, 0.5000000000000001, 0.5000000000000001, -0.5, -0.5), (0.036339070201352526, -0.3376603189182231, 0.09660152482467105, 0.5000000000000001, 0.5000000000000001, -0.5, -0.5), (0.008683313817436984, -0.3177510317024007, 0.09013778372319163, 0.5000000000000001, 0.5000000000000001, -0.5, -0.5), (-0.023562216687914805, -0.29271791732973335, 0.08379294591206257, 0.5000000000000001, 0.5000000000000001, -0.5, -0.5), (-0.129947487967703, -0.2041879575088263, 0.15141524292211142, 0.5000000000000001, 0.5000000000000001, -0.5, -0.5), (-0.27068240284563805, -0.02959183594400629, 0.09854469682125604, 0.999999999999875, 4.999999999998125e-07, 1.5308084989332347e-23, 3.0616169978664694e-17), (-0.3009345016955633, 0.006301338876568242, 0.11601075423581159, 0.999999999999875, 4.999999999998125e-07, 1.5308084989332347e-23, 3.0616169978664694e-17), (-0.2922701008125606, 0.010341625910052995, 0.1599590956927574, 0.999999999999875, 4.999999999998125e-07, 1.5308084989332347e-23, 3.0616169978664694e-17), (-0.2577325406245109, -0.01842459511839839, 0.20042335833038444, 0.999999999999875, 4.999999999998125e-07, 1.5308084989332347e-23, 3.0616169978664694e-17), (-0.211102161476801, -0.048191241859596105, 0.23525294350397855, 0.999999999999875, 4.999999999998125e-07, 1.5308084989332347e-23, 3.0616169978664694e-17), (-0.037441492828559714, -0.028803707356183414, 0.3791158377131998, 0.707106781185973, 3.5355339063663785e-07, -0.7071067811866801, -7.071067811433822e-07), (-0.01082108675708681, -0.010493362122495353, 0.3919457305685165, 0.707106781185973, 3.5355339063663785e-07, -0.7071067811866801, -7.071067811433822e-07), (0.007748720243978214, 0.012974340157772747, 0.3866242356683392, 0.707106781185973, 3.5355339063663785e-07, -0.7071067811866801, -7.071067811433822e-07), (0.12673319837867092, -0.0033687233281112228, 0.29999898807907105, 0.707106781185973, 3.5355339063663785e-07, -0.7071067811866801, -7.071067811433822e-07), (0.2792912545274155, -0.0163942285837505, 0.10351501965714385, -4.5924254967997035e-23, -9.184850993599407e-17, 0.999999999999875, 4.999999999998125e-07), (0.29750055753698856, 0.007902617516020146, 0.1033478792228592, -4.5924254967997035e-23, -9.184850993599407e-17, 0.999999999999875, 4.999999999998125e-07), (0.2927154984410495, -0.010134417185127368, 0.09959488847181251, -4.5924254967997035e-23, -9.184850993599407e-17, 0.999999999999875, 4.999999999998125e-07), (0.260835637289361, 0.024999795250587778, 0.09285966407669236, -4.5924254967997035e-23, -9.184850993599407e-17, 0.999999999999875, 4.999999999998125e-07), (0.0, 0.2732030570220154, 0.11923296654225635, 0.5, -0.5, -0.5, 0.5), (0.00768873111076583, 0.3090412358492467, 0.09701704210422331, 0.5, -0.5, -0.5, 0.5), (0.011533096570039612, 0.32696032481488513, 0.08590908016290581, 0.5, -0.5, -0.5, 0.5)]
        first_pose = (-1.2055292081832886, -0.17412646114826202, 0.7930378317832947, -0.18334272503852844, -1.076518492482137e-05, 0.9830490946769714, 2.7753067115554586e-05)
        path_msg = self.to_path_msg(poses, first_pose)

        time.sleep(5)

        self.publish_path(path_msg)

    def to_path_msg(self,
                    path_points: list[tuple],
                    first_pose: tuple) -> Path:
        path_msg = Path()
        current_time = self.get_clock().now().to_msg()
        path_msg.header = Header(frame_id='base_link', stamp=current_time)

        path_poses_tuple = [first_pose, *path_points]

        # change position of the object based on the transformation
        for i, pose in enumerate(path_poses_tuple[1:]):
            new_tuple = list(pose)
            new_tuple[0] += -0.4
            new_tuple[1] += -0.4
            path_poses_tuple[i] = tuple(new_tuple)

        for pose in path_poses_tuple:
            pose_stamped = PoseStamped()
            pose_stamped.header = Header(frame_id='base_link',
                                         stamp=current_time)
            pose_stamped.pose.position.x = pose[0]
            pose_stamped.pose.position.y = pose[1]
            pose_stamped.pose.position.z = pose[2]
            pose_stamped.pose.orientation.x = pose[3]
            pose_stamped.pose.orientation.y = pose[4]
            pose_stamped.pose.orientation.z = pose[5]
            pose_stamped.pose.orientation.w = pose[6]
            path_msg.poses.append(pose_stamped)

        return path_msg

    def robot_states_callback(self, msg):
        pass

    def move_group_planner_and_executer(self, pose: PoseStamped):
        #  set plan start state to current state
        self.logger.info('PLANNING AND EXECUTING TO GIVEN POSE')
        self.move_group.set_start_state_to_current_state()
        
        # plan to goal
        base_quaternion = pose.pose.orientation
        base_quaternion = R.from_quat(
            np.array([base_quaternion.x, base_quaternion.y,
                      base_quaternion.z, base_quaternion.w]))

        reached = False
        cameras = self.possible_camera_poses.copy()

        while not reached and cameras:
            camera = cameras.pop()

            camera_rot = base_quaternion * camera

            camera_quat = camera_rot.as_quat()

            pose.pose.orientation = Quaternion(
                x=camera_quat[0], y=camera_quat[1],
                z=camera_quat[2], w=camera_quat[3])
            self.move_group.set_goal_state(
                pose_stamped_msg=pose, pose_link="realsense_front_link")

            self.current_point_pub.publish(pose)

            reached = moveit_funcs.plan_and_execute(
                self.futama2,
                self.move_group,
                self.logger,
                sleep_time=0.5)

            if reached:
                self.reached_point_pub.publish(pose)

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
                         {total_time} seconds')
        self.logger.info(f'{self.reached_points}')

def main(args=None):
    rclpy.init(args=args)
    node = AutoInsp()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
