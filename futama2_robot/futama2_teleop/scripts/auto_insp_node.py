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

from geometry_msgs.msg import PoseStamped



class AutoInsp(Node):
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

        self.futama2 = MoveItPy(node_name="auto_insp")
        self.move_group = self.futama2.get_planning_component('front')
        self.planning_scene_monitor = self.futama2.get_planning_scene_monitor()
        self.logger = get_logger('moveit_py.pose_goal')
        self.logger.info('MoveItPy instance created')

        poses = [(0.34828265137358727, 0.04334721225735272, 0.09770813834758717, -3.5355338990360336e-07, -0.7071067798072067, 0.7071067825657115, 3.535533912828557e-07), (0.3307135540413395, 0.06215361945625533, 0.10120694284110104, -3.5355338990360336e-07, -0.7071067798072067, 0.7071067825657115, 3.535533912828557e-07), (0.30289350997919445, 0.08133009826091775, 0.10988117813610833, -3.5355338990360336e-07, -0.7071067798072067, 0.7071067825657115, 3.535533912828557e-07), (0.2647076220284396, 0.10304996094489774, 0.12319445782378577, -3.5355338990360336e-07, -0.7071067798072067, 0.7071067825657115, 3.535533912828557e-07), (0.19999899403953553, 0.12673302281609622, 0.09663136941287138, -3.5355338990360336e-07, -0.7071067798072067, 0.7071067825657115, 3.535533912828557e-07), (0.02499999850988395, 0.26083562983889563, 0.09285966407675905, -2.1648901405887332e-17, 2.1648901405887332e-17, 0.7071067811865475, -0.7071067811865475), (-0.010234209321916614, 0.29250096693927674, 0.11114165497564422, -2.1648901405887332e-17, 2.1648901405887332e-17, 0.7071067811865475, -0.7071067811865475), (0.023168887753034703, 0.26476245952917826, 0.1732333701402631, -2.1648901405887332e-17, 2.1648901405887332e-17, 0.7071067811865475, -0.7071067811865475), (0.0008931754187249718, 0.20913838088840572, 0.2337460887650202, -2.1648901405887332e-17, 2.1648901405887332e-17, 0.7071067811865475, -0.7071067811865475), (0.0, 0.08076680322098062, 0.37320305851213176, 0.707106781185973, 3.5355339063663785e-07, -0.7071067811866801, -7.071067811433822e-07), (-0.002651946768384376, -0.01709130239394043, 0.3777963650539413, 0.707106781185973, 3.5355339063663785e-07, -0.7071067811866801, -7.071067811433822e-07), (-0.19999899403953553, 0.04226438231972314, 0.196631422504984, 0.7071067825657116, 3.535533912828557e-07, 3.5355338990360336e-07, 0.7071067798072067), (-0.24222292240392146, 0.03861564325575525, 0.1631814324532903, 0.7071067825657116, 3.535533912828557e-07, 3.5355338990360336e-07, 0.7071067798072067), (-0.27643514923884205, 0.0317192356180758, 0.1291348646940886, 0.7071067825657116, 3.535533912828557e-07, 3.5355338990360336e-07, 0.7071067798072067), (-0.29833953760295995, 0.020026722429623563, 0.0924883958871602, 0.7071067825657116, 3.535533912828557e-07, 3.5355338990360336e-07, 0.7071067798072067), (-0.2910790591639439, 0.004107708209459295, 0.08910269406893434, 0.7071067825657116, 3.535533912828557e-07, 3.5355338990360336e-07, 0.7071067798072067), (-0.2487325880968339, -0.014128043573325172, 0.14268673358119063, 0.7071067825657116, 3.535533912828557e-07, 3.5355338990360336e-07, 0.7071067798072067), (-0.05201731422844941, -0.2115491256377624, 0.1355959165240221, 0.7071067811865475, 0.7071067811865475, 2.1648901405887332e-17, 2.1648901405887332e-17), (0.020457711779740553, -0.27057661006778805, 0.11653412176362321, 0.7071067811865475, 0.7071067811865475, 2.1648901405887332e-17, 2.1648901405887332e-17), (0.011722850374638027, -0.2893085808007223, 0.1020341592340844, 0.7071067811865475, 0.7071067811865475, 2.1648901405887332e-17, 2.1648901405887332e-17), (-0.006642708807921227, -0.3002029795477147, 0.09560412702187897, 0.7071067811865475, 0.7071067811865475, 2.1648901405887332e-17, 2.1648901405887332e-17)]
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
            new_tuple[0] += -1.0
            new_tuple[1] += 0.0
            new_tuple[2] += 0.0
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
        self.current_pose_msg.header.frame_id = msg.header.frame_id
        self.current_pose_msg.header.stamp = msg.header.stamp

        self.current_pose_msg.pose.position.x = msg.pose.pose.position.x
        self.current_pose_msg.pose.position.y = msg.pose.pose.position.y
        self.current_pose_msg.pose.position.z = msg.pose.pose.position.z

        self.current_pose_msg.pose.orientation.x = msg.pose.pose.orientation.x
        self.current_pose_msg.pose.orientation.y = msg.pose.pose.orientation.y
        self.current_pose_msg.pose.orientation.z = msg.pose.pose.orientation.z
        self.current_pose_msg.pose.orientation.w = msg.pose.pose.orientation.w

    def move_group_planner_and_executer(self, pose: PoseStamped):
        #  set plan start state to current state
        self.logger.info('PLANNING AND EXECUTING TO GIVEN POSE')
        self.move_group.set_start_state_to_current_state()
        self.move_group.set_goal_state(
            pose_stamped_msg=pose, pose_link="realsense_front_link")
        # plan to goal
        moveit_funcs.plan_and_execute(
            self.futama2,
            self.move_group,
            self.logger,
            sleep_time=0.5)

        self.logger.info('PLANNING AND EXECUTING TO GIVEN POSE')

    def publish_path(self, path_msg: Path):
        self.path_pub.publish(path_msg)

        for i in path_msg.poses:
            self.move_group_planner_and_executer(i)
            time.sleep(2.0)

def main(args=None):
    rclpy.init(args=args)
    node = AutoInsp()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
