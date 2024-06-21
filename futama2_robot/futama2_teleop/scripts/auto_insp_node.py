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

max_vel = 0.35                              # [m/s]
min_depth_dist = 350                        # [mm]
const_eq = max_vel/(min_depth_dist)         # [-], tested with 0.001
depth_dist = 0                              # [mm]
start_time = 0                              # [s]

pose_origin = (-1.2055292081832886, -0.17412646114826202, 0.7930378317832947,
               -0.18334272503852844, -1.076518492482137e-05, 0.9830490946769714, 2.7753067115554586e-05)
previous_pose = [0, 0, 0,
                 0, 0, 0, 0]

class AutoInsp(Node):
    def __init__(self):

        global start_time

        super().__init__('auto_insp')

        # Subscriber: Odometry for robot states, useful to compare msg poses with commanded goals
        self.odom_sub = self.create_subscription(
            Odometry, '/rtabmap/odom', self.robot_states_callback, 10)
        self.odom_sub  # prevent unused variable warning

        # Subscriber: TF
        #self.tf_buffer = Buffer()
        #self.tf_listener = TransformListener(self.tf_buffer, self)

        #from_frame_rel = 'base_link'
        #to_frame_rel = 'object_link'
        #t = self.tf_buffer.lookup_transform(
        #    to_frame_rel,
        #    from_frame_rel,
        #    rclpy.time.Time())

        # Publisher
        self.path_pub = self.create_publisher(Path, 'path', 10)
        self.path_pub # prevent unused variable warning
  
        self.futama2 = MoveItPy(node_name="auto_insp")
        
        self.move_group = self.futama2.get_planning_component("front")
        self.planning_scene_monitor = self.futama2.get_planning_scene_monitor()
        self.logger = get_logger("moveit_py.pose_goal")
        self.logger.info("MoveItPy instance created")

        self.path_msg = Path()
        current_time = self.get_clock().now().to_msg()
        self.path_msg.header = Header(frame_id="base_link", stamp=current_time)

        self.path_poses_tuple = [pose_origin,(-0.016145400763403678, -1.0999990014901162, 0.17900748940026634, 0.39386246700844013, -0.39386246700844013, 0.8080135663928221, -0.19198643360717788), (0.07432726641042073, -0.027669960679839067, 1.1999989880790711, 0.8631250784721117, 0.3437153795061271, -0.13687464625030007, 0.3437152426316371), (1.0999989940395356, 0.07323824343380389, 0.15568264055700348, 7.027504675202831e-07, 0.9999999999996487, 4.570233080867851e-13, 4.570473147496338e-07), (-0.0022594261760940718, 1.099998986588955, 0.14449381438343087, -0.3237229985301906, -0.3237229985301906, 0.8810556655170242, 0.11894433448297581), (-1.0999989940395354, 0.07249259249029608, 0.14786869354756083, 0.9999999999996299, 7.402177812910198e-07, 4.38515014054426e-07, 4.385380947267745e-13)]
        # the origin should be added manually to the list of positions every time you get new ones! (for now)
        for i in range(1,len(self.path_poses_tuple)):
            pose = self.path_poses_tuple[i]
            new_tuple = list(pose)
            new_tuple[0] += -1.0    # TO DO change position of the object based on the transformation
            new_tuple[1] += 0.0
            new_tuple[2] += 0.0
            self.path_poses_tuple[i] = tuple(new_tuple)

        for self.pose in self.path_poses_tuple:
            self.pose_stamped = PoseStamped()
            self.pose_stamped.header = Header(frame_id="base_link", stamp=current_time)
            self.pose_stamped.pose.position.x = self.pose[0]        # it is shifted -1m in x because the 
            self.pose_stamped.pose.position.y = self.pose[1]
            self.pose_stamped.pose.position.z = self.pose[2]
            self.pose_stamped.pose.orientation.x = self.pose[3]
            self.pose_stamped.pose.orientation.y = self.pose[4]
            self.pose_stamped.pose.orientation.z = self.pose[5]
            self.pose_stamped.pose.orientation.w = self.pose[6]
            self.path_msg.poses.append(self.pose_stamped)

        self.current_pose_msg = PoseStamped()

        time.sleep(5.0) # here is the key for letting the other modules load (octomap and urdf)
        self.path_pub.publish(self.path_msg)    # show the path some seconds after the initialization
        self.auto_insp_mode()

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

    def move_group_planner_and_executer(self, pose_cmd):
        #  set plan start state to current state
        self.logger.info("PLANNING AND EXECUTING TO GIVEN POSE")
        self.move_group.set_start_state_to_current_state()
        pose_goal = PoseStamped()
        pose_goal.header.frame_id = "base_link"
        pose_goal.pose.position.x = pose_cmd[0]
        pose_goal.pose.position.y = pose_cmd[1]
        pose_goal.pose.position.z = pose_cmd[2]
        pose_goal.pose.orientation.x = pose_cmd[3]
        pose_goal.pose.orientation.y = pose_cmd[4]
        pose_goal.pose.orientation.z = pose_cmd[5]
        pose_goal.pose.orientation.w = pose_cmd[6]
        self.move_group.set_goal_state(
            pose_stamped_msg=pose_goal, pose_link="realsense_front_link")
        # plan to goal
        moveit_funcs.plan_and_execute(
            self.futama2, self.move_group, self.logger, sleep_time=0.5)
        self.logger.info("PLANNING AND EXECUTING TO GIVEN POSE")

    def auto_insp_mode(self):
        for i in self.path_poses_tuple:
            self.move_group_planner_and_executer(i)
            time.sleep(2.0)
        self.destroy_node()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = AutoInsp()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
