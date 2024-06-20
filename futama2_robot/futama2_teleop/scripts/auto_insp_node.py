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
from sensor_msgs.msg import Image
import numpy as np
import time
from controller_manager_msgs.srv import SwitchController
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
#from pathlib import Path
#from object_inspection_pathfinding import Camera, solve_wmp, PathPlotter

from futama2_utils import moveit_funcs
from rclpy.logging import get_logger
from moveit.planning import MoveItPy

max_vel = 0.35                              # [m/s]
min_depth_dist = 350                        # [mm]
const_eq = max_vel/(min_depth_dist)         # [-], tested with 0.001
depth_dist = 0                              # [mm]
start_time = 0                              # [s]

pose_origin = [-1.2055292081832886, -0.17412646114826202, 0.7930378317832947,
               -0.18334272503852844, -1.076518492482137e-05, 0.9830490946769714, 2.7753067115554586e-05]
pose_example1 = [-1.602797494407978, -0.08713420711220163, 0.1624797976718972,
                             -0.2173614367016838, 0.02425603873074822, 0.9749612559673085, -0.04020198729814948]
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

        self.futama2 = MoveItPy(node_name="auto_insp")
        
        self.move_group = self.futama2.get_planning_component("front")
        self.planning_scene_monitor = self.futama2.get_planning_scene_monitor()
        self.logger = get_logger("moveit_py.pose_goal")
        self.logger.info("MoveItPy instance created")

        
        # TO DO -> define the origin position based on the first starting point of the planning scene 
        #stl_file = Path("3d_cube.stl")  # using a cube as an example
        #camera = Camera(30, 5, 10, 50, 1.5)
        #path, orientations = solve_wmp(stl_file.absolute().resolve(), camera, 5, 3, False, True)
        #path_plotter = PathPlotter(stl_file)
        #path_plotter.plot_path(path, orientations)

        self.current_pose_msg = PoseStamped()

        time.sleep(5.0) # here is the key for letting the other modules load (octomap and urdf)

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
        self.move_group_planner_and_executer(pose_origin)
        time.sleep(2.0)
        self.move_group_planner_and_executer(pose_example1)
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
