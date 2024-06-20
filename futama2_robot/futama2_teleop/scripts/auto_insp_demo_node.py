#!/usr/bin/env python3

# SPDX-FileCopyrightText: 2024 German Aerospace Center <adrian.ricardezortigosa@dlr.de>
#
# SPDX-License-Identifier: MIT

"""
Node that executes the automatic robotic inspection process around the wing.
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

from futama2_utils import moveit_funcs
from rclpy.logging import get_logger
from moveit.planning import MoveItPy

max_vel = 0.35                               # [m/s]
min_depth_dist = 350                        # [mm]
const_eq = max_vel/(min_depth_dist)         # [-], tested with 0.001
depth_dist = 0                              # [mm]
start_time = 0                              # [s]

pose_origin = [-1.2055292081832886, -0.17412646114826202, 0.7930378317832947,
               -0.18334272503852844, -1.076518492482137e-05, 0.9830490946769714, 2.7753067115554586e-05]
pose_confined_space_front = [-1.602797494407978, -0.08713420711220163, 0.1624797976718972,
                             -0.2173614367016838, 0.02425603873074822, 0.9749612559673085, -0.04020198729814948]
previous_pose = [0, 0, 0,
                 0, 0, 0, 0]

side_limits = [-0.9, 0.35, 1.0, 0.05]     # left, right, top, and bottom

n_steps_sides = 6
step_distance_sides = abs(side_limits[0]-side_limits[1])/n_steps_sides

distance_goal_y = 0
distance_travelled_y = 0
distance_goal_z = 0
distance_travelled_z = 0
count = 0

class AutoInsp(Node):
    def __init__(self):

        global start_time

        super().__init__('auto_insp_demo')

        # Publisher: Joy
        self.joy_pub = self.create_publisher(Joy, 'joy', 10)
        self.joy_cmd = Joy()
        self.joy_cmd.axes = [0, 0, 0, 0, 0, 0]
        self.joy_cmd.buttons = [0, 0]

        # Subscriber: Centric Depth Point
        self.depth_sub = self.create_subscription(
            Image, '/camera/camera/aligned_depth_to_color/image_raw', self.convert_depth_image_callback, 10)
        self.depth_sub  # prevent unused variable warning

        # Subscriber: Odometry for robot states
        self.odom_sub = self.create_subscription(
            Odometry, '/rtabmap/odom', self.robot_states_callback, 10)
        self.odom_sub  # prevent unused variable warning

        # Service client: Switch Controller
        self.switchcontroller_client = self.create_client(
            SwitchController, '/controller_manager/switch_controller')
        while not self.switchcontroller_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.futama2 = MoveItPy(node_name="auto_insp_demo")
        self.move_group = self.futama2.get_planning_component("front")
        self.planning_scene_monitor = self.futama2.get_planning_scene_monitor()
        self.logger = get_logger("moveit_py.pose_goal")
        self.logger.info("MoveItPy instance created")

        self.current_pose_msg = PoseStamped()

        time.sleep(5.0) # here is the key for letting the other modules load (octomap and urdf)

        # start the robot from origin, and avoid robot state update error to be able to use the forward position controller later
        self.switch_to_trajectory_mode()
        self.move_group_planner_and_executer(pose_origin)
        self.switch_to_forward_mode()

        time.sleep(1.0)

        self.auto_insp_demo_mode()

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

    def lin_x_calc_basedOnDepth(self):
        self.joy_cmd.axes[0] = max_vel-const_eq*depth_dist
        # self.joy_cmd.axes[0] = 0       # for debugging purposes (no UR10e connected with Ethernet)
        if self.joy_cmd.axes[0] > max_vel:
            self.joy_cmd.axes[0] = max_vel
        elif self.joy_cmd.axes[0] < -max_vel:
            self.joy_cmd.axes[0] = -max_vel

    def vel_cmd_publisher(self):
        current_time = self.get_clock().now().to_msg()
        self.joy_cmd.header.stamp = current_time
        self.joy_cmd.axes
        self.joy_pub.publish(self.joy_cmd)

    def move_forward_until_depth(self):
        global previous_pose
        self.lin_x_calc_basedOnDepth()
        self.vel_cmd_publisher()

        status = 'executing'

        # if the robot has reached certain distance to the wing
        if depth_dist <= min_depth_dist+20 and depth_dist > 0:
            previous_pose[0] = self.current_pose_msg.pose.position.x
            previous_pose[1] = self.current_pose_msg.pose.position.y
            previous_pose[2] = self.current_pose_msg.pose.position.z
            self.joy_cmd.axes[0] = 0
            status = 'goal_reached'

        return status

    def receiving_spacemouse_commands(self):
        status = 'executing'

        return status

    def move_distance_y(self, goal_y, start_y):
        global distance_goal_y, distance_travelled_y, previous_pose, count
        if goal_y > start_y:
            self.joy_cmd.axes[1] = max_vel
            self.joy_cmd.axes[5] = -max_vel
        else:
            self.joy_cmd.axes[1] = -max_vel
            self.joy_cmd.axes[5] = max_vel

        # self.lin_x_calc_basedOnDepth()
        self.joy_cmd.axes[2] = 0
        self.joy_cmd.axes[4] = 0
        self.vel_cmd_publisher()

        status = 'executing'

        distance_goal_y = abs(goal_y - start_y)
        distance_travelled_y = abs(
            start_y - self.current_pose_msg.pose.position.y)

        if abs(distance_travelled_y) >= abs(distance_goal_y):
            previous_pose[0] = self.current_pose_msg.pose.position.x
            previous_pose[1] = self.current_pose_msg.pose.position.y
            previous_pose[2] = self.current_pose_msg.pose.position.z
            status = 'goal reached'

        return status

    def move_distance_z(self, goal_z, start_z):
        global distance_goal_z, distance_travelled_z
        if goal_z > start_z:
            self.joy_cmd.axes[2] = max_vel
            self.joy_cmd.axes[4] = max_vel
        else:
            self.joy_cmd.axes[2] = -max_vel
            self.joy_cmd.axes[4] = -max_vel

        self.joy_cmd.axes[1] = 0
        self.joy_cmd.axes[5] = 0
        self.vel_cmd_publisher()

        status = 'executing'

        distance_goal_z = abs(goal_z - start_z)
        distance_travelled_z = abs(
            start_z - self.current_pose_msg.pose.position.z)

        if abs(distance_travelled_z) >= abs(distance_goal_z):
            previous_pose[0] = self.current_pose_msg.pose.position.x
            previous_pose[1] = self.current_pose_msg.pose.position.y
            previous_pose[2] = self.current_pose_msg.pose.position.z
            status = 'goal_reached'

        return status

    def roll_pitch_circle(self):
        global start_time
        if time.time() - start_time <= 2:
            self.joy_cmd.axes = [0, 0, 0, 0, 0, 0.5]
            self.vel_cmd_publisher()
        elif time.time() - start_time >= 2:
            for i in range(10000):
                self.joy_cmd.axes = [0, 0, 0, -0.5, 0.5, -0.5]
                self.vel_cmd_publisher()
            for i in range(10000):
                self.joy_cmd.axes = [0, 0, 0, -0.5, -0.5, -0.5]
                self.vel_cmd_publisher()
            for i in range(10000):
                self.joy_cmd.axes = [0, 0, 0, 0.5, -0.5, 0.5]
                self.vel_cmd_publisher()
            for i in range(10000):
                self.joy_cmd.axes = [0, 0, 0, 0.5, 0.5, 0.5]
                self.vel_cmd_publisher()
            for i in range(10000):
                self.joy_cmd.axes = [0, 0, 0, 0, 0, -0.5]
                self.vel_cmd_publisher()
            self.joy_cmd.axes = [0, 0, 0, 0, 0, 0]
            self.vel_cmd_publisher()

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

    def switch_to_trajectory_mode(self):
        self.req = SwitchController.Request()
        self.req.activate_controllers = {"joint_trajectory_controller"}
        self.req.deactivate_controllers = {"forward_position_controller"}
        self.future = self.switchcontroller_client.call_async(self.req)
        self.logger.info("CONTROLLER TO JOINT TRAJECTORY SWITCHED!")
        time.sleep(1)

    def switch_to_forward_mode(self):
        self.req = SwitchController.Request()
        self.req.activate_controllers = {"forward_position_controller"}
        self.req.deactivate_controllers = {"joint_trajectory_controller"}
        self.future = self.switchcontroller_client.call_async(self.req)
        self.logger.info("CONTROLLER TO FORWARD POSITION SWITCHED!")
        time.sleep(1)

    def auto_insp_demo_mode(self):
        while self.move_forward_until_depth() == 'executing':
            rclpy.spin_once(self)
        while self.move_distance_y(side_limits[0], previous_pose[1]) == 'executing':
            rclpy.spin_once(self)
        while self.move_distance_z(side_limits[3], previous_pose[2]) == 'executing':
            rclpy.spin_once(self)
        while self.move_distance_z(side_limits[2], previous_pose[2]) == 'executing':
            rclpy.spin_once(self)

        zigzag_iterations = int(n_steps_sides/2)
        for i in range(zigzag_iterations):
            while self.move_distance_y(previous_pose[1]+step_distance_sides, previous_pose[1]) == 'executing':
                rclpy.spin_once(self)
            while self.move_distance_z(side_limits[3], previous_pose[2]) == 'executing':
                rclpy.spin_once(self)
            while self.move_distance_y(previous_pose[1]+step_distance_sides, previous_pose[1]) == 'executing':
                rclpy.spin_once(self)
            while self.move_distance_z(side_limits[2], previous_pose[2]) == 'executing':
                rclpy.spin_once(self)
        self.switch_to_trajectory_mode()
        self.move_group_planner_and_executer(pose_origin)
        time.sleep(2.0)
        self.move_group_planner_and_executer(pose_confined_space_front)
        time.sleep(2.0)
        self.switch_to_forward_mode()
        while self.receiving_spacemouse_commands() == 'executing':
            print("forward_controller_active")
        # currently, you need to run the spacenav node manually, TO DO run it from here when the automatic
        # inspection has been performed
        self.destroy_node()
        rclpy.shutdown()

    def convert_depth_image_callback(self, ros_image):
        global depth_dist

        bridge = CvBridge()
        depth_image = bridge.imgmsg_to_cv2(
            ros_image, desired_encoding="passthrough")
        depth_array = np.array(depth_image, dtype=np.float32)
        center_idx = np.array(depth_array.shape) / 2
        depth_dist = depth_array[int(center_idx[0]), int(center_idx[1])]

def main(args=None):
    rclpy.init(args=args)
    node = AutoInsp()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
