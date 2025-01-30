#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from moveit.planning import MoveItPy, PlanRequestParameters
from geometry_msgs.msg import PoseStamped

# ✅ Define Target Poses
POSE_ORIGIN = [0.1743, -1.2168, 0.8495, 0.0339, 0.0339, -0.7062, 0.7063]
POSE_CONFINED_SPACE_FRONT = [-0.3134, -1.5278, 0.0776, 0.2945, 0.2946, -0.6428, 0.6429]

class MinimalMotionPlanner(Node):
    def __init__(self):
        super().__init__('minimal_motion_planner')

        # ✅ Initialize MoveItPy
        self.moveit_py = MoveItPy(node_name="minimal_motion_planner")
        self.move_group = self.moveit_py.get_planning_component("front")  # Adjust to your planning group

        # ✅ Load Motion Planning Parameters (Pass moveit_py object!)
        self.plan_parameters = PlanRequestParameters(self.moveit_py, "ompl")
        self.plan_parameters.planner_id = "RRTConnectkConfigDefault"
        self.plan_parameters.planning_time = 5.0
        self.plan_parameters.planning_attempts = 5
        self.plan_parameters.max_velocity_scaling_factor = 1.0
        self.plan_parameters.max_acceleration_scaling_factor = 1.0

        self.get_logger().info("✅ MoveItPy Initialized!")

        # ✅ Move the robot between two positions
        self.execute_motion_sequence()

    def move_to_pose(self, pose_cmd):
        """Plans and executes a move to a target pose."""
        self.get_logger().info(f"📌 Planning motion to: {pose_cmd}")

        # ✅ Create a PoseStamped message
        pose_goal = PoseStamped()
        pose_goal.header.frame_id = "base_link"
        pose_goal.pose.position.x, pose_goal.pose.position.y, pose_goal.pose.position.z = pose_cmd[:3]
        pose_goal.pose.orientation.x, pose_goal.pose.orientation.y, pose_goal.pose.orientation.z, pose_goal.pose.orientation.w = pose_cmd[3:]

        # ✅ Set goal state
        self.move_group.set_goal_state(pose_stamped_msg=pose_goal, pose_link="realsense_front_link")

        # ✅ Plan and Execute
        plan_result = self.move_group.plan(single_plan_parameters=self.plan_parameters)
        if plan_result and plan_result.trajectory:
            self.get_logger().info("✅ Planning successful, executing...")
            self.moveit_py.execute(plan_result.trajectory)
            return True
        else:
            self.get_logger().error("❌ Planning failed")
            return False

    def execute_motion_sequence(self):
        """Executes a sequence of movements."""
        if self.move_to_pose(POSE_ORIGIN):
            self.get_logger().info("✅ Successfully moved to POSE_ORIGIN")
        if self.move_to_pose(POSE_CONFINED_SPACE_FRONT):
            self.get_logger().info("✅ Successfully moved to POSE_CONFINED_SPACE_FRONT")

def main(args=None):
    rclpy.init(args=args)
    node = MinimalMotionPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
