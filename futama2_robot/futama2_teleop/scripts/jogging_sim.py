#!/usr/bin/env python3

from rclpy import init, spin, Parameter
import rclpy.duration
from rclpy.node import Node
from rclpy.logging import get_logger
import rclpy
from geometry_msgs.msg import Twist, PoseStamped
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import RobotState, MoveItErrorCodes
import rclpy.time
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros.transform_broadcaster import TransformBroadcaster
import tf_transformations as tf
import numpy as np

def msg2tf(msg):
    return np.array([msg.x, msg.y, msg.z, getattr(msg, 'w', 0)])

POSE_ORIGIN = [0.17433129251003265, -1.216812252998352, 0.849524974822998, 
               0.033915333449840546, 0.03393150493502617, -0.7062444090843201, 0.7063407301902771]
POSE_CONFINED_SPACE_FRONT = [-0.3134135603904724, -1.5278343439102173, 0.07761852443218231,
                             0.29451873898506165, 0.2945699691772461, -0.6427989602088928, 0.6428815722465515]

class Jog(Node):
    def __init__(self) -> None:
        super().__init__("jog_node", parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)])
        self.log = get_logger("jog")
        self.pub = self.create_publisher(Float64MultiArray, "/position_controller/commands", 1)
        self.ik = self.create_client(GetPositionIK, "/compute_ik")
        self.tf_buffer = Buffer()
        self.tfl = TransformListener(self.tf_buffer, self)
        self.tfb = TransformBroadcaster(self)
        self.fk_future = None
        self.ik_future = None
        self.robot_state = None
        self.ee_pose = None
        self.ee_link = "realsense_center_link"
        self.frame = "base_link"
        self.joints_arm = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
        self.total_joints = 18  # Updated to match O3DE expectations
        self.target_pos = np.array(POSE_ORIGIN[:3])
        self.target_quat = np.array(POSE_ORIGIN[3:])

        self.log.info("Waiting for transform")        
        future = self.tf_buffer.wait_for_transform_async(self.frame, self.ee_link, rclpy.time.Time())
        rclpy.spin_until_future_complete(self, future)
        
        self.log.info("Waiting for service")
        self.ik.wait_for_service(timeout_sec=5.0)
        self.create_subscription(JointState, "/joint_states", self.get_joint_states, 1)
        self.create_subscription(Twist, "/jog_control", self.get_jog_msg, 1)
        self.create_timer(0.01, self.compute_joint_positions)
    
    def send_cmd(self, target_state: RobotState):
        # Ensure we're sending exactly 6 joint positions
        command = [0.0] * len(self.joints_arm)  # Initialize with default values

        for i, joint in enumerate(self.joints_arm):
            if joint in target_state.joint_state.name:
                index = target_state.joint_state.name.index(joint)
                command[i] = target_state.joint_state.position[index]

        self.log.info(f"Publishing command: {command}")  # Debugging output
        self.pub.publish(Float64MultiArray(data=command))

    
    def compute_joint_positions(self):
        if not self.robot_state:
            return
        
        pose = PoseStamped()
        pose.header.frame_id = self.frame
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = self.target_pos
        pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = self.target_quat
        pose.header.stamp = self.get_clock().now().to_msg()
        
        if self.ik_future is None:
            request = GetPositionIK.Request()
            request.ik_request.robot_state = self.robot_state
            request.ik_request.group_name = "front"
            request.ik_request.ik_link_name = self.ee_link
            request.ik_request.pose_stamped = pose
            request.ik_request.timeout = rclpy.duration.Duration(seconds=0.1).to_msg()
            self.ik_future = self.ik.call_async(request)
        else:
            if self.ik_future.done():
                result = self.ik_future.result()
                if result.error_code.val == MoveItErrorCodes.SUCCESS:
                    self.send_cmd(result.solution)
                else:
                    self.log.error(f"IK error: {result.error_code.val}")
                self.ik_future = None
            elif self.ik_future.cancelled():
                self.ik_future = None
    
    def get_joint_states(self, msg: JointState):
        self.robot_state = RobotState()
        self.robot_state.joint_state = msg
    
    def get_jog_msg(self, msg: Twist):
        o = msg2tf(msg.linear)
        mat = tf.quaternion_matrix(self.target_quat)
        self.target_pos += mat[0:3,0] * o[1] + mat[0:3,1] * o[2] + mat[0:3,2] * o[0]
        rot = tf.quaternion_from_euler(msg.angular.x, msg.angular.y, msg.angular.z)
        self.target_quat = tf.quaternion_multiply(self.target_quat, rot)
        self.target_quat /= np.linalg.norm(self.target_quat)
    
    def move_to_pose(self, pose):
        self.target_pos = np.array(pose[:3])
        self.target_quat = np.array(pose[3:])

def main(args=None):
    init(args=args)
    jogger = Jog()
    jogger.move_to_pose(POSE_ORIGIN)  # Move to origin initially
    try:
        spin(jogger)
    except KeyboardInterrupt:
        pass
    jogger.destroy_node()

if __name__ == "__main__":
    main()