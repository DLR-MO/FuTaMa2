#!/usr/bin/env python3

# SPDX-FileCopyrightText: 2025 German Aerospace Center <adrian.ricardezortigosa@dlr.de>
#
# SPDX-License-Identifier: MIT

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from controller_manager_msgs.srv import SwitchController
from control_msgs.msg import JointJog
from moveit_msgs.srv import ServoCommandType

BUTTONS_PRO = {
    "Menu": 0,
    "Fit": 1,
    "T": 2,
    "R": 4,
    "F": 5,
    "Alt": 23,
    "Ctrl": 25,
    "Shift": 24,
    "ESC": 22,
    "Joystick Rotation": 26,
    "1": 12,
    "2": 13,
    "3": 14,
    "4": 15,
    "Square": 8
}

BUTTONS_SIMPLE = {
    "Right": 0,
    "Left": 1,
}

JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint"
]

BUTTON_TO_JOINT = {
    "Square": JOINT_NAMES[0],
    "T": JOINT_NAMES[1],
    "Fit": JOINT_NAMES[2],
    "F": JOINT_NAMES[3],
    "R": JOINT_NAMES[4],
    "Joystick Rotation": JOINT_NAMES[5]
}

BUTTON_TO_MODE = {
    "1": "rox_teleop_mode",
    "3": "ur_cartesian_mode",
    "4": "ur_joint_mode"
}

class SpacenavToCmdVel(Node):
    def __init__(self):
        super().__init__('spacenav_to_cmd_vel')

        self.servo_mode_client = self.create_client(ServoCommandType, "/servo_node/switch_command_type")
        self.spacemouse_mdl = self.declare_parameter("spacemouse_mdl", "simple").get_parameter_value().string_value
        self.get_logger().info(f"Using SpaceMouse model: {self.spacemouse_mdl}")

        self.sub_spacemouse_filtered = self.create_subscription(Joy, '/spacenav/joy/soft_filter', self.soft_filter_spacemouse_joy_callback, 10)
        self.sub_spacemouse_raw = self.create_subscription(Joy, '/spacenav/joy', self.raw_spacemouse_joy_callback, 10)
        self.sub_spacemouse_normal_filter = self.create_subscription(Joy, '/spacenav/joy/normal_filter', self.normal_filter_spacemouse_joy_callback, 10)

        self.rox_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.ur_publisher = self.create_publisher(Joy, '/joy_ur', 10)
        self.joint_jog_publisher = self.create_publisher(JointJog, '/servo_node/delta_joint_cmds', 10)
        self.controller_switch_client = self.create_client(SwitchController, '/controller_manager/switch_controller')

        self.previous_button_states = [0] * (27 if self.spacemouse_mdl == "pro" else 2)
        self.current_mode = "rox_teleop_mode"
        self.simple_mode_counter = 1
        self.active_joint = None

        self.get_logger().info("Spacenav Joy to CmdVel Node Started!")

    def switch_servo_to_joint_mode(self):
        self._switch_servo_command_type(0)

    def switch_servo_to_cartesian_mode(self):
        self._switch_servo_command_type(1)

    def _switch_servo_command_type(self, command_type):
        if not self.servo_mode_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("servo_node/switch_command_type service not available.")
            return
        req = ServoCommandType.Request()
        req.command_type = command_type
        future = self.servo_mode_client.call_async(req)
        future.add_done_callback(lambda fut: self.get_logger().info("Switched servo_node to {} mode.".format("JOINT" if command_type == 0 else "CARTESIAN")))

    def ensure_button_length(self, msg):
        if len(msg.buttons) < 27:
            msg.buttons.extend([0] * (27 - len(msg.buttons)))

    def normal_filter_spacemouse_joy_callback(self, msg):
        if len(msg.axes) < 6:
            self.get_logger().warn("Received Joy message with insufficient axes (expected 6).")
            return

        if self.current_mode == "rox_teleop_mode" and not self.active_joint:
            twist_msg = Twist()
            twist_msg.linear.x = msg.axes[0]
            twist_msg.linear.y = msg.axes[1]
            twist_msg.angular.z = msg.axes[5]
            self.rox_publisher.publish(twist_msg)

    def soft_filter_spacemouse_joy_callback(self, msg):
        if len(msg.axes) < 6:
            self.get_logger().warn("Received Joy message with insufficient axes (expected 6).")
            return

        if (self.current_mode == "ur_cartesian_mode" or self.current_mode == "ur_joint_mode") and not self.active_joint:
            self.ur_publisher.publish(msg)

        if self.active_joint:
            joint_cmd = JointJog()
            joint_cmd.joint_names = [self.active_joint]
            avg_vel = (msg.axes[3] + msg.axes[4] + msg.axes[5]) / 2.0
            joint_cmd.velocities = [avg_vel]
            joint_cmd.header.stamp = self.get_clock().now().to_msg()
            joint_cmd.header.frame_id = "base_link"
            #self.get_logger().info(f"Joint control: {self.active_joint}, vel={avg_vel:.3f}")
            self.joint_jog_publisher.publish(joint_cmd)

    def raw_spacemouse_joy_callback(self, msg):
        self.ensure_button_length(msg)

        if self.spacemouse_mdl == "pro":
            for btn_name, joint_name in BUTTON_TO_JOINT.items():
                idx = BUTTONS_PRO.get(btn_name)
                if idx is not None and msg.buttons[idx] == 0 and self.previous_button_states[idx] == 1:
                    if self.active_joint == joint_name:
                        self.get_logger().info(f"✋ Joint control disabled for {joint_name}, EE actuation re-enabled")
                        self.active_joint = None
                        self.switch_servo_to_cartesian_mode()
                    else:
                        self.get_logger().info(f"🧾 Joint control enabled for {joint_name}, EE actuation disabled")
                        self.active_joint = joint_name
                        self.switch_servo_to_joint_mode()

            for btn_name, mode in BUTTON_TO_MODE.items():
                idx = BUTTONS_PRO.get(btn_name)
                if idx is not None and msg.buttons[idx] == 1 and self.previous_button_states[idx] == 0:
                    if mode == "rox_teleop_mode":
                        self.current_mode = "rox_teleop_mode"
                        self.active_joint = None
                        self.get_logger().info("Switched to 'rox' mode: publishing to /cmd_vel")
                    elif mode == "ur_cartesian_mode":
                        self.get_logger().info("Switching to forward_position_controller")
                        self.switch_controller("joint_trajectory_controller", "forward_position_controller")
                        self.active_joint = None
                        self.switch_servo_to_cartesian_mode()
                        self.current_mode = "ur_cartesian_mode"
                    elif mode == "ur_joint_mode":
                        self.get_logger().info("Switching to joint_trajectory_controller")
                        self.switch_controller("forward_position_controller", "joint_trajectory_controller")
                        self.active_joint = None
                        self.switch_servo_to_cartesian_mode()
                        self.current_mode = "ur_joint_mode"

        elif self.spacemouse_mdl == "simple":
            current_button = msg.buttons[BUTTONS_SIMPLE["Left"]]

            if self.previous_button_states[BUTTONS_SIMPLE["Left"]] == 0 and current_button == 1:
                self.simple_mode_counter += 1
                if self.simple_mode_counter > 3:
                    self.simple_mode_counter = 1

                if self.simple_mode_counter == 1:
                    self.get_logger().info("Simple: Switching to trajectory mode")
                    self.switch_controller("forward_position_controller", "joint_trajectory_controller")
                    self.current_mode = "rox_teleop_mode"

                elif self.simple_mode_counter == 2:
                    self.get_logger().info("Simple: Switching to cartesian mode")
                    self.switch_controller("joint_trajectory_controller", "forward_position_controller")
                    self.current_mode = "ur_cartesian_mode"
                    self.switch_servo_to_cartesian_mode()

                elif self.simple_mode_counter == 3:
                    self.get_logger().info("Simple: Switching to joint mode")
                    self.switch_controller("joint_trajectory_controller", "forward_position_controller")
                    self.current_mode = "ur_joint_mode"
                    self.switch_servo_to_joint_mode()

        self.previous_button_states = msg.buttons[:]

    def switch_controller(self, stop_controller: str, start_controller: str):
        if not self.controller_switch_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("controller_manager/switch_controller service not available.")
            return

        request = SwitchController.Request()
        request.deactivate_controllers = [stop_controller]
        request.activate_controllers = [start_controller]
        request.strictness = SwitchController.Request.BEST_EFFORT

        future = self.controller_switch_client.call_async(request)

        def callback(fut):
            if fut.result() is not None and fut.result().ok:
                self.get_logger().info(f"Switched to '{start_controller}' successfully.")
            else:
                self.get_logger().warn(f"Failed to switch to '{start_controller}'.")

        future.add_done_callback(callback)

def main(args=None):
    rclpy.init(args=args)
    node = SpacenavToCmdVel()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()