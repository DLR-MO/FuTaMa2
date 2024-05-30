#!/usr/bin/env python3

# SPDX-FileCopyrightText: 2024 German Aerospace Center <adrian.ricardezortigosa@dlr.de>
#
# SPDX-License-Identifier: MIT

"""
Node that allows foto capturing while performing robotic inspection.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from cv_bridge import CvBridge
import cv2
from sensor_msgs.msg import Image
from rclpy.logging import get_logger

bridge = CvBridge()
joy_msg = Joy()
joy_msg.buttons = [0, 0]
previous_msg_button_right = 0
previous_msg_button_left = 0

class FotoCapture(Node):
    def __init__(self):

        global start_time, demo_steps
        super().__init__('foto_capture')

        self.declare_parameter('insp_mode','manual')
        self.auto_insp = self.get_parameter('insp_mode').get_parameter_value().string_value
        self.foto_capture_buttons_topic = ""

        if self.auto_insp == 'manual':
            self.foto_capture_buttons_topic = "/joy"         # /joy is free for the spacemouse
        elif self.auto_insp == 'automatic':
            self.foto_capture_buttons_topic = "/spacenav/joy" # /joy is busy by the auto twist, /spacenav/joy instead
        # Subscriber: Joy
        self.joy_sub = self.create_subscription(
            Joy, self.foto_capture_buttons_topic, self.buttons_callback, 10)
        self.joy_sub  # prevent unused variable warning

        # Subscriber: Image Rect Raw
        self.img_rect_raw_sub = self.create_subscription(
            Image, '/camera/camera/color/image_rect_raw', self.save_image_callback, 10)
        self.img_rect_raw_sub  # prevent unused variable warning

        self.logger = get_logger("foto_capture_node")

        rclpy.spin(self)

    def save_image_callback(self, msg):
        global previous_msg_button_right, previous_msg_button_left
        if previous_msg_button_left == 0 and previous_msg_button_right == 0 and joy_msg.buttons[0] == 1 and joy_msg.buttons[1] == 1:
            self.cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
            # Save your OpenCV2 image as a jpeg 
            time = msg.header.stamp
            cv2.imwrite('FuTaMa2'+str(time)+'.jpeg', self.cv2_img) # save the image
            cv2.waitKey(1)
            self.logger.info("Picture saved in specified path!")
            # set the previous state to avoid saving multiple images
            previous_msg_button_right = 1
            previous_msg_button_left = 1

    def buttons_callback(self, msg):
        global joy_msg, previous_msg_button_right, previous_msg_button_left

        # Button states management ("pull-up" & "pull-down")
        if previous_msg_button_right == 0 and msg.buttons[0] == 1:
            joy_msg.buttons[0] = 1
        elif previous_msg_button_right == 1 and msg.buttons[0] == 0:
            joy_msg.buttons[0] = 0
            previous_msg_button_right = 0
        
        if previous_msg_button_left == 0 and msg.buttons[1] == 1:
            joy_msg.buttons[1] = 1
        elif previous_msg_button_left == 1 and msg.buttons[1] == 0:
            joy_msg.buttons[1] = 0
            previous_msg_button_left = 0

def main(args=None):
    rclpy.init(args=args)
    node = FotoCapture()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()