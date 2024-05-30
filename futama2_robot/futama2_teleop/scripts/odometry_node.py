#!/usr/bin/env python3

# SPDX-FileCopyrightText: 2024 German Aerospace Center <adrian.ricardezortigosa@dlr.de>
#
# SPDX-License-Identifier: MIT

"""
Node that creates the odometry message (red arrows in Rviz) as RTABMap input and other features.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tf2_ros import TransformBroadcaster, TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from nav_msgs.msg import Odometry

global twist_msg

class OdometryPublisher(Node):
    def __init__(self):

        super().__init__('odom_node')

        # Publisher
        self.odom_pub = self.create_publisher(Odometry, 'rtabmap/odom', 30)
        self.odom_msg = Odometry()

        # Subscriber
        self.subscription = self.create_subscription(
            Twist, '/spacenav/twist', self.listener_callback, 10)
        self.subscription  # prevent unused variable warning

        # Declare and acquire `target_frame` parameter
        self.target_frame = self.declare_parameter(
            'target_frame', 'base_link').get_parameter_value().string_value

        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        self.odom_msg.header.frame_id = 'base_link'      # DON'T MODIFY
        self.odom_msg.child_frame_id = 'camera_link'      # DON'T MODIFY

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Call on_timer function every second
        self.timer = self.create_timer(0.01, self.on_timer)

    def listener_callback(self, msg):
        global twist_msg
        twist_msg = Twist()
        twist_msg = msg

        self.odom_msg.twist.twist.linear.x = twist_msg.linear.x
        self.odom_msg.twist.twist.linear.y = twist_msg.linear.y
        self.odom_msg.twist.twist.linear.z = twist_msg.linear.z
        self.odom_msg.twist.twist.angular.x = twist_msg.angular.x
        self.odom_msg.twist.twist.angular.y = twist_msg.angular.y
        self.odom_msg.twist.twist.angular.z = twist_msg.angular.z

    def on_timer(self):
        # Store frame names in variables that will be used to
        # compute transformations
        from_frame_rel = self.target_frame
        to_frame_rel = 'camera_link'

        try:
            t = self.tf_buffer.lookup_transform(
                from_frame_rel,
                to_frame_rel,
                rclpy.time.Time())

            current_time = self.get_clock().now().to_msg()

            # store the values of the transfom into the Odometry msg
            self.odom_msg.pose.pose.position.x = t.transform.translation.x
            self.odom_msg.pose.pose.position.y = t.transform.translation.y
            self.odom_msg.pose.pose.position.z = t.transform.translation.z
            self.odom_msg.pose.pose.orientation.x = t.transform.rotation.x
            self.odom_msg.pose.pose.orientation.y = t.transform.rotation.y
            self.odom_msg.pose.pose.orientation.z = t.transform.rotation.z
            self.odom_msg.pose.pose.orientation.w = t.transform.rotation.w
            self.odom_msg.header.stamp = current_time
            self.odom_pub.publish(self.odom_msg)

        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return

def main(args=None):
    rclpy.init(args=args)
    node = OdometryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
