// SPDX-FileCopyrightText: 2024 German Aerospace Center <adrian.ricardezortigosa@dlr.de>
//
// SPDX-License-Identifier: MIT

// Node that connects the spacemouse with the robot respective controller (e.g. cartesian / joints)

#include <control_msgs/msg/joint_jog.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <moveit_msgs/srv/servo_command_type.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/joy.hpp>

namespace futama2_teleop
{
  // Enums for button names -> axis/button array index
  enum Axis
  {
    X = 0,
    Y = 1,
    Z = 2,
    RX = 3,
    RY = 4,
    RZ = 5,
  };

  class JoystickTeleop : public rclcpp::Node
  {
  public:
    explicit JoystickTeleop(const rclcpp::NodeOptions &options)
        : Node("joystick_teleop", options), cartesian_control_(true)
    {
      frame_to_publish_ = declare_parameter<std::string>("frame_to_publish", "base_link");

      // Setup pub/sub
      // todo set QoS for pub and sub in a sensitive way
      joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
          "/joy", 10, [this](const sensor_msgs::msg::Joy::ConstSharedPtr &msg)
          { return joyCB(msg); });

      twist_pub_ =
          create_publisher<geometry_msgs::msg::TwistStamped>("/servo_node/delta_twist_cmds", 10);
      joint_pub_ = create_publisher<control_msgs::msg::JointJog>("/servo_node/delta_joint_cmds", 10);

      // Create a service client to start the ServoNode
      servo_type_client_ =
          create_client<moveit_msgs::srv::ServoCommandType>("/servo_node/switch_command_type");
      if (!servo_type_client_->wait_for_service(std::chrono::seconds(5)))
      {
        RCLCPP_WARN(
            get_logger(), "Could not connect to servo_node. Will try again with indefinite timeout.");
        servo_type_client_->wait_for_service();
      }
      RCLCPP_INFO(get_logger(), "Connected to servo_node.");
      moveit_msgs::srv::ServoCommandType::Request::SharedPtr req =
          std::make_shared<moveit_msgs::srv::ServoCommandType::Request>();
      req->command_type = 1;
      servo_type_client_->async_send_request(req);
      RCLCPP_INFO(get_logger(), "Switched to Cartesian servo mode.");
    }

    void joyCB(const sensor_msgs::msg::Joy::ConstSharedPtr &msg)
    {
      // Convert the joystick message to Twist or JointJog and publish
      if (cartesian_control_)
      {
        // Convert the joystick message to Twist and publish
        geometry_msgs::msg::TwistStamped twist_msg;
        twist_msg.header.frame_id = frame_to_publish_;
        twist_msg.header.stamp = now();
        // todo add some kind of scaling factor here?
        twist_msg.twist.linear.x = msg->axes[X];
        twist_msg.twist.linear.y = msg->axes[Y];
        twist_msg.twist.linear.z = msg->axes[Z];
        twist_msg.twist.angular.x = msg->axes[RX];
        twist_msg.twist.angular.y = msg->axes[RY];
        twist_msg.twist.angular.z = msg->axes[RZ];
        twist_pub_->publish(twist_msg);
      }
      else // todo this is never executed. add input to switch modes
      {
        // Convert the joystick message to JointJog and publish
        control_msgs::msg::JointJog joint_msg;
        joint_msg.header.stamp = now();
        // todo make joint switchable somehow
        joint_msg.joint_names.push_back("shoulder_pan_joint");
        // todo add some kind of scaling factor here?
        joint_msg.velocities.push_back(msg->axes[X]);
        joint_pub_->publish(std::move(joint_msg));
      }
    }

  private:
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
    rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_pub_;
    rclcpp::Client<moveit_msgs::srv::ServoCommandType>::SharedPtr servo_type_client_;

    std::string frame_to_publish_;
    bool cartesian_control_;
  };

} // namespace futama2_teleop
// Register the component with class_loader
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(futama2_teleop::JoystickTeleop)
