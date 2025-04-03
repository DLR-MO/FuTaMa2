// SPDX-FileCopyrightText: 2024 German Aerospace Center <adrian.ricardezortigosa@dlr.de>
// SPDX-License-Identifier: MIT

#include <control_msgs/msg/joint_jog.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <moveit_msgs/srv/servo_command_type.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

namespace futama2_teleop
{
  enum Axis
  {
    X = 0,
    Y = 1,
    Z = 2,
    RX = 4,
    RY = 3,
    RZ = 5,
  };

  class JoystickTeleopPerceptCollAvoidance : public rclcpp::Node
  {
  public:
    explicit JoystickTeleopPerceptCollAvoidance(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
        : Node("joystick_teleop_percept_coll_avoidance", options), cartesian_control_(true)
    {
      frame_to_publish_ = declare_parameter<std::string>("frame_to_publish", "base_link");

      // Parameters
      SLOWDOWN_START_DISTANCE = 0.30;  // Start reducing speed
      STOP_DISTANCE = 0.09;            // Stop completely if below this
      MIN_DISTANCE_SCALING = 0.0001;
      MAX_SCALING = 0.5;
      COLLISION_VECTOR_TOPIC = "/collision_vector";

      joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
          "/joy_ur", 10, [this](const sensor_msgs::msg::Joy::ConstSharedPtr &msg)
          { return joyCB(msg); });

      collision_vector_sub_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
          COLLISION_VECTOR_TOPIC, 10,
          [this](const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
          {
            latest_collision_vector_ = *msg;
            received_collision_vector_ = true;
          });

      twist_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("/servo_node/delta_twist_cmds", 10);
      joint_pub_ = create_publisher<control_msgs::msg::JointJog>("/servo_node/delta_joint_cmds", 10);

      servo_type_client_ = create_client<moveit_msgs::srv::ServoCommandType>("/servo_node/switch_command_type");
      if (!servo_type_client_->wait_for_service(std::chrono::seconds(5)))
      {
        RCLCPP_WARN(get_logger(), "Could not connect to servo_node. Will try again with indefinite timeout.");
        servo_type_client_->wait_for_service();
      }
      RCLCPP_INFO(get_logger(), "Connected to servo_node.");
      auto req = std::make_shared<moveit_msgs::srv::ServoCommandType::Request>();
      req->command_type = 1;
      servo_type_client_->async_send_request(req);
      RCLCPP_INFO(get_logger(), "Switched to Cartesian servo mode.");
    }

  private:
    void joyCB(const sensor_msgs::msg::Joy::ConstSharedPtr &msg)
    {
      if (cartesian_control_)
      {
        geometry_msgs::msg::TwistStamped twist_msg;
        twist_msg.header.frame_id = frame_to_publish_;
        twist_msg.header.stamp = now();

        twist_msg.twist.linear.x = msg->axes[Y];
        twist_msg.twist.linear.y = -msg->axes[X];
        twist_msg.twist.linear.z = msg->axes[Z];
        twist_msg.twist.angular.x = msg->axes[RX];
        twist_msg.twist.angular.y = -msg->axes[RY];
        twist_msg.twist.angular.z = msg->axes[RZ];

        twist_msg.twist = scale_twist_by_collision_vector(twist_msg.twist);

        twist_pub_->publish(twist_msg);
      }
      else
      {
        control_msgs::msg::JointJog joint_msg;
        joint_msg.header.stamp = now();
        joint_msg.joint_names.push_back("shoulder_pan_joint");
        joint_msg.velocities.push_back(msg->axes[X]);
        joint_pub_->publish(std::move(joint_msg));
      }
    }

    geometry_msgs::msg::Twist scale_twist_by_collision_vector(const geometry_msgs::msg::Twist &original_twist)
    {
      geometry_msgs::msg::Twist adjusted = original_twist;

      if (!received_collision_vector_)
      {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "No minimal distance vector received yet.");
        return adjusted;
      }

      const auto &cv = latest_collision_vector_.vector;
      double distance = std::sqrt(cv.x * cv.x + cv.y * cv.y + cv.z * cv.z);

      double dot = original_twist.linear.x * cv.x +
                   original_twist.linear.y * cv.y +
                   original_twist.linear.z * cv.z;

      double twist_mag = std::sqrt(
          original_twist.linear.x * original_twist.linear.x +
          original_twist.linear.y * original_twist.linear.y +
          original_twist.linear.z * original_twist.linear.z);

      if (twist_mag < 1e-5 || distance < 1e-5)
        return adjusted;

      double cos_theta = dot / (twist_mag * distance);

      // ❗ Hard Stop: if too close AND aligned
      if (distance < STOP_DISTANCE && cos_theta > 0.3)
      {
        adjusted.linear.x = 0.0;
        adjusted.linear.y = 0.0;
        adjusted.linear.z = 0.0;

        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 100,
                             "\n!!! HARD STOP ACTIVATED !!!\n"
                             "Distance: %.3f m < STOP_DISTANCE %.3f\n"
                             "Cos(theta): %.3f → motion into obstacle\n"
                             "Velocity set to ZERO for safety.",
                             distance, STOP_DISTANCE, cos_theta);
        return adjusted;
      }

      // 🟠 Slow down if within threshold AND heading toward obstacle
      if (distance < SLOWDOWN_START_DISTANCE && cos_theta > 0.3)
      {
        double proximity = std::clamp((SLOWDOWN_START_DISTANCE - distance) / (SLOWDOWN_START_DISTANCE - STOP_DISTANCE), 0.0, 1.0);
        double scale = std::clamp(MAX_SCALING * (1.0 - cos_theta * proximity), MIN_DISTANCE_SCALING, MAX_SCALING);

        adjusted.linear.x *= scale;
        adjusted.linear.y *= scale;
        adjusted.linear.z *= scale;

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 100,
                             "\n--- COLLISION AVOIDANCE ACTIVE ---\n"
                             "Distance: %.3f m | Cos(theta): %.3f\n"
                             "Proximity (scaled): %.3f\n"
                             "Scaling factor: %.3f\n"
                             "Collision vector: [%.3f, %.3f, %.3f]\n"
                             "Original twist:  [%.3f, %.3f, %.3f]\n"
                             "Adjusted twist:  [%.3f, %.3f, %.3f]",
                             distance, cos_theta, proximity, scale,
                             cv.x, cv.y, cv.z,
                             original_twist.linear.x, original_twist.linear.y, original_twist.linear.z,
                             adjusted.linear.x, adjusted.linear.y, adjusted.linear.z);
      }

      return adjusted;
    }

    // ROS Members
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr collision_vector_sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
    rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_pub_;
    rclcpp::Client<moveit_msgs::srv::ServoCommandType>::SharedPtr servo_type_client_;

    std::string frame_to_publish_;
    bool cartesian_control_;

    // Collision logic
    geometry_msgs::msg::Vector3Stamped latest_collision_vector_;
    bool received_collision_vector_ = false;

    // Thresholds
    double SLOWDOWN_START_DISTANCE;
    double STOP_DISTANCE;
    double MIN_DISTANCE_SCALING;
    double MAX_SCALING;
    std::string COLLISION_VECTOR_TOPIC;
  };

} // namespace futama2_teleop

// Optionally register as component
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(futama2_teleop::JoystickTeleopPerceptCollAvoidance)

// ---------- MAIN ----------
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<futama2_teleop::JoystickTeleopPerceptCollAvoidance>());
  rclcpp::shutdown();
  return 0;
}
