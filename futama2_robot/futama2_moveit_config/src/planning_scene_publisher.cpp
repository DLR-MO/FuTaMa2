// SPDX-FileCopyrightText: 2024 German Aerospace Center <adrian.ricardezortigosa@dlr.de>
//
// SPDX-License-Identifier: MIT

#include <rclcpp/version.h>

#include <moveit_msgs/msg/planning_scene.hpp>
#include <rclcpp/client.hpp>
#include <rclcpp/experimental/buffers/intra_process_buffer.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/qos.hpp>
#include <std_srvs/srv/trigger.hpp>
#if RCLCPP_VERSION_GTE(20, 0, 0)
#include <rclcpp/event_handler.hpp>
#else
#include <rclcpp/qos_event.hpp>
#endif
#include <rclcpp/subscription.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/utilities.hpp>
#include <thread>

// For the wing as a collision object
#include "geometric_shapes/mesh_operations.h"
#include "geometric_shapes/shape_operations.h"
#include "geometric_shapes/shapes.h"

namespace futama2_moveit_config
{
  class PlanningScenePublisher : public rclcpp::Node
  {
  public:
    PlanningScenePublisher(const rclcpp::NodeOptions &options)
        : Node("planning_scene_publisher", options)
    {
      this->declare_parameter("mode", rclcpp::PARAMETER_STRING);

      // transient local publisher so that planning scene does not get lost
      collision_pub_ = create_publisher<moveit_msgs::msg::PlanningScene>(
          "/planning_scene", rclcpp::QoS(10).transient_local());
      
      // Declaration of mode parameter
      rclcpp::Parameter mode_param = this->get_parameter("mode");
      std::string mode = mode_param.as_string();
      // If the mock hardware is used, then simulate wing in rviz, otherwise, remove it from the planning scene
      if(mode=="mock")
        include_wing_ = declare_parameter<bool>("include_wing", true);
      else
        include_wing_ = declare_parameter<bool>("include_wing", false);

      // Load the collision scene asynchronously
      collision_pub_thread_ = std::thread([this]()
                                          {
      // Create collision object, in the way of servoing
      moveit_msgs::msg::CollisionObject collision_object;
      Eigen::Vector3d scale(1.0, 1.0, 1.0);
      shapes::Mesh * m = shapes::createMeshFromResource(
        "package://futama2_description/meshes/collision/GOM_Wing_1_hyperreduced.stl",scale);

      collision_object.header.frame_id = "world";
      collision_object.id = "box";

      // Aluminum base BOX robot object
      shape_msgs::msg::SolidPrimitive aluminum_base_ur;
      aluminum_base_ur.type = aluminum_base_ur.BOX;
      aluminum_base_ur.dimensions = {1.22, 1.22, 0.916};

      geometry_msgs::msg::Pose aluminum_base_ur_pose;
      aluminum_base_ur_pose.position.x = 0.47;
      aluminum_base_ur_pose.position.y = 0.46;
      aluminum_base_ur_pose.position.z = -0.46;

      collision_object.primitives.push_back(aluminum_base_ur);
      collision_object.primitive_poses.push_back(aluminum_base_ur_pose);

      // Floor mesh
      shape_msgs::msg::SolidPrimitive floor;
      floor.type = floor.BOX;

      floor.dimensions = {4.0, 4.0, 0.01};

      geometry_msgs::msg::Pose floor_pose;
      floor_pose.position.x = 0.0;
      floor_pose.position.y = 0.0;
      floor_pose.position.z = aluminum_base_ur_pose.position.z*2-floor.dimensions[2];

      collision_object.primitives.push_back(floor);
      collision_object.primitive_poses.push_back(floor_pose);

      // Wing meshes (.stl, columnts, rows)
      if (include_wing_) {
        shape_msgs::msg::Mesh wing_mesh_1;
        shape_msgs::msg::Mesh wing_mesh_2;
        shapes::ShapeMsg wing_mesh_msg;
        shapes::constructMsgFromShape(m, wing_mesh_msg);
        wing_mesh_1 = boost::get<shape_msgs::msg::Mesh>(wing_mesh_msg);
        wing_mesh_2 = boost::get<shape_msgs::msg::Mesh>(wing_mesh_msg);
        collision_object.meshes.resize(2);
        collision_object.meshes[0] = wing_mesh_1;
        collision_object.meshes[1] = wing_mesh_2;
        collision_object.mesh_poses.resize(2);

        collision_object.mesh_poses[0].position.x = -1.7;
        collision_object.mesh_poses[0].position.y = 0.0;
        collision_object.mesh_poses[0].position.z = -0.35;

        collision_object.mesh_poses[0].orientation.w = 0.62;
        collision_object.mesh_poses[0].orientation.x = 0.34;
        collision_object.mesh_poses[0].orientation.y = 0.34;
        collision_object.mesh_poses[0].orientation.z = 0.62;

        collision_object.mesh_poses[1].position.x = collision_object.mesh_poses[0].position.x - 0.30;
        collision_object.mesh_poses[1].position.y = collision_object.mesh_poses[0].position.y;
        collision_object.mesh_poses[1].position.z = collision_object.mesh_poses[0].position.z;

        collision_object.mesh_poses[1].orientation.w = collision_object.mesh_poses[0].orientation.w;
        collision_object.mesh_poses[1].orientation.x = collision_object.mesh_poses[0].orientation.x;
        collision_object.mesh_poses[1].orientation.y = collision_object.mesh_poses[0].orientation.y;
        collision_object.mesh_poses[1].orientation.z = collision_object.mesh_poses[0].orientation.z;

        // Wing column 1 object
        shape_msgs::msg::SolidPrimitive wing_column_1;
        wing_column_1.type = wing_column_1.BOX;
        wing_column_1.dimensions = {0.3, 0.1, 1.8};

        geometry_msgs::msg::Pose wing_column_1_pose;
        wing_column_1_pose.position.x = -1.9;
        wing_column_1_pose.position.y = -0.47;
        wing_column_1_pose.position.z = -collision_object.mesh_poses[0].position.z - 0.3;

        // Wing row middle up front object
        shape_msgs::msg::SolidPrimitive wing_row_middle_up_front;
        wing_row_middle_up_front.type = wing_row_middle_up_front.BOX;
        wing_row_middle_up_front.dimensions = {0.05, 2.6, 0.03};

        geometry_msgs::msg::Pose wing_row_middle_up_front_pose;
        wing_row_middle_up_front_pose.position.x = -1.79;
        wing_row_middle_up_front_pose.position.y = -0.7;
        wing_row_middle_up_front_pose.position.z = wing_column_1_pose.position.z + 0.25;

        // Wing row middle down front object
        shape_msgs::msg::SolidPrimitive wing_row_middle_down_front;
        wing_row_middle_down_front.type = wing_row_middle_down_front.BOX;
        wing_row_middle_down_front.dimensions = {0.05, 2.6, 0.03};

        geometry_msgs::msg::Pose wing_row_middle_down_front_pose;
        wing_row_middle_down_front_pose.position.x =
          wing_row_middle_up_front_pose.position.x + 0.03;
        wing_row_middle_down_front_pose.position.y = wing_row_middle_up_front_pose.position.y;
        wing_row_middle_down_front_pose.position.z = wing_row_middle_up_front_pose.position.z - 0.4;

        // Wing row top front object
        shape_msgs::msg::SolidPrimitive wing_row_top_front;
        wing_row_top_front.type = wing_row_top_front.BOX;
        wing_row_top_front.dimensions = {0.05, 2.6, 0.03};

        geometry_msgs::msg::Pose wing_row_top_front_pose;
        wing_row_top_front_pose.position.x = wing_row_middle_up_front_pose.position.x - 0.035;
        wing_row_top_front_pose.position.y = wing_row_middle_up_front_pose.position.y;
        wing_row_top_front_pose.position.z = wing_row_middle_up_front_pose.position.z + 0.23;

        // Wing row bottom front object
        shape_msgs::msg::SolidPrimitive wing_row_bottom_front;
        wing_row_bottom_front.type = wing_row_bottom_front.BOX;
        wing_row_bottom_front.dimensions = {0.07, 2.6, 0.03};

        geometry_msgs::msg::Pose wing_row_bottom_front_pose;
        wing_row_bottom_front_pose.position.x = wing_row_middle_down_front_pose.position.x + 0.025;
        wing_row_bottom_front_pose.position.y = wing_row_middle_down_front_pose.position.y;
        wing_row_bottom_front_pose.position.z = wing_row_middle_down_front_pose.position.z - 0.19;

        // Wing row middle back object
        shape_msgs::msg::SolidPrimitive wing_row_middle_back;
        wing_row_middle_back.type = wing_row_middle_back.BOX;
        wing_row_middle_back.dimensions = {0.05, 2.8, 0.09};

        geometry_msgs::msg::Pose wing_row_middle_back_pose;
        wing_row_middle_back_pose.position.x = wing_row_middle_up_front_pose.position.x - 0.2;
        wing_row_middle_back_pose.position.y = wing_row_middle_up_front_pose.position.y;
        wing_row_middle_back_pose.position.z = wing_row_middle_up_front_pose.position.z - 0.12;

        // Wing row middle up back object
        shape_msgs::msg::SolidPrimitive wing_row_middle_up_back;
        wing_row_middle_up_back.type = wing_row_middle_up_back.BOX;
        wing_row_middle_up_back.dimensions = {0.05, 2.6, 0.09};

        geometry_msgs::msg::Pose wing_row_middle_up_back_pose;
        wing_row_middle_up_back_pose.position.x = wing_row_middle_back_pose.position.x - 0.04;
        wing_row_middle_up_back_pose.position.y = wing_row_middle_back_pose.position.y;
        wing_row_middle_up_back_pose.position.z = wing_row_middle_back_pose.position.z + 0.24;

        // Wing row middle up2 back object
        shape_msgs::msg::SolidPrimitive wing_row_middle_up2_back;
        wing_row_middle_up2_back.type = wing_row_middle_up2_back.BOX;
        wing_row_middle_up2_back.dimensions = {0.05, 2.6, 0.03};

        geometry_msgs::msg::Pose wing_row_middle_up2_back_pose;
        wing_row_middle_up2_back_pose.position.x = wing_row_middle_up_back_pose.position.x - 0.035;
        wing_row_middle_up2_back_pose.position.y = wing_row_middle_up_back_pose.position.y;
        wing_row_middle_up2_back_pose.position.z = wing_row_middle_up_back_pose.position.z + 0.21;

        // Wing row middle down back object
        shape_msgs::msg::SolidPrimitive wing_row_middle_down_back;
        wing_row_middle_down_back.type = wing_row_middle_down_back.BOX;
        wing_row_middle_down_back.dimensions = {0.05, 2.6, 0.07};

        geometry_msgs::msg::Pose wing_row_middle_down_back_pose;
        wing_row_middle_down_back_pose.position.x = wing_row_middle_back_pose.position.x + 0.01;
        wing_row_middle_down_back_pose.position.y = wing_row_middle_back_pose.position.y;
        wing_row_middle_down_back_pose.position.z = wing_row_middle_back_pose.position.z - 0.25;

        // Wing row middle down2 back object
        shape_msgs::msg::SolidPrimitive wing_row_middle_down2_back;
        wing_row_middle_down2_back.type = wing_row_middle_down2_back.BOX;
        wing_row_middle_down2_back.dimensions = {0.05, 2.6, 0.08};

        geometry_msgs::msg::Pose wing_row_middle_down2_back_pose;
        wing_row_middle_down2_back_pose.position.x =
          wing_row_middle_down_back_pose.position.x + 0.007;
        wing_row_middle_down2_back_pose.position.y = wing_row_middle_down_back_pose.position.y;
        wing_row_middle_down2_back_pose.position.z =
          wing_row_middle_down_back_pose.position.z - 0.35;

        // Push meshes
        collision_object.meshes.push_back(wing_mesh_1);
        collision_object.meshes.push_back(wing_mesh_2);
        collision_object.mesh_poses.push_back(collision_object.mesh_poses[0]);
        collision_object.mesh_poses.push_back(collision_object.mesh_poses[1]);
        collision_object.primitives.push_back(wing_column_1);
        collision_object.primitive_poses.push_back(wing_column_1_pose);
        collision_object.primitives.push_back(wing_row_middle_up_front);
        collision_object.primitive_poses.push_back(wing_row_middle_up_front_pose);
        collision_object.primitives.push_back(wing_row_middle_down_front);
        collision_object.primitive_poses.push_back(wing_row_middle_down_front_pose);
        collision_object.primitives.push_back(wing_row_top_front);
        collision_object.primitive_poses.push_back(wing_row_top_front_pose);
        collision_object.primitives.push_back(wing_row_bottom_front);
        collision_object.primitive_poses.push_back(wing_row_bottom_front_pose);
        collision_object.primitives.push_back(wing_row_middle_back);
        collision_object.primitive_poses.push_back(wing_row_middle_back_pose);
        collision_object.primitives.push_back(wing_row_middle_up_back);
        collision_object.primitive_poses.push_back(wing_row_middle_up_back_pose);
        collision_object.primitives.push_back(wing_row_middle_up2_back);
        collision_object.primitive_poses.push_back(wing_row_middle_up2_back_pose);
        collision_object.primitives.push_back(wing_row_middle_down_back);
        collision_object.primitive_poses.push_back(wing_row_middle_down_back_pose);
        collision_object.primitives.push_back(wing_row_middle_down2_back);
        collision_object.primitive_poses.push_back(wing_row_middle_down2_back_pose);

        collision_object.operation = collision_object.ADD;
      }
      moveit_msgs::msg::PlanningSceneWorld psw;
      psw.collision_objects.push_back(collision_object);

      auto ps = std::make_unique<moveit_msgs::msg::PlanningScene>();
      ps->world = psw;
      ps->is_diff = true;
      int subscriber = collision_pub_->get_subscription_count();
      collision_pub_->publish(std::move(ps));
      //todo this while part is only necessary because the moveit move_group does not subscribe with transient local durability on planning scene
      while (rclcpp::ok()) {
        if (subscriber != collision_pub_->get_subscription_count()) {
          // got new subscriber, republish
          subscriber = collision_pub_->get_subscription_count();
          collision_pub_->publish(std::move(ps));
        }
        sleep(1);
      } });
    }

    ~PlanningScenePublisher() override
    {
      if (collision_pub_thread_.joinable())
        collision_pub_thread_.join();
    }

  private:
    rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr collision_pub_;

    std::thread collision_pub_thread_;
    bool include_wing_;
  }; // class PlanningScenePublisher

} // namespace futama2_moveit_config

// Register the component with class_loader
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(futama2_moveit_config::PlanningScenePublisher)
