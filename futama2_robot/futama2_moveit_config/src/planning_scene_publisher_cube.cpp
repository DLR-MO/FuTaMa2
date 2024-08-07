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
  class PlanningScenePublisherCube : public rclcpp::Node
  {
  public:
    PlanningScenePublisherCube(const rclcpp::NodeOptions &options)
        : Node("planning_scene_publisher", options)
    {
      this->declare_parameter("mode", rclcpp::PARAMETER_STRING);

      // transient local publisher so that planning scene does not get lost
      collision_pub_ = create_publisher<moveit_msgs::msg::PlanningScene>(
          "/planning_scene", rclcpp::QoS(10).transient_local());
      
      // Declaration of mode parameter
      rclcpp::Parameter mode_param = this->get_parameter("mode");
      std::string mode = mode_param.as_string();

      // Load the collision scene asynchronously
      collision_pub_thread_ = std::thread([this]()
                                          {
      // Create collision object, in the way of servoing
      moveit_msgs::msg::CollisionObject collision_object;
      Eigen::Vector3d scale(0.001, 0.001, 0.001);
      shapes::Mesh * m = shapes::createMeshFromResource(
        "package://futama2_description/meshes/collision/dlr_robo_test.stl",scale);

      collision_object.header.frame_id = "world";
      collision_object.id = "box";

      // Aluminum base BOX for robot
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

      // Base BOX for object
      shape_msgs::msg::SolidPrimitive base_object;
      base_object.type = base_object.BOX;
      base_object.dimensions = {0.5, 0.5, 1};

      geometry_msgs::msg::Pose base_object_pose;
      base_object_pose.position.x = -0.4;
      base_object_pose.position.y = -0.4;
      base_object_pose.position.z = -0.5;

      collision_object.primitives.push_back(base_object);
      collision_object.primitive_poses.push_back(base_object_pose);

      // Cube mesh (.stl, columnts, rows)
      shape_msgs::msg::Mesh cube_mesh;
      shapes::ShapeMsg cube_mesh_msg;
      shapes::constructMsgFromShape(m, cube_mesh_msg);
      cube_mesh = boost::get<shape_msgs::msg::Mesh>(cube_mesh_msg);
      collision_object.meshes.resize(1);
      collision_object.meshes[0] = cube_mesh;
      collision_object.mesh_poses.resize(1);

      collision_object.mesh_poses[0].position.x = -0.4;
      collision_object.mesh_poses[0].position.y = -0.4;
      collision_object.mesh_poses[0].position.z = 0;

      collision_object.mesh_poses[0].orientation.w = 1.0;
      collision_object.mesh_poses[0].orientation.x = 0.0;
      collision_object.mesh_poses[0].orientation.y = 0.0;
      collision_object.mesh_poses[0].orientation.z = 0.0;

      // Push meshes
      collision_object.meshes.push_back(cube_mesh);
      collision_object.mesh_poses.push_back(collision_object.mesh_poses[0]);
      collision_object.operation = collision_object.ADD;
      
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

    ~PlanningScenePublisherCube() override
    {
      if (collision_pub_thread_.joinable())
        collision_pub_thread_.join();
    }

  private:
    rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr collision_pub_;

    std::thread collision_pub_thread_;
  }; // class PlanningScenePublisher

} // namespace futama2_moveit_config

// Register the component with class_loader
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(futama2_moveit_config::PlanningScenePublisherCube)
