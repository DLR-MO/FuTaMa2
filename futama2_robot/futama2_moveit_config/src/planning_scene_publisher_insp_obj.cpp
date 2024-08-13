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
  class PlanningScenePublisherInspObj : public rclcpp::Node
  {
  public:
    PlanningScenePublisherInspObj(const rclcpp::NodeOptions &options)
        : Node("planning_scene_publisher_insp_obj", options)
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
      // Create collision object wing, in the way of servoing
      moveit_msgs::msg::CollisionObject collision_object;
      Eigen::Vector3d scale_wing(1.0, 1.0, 1.0);
      shapes::Mesh * m_wing = shapes::createMeshFromResource(
        "package://futama2_description/meshes/collision/GOM_Wing_1_hyperreduced.stl",scale_wing);

      collision_object.header.frame_id = "world";
      collision_object.id = "box";

      // Wing mesh (.stl, columnts, rows)
      shape_msgs::msg::Mesh wing_mesh;
      shapes::ShapeMsg wing_mesh_msg;
      shapes::constructMsgFromShape(m_wing, wing_mesh_msg);
      wing_mesh = boost::get<shape_msgs::msg::Mesh>(wing_mesh_msg);
      collision_object.meshes.resize(2);
      collision_object.meshes[0] = wing_mesh;
      collision_object.mesh_poses.resize(2);
      collision_object.mesh_poses[0].position.x = -0.30 - 0.15;
      collision_object.mesh_poses[0].position.y = -1.6 - 0.14;
      collision_object.mesh_poses[0].position.z = -0.35;
      collision_object.mesh_poses[0].orientation.w = 0.0;
      collision_object.mesh_poses[0].orientation.x = 0.0;
      collision_object.mesh_poses[0].orientation.y = 0.48;
      collision_object.mesh_poses[0].orientation.z = 0.88;

      // Push meshes
      collision_object.meshes.push_back(wing_mesh);
      collision_object.mesh_poses.push_back(collision_object.mesh_poses[0]);
    
      collision_object.operation = collision_object.ADD;

      // Create inspection object, in the way of servoing
      Eigen::Vector3d scale_inspection_object(1 , 1, 1);
      shapes::Mesh * m_inspection_object = shapes::createMeshFromResource(
        "package://futama2_description/meshes/collision/dlr_robo_test.stl",scale_inspection_object);

      // Obstacle mesh (.stl, columnts, rows)
      shape_msgs::msg::Mesh inspection_object_mesh;
      shapes::ShapeMsg inspection_object_mesh_msg;
      shapes::constructMsgFromShape(m_inspection_object, inspection_object_mesh_msg);
      inspection_object_mesh = boost::get<shape_msgs::msg::Mesh>(inspection_object_mesh_msg);
      collision_object.meshes[1] = inspection_object_mesh;

      collision_object.mesh_poses[1].position.x = - 0.698 + 0.15;
      collision_object.mesh_poses[1].position.y = 1.01 - 0.14;
      collision_object.mesh_poses[1].position.z = 0.098;

      collision_object.mesh_poses[1].orientation.w = 0.0;
      collision_object.mesh_poses[1].orientation.x = 0.0;
      collision_object.mesh_poses[1].orientation.y = 0.0;
      collision_object.mesh_poses[1].orientation.z = 1.0;

      // Push meshes
      collision_object.meshes.push_back(inspection_object_mesh);
      collision_object.mesh_poses.push_back(collision_object.mesh_poses[1]);
      collision_object.operation = collision_object.ADD;

      // Aluminum base BOX for robot
      shape_msgs::msg::SolidPrimitive aluminum_base_ur;
      aluminum_base_ur.type = aluminum_base_ur.BOX;
      aluminum_base_ur.dimensions = {1.22, 1.22, 0.916};

      geometry_msgs::msg::Pose aluminum_base_ur_pose;
      aluminum_base_ur_pose.position.x = -0.46;
      aluminum_base_ur_pose.position.y = 0.47;
      aluminum_base_ur_pose.position.z = -0.45;

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
    
      // Workstation table
      shape_msgs::msg::SolidPrimitive workstation_table;
      workstation_table.type = workstation_table.BOX;
      workstation_table.dimensions = {1.60, 0.80, 0.916};

      geometry_msgs::msg::Pose workstation_table_pose;
      workstation_table_pose.position.x = 0.95;
      workstation_table_pose.position.y = 0.26;
      workstation_table_pose.position.z = -0.45;

      collision_object.primitives.push_back(workstation_table);
      collision_object.primitive_poses.push_back(workstation_table_pose);
      
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

    ~PlanningScenePublisherInspObj() override
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
RCLCPP_COMPONENTS_REGISTER_NODE(futama2_moveit_config::PlanningScenePublisherInspObj)
