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
      // Create collision objects, in the way of servoing
      moveit_msgs::msg::CollisionObject collision_object;
      collision_object.header.frame_id = "world";
      collision_object.id = "box";
      collision_object.meshes.resize(3);      // wing_front, wing back, and insp obj (so far)
      collision_object.mesh_poses.resize(3);  // wing_front, wing back, and insp obj (so far)

      // --------------- WING(S) --------------------------------------------------------
      Eigen::Vector3d scale_wing(1.0, 1.0, 1.0);
      shapes::Mesh * m_wing = shapes::createMeshFromResource(
        "package://futama2_description/meshes/collision/GOM_Wing_1_hyperreduced.stl",scale_wing); // one mesh for both
      // Initializing meshes from STL
      shape_msgs::msg::Mesh wing_front_mesh;
      shape_msgs::msg::Mesh wing_back_mesh;
      // Initializing mesh messages
      shapes::ShapeMsg wing_front_mesh_msg;
      shapes::ShapeMsg wing_back_mesh_msg;
      // Construct meshes from shapes
      shapes::constructMsgFromShape(m_wing, wing_front_mesh_msg);
      shapes::constructMsgFromShape(m_wing, wing_back_mesh_msg);
      wing_front_mesh = boost::get<shape_msgs::msg::Mesh>(wing_front_mesh_msg);
      wing_back_mesh = boost::get<shape_msgs::msg::Mesh>(wing_back_mesh_msg);
      // Assigning
      collision_object.meshes[0] = wing_front_mesh;
      collision_object.meshes[1] = wing_back_mesh;
      // Positions and orientations of meshes
      // Wing 1
      collision_object.mesh_poses[0].position.x = -0.45;
      collision_object.mesh_poses[0].position.y = -1.74;
      collision_object.mesh_poses[0].position.z = -0.35;
      collision_object.mesh_poses[0].orientation.w = 0.0;
      collision_object.mesh_poses[0].orientation.x = 0.0;
      collision_object.mesh_poses[0].orientation.y = 0.48;
      collision_object.mesh_poses[0].orientation.z = 0.88;
      // Wing 2
      collision_object.mesh_poses[1].position.x = collision_object.mesh_poses[0].position.x;
      collision_object.mesh_poses[1].position.y = collision_object.mesh_poses[0].position.y - 0.30;
      collision_object.mesh_poses[1].position.z = collision_object.mesh_poses[0].position.z;
      collision_object.mesh_poses[1].orientation.w = collision_object.mesh_poses[0].orientation.w;
      collision_object.mesh_poses[1].orientation.x = collision_object.mesh_poses[0].orientation.x;
      collision_object.mesh_poses[1].orientation.y = collision_object.mesh_poses[0].orientation.y;
      collision_object.mesh_poses[1].orientation.z = collision_object.mesh_poses[0].orientation.z;
      // Mesh push
      collision_object.meshes.push_back(wing_front_mesh);
      collision_object.meshes.push_back(wing_back_mesh);
      collision_object.mesh_poses.push_back(collision_object.mesh_poses[0]);
      collision_object.mesh_poses.push_back(collision_object.mesh_poses[1]);

      // Add primitives and additional objects here as per original code
      // Wing column 1 object (reference column for the other columns and rows)
      shape_msgs::msg::SolidPrimitive wing_column_1;
      wing_column_1.type = wing_column_1.BOX;
      wing_column_1.dimensions = {0.3, 0.1, 1.8};
      geometry_msgs::msg::Pose wing_column_1_pose;
      wing_column_1_pose.position.x = collision_object.mesh_poses[0].position.x-0.25;
      wing_column_1_pose.position.y = collision_object.mesh_poses[0].position.y-0.27;
      wing_column_1_pose.position.z = -collision_object.mesh_poses[0].position.z - 0.3;
      wing_column_1_pose.orientation.w = 0.707;
      wing_column_1_pose.orientation.x = 0.0;
      wing_column_1_pose.orientation.y = 0.0;
      wing_column_1_pose.orientation.z = 0.707;
      collision_object.primitives.push_back(wing_column_1);           // Push
      collision_object.primitive_poses.push_back(wing_column_1_pose); // Push
      // Wing column 2 object
      shape_msgs::msg::SolidPrimitive wing_column_2;
      wing_column_2.type = wing_column_2.BOX;
      wing_column_2.dimensions = {0.3, 0.1, 1.8};
      geometry_msgs::msg::Pose wing_column_2_pose;
      wing_column_2_pose.position.x = wing_column_1_pose.position.x+0.73;
      wing_column_2_pose.position.y = wing_column_1_pose.position.y;
      wing_column_2_pose.position.z = wing_column_1_pose.position.z;
      wing_column_2_pose.orientation.w = wing_column_1_pose.orientation.w;
      wing_column_2_pose.orientation.x = wing_column_1_pose.orientation.x;
      wing_column_2_pose.orientation.y = wing_column_1_pose.orientation.y;
      wing_column_2_pose.orientation.z = wing_column_1_pose.orientation.z;
      collision_object.primitives.push_back(wing_column_2);           // Push
      collision_object.primitive_poses.push_back(wing_column_2_pose); // Push
      // Wing row middle up front object
      shape_msgs::msg::SolidPrimitive wing_row_middle_up_front;
      wing_row_middle_up_front.type = wing_row_middle_up_front.BOX;
      wing_row_middle_up_front.dimensions = {0.05, 2.6, 0.03};
      geometry_msgs::msg::Pose wing_row_middle_up_front_pose;
      wing_row_middle_up_front_pose.position.x = collision_object.mesh_poses[0].position.x+0.7;
      wing_row_middle_up_front_pose.position.y = collision_object.mesh_poses[0].position.y-0.07;
      wing_row_middle_up_front_pose.position.z = wing_column_1_pose.position.z + 0.25;
      wing_row_middle_up_front_pose.orientation.w = wing_column_1_pose.orientation.w;
      wing_row_middle_up_front_pose.orientation.x = wing_column_1_pose.orientation.x;
      wing_row_middle_up_front_pose.orientation.y = wing_column_1_pose.orientation.y;
      wing_row_middle_up_front_pose.orientation.z = wing_column_1_pose.orientation.z;
      collision_object.primitives.push_back(wing_row_middle_up_front);          // Push
      collision_object.primitive_poses.push_back(wing_row_middle_up_front_pose);// Push
      // Wing row middle down front object
      shape_msgs::msg::SolidPrimitive wing_row_middle_down_front;
      wing_row_middle_down_front.type = wing_row_middle_down_front.BOX;
      wing_row_middle_down_front.dimensions = {0.05, 2.6, 0.03};
      geometry_msgs::msg::Pose wing_row_middle_down_front_pose;
      wing_row_middle_down_front_pose.position.x =
        wing_row_middle_up_front_pose.position.x;
      wing_row_middle_down_front_pose.position.y = wing_row_middle_up_front_pose.position.y+ 0.03;
      wing_row_middle_down_front_pose.position.z = wing_row_middle_up_front_pose.position.z - 0.4;
      wing_row_middle_down_front_pose.orientation.w = wing_column_1_pose.orientation.w;
      wing_row_middle_down_front_pose.orientation.x = wing_column_1_pose.orientation.x;
      wing_row_middle_down_front_pose.orientation.y = wing_column_1_pose.orientation.y;
      wing_row_middle_down_front_pose.orientation.z = wing_column_1_pose.orientation.z;
      collision_object.primitives.push_back(wing_row_middle_down_front);          // Push
      collision_object.primitive_poses.push_back(wing_row_middle_down_front_pose);// Push
      // Wing row top front object
      shape_msgs::msg::SolidPrimitive wing_row_top_front;
      wing_row_top_front.type = wing_row_top_front.BOX;
      wing_row_top_front.dimensions = {0.05, 2.6, 0.03};
      geometry_msgs::msg::Pose wing_row_top_front_pose;
      wing_row_top_front_pose.position.x = wing_row_middle_up_front_pose.position.x;
      wing_row_top_front_pose.position.y = wing_row_middle_up_front_pose.position.y- 0.035;
      wing_row_top_front_pose.position.z = wing_row_middle_up_front_pose.position.z + 0.23;
      wing_row_top_front_pose.orientation.w = wing_column_1_pose.orientation.w;
      wing_row_top_front_pose.orientation.x = wing_column_1_pose.orientation.x;
      wing_row_top_front_pose.orientation.y = wing_column_1_pose.orientation.y;
      wing_row_top_front_pose.orientation.z = wing_column_1_pose.orientation.z;
      collision_object.primitives.push_back(wing_row_top_front);           // Push
      collision_object.primitive_poses.push_back(wing_row_top_front_pose); // Push
      // Wing row bottom front object
      shape_msgs::msg::SolidPrimitive wing_row_bottom_front;
      wing_row_bottom_front.type = wing_row_bottom_front.BOX;
      wing_row_bottom_front.dimensions = {0.07, 2.6, 0.03};
      geometry_msgs::msg::Pose wing_row_bottom_front_pose;
      wing_row_bottom_front_pose.position.x = wing_row_middle_down_front_pose.position.x;
      wing_row_bottom_front_pose.position.y = wing_row_middle_down_front_pose.position.y+ 0.025;
      wing_row_bottom_front_pose.position.z = wing_row_middle_down_front_pose.position.z - 0.19;
      wing_row_bottom_front_pose.orientation.w = wing_column_1_pose.orientation.w;
      wing_row_bottom_front_pose.orientation.x = wing_column_1_pose.orientation.x;
      wing_row_bottom_front_pose.orientation.y = wing_column_1_pose.orientation.y;
      wing_row_bottom_front_pose.orientation.z = wing_column_1_pose.orientation.z;
      collision_object.primitives.push_back(wing_row_bottom_front);           // Push
      collision_object.primitive_poses.push_back(wing_row_bottom_front_pose); // Push
      // Wing row middle back object
      shape_msgs::msg::SolidPrimitive wing_row_middle_back;
      wing_row_middle_back.type = wing_row_middle_back.BOX;
      wing_row_middle_back.dimensions = {0.05, 2.8, 0.09};
      geometry_msgs::msg::Pose wing_row_middle_back_pose;
      wing_row_middle_back_pose.position.x = wing_row_middle_up_front_pose.position.x;
      wing_row_middle_back_pose.position.y = wing_row_middle_up_front_pose.position.y- 0.2;
      wing_row_middle_back_pose.position.z = wing_row_middle_up_front_pose.position.z - 0.12;
      wing_row_middle_back_pose.orientation.w = wing_column_1_pose.orientation.w;
      wing_row_middle_back_pose.orientation.x = wing_column_1_pose.orientation.x;
      wing_row_middle_back_pose.orientation.y = wing_column_1_pose.orientation.y;
      wing_row_middle_back_pose.orientation.z = wing_column_1_pose.orientation.z;
      collision_object.primitives.push_back(wing_row_middle_back);          // Push
      collision_object.primitive_poses.push_back(wing_row_middle_back_pose);// Push
      // Wing row middle up back object
      shape_msgs::msg::SolidPrimitive wing_row_middle_up_back;
      wing_row_middle_up_back.type = wing_row_middle_up_back.BOX;
      wing_row_middle_up_back.dimensions = {0.05, 2.6, 0.09};
      geometry_msgs::msg::Pose wing_row_middle_up_back_pose;
      wing_row_middle_up_back_pose.position.x = wing_row_middle_back_pose.position.x;
      wing_row_middle_up_back_pose.position.y = wing_row_middle_back_pose.position.y- 0.04;
      wing_row_middle_up_back_pose.position.z = wing_row_middle_back_pose.position.z + 0.24;
      wing_row_middle_up_back_pose.orientation.w = wing_column_1_pose.orientation.w;
      wing_row_middle_up_back_pose.orientation.x = wing_column_1_pose.orientation.x;
      wing_row_middle_up_back_pose.orientation.y = wing_column_1_pose.orientation.y;
      wing_row_middle_up_back_pose.orientation.z = wing_column_1_pose.orientation.z;
      collision_object.primitives.push_back(wing_row_middle_up_back);           // Push
      collision_object.primitive_poses.push_back(wing_row_middle_up_back_pose); // Push
      // Wing row middle up2 back object
      shape_msgs::msg::SolidPrimitive wing_row_middle_up2_back;
      wing_row_middle_up2_back.type = wing_row_middle_up2_back.BOX;
      wing_row_middle_up2_back.dimensions = {0.05, 2.6, 0.03};
      geometry_msgs::msg::Pose wing_row_middle_up2_back_pose;
      wing_row_middle_up2_back_pose.position.x = wing_row_middle_up_back_pose.position.x;
      wing_row_middle_up2_back_pose.position.y = wing_row_middle_up_back_pose.position.y- 0.035;
      wing_row_middle_up2_back_pose.position.z = wing_row_middle_up_back_pose.position.z + 0.21;
      wing_row_middle_up2_back_pose.orientation.w = wing_column_1_pose.orientation.w;
      wing_row_middle_up2_back_pose.orientation.x = wing_column_1_pose.orientation.x;
      wing_row_middle_up2_back_pose.orientation.y = wing_column_1_pose.orientation.y;
      wing_row_middle_up2_back_pose.orientation.z = wing_column_1_pose.orientation.z;
      collision_object.primitives.push_back(wing_row_middle_up2_back);          // Push
      collision_object.primitive_poses.push_back(wing_row_middle_up2_back_pose);// Push
      // Wing row middle down back object
      shape_msgs::msg::SolidPrimitive wing_row_middle_down_back;
      wing_row_middle_down_back.type = wing_row_middle_down_back.BOX;
      wing_row_middle_down_back.dimensions = {0.05, 2.6, 0.07};
      geometry_msgs::msg::Pose wing_row_middle_down_back_pose;
      wing_row_middle_down_back_pose.position.x = wing_row_middle_back_pose.position.x;
      wing_row_middle_down_back_pose.position.y = wing_row_middle_back_pose.position.y+ 0.01;
      wing_row_middle_down_back_pose.position.z = wing_row_middle_back_pose.position.z - 0.25;
      wing_row_middle_down_back_pose.orientation.w = wing_column_1_pose.orientation.w;
      wing_row_middle_down_back_pose.orientation.x = wing_column_1_pose.orientation.x;
      wing_row_middle_down_back_pose.orientation.y = wing_column_1_pose.orientation.y;
      wing_row_middle_down_back_pose.orientation.z = wing_column_1_pose.orientation.z;
      collision_object.primitives.push_back(wing_row_middle_down_back);           // Push
      collision_object.primitive_poses.push_back(wing_row_middle_down_back_pose); // Push
      // Wing row middle down2 back object
      shape_msgs::msg::SolidPrimitive wing_row_middle_down2_back;
      wing_row_middle_down2_back.type = wing_row_middle_down2_back.BOX;
      wing_row_middle_down2_back.dimensions = {0.05, 2.6, 0.08};
      geometry_msgs::msg::Pose wing_row_middle_down2_back_pose;
      wing_row_middle_down2_back_pose.position.x =
        wing_row_middle_down_back_pose.position.x;
      wing_row_middle_down2_back_pose.position.y = wing_row_middle_down_back_pose.position.y + 0.007;
      wing_row_middle_down2_back_pose.position.z =
        wing_row_middle_down_back_pose.position.z - 0.35;
      wing_row_middle_down2_back_pose.orientation.w = wing_column_1_pose.orientation.w;
      wing_row_middle_down2_back_pose.orientation.x = wing_column_1_pose.orientation.x;
      wing_row_middle_down2_back_pose.orientation.y = wing_column_1_pose.orientation.y;
      wing_row_middle_down2_back_pose.orientation.z = wing_column_1_pose.orientation.z;
      collision_object.primitives.push_back(wing_row_middle_down2_back);          // Push
      collision_object.primitive_poses.push_back(wing_row_middle_down2_back_pose);// Push
      // -----------------------------------------------------------------------

      // ----------------- INSP. OBJ. ------------------------------------------
      Eigen::Vector3d scale_insp_obj(1.0, 1.0, 1.0);
      shapes::Mesh * m_insp_obj = shapes::createMeshFromResource(
        "package://futama2_description/meshes/collision/dlr_robo_test.stl",scale_insp_obj);
      // Initializing meshe from STL
      shape_msgs::msg::Mesh insp_obj_mesh;
      // Initializing mesh message
      shapes::ShapeMsg insp_obj_mesh_msg;
      // Construct meshe from shape
      shapes::constructMsgFromShape(m_insp_obj, insp_obj_mesh_msg);
      insp_obj_mesh = boost::get<shape_msgs::msg::Mesh>(insp_obj_mesh_msg);
      // Assigning
      collision_object.meshes[2] = insp_obj_mesh;
      // Positions and orientations of mesh
      // Insp object
      collision_object.mesh_poses[2].position.x = - 0.698 + 0.15;
      collision_object.mesh_poses[2].position.y = 1.01 - 0.14;
      collision_object.mesh_poses[2].position.z = 0.098;
      collision_object.mesh_poses[2].orientation.w = 0.0;
      collision_object.mesh_poses[2].orientation.x = 0.0;
      collision_object.mesh_poses[2].orientation.y = 0.0;
      collision_object.mesh_poses[2].orientation.z = 1.0;
      // Mesh push
      collision_object.meshes.push_back(insp_obj_mesh);
      collision_object.mesh_poses.push_back(collision_object.mesh_poses[2]);
      // -----------------------------------------------------------------------

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
      
      // Ensure all objects are correctly assigned and maintained

      collision_object.operation = collision_object.ADD;
      
      moveit_msgs::msg::PlanningSceneWorld psw;
      psw.collision_objects.push_back(collision_object);

      auto ps = std::make_unique<moveit_msgs::msg::PlanningScene>();
      ps->world = psw;
      ps->is_diff = true;

      while (rclcpp::ok()) {
          size_t subscriber = collision_pub_->get_subscription_count();
          if (subscriber > 0) {
              auto ps = std::make_unique<moveit_msgs::msg::PlanningScene>();
              ps->world = psw;
              ps->is_diff = true;
              collision_pub_->publish(*ps); // Use copy publish instead of move
          }
          std::this_thread::sleep_for(std::chrono::seconds(1));
      }
    });
  }

    ~PlanningScenePublisher() override
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
RCLCPP_COMPONENTS_REGISTER_NODE(futama2_moveit_config::PlanningScenePublisher)
