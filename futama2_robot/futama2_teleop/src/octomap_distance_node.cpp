#include <rclcpp/rclcpp.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <std_msgs/msg/float64.hpp>
#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/msg/marker.hpp>

using std::placeholders::_1;

class OctomapDistanceNode : public rclcpp::Node
{
public:
    OctomapDistanceNode() : Node("octomap_distance_node")
    {
        sub_ = this->create_subscription<moveit_msgs::msg::PlanningScene>(
            "/monitored_planning_scene", 10,
            std::bind(&OctomapDistanceNode::scene_callback, this, _1));

        dist_pub_ = this->create_publisher<std_msgs::msg::Float64>("distance_to_nearest_voxel", 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("nearest_voxel_marker", 10);

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

private:
    rclcpp::Subscription<moveit_msgs::msg::PlanningScene>::SharedPtr sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr dist_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    void scene_callback(const moveit_msgs::msg::PlanningScene::SharedPtr msg)
    {
        const auto &octomap_msg = msg->world.octomap.octomap;

        if (octomap_msg.id.empty() || octomap_msg.data.empty()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                "Skipping message: empty octomap ID or data.");
            return;
        }

        octomap::AbstractOcTree* tree = octomap_msgs::fullMsgToMap(octomap_msg);
        if (!tree) {
            RCLCPP_WARN(this->get_logger(), "Failed to convert Octomap message to OcTree.");
            return;
        }

        auto octree = std::shared_ptr<octomap::OcTree>(dynamic_cast<octomap::OcTree*>(tree));
        if (!octree) {
            RCLCPP_WARN(this->get_logger(), "Octomap is not of type OcTree.");
            delete tree;
            return;
        }

        geometry_msgs::msg::TransformStamped transform;
        try {
            transform = tf_buffer_->lookupTransform("world", "realsense_center_link", tf2::TimePointZero);
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                "Could not transform: %s", ex.what());
            return;
        }

        octomap::point3d query_point(
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z
        );

        double min_dist = std::numeric_limits<double>::max();
        octomap::point3d closest_voxel;

        for (auto it = octree->begin_leafs(), end = octree->end_leafs(); it != end; ++it) {
            if (octree->isNodeOccupied(*it)) {
                double dist = it.getCoordinate().distance(query_point);
                if (dist < min_dist) {
                    min_dist = dist;
                    closest_voxel = it.getCoordinate();
                }
            }
        }

        // 1. Publish distance
        std_msgs::msg::Float64 msg_out;
        msg_out.data = min_dist;
        dist_pub_->publish(msg_out);

        // 2. Publish marker
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = this->now();
        marker.ns = "nearest_voxel_arrow";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::ARROW;
        marker.action = visualization_msgs::msg::Marker::ADD;

        geometry_msgs::msg::Point start, end;
        start.x = query_point.x();
        start.y = query_point.y();
        start.z = query_point.z();
        end.x = closest_voxel.x();
        end.y = closest_voxel.y();
        end.z = closest_voxel.z();

        marker.points.push_back(start);
        marker.points.push_back(end);

        marker.scale.x = 0.01;  // shaft diameter
        marker.scale.y = 0.02;  // head diameter
        marker.scale.z = 0.02;  // head length

        marker.color.r = 1.0;
        marker.color.g = 0.2;
        marker.color.b = 0.0;
        marker.color.a = 1.0;

        marker.lifetime = rclcpp::Duration::from_seconds(0.5);
        marker_pub_->publish(marker);

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "Distance to nearest voxel from realsense_center_link: %.3f m", min_dist);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OctomapDistanceNode>());
    rclcpp::shutdown();
    return 0;
}
