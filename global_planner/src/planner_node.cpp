#include <rclcpp/rclcpp.hpp>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "global_planner/ompl_rrt_core.hpp"

class GlobalPlannerNode : public rclcpp::Node {
public:
    GlobalPlannerNode() : Node("global_planner_node") {
        // Subscribe to the binary OctoMap coming from octomap_server
        map_sub_ = this->create_subscription<octomap_msgs::msg::Octomap>(
            "/octomap_binary", 10, std::bind(&GlobalPlannerNode::map_callback, this, std::placeholders::_1));

        // Publisher to show the final path in RViz
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/global_path", 10);

        // For this demo, we'll trigger the planner 2 seconds after the map is received
        // In a real system, this would be triggered by a Goal Pose or an Action
        timer_ = this->create_wall_timer(
            std::chrono::seconds(5), std::bind(&GlobalPlannerNode::plan_demo_path, this));

        RCLCPP_INFO(this->get_logger(), "Global Planner Node initialized. Waiting for OctoMap...");
    }

private:
    void map_callback(const octomap_msgs::msg::Octomap::SharedPtr msg) {
        // Get the abstract pointer
        octomap::AbstractOcTree* abstract_tree = octomap_msgs::binaryMsgToMap(*msg);
        
        if (abstract_tree) {
            // Cast it to the actual OcTree type
            octomap::OcTree* octree = dynamic_cast<octomap::OcTree*>(abstract_tree);
            
            if (octree) {
                octree_ptr_ = std::shared_ptr<octomap::OcTree>(octree);
                RCLCPP_INFO(this->get_logger(), "OctoMap stored. Nodes: %zu", octree_ptr_->size());
            } else {
                RCLCPP_ERROR(this->get_logger(), "Abstract tree is not an OcTree!");
                delete abstract_tree; // Clean up if cast fails
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to deserialize OctoMap binary message.");
        }
    }

    void plan_demo_path() {
        if (!octree_ptr_) {
            RCLCPP_WARN(this->get_logger(), "No map received yet. Cannot plan.");
            return;
        }

        // Initialize our Library with the current map
        global_planner::OmplPlanner planner(octree_ptr_);

        // Define a start and end point (adjust these to fit your office3.pcd)
        std::vector<double> start = {0.0, 0.0, 0.8};
        std::vector<double> goal = {7.0, -13.0, 1.2};
        std::vector<std::vector<double>> result_path;

        RCLCPP_INFO(this->get_logger(), "Attempting to plan path...");

        if (planner.planPath(start, goal, result_path)) {
            RCLCPP_INFO(this->get_logger(), "Path Found! Points: %zu", result_path.size());

            // Convert our library results into a ROS Path message
            nav_msgs::msg::Path path_msg;
            path_msg.header.stamp = this->now();
            path_msg.header.frame_id = "map";

            for (const auto& point : result_path) {
                geometry_msgs::msg::PoseStamped pose;
                pose.pose.position.x = point[0];
                pose.pose.position.y = point[1];
                pose.pose.position.z = point[2];
                path_msg.poses.push_back(pose);
            }
            path_pub_->publish(path_msg);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Planner failed to find a path.");
        }
    }

    rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr map_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<octomap::OcTree> octree_ptr_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GlobalPlannerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}