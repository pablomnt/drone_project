#include <rclcpp/rclcpp.hpp>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include "global_planner/sfc_optimizer.hpp"

#include "global_planner/ompl_rrt_core.hpp"

class GlobalPlannerNode : public rclcpp::Node {
public:
    GlobalPlannerNode() : Node("global_planner_node") {
        
        // Declare node parameters for the goal coordinates
        this->declare_parameter<double>("goal_x", -1.5);
        this->declare_parameter<double>("goal_y", -13.0);
        this->declare_parameter<double>("goal_z", 1.2);

        // Subscribe to the binary OctoMap topic
        map_sub_ = this->create_subscription<octomap_msgs::msg::Octomap>(
            "/octomap_binary", 10, std::bind(&GlobalPlannerNode::map_callback, this, std::placeholders::_1));

        // Initialize publishers for the path and the goal marker
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/global_path", 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/goal_marker", 10);

        // Create a timer to trigger the planning process
        timer_ = this->create_wall_timer(
            std::chrono::seconds(5), std::bind(&GlobalPlannerNode::plan_demo_path, this));

        RCLCPP_INFO(this->get_logger(), "Global Planner Node initialized. Waiting for OctoMap...");
    }

private:
    void map_callback(const octomap_msgs::msg::Octomap::SharedPtr msg) {
        
        // Convert the ROS message to an abstract OctoMap pointer
        octomap::AbstractOcTree* abstract_tree = octomap_msgs::binaryMsgToMap(*msg);
        
        if (abstract_tree) {
            
            // Cast the abstract tree to an OcTree type
            octomap::OcTree* octree = dynamic_cast<octomap::OcTree*>(abstract_tree);
            
            if (octree) {
                // Store the map in a shared pointer for the planner
                octree_ptr_ = std::shared_ptr<octomap::OcTree>(octree);
                RCLCPP_INFO(this->get_logger(), "OctoMap stored. Nodes: %zu", octree_ptr_->size());
            } else {
                RCLCPP_ERROR(this->get_logger(), "Abstract tree is not an OcTree!");
                // Free memory if the cast fails
                delete abstract_tree; 
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to deserialize OctoMap binary message.");
        }
    }

    void publish_goal_marker(double x, double y, double z) {
        
        // Create and populate a visualization marker for RViz
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->now();
        marker.ns = "goal_namespace";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        
        marker.pose.position.x = x;
        marker.pose.position.y = y;
        marker.pose.position.z = z;
        marker.pose.orientation.w = 1.0;
        
        // Set the scale of the sphere
        marker.scale.x = 0.5;
        marker.scale.y = 0.5;
        marker.scale.z = 0.5;
        
        // Set the color to solid red
        marker.color.a = 1.0; 
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        
        marker_pub_->publish(marker);
    }

    void plan_demo_path() {
        if (!octree_ptr_) {
            RCLCPP_WARN(this->get_logger(), "No map received yet. Cannot plan.");
            return;
        }

        // Instantiate the planner with the received map
        global_planner::OmplPlanner planner(octree_ptr_);

        // Retrieve the goal coordinates from the node parameters
        double gx = this->get_parameter("goal_x").as_double();
        double gy = this->get_parameter("goal_y").as_double();
        double gz = this->get_parameter("goal_z").as_double();

        // Publish the visual marker at the goal location
        publish_goal_marker(gx, gy, gz);

        // Define the start and goal vectors
        std::vector<double> start = {-3.0, 12.5, 0.8};
        std::vector<double> goal = {gx, gy, gz};
        std::vector<std::vector<double>> result_path;

        RCLCPP_INFO(this->get_logger(), "Attempting to plan path...");

    if (planner.planPath(start, goal, result_path)) {
        RCLCPP_INFO(this->get_logger(), "RRT* Path Found! Points: %zu", result_path.size());

        // --- NEW SFC PIPELINE ---
        global_planner::SfcOptimizer optimizer(octree_ptr_);
        std::vector<std::vector<double>> smooth_trajectory;
        
        // Target 2.0 m/s average speed
        if (optimizer.generateTrajectory(result_path, 2.0, smooth_trajectory)) {
            RCLCPP_INFO(this->get_logger(), "Trajectory Optimized! Points: %zu", smooth_trajectory.size());

            // Publish the OPTIMIZED trajectory to RViz, not the raw RRT* path
            nav_msgs::msg::Path path_msg;
            path_msg.header.stamp = this->now();
            path_msg.header.frame_id = "map";

            for (const auto& point : smooth_trajectory) {
                geometry_msgs::msg::PoseStamped pose;
                pose.pose.position.x = point[0];
                pose.pose.position.y = point[1];
                pose.pose.position.z = point[2];
                path_msg.poses.push_back(pose);
            }
            path_pub_->publish(path_msg);
        }
    }
        else {
            RCLCPP_ERROR(this->get_logger(), "Planner failed to find a path.");
        }
    }

    rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr map_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<octomap::OcTree> octree_ptr_;
};

int main(int argc, char** argv) {
    
    // Initialize the ROS context and spin the node
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GlobalPlannerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}