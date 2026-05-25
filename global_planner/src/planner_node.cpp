#include <rclcpp/rclcpp.hpp>
#include <ompl/util/Console.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>


#include "global_planner/ompl_rrt_core.hpp"
#include "global_planner/nlopt_trajectory_optimizer.hpp"

class GlobalPlannerNode : public rclcpp::Node {
public:
    GlobalPlannerNode() : Node("global_planner_node") {
        
        this->declare_parameter<double>("goal_x", -1.5);
        this->declare_parameter<double>("goal_y", -13.0);
        this->declare_parameter<double>("goal_z", 1.2);

        map_sub_ = this->create_subscription<octomap_msgs::msg::Octomap>(
            "/octomap_binary", 10, std::bind(&GlobalPlannerNode::map_callback, this, std::placeholders::_1));

        raw_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/raw_planned_path", 10);
        waypoints_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/waypoints", 10);
        smooth_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/smooth_trajectory", 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/goal_marker", 10);

        timer_ = this->create_wall_timer(
            std::chrono::seconds(5), std::bind(&GlobalPlannerNode::plan_demo_path, this));

        RCLCPP_INFO(this->get_logger(), "Global Planner Node initialized");
    }

private:
    void map_callback(const octomap_msgs::msg::Octomap::SharedPtr msg) {
        
        octomap::AbstractOcTree* abstract_tree = octomap_msgs::binaryMsgToMap(*msg);
        
        if (abstract_tree) {
            
            octomap::OcTree* octree = dynamic_cast<octomap::OcTree*>(abstract_tree);
            
            if (octree) {
                octree_ptr_ = std::shared_ptr<octomap::OcTree>(octree);
            } else {
                delete abstract_tree; 
            }
        }
    }

    void publish_goal_marker(double x, double y, double z) {
        
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
        
        marker.scale.x = 0.5;
        marker.scale.y = 0.5;
        marker.scale.z = 0.5;
        
        marker.color.a = 1.0; 
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        
        marker_pub_->publish(marker);
    }

    void plan_demo_path() {
        if (!octree_ptr_) {
            return;
        }

        global_planner::OmplPlanner planner(octree_ptr_);

        double gx = this->get_parameter("goal_x").as_double();
        double gy = this->get_parameter("goal_y").as_double();
        double gz = this->get_parameter("goal_z").as_double();

        publish_goal_marker(gx, gy, gz);

        std::vector<double> start = {-3.0, 12.5, 0.8};
        std::vector<double> goal = {gx, gy, gz};
        std::vector<std::vector<double>> result_path;

        if (planner.planPath(start, goal, result_path)) {
            
            nav_msgs::msg::Path raw_path_msg;
            raw_path_msg.header.stamp = this->now();
            raw_path_msg.header.frame_id = "map";

            visualization_msgs::msg::MarkerArray waypoints_msg;

            for (size_t i = 0; i < result_path.size(); ++i) {
                geometry_msgs::msg::PoseStamped pose;
                pose.pose.position.x = result_path[i][0];
                pose.pose.position.y = result_path[i][1];
                pose.pose.position.z = result_path[i][2];
                raw_path_msg.poses.push_back(pose);

                visualization_msgs::msg::Marker marker;
                marker.header = raw_path_msg.header;
                marker.ns = "waypoints";
                marker.id = i;
                marker.type = visualization_msgs::msg::Marker::SPHERE;
                marker.action = visualization_msgs::msg::Marker::ADD;
                marker.pose.position.x = result_path[i][0];
                marker.pose.position.y = result_path[i][1];
                marker.pose.position.z = result_path[i][2];
                marker.scale.x = 0.3;
                marker.scale.y = 0.3;
                marker.scale.z = 0.3;
                marker.color.a = 1.0;
                marker.color.r = 0.0;
                marker.color.g = 1.0;
                marker.color.b = 0.0;
                waypoints_msg.markers.push_back(marker);
            }

            raw_path_pub_->publish(raw_path_msg);
            waypoints_pub_->publish(waypoints_msg);
            
            global_planner::NLOptTrajectoryOptimizer optimizer;
            std::vector<std::vector<double>> smooth_trajectory;
            
            if (optimizer.generateOptimizedTrajectory(result_path, smooth_trajectory)) {
                RCLCPP_INFO(this->get_logger(), "OPTIMIZATION HAS RAN SUCCESSFULLY");

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
                smooth_path_pub_->publish(path_msg);
            }
        }
    }

    rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr map_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr raw_path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr smooth_path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr waypoints_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<octomap::OcTree> octree_ptr_;
};

int main(int argc, char** argv) {
    
    ompl::msg::setLogLevel(ompl::msg::LOG_WARN);
    
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GlobalPlannerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}