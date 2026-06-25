#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <chrono>

class DatasetPCNode : public rclcpp::Node {
public:
    DatasetPCNode() : Node("dataset_pc_node") {
        // Set up the Publisher (Note the updated topic name)
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/dataset_pc/points", 10);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        
        std::string file_path = "/home/pablo/ws_paramio/src/pc_publisher/data/office3.pcd"; 

        if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(file_path, *cloud) == -1) {
            RCLCPP_ERROR(this->get_logger(), "Couldn't read the PCD file! Check the path.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Successfully loaded point cloud with %lu points.", cloud->points.size());

        // Convert PCL object to ROS 2 Message
        pcl::toROSMsg(*cloud, ros_cloud_msg_);
        ros_cloud_msg_.header.frame_id = "map"; // Set the reference frame

        // Publish at 1 Hz using a timer
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&DatasetPCNode::publish_cloud, this));
    }

private:
    void publish_cloud() {
        ros_cloud_msg_.header.stamp = this->now();
        publisher_->publish(ros_cloud_msg_);
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    sensor_msgs::msg::PointCloud2 ros_cloud_msg_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DatasetPCNode>());
    rclcpp::shutdown();
    return 0;
}