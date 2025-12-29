#include "cloudPreprocessor.h"

CloudPreprocessor::CloudPreprocessor(std::string name)
    : Node(name), frame_count_(0), avg_input_points_(0.0), avg_output_points_(0.0)
{
    // Declare parameters
    this->declare_parameter("voxel_size", rclcpp::ParameterValue(0.1f));
    this->declare_parameter("debug_info", rclcpp::ParameterValue(false));
    this->declare_parameter("input_topic", rclcpp::ParameterValue("/livox/lidar/pointcloud"));
    this->declare_parameter("output_topic", rclcpp::ParameterValue("/livox/lidar/pointcloud_filtered"));

    // Get parameters
    this->get_parameter("voxel_size", voxel_size_);
    this->get_parameter("debug_info", debug_info_);

    std::string input_topic, output_topic;
    this->get_parameter("input_topic", input_topic);
    this->get_parameter("output_topic", output_topic);

    // Setup voxel filter
    voxel_filter_.setLeafSize(voxel_size_, voxel_size_, voxel_size_);

    // Create subscriber
    sub_raw_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        input_topic, 
        rclcpp::SensorDataQoS(),
        std::bind(&CloudPreprocessor::cloudHandler, this, std::placeholders::_1));

    // Create publisher
    // Use Reliable to ensure compatibility with RViz2 and LeGO-LOAM default subscriptions
    pub_filtered_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        output_topic, 
        rclcpp::QoS(rclcpp::KeepLast(10)).reliable());

    RCLCPP_INFO(this->get_logger(), "CloudPreprocessor initialized");
    RCLCPP_INFO(this->get_logger(), "  Input topic: %s", input_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "  Output topic: %s", output_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "  Voxel size: %.3f m", voxel_size_);
    RCLCPP_INFO(this->get_logger(), "  Debug info: %s", debug_info_ ? "true" : "false");
}

void CloudPreprocessor::cloudHandler(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    // Convert ROS message to PCL point cloud
    pcl::PointCloud<PointType>::Ptr cloud_in(new pcl::PointCloud<PointType>);
    pcl::fromROSMsg(*msg, *cloud_in);

    // Record input point count
    uint64_t input_size = cloud_in->points.size();

    // Check if cloud is empty
    if (input_size == 0) {
        RCLCPP_WARN(this->get_logger(), "Received empty point cloud, skipping");
        return;
    }

    // Apply voxel grid filter
    pcl::PointCloud<PointType>::Ptr cloud_filtered(new pcl::PointCloud<PointType>);
    voxel_filter_.setInputCloud(cloud_in);
    voxel_filter_.filter(*cloud_filtered);

    // Record output point count
    uint64_t output_size = cloud_filtered->points.size();

    // Update statistics
    if (frame_count_ == 0) {
        avg_input_points_ = static_cast<double>(input_size);
        avg_output_points_ = static_cast<double>(output_size);
    } else {
        // Use exponential moving average for smoother statistics
        double alpha = 0.05;
        avg_input_points_ = (1.0 - alpha) * avg_input_points_ + alpha * input_size;
        avg_output_points_ = (1.0 - alpha) * avg_output_points_ + alpha * output_size;
    }
    frame_count_++;

    // Print debug info every 100 frames
    if (debug_info_ && (frame_count_ % 100 == 0)) {
        double compression_ratio = static_cast<double>(output_size) / input_size * 100.0;
        RCLCPP_INFO(this->get_logger(),
            "Frame %lu | Input: %lu pts | Output: %lu pts | Ratio: %.1f%% | Avg: %.0f -> %.0f",
            frame_count_, input_size, output_size, compression_ratio,
            avg_input_points_, avg_output_points_);
    }

    // Convert filtered cloud back to ROS message and publish
    sensor_msgs::msg::PointCloud2 cloud_out_msg;
    pcl::toROSMsg(*cloud_filtered, cloud_out_msg);
    cloud_out_msg.header = msg->header;  // Preserve original header (timestamp, frame_id)
    
    pub_filtered_cloud_->publish(cloud_out_msg);
}
