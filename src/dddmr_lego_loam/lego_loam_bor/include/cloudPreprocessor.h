#ifndef CLOUD_PREPROCESSOR_H
#define CLOUD_PREPROCESSOR_H

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

typedef pcl::PointXYZI PointType;

class CloudPreprocessor : public rclcpp::Node
{
public:
    CloudPreprocessor(std::string name);
    ~CloudPreprocessor() = default;

private:
    // Callbacks
    void cloudHandler(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_filtered_cloud_;

    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_raw_cloud_;

    // PCL filters
    pcl::VoxelGrid<PointType> voxel_filter_;

    // Parameters
    float voxel_size_;
    bool debug_info_;

    // Statistics
    uint64_t frame_count_;
    double avg_input_points_;
    double avg_output_points_;
};

#endif  // CLOUD_PREPROCESSOR_H
