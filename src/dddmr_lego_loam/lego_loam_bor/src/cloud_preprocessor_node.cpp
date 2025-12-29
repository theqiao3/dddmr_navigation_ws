#include "cloudPreprocessor.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CloudPreprocessor>("cloud_preprocessor");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
