#include <yolo_ros2_image_sub.h>

YoloRos2ImageSub::YoloRos2ImageSub(std::string name):Node(name){
  

  this->declare_parameter("trt_model_path", rclcpp::ParameterValue(""));
  this->get_parameter("trt_model_path", trt_model_path_);
  RCLCPP_INFO(this->get_logger(), "trt_model_path: %s" , trt_model_path_.c_str());

  sub_img_ = this->create_subscription<sensor_msgs::msg::Image>(
              "image", rclcpp::QoS(rclcpp::KeepLast(1)).durability_volatile().best_effort(),
              std::bind(&YoloRos2ImageSub::cbImg, this, std::placeholders::_1) );
  
  pub_annotated_img_ = this->create_publisher<sensor_msgs::msg::Image>("annotated_image", 1);

  YoloV8Config config;
  yolov8_ = std::make_shared<YoloV8>("", trt_model_path_, config);

}

void YoloRos2ImageSub::cbImg(const sensor_msgs::msg::Image::SharedPtr img){

  try
  {
    cv_image_ = cv_bridge::toCvCopy(img, img->encoding);
  }
  catch (cv_bridge::Exception& e)
  {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  // Run inference
  const auto objects = yolov8_->detectObjects(cv_image_->image);

  // Draw the bounding boxes on the image
  yolov8_->drawObjectLabels(cv_image_->image, objects);

  cv_bridge::CvImage img_bridge;
  img_bridge.image = cv_image_->image;
  img_bridge.encoding = sensor_msgs::image_encodings::TYPE_8UC3;
  sensor_msgs::msg::Image::SharedPtr msg = img_bridge.toImageMsg();
  pub_annotated_img_->publish(*msg);

}

// Node execution starts here
int main(int argc, char * argv[])
{
  // Initialize ROS 2
  rclcpp::init(argc, argv);
  
  YoloRos2ImageSub YR2IS = YoloRos2ImageSub("yolo_ros2_img");

  rclcpp::executors::MultiThreadedExecutor::SharedPtr mulexecutor_;

  mulexecutor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

  mulexecutor_->add_node(YR2IS.get_node_base_interface());
   
  mulexecutor_->spin();

  // Shutdown the node when finished
  rclcpp::shutdown();
  return 0;

}