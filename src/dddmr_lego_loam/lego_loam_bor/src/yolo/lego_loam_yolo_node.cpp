#include "imageProjection.h"
#include "featureAssociation.h"
#include "mapOptimization.h"

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"

//interactive
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float32.hpp"
#include "interactive_pose_graph_editor.h"

using namespace std::chrono_literals;

int main(int argc, char** argv) {

  rclcpp::init(argc, argv);

  Channel<ProjectionOut> projection_out_channel(true);
  auto IP = std::make_shared<ImageProjection>("lego_loam_ip", projection_out_channel);
  Channel<AssociationOut> association_out_channel(false);
  auto FA = std::make_shared<FeatureAssociation>("lego_loam_fa", projection_out_channel, association_out_channel);
  auto MO = std::make_shared<MapOptimization>("lego_loam_mo", association_out_channel);
  auto IPGE = std::make_shared<InteractivePoseGraphEditor>("interactive_pose_graph_editor", MO);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(IP);
  executor.add_node(FA);
  executor.add_node(MO);
  executor.add_node(IPGE);
  IP->tfInitial();
  FA->tfInitial();
  executor.spin();
  rclcpp::shutdown();

  return 0;
}


