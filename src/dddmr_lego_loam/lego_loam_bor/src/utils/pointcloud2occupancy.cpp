/*
* BSD 3-Clause License

* Copyright (c) 2024, DDDMobileRobot

* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:

* 1. Redistributions of source code must retain the above copyright notice, this
*    list of conditions and the following disclaimer.

* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.

* 3. Neither the name of the copyright holder nor the names of its
*    contributors may be used to endorse or promote products derived from
*    this software without specific prior written permission.

* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#include "rclcpp/rclcpp.hpp"

#include "pcl/common/transforms.h"
#include "pcl/PCLPointCloud2.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <iostream>
#include <fstream>
#include <sstream>
#include <filesystem>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

using namespace std::chrono_literals;

class PointCloud2Occupancy : public rclcpp::Node
{

  public:

    PointCloud2Occupancy();
 
  private:
    
    void findMinMaxXY();
    void createOccupancy();

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_map_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_occupancy_;

    std::string pcd_dir_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_;
    double min_y_, min_x_;
    double max_y_, max_x_;
    int min_y_int_, min_x_int_;
    int max_y_int_, max_x_int_;
};


PointCloud2Occupancy::PointCloud2Occupancy():Node("pointcloud_2_occupancy"){


  pub_map_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("mapcloud",
              rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  pub_occupancy_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("mapcloud_occupancy",
              rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  this->declare_parameter("pcd_dir", rclcpp::ParameterValue("/root/dddmr_bags/meydan_hotel_pg/map.pcd"));
  this->get_parameter("pcd_dir", pcd_dir_);
  RCLCPP_INFO(this->get_logger(), "pcd_dir: %s" , pcd_dir_.c_str());

  if(!std::filesystem::exists(pcd_dir_))
  {
    RCLCPP_INFO(get_logger(), "File: %s not exist, exit.", pcd_dir_.c_str());
    return;
  }

  RCLCPP_INFO(get_logger(), "Reading file from: %s", pcd_dir_.c_str());
  cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>);

  if (pcl::io::loadPCDFile<pcl::PointXYZI> (pcd_dir_, *cloud_) == -1) //@ load the file
  {
    RCLCPP_ERROR(get_logger(), "Couldn't read pcd file: %s", pcd_dir_.c_str());
  }

  sensor_msgs::msg::PointCloud2 ros_msg_map_cloud;
  pcl::toROSMsg(*cloud_, ros_msg_map_cloud);
  ros_msg_map_cloud.header.frame_id = "map";
  pub_map_->publish(ros_msg_map_cloud);

  findMinMaxXY();
  createOccupancy();

}

void PointCloud2Occupancy::findMinMaxXY(){
  
  min_y_ = 1000000;
  min_x_ = 1000000;
  max_y_ = -1000000;
  max_x_ = -1000000;

  for(auto i=cloud_->points.begin(); i!=cloud_->points.end();i++){
    if((*i).y<min_y_)
      min_y_ = (*i).y;
    if((*i).x<min_x_)
      min_x_ = (*i).x;
    if((*i).y>max_y_)
      max_y_ = (*i).y;
    if((*i).x>max_x_)
      max_x_ = (*i).x;
  }

  RCLCPP_INFO(this->get_logger(), "MinX: %.2f, MinY: %.2f, MaxX: %.2f, MaxY: %.2f", min_x_, min_y_, max_x_, max_y_);
  
  min_y_int_ = int(min_y_/0.05)-1;
  min_x_int_ = int(min_x_/0.05)-1;
  max_y_int_ = int(max_y_/0.05)+1;
  max_x_int_ = int(max_x_/0.05)+1;
}

void PointCloud2Occupancy::createOccupancy(){

  nav_msgs::msg::OccupancyGrid map;
  map.header.frame_id = "map";
  map.info.resolution = 0.05;
  map.info.width = max_x_int_ - min_x_int_;
  map.info.height = max_y_int_ - min_y_int_;
  map.info.origin.position.x = min_x_int_*0.05;
  map.info.origin.position.y = min_y_int_*0.05;
  map.info.origin.orientation.w = 1.0;
  map.data.resize(map.info.width * map.info.height);
  for(unsigned int y = 0; y < map.info.height; y++) {
    for(unsigned int x = 0; x < map.info.width; x++) {
      unsigned int i = x + y * map.info.width;
      map.data[i] = 0;
    }
  }
  for(auto it=cloud_->points.begin(); it!=cloud_->points.end();it++){
    int x_index = (*it).x/0.05 - min_x_int_;
    int y_index = (*it).y/0.05 - min_y_int_;
    unsigned int i = x_index + y_index * map.info.width;
    map.data[i] = 100;
  }
  pub_occupancy_->publish(map);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloud2Occupancy>());
  rclcpp::shutdown();
  return 0;
}
