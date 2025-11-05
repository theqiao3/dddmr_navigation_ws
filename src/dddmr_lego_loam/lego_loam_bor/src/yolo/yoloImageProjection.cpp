// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <boost/circular_buffer.hpp>
#include "yoloImageProjection.h"

using std::placeholders::_1;

YoloImageProjection::YoloImageProjection(std::string name, Channel<ProjectionOut>& output_channel)
    : Node(name), _output_channel(output_channel), first_frame_processed_(0), got_baselink2sensor_tf_(false)
{

  //supress the no intensity found log
  pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
  clock_ = this->get_clock();
  last_save_depth_img_time_ = 0;

  tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
  _pub_projected_image = this->create_publisher<sensor_msgs::msg::Image>("projected_image", 1);

  _sub_laser_cloud = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "lslidar_point_cloud", 2,
        std::bind(&YoloImageProjection::cloudHandler, this, std::placeholders::_1));

  _pub_full_info_cloud = this->create_publisher<sensor_msgs::msg::PointCloud2>
      ("full_cloud_info", 1);  

  _pub_ground_cloud = this->create_publisher<sensor_msgs::msg::PointCloud2>
      ("ground_cloud", 1);  

  _pub_segmented_cloud = this->create_publisher<sensor_msgs::msg::PointCloud2>
      ("segmented_cloud", 1); 

  _pub_segmented_cloud_pure = this->create_publisher<sensor_msgs::msg::PointCloud2>
      ("segmented_cloud_pure", 1);

  _pub_segmented_cloud_info = this->create_publisher<cloud_msgs::msg::CloudInfo>
      ("segmented_cloud_info", 1); 

  _pub_outlier_cloud = this->create_publisher<sensor_msgs::msg::PointCloud2>
      ("outlier_cloud", 1); 
  
  pub_annotated_img_ = this->create_publisher<sensor_msgs::msg::Image>("annotated_image", 1);
  
  declare_parameter("laser.num_vertical_scans", rclcpp::ParameterValue(0));
  this->get_parameter("laser.num_vertical_scans", _vertical_scans);
  RCLCPP_INFO(this->get_logger(), "laser.num_vertical_scans: %d", _vertical_scans);
  
  declare_parameter("laser.num_horizontal_scans", rclcpp::ParameterValue(0));
  this->get_parameter("laser.num_horizontal_scans", _horizontal_scans);
  RCLCPP_INFO(this->get_logger(), "laser.num_horizontal_scans: %d", _horizontal_scans);
  
  declare_parameter("laser.scan_period", rclcpp::ParameterValue(0.1));
  this->get_parameter("laser.scan_period", _scan_period);
  RCLCPP_INFO(this->get_logger(), "laser.scan_period: %.2f", _scan_period);
  
  declare_parameter("laser.vertical_angle_bottom", rclcpp::ParameterValue(0.0));
  this->get_parameter("laser.vertical_angle_bottom", _ang_bottom);
  RCLCPP_INFO(this->get_logger(), "laser.vertical_angle_bottom: %.2f", _ang_bottom);

  float vertical_angle_top;
  declare_parameter("laser.vertical_angle_top", rclcpp::ParameterValue(0.0));
  this->get_parameter("laser.vertical_angle_top", vertical_angle_top);
  RCLCPP_INFO(this->get_logger(), "laser.vertical_angle_top: %.2f", vertical_angle_top);

  _ang_resolution_X = (M_PI*2) / (_horizontal_scans);
  _ang_resolution_Y = DEG_TO_RAD*(vertical_angle_top - _ang_bottom) / float(_vertical_scans-1);
  _ang_bottom = -( _ang_bottom - 0.1) * DEG_TO_RAD;
  _segment_alpha_X = _ang_resolution_X;
  _segment_alpha_Y = _ang_resolution_Y;
  
  declare_parameter("imageProjection.segment_theta", rclcpp::ParameterValue(0.0));
  this->get_parameter("imageProjection.segment_theta", _segment_theta);
  RCLCPP_INFO(this->get_logger(), "imageProjection.segment_theta: %.2f", _segment_theta);
  _segment_theta *= DEG_TO_RAD;
  
  declare_parameter("imageProjection.segment_valid_point_num", rclcpp::ParameterValue(0));
  this->get_parameter("imageProjection.segment_valid_point_num", _segment_valid_point_num);
  RCLCPP_INFO(this->get_logger(), "imageProjection.segment_valid_point_num: %d", _segment_valid_point_num);
  
  declare_parameter("imageProjection.segment_valid_line_num", rclcpp::ParameterValue(0));
  this->get_parameter("imageProjection.segment_valid_line_num",_segment_valid_line_num);
  RCLCPP_INFO(this->get_logger(), "imageProjection.segment_valid_line_num: %d", _segment_valid_line_num);

  declare_parameter("laser.ground_scan_index", rclcpp::ParameterValue(0));
  this->get_parameter("laser.ground_scan_index", _ground_scan_index);
  RCLCPP_INFO(this->get_logger(), "laser.ground_scan_index: %d", _ground_scan_index);

  declare_parameter("laser.odom_type", rclcpp::ParameterValue(""));
  this->get_parameter("laser.odom_type", odom_type_);
  RCLCPP_INFO(this->get_logger(), "laser.odom_type: %s", odom_type_.c_str());
  
  declare_parameter("laser.baselink_frame", rclcpp::ParameterValue(""));
  this->get_parameter("laser.baselink_frame", baselink_frame_);
  RCLCPP_INFO(this->get_logger(), "laser.baselink_frame: %s", baselink_frame_.c_str());
  
  declare_parameter("imageProjection.maximum_detection_range", rclcpp::ParameterValue(0.0));
  this->get_parameter("imageProjection.maximum_detection_range", _maximum_detection_range);
  RCLCPP_INFO(this->get_logger(), "imageProjection.maximum_detection_range: %.2f", _maximum_detection_range);

  declare_parameter("imageProjection.minimum_detection_range", rclcpp::ParameterValue(0.3));
  this->get_parameter("imageProjection.minimum_detection_range", _minimum_detection_range);
  RCLCPP_INFO(this->get_logger(), "imageProjection.minimum_detection_range: %.2f", _minimum_detection_range);

  declare_parameter("imageProjection.distance_for_patch_between_rings", rclcpp::ParameterValue(1.0));
  this->get_parameter("imageProjection.distance_for_patch_between_rings", distance_for_patch_between_rings_);
  RCLCPP_INFO(this->get_logger(), "imageProjection.distance_for_patch_between_rings: %.2f", distance_for_patch_between_rings_);

  declare_parameter("imageProjection.to_fa", rclcpp::ParameterValue(true));
  this->get_parameter("imageProjection.to_fa", to_fa_);
  RCLCPP_INFO(this->get_logger(), "imageProjection.to_fa: %d", to_fa_);
  
  declare_parameter("imageProjection.time_step_between_depth_image", rclcpp::ParameterValue(0.5));
  this->get_parameter("imageProjection.time_step_between_depth_image", time_step_between_depth_image_);
  RCLCPP_INFO(this->get_logger(), "imageProjection.time_step_between_depth_image: %.2f", time_step_between_depth_image_);

  declare_parameter("imageProjection.stitcher_num", rclcpp::ParameterValue(0));
  this->get_parameter("imageProjection.stitcher_num", stitcher_num_);
  RCLCPP_INFO(this->get_logger(), "imageProjection.stitcher_num: %d", stitcher_num_);
  
  this->declare_parameter("imageProjection.trt_model_path", rclcpp::ParameterValue(""));
  this->get_parameter("imageProjection.trt_model_path", trt_model_path_);
  RCLCPP_INFO(this->get_logger(), "imageProjection.trt_model_path: %s" , trt_model_path_.c_str());

  YoloV8Config config;
  yolov8_ = std::make_shared<YoloV8>("", trt_model_path_, config);

  const size_t cloud_size = _vertical_scans * _horizontal_scans;

  _laser_cloud_in.reset(new pcl::PointCloud<PointType>());
  _full_cloud.reset(new pcl::PointCloud<PointType>());
  _full_info_cloud.reset(new pcl::PointCloud<PointType>());

  _ground_cloud.reset(new pcl::PointCloud<PointType>());
  _segmented_cloud.reset(new pcl::PointCloud<PointType>());
  _segmented_cloud_pure.reset(new pcl::PointCloud<PointType>());
  _outlier_cloud.reset(new pcl::PointCloud<PointType>());
  patched_ground_.reset(new pcl::PointCloud<PointType>());
  patched_ground_edge_.reset(new pcl::PointCloud<PointType>());

  _full_cloud->points.resize(cloud_size);
  _full_info_cloud->points.resize(cloud_size);
  
  dsf_patched_ground_.setLeafSize(0.1, 0.1, 0.1);
}


void YoloImageProjection::resetParameters() {
  const size_t cloud_size = _vertical_scans * _horizontal_scans;
  PointType nanPoint;
  nanPoint.x = std::numeric_limits<float>::quiet_NaN();
  nanPoint.y = std::numeric_limits<float>::quiet_NaN();
  nanPoint.z = std::numeric_limits<float>::quiet_NaN();

  _laser_cloud_in->clear();
  _ground_cloud->clear();
  _segmented_cloud->clear();
  _segmented_cloud_pure->clear();
  _outlier_cloud->clear();

  _range_mat.resize(_vertical_scans, _horizontal_scans);
  _ground_mat.resize(_vertical_scans, _horizontal_scans);
  _label_mat.resize(_vertical_scans, _horizontal_scans);

  _range_mat.fill(FLT_MAX);
  _ground_mat.setZero();
  _label_mat.setZero();

  _label_count = 1;

  std::fill(_full_cloud->points.begin(), _full_cloud->points.end(), nanPoint);
  std::fill(_full_info_cloud->points.begin(), _full_info_cloud->points.end(),
            nanPoint);

  _seg_msg.start_ring_index.assign(_vertical_scans, 0);
  _seg_msg.end_ring_index.assign(_vertical_scans, 0);

  _seg_msg.segmented_cloud_ground_flag.assign(cloud_size, false);
  _seg_msg.segmented_cloud_col_ind.assign(cloud_size, 0);
  _seg_msg.segmented_cloud_range.assign(cloud_size, 0);
}

void YoloImageProjection::tfInitial(){

  //@Initialize transform listener and broadcaster
  tf_listener_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  tf2Buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    this->get_node_base_interface(),
    this->get_node_timers_interface(),
    tf_listener_group_);
  tf2Buffer_->setCreateTimerInterface(timer_interface);
  tfl_ = std::make_shared<tf2_ros::TransformListener>(*tf2Buffer_);
}

bool YoloImageProjection::allEssentialTFReady(std::string sensor_frame){
  
  if(!got_baselink2sensor_tf_){
    sensor_frame_ = sensor_frame;
    try
    {
      trans_b2s_ = tf2Buffer_->lookupTransform(
          baselink_frame_, sensor_frame_, tf2::TimePointZero);
      
      tf2_trans_b2s_.setRotation(tf2::Quaternion(trans_b2s_.transform.rotation.x, trans_b2s_.transform.rotation.y, trans_b2s_.transform.rotation.z, trans_b2s_.transform.rotation.w));
      tf2_trans_b2s_.setOrigin(tf2::Vector3(trans_b2s_.transform.translation.x, trans_b2s_.transform.translation.y, trans_b2s_.transform.translation.z));
      
      tf2::Matrix3x3 m(tf2_trans_b2s_.getRotation());
      double roll, pitch, yaw;
      m.getRPY(roll, _sensor_mount_angle, yaw);
      
      //@ we got pitch, remove pitch in the tf
      tf2::Quaternion zero_pitch;
      zero_pitch.setRPY(0, 0, yaw);
      tf2_trans_b2s_.setRotation(zero_pitch);
      trans_b2s_.transform.rotation.x = tf2_trans_b2s_.getRotation().x();
      trans_b2s_.transform.rotation.y = tf2_trans_b2s_.getRotation().y();
      trans_b2s_.transform.rotation.z = tf2_trans_b2s_.getRotation().z();
      trans_b2s_.transform.rotation.w = tf2_trans_b2s_.getRotation().w();

      //@ pubish a static tf for a frame that removing pitch
      geometry_msgs::msg::TransformStamped trans_sensor2sensor_no_pitch;
      trans_sensor2sensor_no_pitch.header.frame_id = sensor_frame;
      trans_sensor2sensor_no_pitch.child_frame_id = sensor_frame+"_pitch_removed";
      trans_sensor2sensor_no_pitch.transform.translation.x = 0.0; trans_sensor2sensor_no_pitch.transform.translation.y = 0.0; trans_sensor2sensor_no_pitch.transform.translation.z = 0.0;
      tf2::Quaternion compensate_pitch;
      compensate_pitch.setRPY(0, -1.0*_sensor_mount_angle, 0);
      trans_sensor2sensor_no_pitch.transform.rotation.x = compensate_pitch.x();
      trans_sensor2sensor_no_pitch.transform.rotation.y = compensate_pitch.y();
      trans_sensor2sensor_no_pitch.transform.rotation.z = compensate_pitch.z();
      trans_sensor2sensor_no_pitch.transform.rotation.w = compensate_pitch.w();
      
      tf_static_broadcaster_->sendTransform(trans_sensor2sensor_no_pitch);


      // camera to sensor
      tf2::Quaternion qc2s;
      qc2s.setRPY(0,-1.570795,-1.570795);
      tf2_trans_c2s_.setOrigin(tf2::Vector3(0, 0, 0));
      tf2_trans_c2s_.setRotation(qc2s);
      
      //camera to base_link/base_footprint
      tf2_trans_c2b_.mult(tf2_trans_c2s_, tf2_trans_b2s_.inverse());
      got_baselink2sensor_tf_= true;


      trans_c2s_.transform.translation.x = tf2_trans_c2s_.getOrigin().x(); 
      trans_c2s_.transform.translation.y = tf2_trans_c2s_.getOrigin().y(); 
      trans_c2s_.transform.translation.z = tf2_trans_c2s_.getOrigin().z();
      trans_c2s_.transform.rotation.x = tf2_trans_c2s_.getRotation().x();
      trans_c2s_.transform.rotation.y = tf2_trans_c2s_.getRotation().y();
      trans_c2s_.transform.rotation.z = tf2_trans_c2s_.getRotation().z();
      trans_c2s_.transform.rotation.w = tf2_trans_c2s_.getRotation().w();
      
      trans_c2b_.child_frame_id = baselink_frame_;
      trans_c2b_.transform.translation.x = tf2_trans_c2b_.getOrigin().x(); 
      trans_c2b_.transform.translation.y = tf2_trans_c2b_.getOrigin().y(); 
      trans_c2b_.transform.translation.z = tf2_trans_c2b_.getOrigin().z();
      trans_c2b_.transform.rotation.x = tf2_trans_c2b_.getRotation().x();
      trans_c2b_.transform.rotation.y = tf2_trans_c2b_.getRotation().y();
      trans_c2b_.transform.rotation.z = tf2_trans_c2b_.getRotation().z();
      trans_c2b_.transform.rotation.w = tf2_trans_c2b_.getRotation().w();
      return true;
    }
    catch (tf2::TransformException& e)
    {
      RCLCPP_ERROR(this->get_logger(), "Could not get footprint frame to sensor frame, did you launch a static broadcaster node for the tf between footprint to sensor?");
      return false;
    }
  }
  else{
    return true;
  }
}

void YoloImageProjection::cloudHandler(
    const sensor_msgs::msg::PointCloud2::SharedPtr laserCloudMsg){

  if(!allEssentialTFReady(laserCloudMsg->header.frame_id))
    return;

  resetParameters();

  // Copy and remove NAN points
  pcl::fromROSMsg(*laserCloudMsg, *_laser_cloud_in);
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*_laser_cloud_in, *_laser_cloud_in, indices);

  //@if not stitch, save copy time
  pcl::PointCloud<PointType>::Ptr pcl_stitched_msg (new pcl::PointCloud<PointType>);
  if(stitcher_num_<=0){
  }
  else{
    if(pcl_stitcher_.size()<stitcher_num_){
      pcl_stitcher_.push_back(*_laser_cloud_in);
    }
    else{
      pcl_stitcher_.pop_front();
      pcl_stitcher_.push_back(*_laser_cloud_in);
    }
    
    for(auto si=pcl_stitcher_.begin(); si!=pcl_stitcher_.end();si++){
      *pcl_stitched_msg += (*si);
    }
    *_laser_cloud_in = *pcl_stitched_msg;
  }

  _seg_msg.header = laserCloudMsg->header;
  _seg_msg.header.stamp = laserCloudMsg->header.stamp;
  _seg_msg.header.frame_id = laserCloudMsg->header.frame_id+"_pitch_removed";

  // transform tilted lidar back to horizontal
  
  geometry_msgs::msg::TransformStamped trans_lidar2horizontal;
  tf2::Quaternion q;
  q.setRPY( 0, _sensor_mount_angle, 0);
  trans_lidar2horizontal.transform.rotation.x = q.x(); trans_lidar2horizontal.transform.rotation.y = q.y();
  trans_lidar2horizontal.transform.rotation.z = q.z(); trans_lidar2horizontal.transform.rotation.w = q.w();
  Eigen::Affine3d trans_lidar2horizontal_af3 = tf2::transformToEigen(trans_lidar2horizontal);
  pcl::transformPointCloud(*_laser_cloud_in, *_laser_cloud_in, trans_lidar2horizontal_af3);
  
  findStartEndAngle();
  // Range image projection
  projectPointCloud();
  // Mark ground points
  groundRemoval();
  // Point cloud segmentation
  cloudSegmentation();
  //publish (optionally)
  publishClouds();
}


void YoloImageProjection::projectPointCloud() {
  
  //cv image
  cv::Mat projected_image(_vertical_scans, _horizontal_scans, CV_8UC1, cv::Scalar(0));
  
  // range image projection
  const size_t cloudSize = _laser_cloud_in->points.size();

  //if(_vertical_scans*_horizontal_scans!=_laser_cloud_in->points.size())
  //  RCLCPP_ERROR(this->get_logger(), "Number of lidar points %lu does not match the %d*%d", _laser_cloud_in->points.size(), _vertical_scans, _horizontal_scans);

  for (size_t i = 0; i < cloudSize; ++i) {
    PointType thisPoint = _laser_cloud_in->points[i];

    float range = sqrt(thisPoint.x * thisPoint.x +
                       thisPoint.y * thisPoint.y +
                       thisPoint.z * thisPoint.z);

    // find the row and column index in the image for this point
    float verticalAngle = std::asin(thisPoint.z / range);
        //std::atan2(thisPoint.z, sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y));

    int rowIdn = (verticalAngle + _ang_bottom) / _ang_resolution_Y;
    if (rowIdn < 0 || rowIdn >= _vertical_scans) {
      continue;
    }

    float horizonAngle = std::atan2(thisPoint.x, thisPoint.y);
    
    //if(horizonAngle>=0.7854 && horizonAngle<=1.57+0.7854 && range<6.0){
    //  continue;
    //}

    int columnIdn = -round((horizonAngle) / _ang_resolution_X) + _horizontal_scans * 0.5;

    if (columnIdn >= _horizontal_scans){
      columnIdn -= _horizontal_scans;
    }

    if (columnIdn < 0 || columnIdn >= _horizontal_scans){
      continue;
    }

    if (range < _minimum_detection_range || range > _maximum_detection_range){
      continue;
    }

    _range_mat(rowIdn, columnIdn) = range;
    
    //@ generate projected_image
    //@ the rowIdn for _range_mat is from top to bottom, which means the line 0 is the first row
    //@ to visualize the depth image more intuitively, we make line 0 to the last rowIdn
    // for columnIdn, we need to rotate it 180 degree
    int viscolumnIdn = -round((horizonAngle) / _ang_resolution_X) + _horizontal_scans * 0.5 + _horizontal_scans * 0.25;
    if(viscolumnIdn>=_horizontal_scans)
      viscolumnIdn = viscolumnIdn - _horizontal_scans;
    if(rowIdn>0 && rowIdn<=_vertical_scans && viscolumnIdn<_horizontal_scans && viscolumnIdn>=0)
      projected_image.at<unsigned char>(_vertical_scans-rowIdn, viscolumnIdn) = static_cast<unsigned char>(range*51);

    thisPoint.intensity = (float)rowIdn + (float)columnIdn / 10000.0;
    size_t index = columnIdn + rowIdn * _horizontal_scans;
    _full_cloud->points[index] = thisPoint;
    // the corresponding range of a point is saved as "intensity"
    _full_info_cloud->points[index] = thisPoint;
    _full_info_cloud->points[index].intensity = range;
  }

  cv_bridge::CvImage img_bridge;
  img_bridge.image = projected_image;
  img_bridge.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
  sensor_msgs::msg::Image::SharedPtr msg = img_bridge.toImageMsg();
  _pub_projected_image->publish(*msg);


  cv::Mat colorImage;
  cv::cvtColor(projected_image, colorImage, cv::COLOR_GRAY2BGR);
  // Run inference
  const auto objects = yolov8_->detectObjects(colorImage);

  // Draw the bounding boxes on the image
  yolov8_->drawObjectLabels(colorImage, objects);

  cv_bridge::CvImage img_annotated;
  img_annotated.image = colorImage;
  img_annotated.encoding = sensor_msgs::image_encodings::TYPE_8UC3;
  sensor_msgs::msg::Image::SharedPtr ros2_annotated_img = img_annotated.toImageMsg();
  pub_annotated_img_->publish(*ros2_annotated_img);

  //@ write depth img
  double imge_time = _seg_msg.header.stamp.sec + _seg_msg.header.stamp.nanosec/1e9;

  if(!to_fa_ && imge_time - last_save_depth_img_time_ >= time_step_between_depth_image_){
    /*
    cv::Mat mat_color_8 = cv::Mat(projected_image.rows, projected_image.cols, CV_8UC3);   // container for false-color version

    for (int di=0; di<projected_image.rows; di++){
      for (int dj=0; dj<projected_image.cols; dj++){
        ushort val = projected_image.at<ushort>(di,dj);
        mat_color_8.at<cv::Vec3b>(di,dj)[0] = val;
        mat_color_8.at<cv::Vec3b>(di,dj)[1] = val;
        mat_color_8.at<cv::Vec3b>(di,dj)[2] = val;
      }
    }
    */
    cv::Mat colorImage;
    cv::cvtColor(projected_image, colorImage, cv::COLOR_GRAY2BGR);
    std::string timestamp;
    std::stringstream ss;
    ss << _seg_msg.header.stamp.sec << "_" << std::setw(9) << std::setfill('0') << _seg_msg.header.stamp.nanosec;
    timestamp = ss.str();
    std::string file_name = mapping_dir_string_ + "/" + timestamp + ".png";
    cv::imwrite(file_name, colorImage);
    last_save_depth_img_time_ = imge_time;
  }

}

void YoloImageProjection::findStartEndAngle() {
  // start and end orientation of this cloud
  auto point = _laser_cloud_in->points.front();
  _seg_msg.start_orientation = -std::atan2(point.y, point.x);

  point = _laser_cloud_in->points.back();
  _seg_msg.end_orientation = -std::atan2(point.y, point.x) + 2 * M_PI;

  if (_seg_msg.end_orientation - _seg_msg.start_orientation > 3 * M_PI) {
    _seg_msg.end_orientation -= 2 * M_PI;
  } else if (_seg_msg.end_orientation - _seg_msg.start_orientation < M_PI) {
    _seg_msg.end_orientation += 2 * M_PI;
  }
  _seg_msg.orientation_diff =
      _seg_msg.end_orientation - _seg_msg.start_orientation;
}

void YoloImageProjection::groundRemoval() {
  // _ground_mat
  // -1, no valid info to check if ground of not
  //  0, initial value, after validation, means not ground
  //  1, ground
  patched_ground_->points.clear();
  patched_ground_edge_->points.clear();
  for (size_t j = 0; j < _horizontal_scans; ++j) {
    size_t ring_edge = 0;
    size_t closest_ring_edge = _ground_scan_index;
    bool do_patch = false;
    for (size_t i = 0; i < _ground_scan_index; ++i) {
      size_t lowerInd = j + (i)*_horizontal_scans;
      size_t upperInd = j + (i + 1) * _horizontal_scans;

      if (_full_cloud->points[lowerInd].intensity == -1 ||
          _full_cloud->points[upperInd].intensity == -1) {
        // no info to check, invalid points
        _ground_mat(i, j) = -1;
        continue;
      }

      float dX =
          _full_cloud->points[upperInd].x - _full_cloud->points[lowerInd].x;
      float dY =
          _full_cloud->points[upperInd].y - _full_cloud->points[lowerInd].y;
      float dZ =
          _full_cloud->points[upperInd].z - _full_cloud->points[lowerInd].z;

      float vertical_angle = std::atan2(dZ , sqrt(dX * dX + dY * dY + dZ * dZ));

      // TODO: review this change

      if ( (vertical_angle + _sensor_mount_angle) <= 10 * DEG_TO_RAD) {
        _ground_mat(i, j) = 1;
        _ground_mat(i + 1, j) = 1;
        //x = _full_cloud->points[lowerInd].x + dX*t
        //y = _full_cloud->points[lowerInd].y + dY*t
        //z = _full_cloud->points[lowerInd].z + dZ*t
        if(i<closest_ring_edge && i!=_ground_scan_index) //we dont casting the last one
          closest_ring_edge = i;

        float ds = sqrt(dX*dX + dY*dY + dZ*dZ);
        
        //@ if distance between is too large, we do not patch, because of too many unknown betweem rings
        if(ds<distance_for_patch_between_rings_){
          ring_edge = i+1;
          float dt = 1.0/(ds/0.1+1);
          for(float t=0; t<=1.0; t+=dt){
            PointType a_pt;
            a_pt.intensity = 0.0;
            a_pt.x = _full_cloud->points[lowerInd].x + dX*t;
            a_pt.y = _full_cloud->points[lowerInd].y + dY*t;
            a_pt.z = _full_cloud->points[lowerInd].z + dZ*t;
            patched_ground_->push_back(a_pt);
          }
          PointType a_pt;
          a_pt.intensity = 0.0;
          a_pt.x = _full_cloud->points[lowerInd].x + dX;
          a_pt.y = _full_cloud->points[lowerInd].y + dY;
          a_pt.z = _full_cloud->points[lowerInd].z + dZ;
          patched_ground_->push_back(a_pt);
          do_patch = true;
        }
      }
    }
    size_t ringEdgeInd = j + ring_edge*_horizontal_scans;
    PointType a_pt;
    a_pt.x = _full_cloud->points[ringEdgeInd].x;
    a_pt.y = _full_cloud->points[ringEdgeInd].y;
    a_pt.z = _full_cloud->points[ringEdgeInd].z;
    a_pt.intensity = 100;
    patched_ground_edge_->push_back(a_pt);

    if(do_patch && first_frame_processed_<5 && closest_ring_edge < _ground_scan_index){
      //@ patch ground from closest ring edge to base_link
      size_t closest_ring_edgeInd = j + (closest_ring_edge)*_horizontal_scans;
      float dXf = -
          _full_cloud->points[closest_ring_edgeInd].x;
      float dYf = -
          _full_cloud->points[closest_ring_edgeInd].y;
      float dZf = -
          _full_cloud->points[closest_ring_edgeInd].z;

      for(float t=0; t<=1.0; t+=0.05){
        PointType a_ptf;
        a_ptf.intensity = 0.0;
        a_ptf.x = _full_cloud->points[closest_ring_edgeInd].x + dXf*t;
        a_ptf.y = _full_cloud->points[closest_ring_edgeInd].y + dYf*t;
        a_ptf.z = _full_cloud->points[closest_ring_edgeInd].z; // mitigate height difference
        patched_ground_->push_back(a_ptf);
      }
      PointType a_ptf;
      a_ptf.intensity = 0.0;
      a_ptf.x = _full_cloud->points[closest_ring_edgeInd].x + dXf;
      a_ptf.y = _full_cloud->points[closest_ring_edgeInd].y + dYf;
      a_ptf.z = _full_cloud->points[closest_ring_edgeInd].z;
      patched_ground_->push_back(a_ptf);
    }
  }
  //@ we have ring edge, mark intensity for those edge points


  dsf_patched_ground_.setInputCloud(patched_ground_);
  dsf_patched_ground_.filter(*patched_ground_);
  dsf_patched_ground_.setInputCloud(patched_ground_edge_);
  dsf_patched_ground_.filter(*patched_ground_edge_); 
  // extract ground cloud (_ground_mat == 1)
  // mark entry that doesn't need to label (ground and invalid point) for
  // segmentation note that ground remove is from 0~_N_scan-1, need _range_mat
  // for mark label matrix for the 16th scan
  for (size_t i = 0; i < _vertical_scans; ++i) {
    for (size_t j = 0; j < _horizontal_scans; ++j) {
      if (_ground_mat(i, j) == 1 ||
          _range_mat(i, j) == FLT_MAX) {
        _label_mat(i, j) = -1;
      }
    }
  }

  for (size_t i = 0; i <= _ground_scan_index; ++i) {
    for (size_t j = 0; j < _horizontal_scans; ++j) {
      if (_ground_mat(i, j) == 1)
        _ground_cloud->push_back(_full_cloud->points[j + i * _horizontal_scans]);
    }
  }
}

void YoloImageProjection::cloudSegmentation() {
  // segmentation process
  for (size_t i = 0; i < _vertical_scans; ++i)
    for (size_t j = 0; j < _horizontal_scans; ++j)
      if (_label_mat(i, j) == 0) labelComponents(i, j);

  int sizeOfSegCloud = 0;
  // extract segmented cloud for lidar odometry
  for (size_t i = 0; i < _vertical_scans; ++i) {
    _seg_msg.start_ring_index[i] = sizeOfSegCloud - 1 + 5;

    for (size_t j = 0; j < _horizontal_scans; ++j) {
      if (_label_mat(i, j) > 0 || _ground_mat(i, j) == 1) {
        // outliers that will not be used for optimization (always continue)
        if (_label_mat(i, j) == 999999) {
          if (i > _ground_scan_index && j % 5 == 0) {
            _outlier_cloud->push_back(
                _full_cloud->points[j + i * _horizontal_scans]);
            continue;
          } else {
            continue;
          }
        }
        // majority of ground points are skipped
        if (_ground_mat(i, j) == 1) {
          if (j % 5 != 0 && j > 5 && j < _horizontal_scans - 5) continue;
        }
        // mark ground points so they will not be considered as edge features
        // later
        _seg_msg.segmented_cloud_ground_flag[sizeOfSegCloud] =
            (_ground_mat(i, j) == 1);
        // mark the points' column index for marking occlusion later
        _seg_msg.segmented_cloud_col_ind[sizeOfSegCloud] = j;
        // save range info
        _seg_msg.segmented_cloud_range[sizeOfSegCloud] =
            _range_mat(i, j);
        // save seg cloud
        _segmented_cloud->push_back(_full_cloud->points[j + i * _horizontal_scans]);
        // size of seg cloud
        ++sizeOfSegCloud;
      }
    }

    _seg_msg.end_ring_index[i] = sizeOfSegCloud - 1 - 5;
  }

  // extract segmented cloud for visualization
  for (size_t i = 0; i < _vertical_scans; ++i) {
    for (size_t j = 0; j < _horizontal_scans; ++j) {
      if (_label_mat(i, j) > 0 && _label_mat(i, j) != 999999) {
        _segmented_cloud_pure->push_back(
            _full_cloud->points[j + i * _horizontal_scans]);
        _segmented_cloud_pure->points.back().intensity =
            _label_mat(i, j);
      }
    }
  }
}

void YoloImageProjection::labelComponents(int row, int col) {

  const float segmentThetaThreshold = tan(_segment_theta);

  std::vector<bool> lineCountFlag(_vertical_scans, false);
  const size_t cloud_size = _vertical_scans * _horizontal_scans;
  using Coord2D = Eigen::Vector2i;
  boost::circular_buffer<Coord2D> queue(cloud_size);
  boost::circular_buffer<Coord2D> all_pushed(cloud_size);

  queue.push_back({ row,col } );
  all_pushed.push_back({ row,col } );

  const Coord2D neighborIterator[4] = {
      {0, -1}, {-1, 0}, {1, 0}, {0, 1}};

  while (queue.size() > 0) {
    // Pop point
    Coord2D fromInd = queue.front();
    queue.pop_front();

    // Mark popped point
    _label_mat(fromInd.x(), fromInd.y()) = _label_count;
    // Loop through all the neighboring grids of popped grid

    for (const auto& iter : neighborIterator) {
      // new index
      int thisIndX = fromInd.x() + iter.x();
      int thisIndY = fromInd.y() + iter.y();
      // index should be within the boundary
      if (thisIndX < 0 || thisIndX >= _vertical_scans){
        continue;
      }
      // at range image margin (left or right side)
      if (thisIndY < 0){
        thisIndY = _horizontal_scans - 1;
      }
      if (thisIndY >= _horizontal_scans){
        thisIndY = 0;
      }
      // prevent infinite loop (caused by put already examined point back)
      if (_label_mat(thisIndX, thisIndY) != 0){
        continue;
      }

      float d1 = std::max(_range_mat(fromInd.x(), fromInd.y()),
                    _range_mat(thisIndX, thisIndY));
      float d2 = std::min(_range_mat(fromInd.x(), fromInd.y()),
                    _range_mat(thisIndX, thisIndY));

      float alpha = (iter.x() == 0) ? _ang_resolution_X : _ang_resolution_Y;
      float tang = (d2 * sin(alpha) / (d1 - d2 * cos(alpha)));

      if (tang > segmentThetaThreshold) {
        queue.push_back( {thisIndX, thisIndY } );

        _label_mat(thisIndX, thisIndY) = _label_count;
        lineCountFlag[thisIndX] = true;

        all_pushed.push_back(  {thisIndX, thisIndY } );
      }
    }
  }

  // check if this segment is valid
  bool feasibleSegment = false;
  if (all_pushed.size() >= 30){
    feasibleSegment = true;
  }
  else if (all_pushed.size() >= _segment_valid_point_num) {
    int lineCount = 0;
    for (size_t i = 0; i < _vertical_scans; ++i) {
      if (lineCountFlag[i] == true) ++lineCount;
    }
    if (lineCount >= _segment_valid_line_num) feasibleSegment = true;
  }
  // segment is valid, mark these points
  if (feasibleSegment == true) {
    ++_label_count;
  } else {  // segment is invalid, mark these points
    for (size_t i = 0; i < all_pushed.size(); ++i) {
      _label_mat(all_pushed[i].x(), all_pushed[i].y()) = 999999;
    }
  }
}

void YoloImageProjection::publishClouds() {
  const auto& cloudHeader = _seg_msg.header;

  sensor_msgs::msg::PointCloud2 laserCloudTemp;

  auto PublishCloud = [&](rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub,
                          const pcl::PointCloud<PointType>::Ptr& cloud) {
    if (true) {
      pcl::toROSMsg(*cloud, laserCloudTemp);
      laserCloudTemp.header = cloudHeader;
      laserCloudTemp.header.stamp = clock_->now();
      pub->publish(laserCloudTemp);
    }
  };

  //PublishCloud(_pub_outlier_cloud, _outlier_cloud);
  //PublishCloud(_pub_segmented_cloud, _segmented_cloud);
  PublishCloud(_pub_ground_cloud, patched_ground_);
  PublishCloud(_pub_segmented_cloud_pure, _segmented_cloud_pure);
  //PublishCloud(_pub_full_info_cloud, _full_info_cloud);

  if (_pub_segmented_cloud_info->get_subscription_count() != 0) {
    _pub_segmented_cloud_info->publish(_seg_msg);
  }

  //--------------------
  ProjectionOut out;
  out.outlier_cloud.reset(new pcl::PointCloud<PointType>());
  out.segmented_cloud.reset(new pcl::PointCloud<PointType>());
  out.patched_ground.reset(new pcl::PointCloud<PointType>());
  out.patched_ground_edge.reset(new pcl::PointCloud<PointType>());
  out.trans_c2s = trans_c2s_;
  out.trans_c2b = trans_c2b_;
  out.trans_b2s = trans_b2s_;
  out.odom_type = odom_type_;
  out.vertical_scans = _vertical_scans;
  out.horizontal_scans = _horizontal_scans;
  out.scan_period = _scan_period;

  std::swap(out.seg_msg, _seg_msg);
  std::swap(out.outlier_cloud, _outlier_cloud);
  std::swap(out.segmented_cloud, _segmented_cloud);
  std::swap(out.patched_ground, patched_ground_);
  std::swap(out.patched_ground_edge, patched_ground_edge_);
  if(to_fa_)
    _output_channel.send( std::move(out) );
  first_frame_processed_++;

}


