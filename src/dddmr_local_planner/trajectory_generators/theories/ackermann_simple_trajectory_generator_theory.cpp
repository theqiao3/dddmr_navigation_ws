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

#include <trajectory_generators/ackermann_simple_trajectory_generator_theory.h>
#include <rclcpp/rclcpp.hpp>

PLUGINLIB_EXPORT_CLASS(
  trajectory_generators::AckermannSimpleTrajectoryGeneratorTheory,
  trajectory_generators::TrajectoryGeneratorTheory)

namespace trajectory_generators {

AckermannSimpleTrajectoryGeneratorTheory::AckermannSimpleTrajectoryGeneratorTheory()
  : next_sample_index_(0) {
}

void AckermannSimpleTrajectoryGeneratorTheory::onInitialize() {
  // 初始化限制参数
  limits_ = std::make_shared<AckermannTrajectoryGeneratorLimits>();
  
  node_->declare_parameter(name_ + ".min_vel_x", rclcpp::ParameterValue(0.1));
  node_->get_parameter(name_ + ".min_vel_x", limits_->min_vel_x);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "min_vel_x: %.2f", limits_->min_vel_x);

  node_->declare_parameter(name_ + ".max_vel_x", rclcpp::ParameterValue(1.2));
  node_->get_parameter(name_ + ".max_vel_x", limits_->max_vel_x);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "max_vel_x: %.2f", limits_->max_vel_x);

  node_->declare_parameter(name_ + ".min_steering_angle", rclcpp::ParameterValue(-0.436));
  node_->get_parameter(name_ + ".min_steering_angle", limits_->min_steering_angle);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "min_steering_angle: %.4f rad (%.1f deg)",
    limits_->min_steering_angle, limits_->min_steering_angle * 180.0 / M_PI);

  node_->declare_parameter(name_ + ".max_steering_angle", rclcpp::ParameterValue(0.436));
  node_->get_parameter(name_ + ".max_steering_angle", limits_->max_steering_angle);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "max_steering_angle: %.4f rad (%.1f deg)",
    limits_->max_steering_angle, limits_->max_steering_angle * 180.0 / M_PI);

  node_->declare_parameter(name_ + ".acc_lim_x", rclcpp::ParameterValue(1.0));
  node_->get_parameter(name_ + ".acc_lim_x", limits_->acc_lim_x);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "acc_lim_x: %.2f", limits_->acc_lim_x);

  node_->declare_parameter(name_ + ".acc_lim_steering", rclcpp::ParameterValue(1.0));
  node_->get_parameter(name_ + ".acc_lim_steering", limits_->acc_lim_steering);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "acc_lim_steering: %.2f", limits_->acc_lim_steering);

  node_->declare_parameter(name_ + ".deceleration_ratio", rclcpp::ParameterValue(2.0));
  node_->get_parameter(name_ + ".deceleration_ratio", limits_->deceleration_ratio);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "deceleration_ratio: %.2f", limits_->deceleration_ratio);

  // Ackermann特有参数
  node_->declare_parameter(name_ + ".wheelbase", rclcpp::ParameterValue(0.45));
  node_->get_parameter(name_ + ".wheelbase", limits_->wheelbase);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "wheelbase: %.3f m", limits_->wheelbase);

  node_->declare_parameter(name_ + ".track_width", rclcpp::ParameterValue(0.3));
  node_->get_parameter(name_ + ".track_width", limits_->track_width);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "track_width: %.3f m", limits_->track_width);

  node_->declare_parameter(name_ + ".robot_radius", rclcpp::ParameterValue(0.25));
  node_->get_parameter(name_ + ".robot_radius", limits_->robot_radius);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "robot_radius: %.2f", limits_->robot_radius);

  // 初始化规划参数
  params_ = std::make_shared<AckermannTrajectoryGeneratorParams>();

  node_->declare_parameter(name_ + ".controller_frequency", rclcpp::ParameterValue(10.0));
  node_->get_parameter(name_ + ".controller_frequency", params_->controller_frequency);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "controller_frequency: %.2f", params_->controller_frequency);

  node_->declare_parameter(name_ + ".sim_time", rclcpp::ParameterValue(1.0));
  node_->get_parameter(name_ + ".sim_time", params_->sim_time);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "sim_time: %.2f", params_->sim_time);

  node_->declare_parameter(name_ + ".linear_x_sample", rclcpp::ParameterValue(5));
  node_->get_parameter(name_ + ".linear_x_sample", params_->linear_x_sample);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "linear_x_sample: %d", params_->linear_x_sample);

  node_->declare_parameter(name_ + ".steering_angle_sample", rclcpp::ParameterValue(7));
  node_->get_parameter(name_ + ".steering_angle_sample", params_->steering_angle_sample);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "steering_angle_sample: %d", params_->steering_angle_sample);

  node_->declare_parameter(name_ + ".sim_granularity", rclcpp::ParameterValue(0.1));
  node_->get_parameter(name_ + ".sim_granularity", params_->sim_granularity);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "sim_granularity: %.2f", params_->sim_granularity);

  // 解析立方体
  RCLCPP_INFO(node_->get_logger().get_child(name_), "Start to parse cuboid.");
  std::vector<double> p;
  
  std::vector<std::string> cuboid_vertices = {
    "flb", "frb", "flt", "frt", "blb", "brb", "blt", "brt"
  };
  
  for (const auto& vertex : cuboid_vertices) {
    std::string param_name = name_ + ".cuboid." + vertex;
    node_->declare_parameter(param_name, rclcpp::PARAMETER_DOUBLE_ARRAY);
    rclcpp::Parameter cuboid_param = node_->get_parameter(param_name);
    p = cuboid_param.as_double_array();
    
    pcl::PointXYZ pt;
    pt.x = p[0]; pt.y = p[1]; pt.z = p[2];
    limits_->cuboid.push_back(pt);
    RCLCPP_INFO(node_->get_logger().get_child(name_), 
      "Cuboid %s: %.2f, %.2f, %.2f", vertex.c_str(), pt.x, pt.y, pt.z);
  }

  if (limits_->cuboid.size() != 8) {
    RCLCPP_FATAL(node_->get_logger().get_child(name_), "Cuboid is essential (need 8 vertices).");
  }
}

void AckermannSimpleTrajectoryGeneratorTheory::initialise() {
  next_sample_index_ = 0;
  sample_params_.clear();

  double min_vel_x = limits_->min_vel_x;
  double max_vel_x = limits_->max_vel_x;
  double min_steering = limits_->min_steering_angle;
  double max_steering = limits_->max_steering_angle;

  // 根据当前速度和加速度限制调整最大/最小速度
  double sim_period = 1.0 / params_->controller_frequency;
  
  if (shared_data_->current_allowed_max_linear_speed_ > 0.0) {
    max_vel_x = std::min(max_vel_x, shared_data_->current_allowed_max_linear_speed_);
  }

  double current_vel_x = shared_data_->robot_state_.twist.twist.linear.x;
  double max_vel_x_accl = current_vel_x + limits_->acc_lim_x * sim_period;
  double min_vel_x_decel = current_vel_x / limits_->deceleration_ratio;

  max_vel_x = std::min(max_vel_x, max_vel_x_accl);
  min_vel_x = std::max(min_vel_x, min_vel_x_decel);

  if (max_vel_x < min_vel_x) {
    min_vel_x = current_vel_x / limits_->deceleration_ratio;
    max_vel_x = current_vel_x / limits_->deceleration_ratio;
  }

  // 采样速度和转向角
  for (int i = 0; i < params_->linear_x_sample; ++i) {
    double v = min_vel_x + (max_vel_x - min_vel_x) * i / 
      std::max(1, params_->linear_x_sample - 1);
    
    for (int j = 0; j < params_->steering_angle_sample; ++j) {
      double delta = min_steering + (max_steering - min_steering) * j /
        std::max(1, params_->steering_angle_sample - 1);
      
      if (checkConstraints(v, delta)) {
        sample_params_.push_back(std::make_pair(v, delta));
      }
    }
  }

  RCLCPP_DEBUG(node_->get_logger().get_child(name_),
    "Generated %lu trajectory samples", sample_params_.size());
}

bool AckermannSimpleTrajectoryGeneratorTheory::checkConstraints(double v, double delta) {
  // 检查速度和转向角是否在约束范围内
  if (v < limits_->min_vel_x || v > limits_->max_vel_x) {
    return false;
  }
  
  if (delta < limits_->min_steering_angle || delta > limits_->max_steering_angle) {
    return false;
  }
  
  return true;
}

bool AckermannSimpleTrajectoryGeneratorTheory::hasMoreTrajectories() {
  if (sample_params_.empty()) {
    initialise();
  }
  return next_sample_index_ < sample_params_.size();
}

bool AckermannSimpleTrajectoryGeneratorTheory::nextTrajectory(base_trajectory::Trajectory& traj) {
  if (!hasMoreTrajectories()) {
    return false;
  }

  auto [v, delta] = sample_params_[next_sample_index_];
  next_sample_index_++;

  return generateTrajectory(v, delta, traj);
}

void AckermannSimpleTrajectoryGeneratorTheory::propagateTrajectory(
  double& x, double& y, double& theta,
  double v, double delta, double dt) {
  
  // 使用中值法进行积分（提高精度）
  const double L = limits_->wheelbase;
  
  double tan_delta = std::tan(delta);
  double theta_mid = theta + v * tan_delta / L * dt / 2.0;
  
  x += v * std::cos(theta_mid) * dt;
  y += v * std::sin(theta_mid) * dt;
  theta += v * tan_delta / L * dt;
  
  // 归一化角度
  while (theta > M_PI) theta -= 2.0 * M_PI;
  while (theta < -M_PI) theta += 2.0 * M_PI;
}

bool AckermannSimpleTrajectoryGeneratorTheory::pointInCuboid(
  const pcl::PointXYZ& pt,
  const Eigen::Vector3f& transform,
  const pcl::PointCloud<pcl::PointXYZ>& cuboid) const {
  
  // 将点转换到立方体坐标系
  pcl::PointXYZ local_pt;
  local_pt.x = pt.x - transform[0];
  local_pt.y = pt.y - transform[1];
  local_pt.z = pt.z - transform[2];
  
  // 获取立方体边界
  if (cuboid.size() < 8) return false;
  
  double min_x = std::min({cuboid[0].x, cuboid[1].x, cuboid[2].x, cuboid[3].x,
                           cuboid[4].x, cuboid[5].x, cuboid[6].x, cuboid[7].x});
  double max_x = std::max({cuboid[0].x, cuboid[1].x, cuboid[2].x, cuboid[3].x,
                           cuboid[4].x, cuboid[5].x, cuboid[6].x, cuboid[7].x});
  
  double min_y = std::min({cuboid[0].y, cuboid[1].y, cuboid[2].y, cuboid[3].y,
                           cuboid[4].y, cuboid[5].y, cuboid[6].y, cuboid[7].y});
  double max_y = std::max({cuboid[0].y, cuboid[1].y, cuboid[2].y, cuboid[3].y,
                           cuboid[4].y, cuboid[5].y, cuboid[6].y, cuboid[7].y});
  
  double min_z = std::min({cuboid[0].z, cuboid[1].z, cuboid[2].z, cuboid[3].z,
                           cuboid[4].z, cuboid[5].z, cuboid[6].z, cuboid[7].z});
  double max_z = std::max({cuboid[0].z, cuboid[1].z, cuboid[2].z, cuboid[3].z,
                           cuboid[4].z, cuboid[5].z, cuboid[6].z, cuboid[7].z});
  
  return (local_pt.x >= min_x && local_pt.x <= max_x &&
          local_pt.y >= min_y && local_pt.y <= max_y &&
          local_pt.z >= min_z && local_pt.z <= max_z);
}

bool AckermannSimpleTrajectoryGeneratorTheory::generateTrajectory(
  double v, double delta,
  base_trajectory::Trajectory& traj) {

  traj.cost_ = 0.0;
  traj.resetPoints();

  traj.xv_ = v;
  traj.yv_ = 0.0;
  traj.thetav_ = v * std::tan(delta) / limits_->wheelbase;
  traj.time_delta_ = 1.0 / params_->controller_frequency;

  Eigen::Affine3d pos_af3 = tf2::transformToEigen(shared_data_->robot_pose_);

  // 本地坐标系中的轨迹（base_link）
  double x = 0.0, y = 0.0, theta = 0.0;
  double dt = 1.0 / params_->controller_frequency;
  int steps = std::max(1, static_cast<int>(std::ceil(params_->sim_time / dt)));

  // 生成轨迹点（转换到全局坐标系，并同步生成 cuboid）
  for (int i = 0; i < steps; ++i) {
    propagateTrajectory(x, y, theta, v, delta, dt);

    Eigen::Affine3d trans_b2traj_af3(Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ()));
    trans_b2traj_af3.translation().x() = x;
    trans_b2traj_af3.translation().y() = y;

    Eigen::Affine3d trans_gbl2traj_af3 = pos_af3 * trans_b2traj_af3;
    geometry_msgs::msg::TransformStamped trans_gbl2traj = tf2::eigenToTransform(trans_gbl2traj_af3);

    geometry_msgs::msg::PoseStamped ros_pose;
    ros_pose.header = shared_data_->robot_pose_.header;
    ros_pose.pose.position.x = trans_gbl2traj.transform.translation.x;
    ros_pose.pose.position.y = trans_gbl2traj.transform.translation.y;
    ros_pose.pose.position.z = trans_gbl2traj.transform.translation.z;
    ros_pose.pose.orientation = trans_gbl2traj.transform.rotation;

    pcl::PointCloud<pcl::PointXYZ> pc_out;
    pcl::transformPointCloud(limits_->cuboid, pc_out, trans_gbl2traj_af3);

    base_trajectory::cuboid_min_max_t cuboid_min_max;
    pcl::getMinMax3D(pc_out, cuboid_min_max.first, cuboid_min_max.second);

    if (!traj.addPoint(ros_pose, pc_out, cuboid_min_max)) {
      return false;
    }
  }
  
  // 添加评分（基于物理合理性）
  // 1. 转向角大的轨迹评分略高（因为效率低）
  traj.cost_ += 0.1 * std::abs(delta) / limits_->max_steering_angle;
  
  // 2. 速度低的轨迹评分略高（鼓励高速）
  traj.cost_ += 0.05 * (1.0 - v / limits_->max_vel_x);
  
  return true;
}

} // end namespace trajectory_generators
