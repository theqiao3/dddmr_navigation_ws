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

#ifndef ACKERMANN_SIMPLE_TRAJECTORY_GENERATOR_THEORY_H_
#define ACKERMANN_SIMPLE_TRAJECTORY_GENERATOR_THEORY_H_

#include <trajectory_generators/trajectory_generator_theory.h>
#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <cmath>
#include <vector>

namespace trajectory_generators {

// Ackermann轨迹生成器参数限制
struct AckermannTrajectoryGeneratorLimits {
  double min_vel_x;              // 最小线速度
  double max_vel_x;              // 最大线速度
  double min_steering_angle;     // 最小转向角 (弧度)
  double max_steering_angle;     // 最大转向角 (弧度)
  double acc_lim_x;              // 线性加速度限制
  double acc_lim_steering;       // 转向角加速度限制
  double deceleration_ratio;     // 减速比例
  double wheelbase;              // 轴距 (m)
  double track_width;            // 轮距 (m)
  double robot_radius;           // 机器人半径
  pcl::PointCloud<pcl::PointXYZ> cuboid;  // 碰撞检测立方体
};

// Ackermann轨迹生成器参数
struct AckermannTrajectoryGeneratorParams {
  double controller_frequency;   // 控制器频率
  double sim_time;               // 模拟时间
  int linear_x_sample;           // 线速度采样数
  int steering_angle_sample;     // 转向角采样数
  double sim_granularity;        // 模拟粒度
};

class AckermannSimpleTrajectoryGeneratorTheory : public TrajectoryGeneratorTheory {
public:
  AckermannSimpleTrajectoryGeneratorTheory();
  virtual ~AckermannSimpleTrajectoryGeneratorTheory() = default;

  virtual bool hasMoreTrajectories();
  virtual bool nextTrajectory(base_trajectory::Trajectory& _traj);

private:
  void initialise();
  
  // 阿克曼运动学传播
  void propagateTrajectory(
    double& x, double& y, double& theta,
    double v, double delta, double dt);
  
  // 生成单条轨迹
  bool generateTrajectory(
    double v, double delta,
    base_trajectory::Trajectory& traj);
  
  // 检查参数是否在约束范围内
  bool checkConstraints(double v, double delta);
  
  // 计算点是否在立方体内
  bool pointInCuboid(const pcl::PointXYZ& pt, const Eigen::Vector3f& transform,
    const pcl::PointCloud<pcl::PointXYZ>& cuboid) const;

protected:
  virtual void onInitialize();

  std::shared_ptr<AckermannTrajectoryGeneratorLimits> limits_;
  std::shared_ptr<AckermannTrajectoryGeneratorParams> params_;
  
  unsigned int next_sample_index_;
  // 存储采样参数 [v, delta]
  std::vector<std::pair<double, double>> sample_params_;
};

} // end namespace trajectory_generators

#endif // ACKERMANN_SIMPLE_TRAJECTORY_GENERATOR_THEORY_H_
