# Ackermann 机器人全栈调试与测试指南 (ROS2 Humble)

本文档为您提供基于 ROS2 Humble 的 Ackermann 转向机器人（搭载 MID360）的完整调试、测试配置和实施方案。

## 0. ROS2 Humble 兼容性分析

| 模块 | 对应包名 | ROS2 Humble 兼容性 | 说明 |
|------|---------|-------------------|------|
| **建图** | `dddmr_lego_loam` | ✅ **完全兼容** | 基于 LeGO-LOAM 的 ROS2 移植版，支持 MID360 点云格式。 |
| **定位** | `dddmr_mcl_3dl` | ✅ **完全兼容** | 3D 粒子滤波定位，ROS2 版本稳定，支持 IMU 融合。 |
| **规划** | `dddmr_global_planner`<br>`dddmr_local_planner` | ✅ **完全兼容** | 核心算法已适配 ROS2 接口。**注意**：局部规划器已针对 Ackermann 进行了定制开发。 |
| **控制** | `dddmr_p2p_move_base` | ✅ **完全兼容** | 状态机控制逻辑，使用 ROS2 Action 和 Service。 |

---

## 1. 建图模块 (Mapping)

### 方案概述
使用 **LeGO-LOAM** 算法进行 3D 点云建图。针对 MID360 激光雷达的特性（360度视场，非重复扫描）进行配置。

### 1.1 离线建图 (推荐)
**配置文件**: `src/dddmr_lego_loam/config/loam_bag_ackermann_mid360_config.yaml` (需确认或创建)

```yaml
lego_loam:
  ros__parameters:
    # 激光雷达配置 (MID360)
    laser:
      num_vertical_scans: 32
      num_horizontal_scans: 1800
      vertical_angle_bottom: -7.0  # 角度范围
      vertical_angle_top: 52.0
      sensor_mount_angle: 0.0
      ground_scan_index: 20
    
    # 建图特性
    featureAssociation:
      edge_threshold: 0.1
      surf_threshold: 0.1
```

### 1.2 在线建图 (实时)
如果您希望直接在机器人上运行实时建图，请使用以下步骤。

**配置文件**: `src/dddmr_lego_loam/lego_loam_bor/config/loam_ackermann_mid360_config.yaml`
**启动文件**: `src/dddmr_lego_loam/lego_loam_bor/launch/lego_loam_ackermann_mid360.launch.py`

**操作步骤**:

1.  **启动底层驱动**:
    启动机器人底盘驱动（发布 `/odom`）和 MID360 驱动（发布 `/livox/lidar` 和 `/livox/imu`）。
    ```bash
    # 示例
    ros2 launch livox_ros_driver2 msg_MID360_launch.py
    ```

2.  **启动在线建图**:
    ```bash
    ros2 launch lego_loam_bor lego_loam_ackermann_mid360.launch.py
    ```

3.  **保存地图**:
    建图完成后，调用服务保存地图。
    ```bash
    ros2 service call /map_save std_srvs/srv/Trigger
    ```

### 调试与测试步骤

1.  **启动雷达驱动**:
    确保 MID360 驱动正常运行，发布 `/livox/lidar` 或 `/points_raw` 话题。
    ```bash
    ros2 launch livox_ros_driver2 msg_MID360_launch.py
    ```

2.  **录制数据包 (推荐)**:
    在构建地图前，先录制一段包含回环（回到起点）的数据包。
    ```bash
    ros2 bag record -o map_data /livox/lidar /imu/data
    ```

3.  **运行建图**:
    ```bash
    ros2 launch dddmr_lego_loam loam_bag.launch.py config_file:=loam_bag_ackermann_mid360_config.yaml
    ```

4.  **保存地图**:
    建图完成后，保存 PCD 文件。
    ```bash
    ros2 service call /map_save std_srvs/srv/Trigger
    # 或使用 pcl_ros 工具保存
    ```

### 常见问题
*   **地图漂移**: 检查 IMU 外参是否准确，MID360 内部集成了 IMU，需确保时间同步。
*   **Z轴漂移**: 确保 `ground_scan_index` 设置正确，以便算法正确识别地面。

---

## 2. 定位模块 (Localization)

### 方案概述
使用 **MCL-3DL** (Monte Carlo Localization for 3D LiDAR) 进行定位。它结合了里程计（Odometry）、IMU 和 3D 点云地图进行粒子滤波定位。

### 关键配置
**配置文件**: `src/dddmr_mcl_3dl/config/ackermann_mid360_localization.yaml`

```yaml
mcl_3dl:
  ros__parameters:
    map_frame: "map"
    robot_frame: "base_link"
    odom_frame: "odom"
    
    # 粒子滤波器参数
    particle_num: 500          # 粒子数量
    resample_thresholds: 0.5   # 重采样阈值
    
    # 初始位置方差
    init_var_x: 0.5
    init_var_y: 0.5
    init_var_yaw: 0.1
    
    # 似然场计算 (匹配精度)
    dist_weight_x: 1.0
    dist_weight_y: 1.0
    dist_weight_z: 1.0
```

### 调试与测试步骤

1.  **启动定位节点**:
    ```bash
    ros2 launch dddmr_mcl_3dl mcl_3dl.launch.py config_file:=ackermann_mid360_localization.yaml
    ```

2.  **初始化位姿**:
    在 RViz 中使用 "2D Pose Estimate" 工具给定初始位置。

3.  **验证定位精度**:
    *   遥控机器人移动。
    *   观察 `/mcl_3dl/amcl_pose` 与激光雷达扫描 (`/scan_matched` 或类似) 与地图的重合度。
    *   检查 `/mcl_3dl/status` 中的 `convergence_status`。

### 常见问题
*   **粒子发散**: 初始位置给得太偏，或者里程计误差过大。
*   **定位延迟**: 减少 `particle_num` 或增加 `skip_measure`。

---

## 3. 规划模块 (Planning)

### 方案概述
*   **全局规划**: 使用 A* 算法在 3D 栅格地图或 2D 投影地图上规划路径。
*   **局部规划**: 使用 **定制的 Ackermann 轨迹生成器** (已实现) 替代默认的差分驱动规划器。

### 关键配置
**局部规划器配置**: `src/dddmr_local_planner/local_planner/config/ackermann_trajectory_generator.yaml`

```yaml
local_planner:
  ros__parameters:
    trajectory_generator_type: "trajectory_generators::AckermannSimpleTrajectoryGeneratorTheory"
    ackermann_simple_trajectory_generator_theory:
      min_vel_x: 0.1
      max_vel_x: 1.2
      min_steering_angle: -0.436  # -25度
      max_steering_angle: 0.436   # 25度
      wheelbase: 0.45
```

### 调试与测试步骤

1.  **编译更新后的规划器**:
    ```bash
    colcon build --packages-select dddmr_local_planner local_planner
    source install/setup.bash
    ```

2.  **启动规划与控制栈**:
    (需创建组合 launch 文件，或分别启动)

3.  **发布目标点**:
    在 RViz 中使用 "2D Nav Goal"。

4.  **监控局部轨迹**:
    在 RViz 中订阅 `/local_planner/trajectory` (或类似话题)，观察生成的轨迹是否符合 Ackermann 运动学（平滑曲线，无原地旋转）。

5.  **验证控制指令**:
    ```bash
    ros2 topic echo /cmd_vel
    ```
    检查 `angular.z` 是否符合预期（对于 Ackermann 机器人，通常底层驱动需要将 `angular.z` 转换为转向角，或者规划器直接发布包含转向角的自定义消息。**注意：本方案中规划器输出的是标准 Twist 消息，其中 angular.z = v * tan(delta) / L。底层驱动需要根据此公式反解出转向角 delta**）。

### 常见问题
*   **规划失败**: 检查 `cuboid` (碰撞盒) 设置是否过大，导致在狭窄空间无法通过。
*   **轨迹不平滑**: 调整 `acc_lim_steering` (转向加速度限制)。

---

## 4. 控制模块 (Control)

### 方案概述
`dddmr_p2p_move_base` 负责整体的任务调度、状态机管理和异常恢复。

### 关键配置
**配置文件**: `src/dddmr_p2p_move_base/config/p2p_move_base.yaml` (假设存在)

```yaml
p2p_move_base:
  ros__parameters:
    control_frequency: 10.0
    global_planner_timeout: 2.0
    recovery_behavior_enabled: true
```

### 调试与测试步骤

1.  **状态机监控**:
    观察 `/p2p_move_base/status` 或日志，了解当前处于 `PLANNING`, `CONTROLLING`, 还是 `RECOVERY` 状态。

2.  **集成测试**:
    *   给定一个远处的目标点。
    *   观察机器人是否能自主避障、规划路径并到达终点。
    *   人为设置障碍物，测试重规划能力。

---

## 5. 底层驱动适配 (关键)

由于规划器输出的是 `geometry_msgs/Twist` (v, w)，而 Ackermann 机器人需要 (v, δ)，您需要在底层驱动节点（MicroROS 或串口驱动）中添加转换逻辑：

**转换公式**:
$$ \delta = \arctan\left(\frac{\omega \cdot L}{v}\right) $$

其中：
*   $\delta$: 转向角 (发送给舵机)
*   $\omega$: 角速度 (`cmd_vel.angular.z`)
*   $v$: 线速度 (`cmd_vel.linear.x`)
*   $L$: 轴距 (0.45m)

**Python 转换示例**:
```python
import math
def cmd_vel_callback(msg):
    v = msg.linear.x
    omega = msg.angular.z
    L = 0.45
    
    if abs(v) < 0.01:
        steering_angle = 0
    else:
        steering_angle = math.atan(omega * L / v)
    
    # 限制转向角
    steering_angle = max(min(steering_angle, 0.436), -0.436)
    
    publish_to_robot(v, steering_angle)
```

---

## 6. 总结：调试路线图

1.  **Day 1**: 跑通 **建图**。录制数据，生成高质量的点云地图。
2.  **Day 2**: 跑通 **定位**。在构建的地图中，确保机器人能准确定位，不漂移。
3.  **Day 3**: 部署 **Ackermann 规划器**。编译新代码，配置参数，验证生成的轨迹形状。
4.  **Day 4**: **底层驱动适配**与**联合调试**。实现 Twist 到 (v, δ) 的转换，进行实车避障导航测试。
