# LeGO-LOAM 框架分析与 FAST-LIO 替换方案

## 1. LeGO-LOAM 框架与 TF 分析

### 1.1 LeGO-LOAM 的 TF 树结构
在 `mapOptimization.cpp` 中，LeGO-LOAM 维护了以下 TF 链：

1.  **`map` -> `camera_init`**:
    *   这是一个静态或准静态变换，通常用于对齐地图原点。
    *   代码中 `trans_m2ci` 定义了这个变换。

2.  **`camera_init` -> `camera` (sensor)**:
    *   这是算法计算出的核心位姿（激光里程计/SLAM位姿）。
    *   LeGO-LOAM 内部计算的是雷达相对于起始点 (`camera_init`) 的位姿。

3.  **`camera` -> `base_link`**:
    *   这是固定的外参变换（安装位置）。

4.  **`map` -> `odom` 的计算**:
    *   代码逻辑：`T_map_odom = T_map_base * T_odom_base^-1`
    *   其中 `T_map_base` 是通过 `map`->`camera_init`->`camera`->`base_link` 计算出来的。
    *   `T_odom_base` 是输入的轮速计/激光里程计。
    *   **关键点**: LeGO-LOAM 试图发布 `map` -> `odom` 的变换，以闭合标准的 ROS 导航 TF 树 (`map` -> `odom` -> `base_link`)。

### 1.2 为什么 `map` 消失了？
您提到 "运行时已没有 map frame 发布"。这通常是因为 `publishTF()` 函数没有被调用，或者计算 `map2odom` 的前提条件未满足：
*   **数据流中断**: 如果 `runWoLO()` (Run Without Lidar Odometry) 或 `run()` 没有被触发（即没有接收到有效的特征点数据），`publishTF()` 就不会执行。
*   **初始化失败**: 算法可能还在等待第一帧数据或 IMU 初始化。

## 2. 使用 FAST-LIO 替换 LeGO-LOAM

**结论**: **完全可以，且强烈推荐。**

对于 **MID360** 激光雷达，**FAST-LIO2** 是目前最优的开源 SLAM 方案之一。相比 LeGO-LOAM，它有以下优势：
*   **原生支持 MID360**: 能够直接利用 MID360 的内置 IMU 和非重复扫描特性。
*   **紧耦合 LIO**: 鲁棒性更强，不易在剧烈运动中丢失定位。
*   **计算效率高**: 适合嵌入式平台。

### 2.1 替换方案架构

如果您决定替换，架构将发生如下变化：

**当前架构 (LeGO-LOAM)**:
*   输入: PointCloud2, IMU, Odom
*   输出: `map` -> `odom` TF, 栅格地图/点云地图

**新架构 (FAST-LIO2)**:
*   输入: `/livox/lidar` (CustomMsg), `/livox/imu`
*   输出: `odom` -> `base_link` (高频里程计) 或 `map` -> `odom` (如果开启 mapping 模式)

### 2.2 实施步骤

#### 第一步：编译 FAST-LIO
您需要下载适配 ROS2 的 FAST-LIO 版本。
*(注：官方 FAST-LIO 主要支持 ROS1，ROS2 版本通常由社区维护，如 `fast_lio_ros2`)*

#### 第二步：配置 TF
FAST-LIO 默认发布 `camera_init` -> `body`。为了适配导航栈，您需要：
1.  **重映射 Frame ID**: 将 `camera_init` 映射为 `odom`，`body` 映射为 `base_link`。
2.  **或者使用 TF 转换节点**: 启动一个节点将 FAST-LIO 的输出转换为标准的 `odom` -> `base_link`。

#### 第三步：解决 `map` -> `odom`
*   **纯里程计模式**: FAST-LIO 仅作为里程计 (`odom` -> `base_link`)。然后使用 `dddmr_mcl_3dl` 或 `nav2_amcl` 提供 `map` -> `odom` 的定位变换。
*   **SLAM 模式**: 修改 FAST-LIO 代码，使其发布 `map` -> `odom` 变换（类似于 LeGO-LOAM 的做法）。

### 2.3 推荐路线

鉴于您已经有 `dddmr_mcl_3dl` (定位模块)，最稳健的架构是：

1.  **建图阶段**:
    *   使用 **FAST-LIO2** 进行建图。
    *   保存 PCD 地图。
    *   此时不需要 `map` -> `odom` 的实时 TF，只需要记录点云和轨迹。

2.  **导航阶段**:
    *   **定位**: 使用 `dddmr_mcl_3dl` 加载 PCD 地图，发布 `map` -> `odom`。
    *   **里程计**: 使用 **FAST-LIO2** (定位模式) 或 **轮速计** 发布 `odom` -> `base_link`。
    *   **规划**: `dddmr_local_planner` 在 `odom` 坐标系规划，`dddmr_global_planner` 在 `map` 坐标系规划。

## 3. 总结建议

1.  **短期修复**: 如果您想继续用 LeGO-LOAM，请集中解决数据输入问题（确保 PointCloud2 格式正确，话题名称匹配），这是 `map` frame 消失的根本原因。
2.  **长期优化**: 切换到 **FAST-LIO2**。它对 MID360 的支持更好。您可以保留 `dddmr_navigation` 的其他部分（规划、控制），仅替换建图/里程计前端。

如果您决定切换到 FAST-LIO，我可以为您提供具体的 ROS2 FAST-LIO 部署指南。
