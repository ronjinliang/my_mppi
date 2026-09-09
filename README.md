# rm_mppi_controller（master · CPU 版）

一个自研的 **MPPI（Model Predictive Path Integral，模型预测路径积分）** 局部控制器，以 ROS 2 Navigation2 `nav2_core::Controller` 插件的形式接入 `controller_server`，负责在跟踪全局路径的同时实时避障。

本分支（`master`）为 **CPU 纯计算版**：全部采样轨迹的前向仿真与代价计算都在 CPU 上完成，使用 C++17 `std::execution` / oneTBB 做多线程并行，并利用 Eigen（含 `unsupported/Eigen/CXX11/Tensor`）做向量化计算，**不依赖任何 GPU**。

## Demo

实机演示（录屏）：**哨兵（双轮腿机器人）** 搭载本 MPPI 控制器跟迹避障的效果。在 RViz2 中向机器人发布目标点指令后，哨兵沿全局路径实时跟踪，并对路径上的障碍自动绕行避让（动图见仓库根 `demo/sentry.gif`）。

![哨兵实机 MPPI 跟迹避障演示](demo/sentry.gif)

## 分支总览

仓库用三个分支实现了同一套 MPPI 算法，代价模型与控制器接口完全一致，区别仅在**采样并行化的计算后端**：

| 分支 | 计算后端 | 说明 |
| --- | --- | --- |
| `master` | CPU 多线程（`std::execution` + TBB + Eigen） | 本分支，无额外硬件要求，可移植性最好 |
| `mppi_cuda` | NVIDIA CUDA（`sm_89`，即 Jetson Orin 系） | 采样前向仿真上 GPU，吞吐最高 |
| `mppi_opencl` | OpenCL（跨厂商 GPU / iGPU / CPU 设备） | 内核写在 `.cl` 文件，主机端动态编译加载 |

## 特性

- **完整 MPPI 滚动时域优化**：对控制输入序列叠加高斯噪声采样 `K` 条轨迹，向前仿真 `T` 步，按代价加权平均得到最优控制序列。
- **CPU 多线程并行**：`K` 条轨迹之间用 `std::execution::par_unseq` 并行，轨迹内 `T` 步用 `par` 并行；随机数由 `utils.h` 中的 `xorshift32` + 高斯变换生成，线程间种子独立、无锁。
- **代价函数完备**，由四部分构成：
  - 跟踪代价：对参考路径的横向/纵向偏差（阶段项 + 终端项）；
  - 控制输入代价：速度/角速度及其变化率的惩罚（`gamma_*`）；
  - 避障代价：直接读取 Nav2 代价地图，对致命障碍重罚（`collision_cost`）、膨胀区域内按反算距离施加渐变惩罚（`critical_weight` / `collision_margin_distance`）；
  - 安全约束：`limit_input` 对速度/角速度限幅。
- **滚动时域热启动**：上一周期的最优控制序列整体平移一位作为本轮均值（`shift_control_seq`），再叠加噪声采样，收敛快、抖动小。
- **预计算路径弧长表**：`setPlan` 时把全局路径一次性转到 `odom` 系并预计算弧长/朝向，周期内最近路径点查找为 O(1) 级，避免逐点遍历。
- 发布 **`local_plan`**（当前最优轨迹）话题，可在 RViz 中直接可视化。
- 全部控制/代价参数通过 ROS 2 参数系统暴露，可在 `controller_server` 的 YAML 中在线整定。

## 依赖

- ROS 2（Humble 或更新版本）及其 Navigation2（`nav2_core`、`nav2_costmap_2d`、`nav2_util`）
- C++17 编译器（gcc ≥ 10 / clang，需要 `<execution>` 并行算法支持）
- `pluginlib`、`ament_cmake`
- oneTBB（`TBB::tbb`）
- Eigen3（含 unsupported 模块，`Eigen/CXX11/Tensor`）

> 注意：本分支 `CMakeLists.txt` 打开了 `-march=native`、`-mfma` 与 `EIGEN_USE_THREADS`，**换机器后必须重新编译**，否则可能触发非法指令。

## 编译与安装

```bash
# 在 ROS 2 工作空间下（假设 ws 为你的工作空间）
cd ws/src
git clone -b master <仓库地址> my_mppi
# 或直接复制 rm_mppi_controller 包目录到 src 下
cd ..
colcon build --packages-select rm_mppi_controller
source install/setup.bash
```

编译产物为共享库 `rm_mppi_controller_plugin`，已通过 `pluginlib_export_plugin_description_file(nav2_core plugin.xml)` 自动注册插件描述。

## 接入 Nav2（controller_server 配置示例）

在 `controller_server` 的 YAML 中把 `FollowPath` 的插件换成 `rm_mppi_controller::MPPIController` 即可。所有 `FollowPath.*` 参数与代码内 `Options` 的默认值一致，可按实车调参：

```yaml
controller_server:
  ros__parameters:
    use_sim_time: True
    controller_frequency: 30.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    failure_tolerance: 0.3
    progress_checker_plugin: "progress_checker"
    goal_checker_plugins: ["general_goal_checker"]   # "precise_goal_checker"
    controller_plugins: ["FollowPath"]

    # Progress checker parameters
    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.5
      movement_time_allowance: 10.0
    # Goal checker parameters
    general_goal_checker:
      stateful: True
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.5

    # ---- 自研 MPPI（CPU 版）----
    FollowPath:
      plugin: "rm_mppi_controller::MPPIController"
      frequency: 30.0            # 控制频率（Hz），内部 dt = 1/frequency
      max_v: 1.5                 # 最大线速度（m/s）
      max_w: 1.5                 # 最大角速度（rad/s）
      step_T: 30                 # 预测时域长度（步）。CPU 版请从 20~30 起步，视算力上调
      samples_K: 100             # 采样轨迹数。CPU 版 100 已经较慢，加大前请先测单周期耗时
      lambda: 50.0               # MPPI 温度参数：越大越随机、越稳定但越慢；越小越趋最优、易局部最优
      gamma_dv: 0.001            # 速度变化率成本
      gamma_dw: 0.001            # 角速度变化率成本
      gamma_v: 0.001             # 速度成本
      gamma_w: 0.001             # 角速度成本
      sigma_v: 0.1               # 速度采样噪声方差（内部取 sqrt 作为采样标准差）；越大速度越快
      sigma_w: 0.1               # 角速度采样噪声方差（内部取 sqrt 作为采样标准差）
      stage_cost_weight_x: 50.0
      stage_cost_weight_y: 50.0
      stage_cost_weight_yaw: 1.0     # 不能太大，会影响避障
      terminal_cost_weight_x: 50.0
      terminal_cost_weight_y: 50.0
      terminal_cost_weight_yaw: 1.0
      obstacle_cost_weight: 5.0      # 一般避障权重（自由/低代价区间的惩罚比例）
      critical_weight: 100.0         # 严重惩罚权重（安全边距内）
      collision_cost: 1000000.0      # 致命碰撞代价（超过地图致命阈值时，太大可能引入 inf/NaN）
      collision_margin_distance: 0.2 # 安全边距（米）
      near_goal_distance: 0.5        # 接近目标距离（预留：当前 CPU 版尚未启用该避障开关逻辑）
      inflation_radius: 0.55         # 代价地图膨胀半径（与代价地图配置保持一致）
      cost_scaling_factor: 3.0       # 代价衰减系数：越大衰减越快（更敢贴边走）
```

## 参数速查

| 参数 | 含义 | 备注 |
| --- | --- | --- |
| `frequency` | 控制频率，`dt = 1/frequency` | 建议与 `controller_frequency` 一致 |
| `max_v` / `max_w` | 线/角速度限幅 | 超出会被 `limit_input` 截断 |
| `step_T` | 预测时域步数 | 越大看越远，CPU 开销线性增长 |
| `samples_K` | 每周期采样轨迹数 | 并行上限与 CPU 核数相关 |
| `lambda` | 信息温度 | 权衡探索与收敛 |
| `gamma_dv/dw` | 控制量变化率成本 | 抑制抖动 |
| `gamma_v/w` | 控制量成本 | 抑制大油门/大转角 |
| `sigma_v/w` | 采样噪声方差（内部按 sqrt 转采样标准差） | 与探索范围直接相关 |
| `stage/terminal_cost_weight_*` | 跟踪代价权重（x/y/yaw） | `yaw` 权重不宜过大 |
| `obstacle_cost_weight` | 一般避障权重 | |
| `critical_weight` | 安全边距内渐增惩罚 | |
| `collision_cost` | 致命碰撞代价 | 过大易造成数值溢出 |
| `collision_margin_distance` | 安全边距 | |
| `near_goal_distance` | 接近目标阈值 | 已声明并读取，当前 CPU 版尚未启用该逻辑 |
| `inflation_radius` / `cost_scaling_factor` | 膨胀层模型参数 | 用于由代价值反算距障碍距离 |

## 运行流程

1. `setPlan`：接收全局路径（`map` 系）→ 查询 `map → odom` 变换 → 逐点转到 `odom` 系，预计算弧长与切线朝向，存入 `path_points_` / `path_arc_lengths_`。
2. `computeVelocityCommands` 每个控制周期：
   - 由 `path_arc_lengths_` 二分/索引快速找到最近路径点（维护 `prev_waypoints_idx_` 滑动窗口）；
   - `calc_total_costs`：并行采样 `K` 条带噪轨迹并前向仿真、累计各段代价；
   - `calc_weights`：按 `exp(-cost/lambda)` 归一化得到权重；
   - `calc_control_seq`：加权平均噪声得到最优控制增量；
   - 发布 `local_plan`（可视化）；
   - `smooth_control_seq` 平滑、`shift_control_seq` 平移热启动。
3. 路径过短（`< 10` 个点）时输出零速指令等待新路径。

代价地图取 `costmap_ros_->getCostmap()`（`odom` 系、含膨胀层），障碍代价按 Nav2 膨胀代价模型由单元代价反算障碍距离后平滑惩罚，避免代价跳变导致指令抖动或 NaN。

## 目录结构

```
rm_mppi_controller/
├── CMakeLists.txt
├── LICENSE
├── package.xml              # ament 包描述（导出 pluginlib 插件）
├── plugin.xml               # nav2_core controller 插件注册
├── include/rm_mppi_controller/
│   ├── eigen_types.h        # Eigen 常用类型别名（Vec3f/Mat2f 等）
│   ├── mppi_controller.hpp  # MPPIController 声明 + Options 参数结构
│   └── utils.h              # xorshift32 伪随机数 + 高斯采样（CPU 版专用）
└── src/
    └── mppi_controller.cpp  # 控制器实现（~660 行）
```

## License

Apache-2.0（见 `rm_mppi_controller/LICENSE`）。
