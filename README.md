# rm_mppi_controller（mppi_cuda · CUDA 版）

一个自研的 **MPPI（Model Predictive Path Integral，模型预测路径积分）** 局部控制器，以 ROS 2 Navigation2 `nav2_core::Controller` 插件的形式接入 `controller_server`，负责在跟踪全局路径的同时实时避障。

本分支（`mppi_cuda`）为 **CUDA GPU 加速版**：每周期 `K` 条采样轨迹的噪声生成与 `T` 步前向仿真/代价累计整体放入 GPU kernel（`computeTrajectoryCostsKernel`），随机数由 **cuRAND** 在设备端按线程独立生成；代价加权、控制序列合成、平滑与平移仍在 CPU 上完成。相比 `master` 的 CPU 版，可在同一控制频率下把 `samples_K` / `step_T` 提高一个数量级（本仓库实测配置 `K=512`、`T=60`）。

> 本仓库早期根目录下有一个无扩展名的 `README` 文件，内容为本分支的 Nav2 调参配置笔记，已合并进本文件，原文件可删除。

## 分支总览

仓库用三个分支实现了同一套 MPPI 算法，代价模型与控制器接口完全一致，区别仅在**采样并行化的计算后端**：

| 分支 | 计算后端 | 说明 |
| --- | --- | --- |
| `master` | CPU 多线程（`std::execution` + TBB + Eigen） | 无额外硬件要求，可移植性最好 |
| `mppi_cuda` | NVIDIA CUDA（`sm_89`，Ada 架构 GPU） | 本分支，吞吐最高 |
| `mppi_opencl` | OpenCL（跨厂商 GPU / iGPU / CPU 设备） | 内核写在 `.cl` 文件，主机端动态编译加载 |

## 特性

- **完整 MPPI 滚动时域优化**：对控制输入序列叠加高斯噪声采样 `K` 条轨迹，向前仿真 `T` 步，按代价加权平均得到最优控制序列。
- **CUDA 并行采样**：`__global__ computeTrajectoryCostsKernel` 把 `K×T` 的前向仿真与避障/跟踪/输入代价累计全部放到 GPU；每个线程独立初始化 **curand** 随机状态，消除主机端随机数生成与并发瓶颈。
- **代价函数完备**，由四部分构成：
  - 跟踪代价：对参考路径的横向/纵向偏差（阶段项 + 终端项）；
  - 控制输入代价：速度/角速度及其变化率的惩罚（`gamma_*`）；
  - 避障代价：读取 Nav2 代价地图，对致命障碍重罚（`collision_cost`）、膨胀区域内按反算距离施加渐变惩罚（`critical_weight` / `collision_margin_distance`）；
  - 安全约束：`limit_input` 对速度/角速度限幅。
- **滚动时域热启动**：上一周期最优控制序列整体平移一位作为本轮均值（`shift_control_seq`）再叠加噪声采样。
- 发布 **`local_plan`** 话题，可在 RViz 中直接可视化最优轨迹。
- 全部控制/代价参数通过 ROS 2 参数系统暴露，可在 `controller_server` 的 YAML 中在线整定。

## 依赖

- ROS 2（Humble 或更新版本）及其 Navigation2（`nav2_core`、`nav2_costmap_2d`、`nav2_util`）
- **NVIDIA CUDA 工具链**（本项目按 CUDA 13.2 + `sm_89` 编写，见下方注意事项）
- oneTBB（`TBB::tbb`，主机侧并行仍在使用）
- Eigen3（含 unsupported 模块）

> **构建前必读（CUDA 架构）**：`CMakeLists.txt` 中写死了
> ```cmake
> set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -arch=sm_89")
> set(CMAKE_CUDA_COMPILER "/usr/local/cuda-13.2/bin/nvcc")
> ```
> `sm_89` 对应 Ada 架构（如 Jetson Orin 模组 / GeForce RTX 40 系）。**编译目标必须与你的实际 GPU 计算能力一致**，否则运行时会报 `no kernel image is available`。若你的设备不同，请修改 `-arch=sm_XX`（可用 `deviceQuery` 或 `nvidia-smi` 查询计算能力）；若 CUDA 安装路径不同，请修改 `CMAKE_CUDA_COMPILER`。

## 编译与安装

```bash
# 在 ROS 2 工作空间下
cd ws/src
git clone -b mppi_cuda <仓库地址> my_mppi
cd ..
colcon build --packages-select rm_mppi_controller
source install/setup.bash
```

CMake 已做以下处理：`enable_language(CUDA)`、`LINKER_LANGUAGE CUDA`、`CXX/CUDA_STANDARD 17`；链接 `TBB::tbb` 与 `CUDA::cudart` / `CUDA::curand`。产物为共享库 `rm_mppi_controller_plugin`，通过 `pluginlib_export_plugin_description_file(nav2_core plugin.xml)` 自动注册插件。

## 接入 Nav2（controller_server 配置示例）

以下数值为实车调试使用过的配置（`K=512`、`T=60`，可在 Jetson Orin 上跑满 30 Hz），供参考起步：

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

    # ---- 自研 MPPI（CUDA 版）----
    FollowPath:
      plugin: "rm_mppi_controller::MPPIController"
      frequency: 30.0            # 控制频率（Hz），内部 dt = 1/frequency
      max_v: 1.7                 # 最大线速度（m/s）
      max_w: 2.0                 # 最大角速度（rad/s）
      step_T: 60                 # 预测时域长度（步）
      samples_K: 512             # 采样轨迹数（GPU 并行，可开很大）
      lambda: 250.0              # MPPI 温度参数：越大越随机、越稳定但越慢；越小越趋最优、易局部最优
      gamma_dv: 0.01             # 速度变化率成本
      gamma_dw: 0.01             # 角速度变化率成本
      gamma_v: 0.2               # 速度成本
      gamma_w: 0.5               # 角速度成本
      sigma_v: 0.7               # 速度采样噪声方差（内部取 sqrt 作为采样标准差）；越大速度越快
      sigma_w: 0.9               # 角速度采样噪声方差（内部取 sqrt 作为采样标准差）
      stage_cost_weight_x: 50.0
      stage_cost_weight_y: 50.0
      stage_cost_weight_yaw: 1.0     # 不能太大，会影响避障
      terminal_cost_weight_x: 50.0
      terminal_cost_weight_y: 50.0
      terminal_cost_weight_yaw: 1.0
      obstacle_cost_weight: 100.0    # 一般避障权重
      critical_weight: 500.0         # 严重惩罚权重（安全边距内）
      collision_cost: 10000.0        # 致命碰撞代价（太大易引入 inf/NaN 报错）
      collision_margin_distance: 0.5 # 安全边距（米）
      near_goal_distance: 0.5        # 接近目标距离（该距离内停用一般避障项）
      inflation_radius: 0.5          # 代价地图膨胀半径
      cost_scaling_factor: 3.0       # 代价衰减系数：越大衰减越快（更敢贴边走）
```

> 主要依赖只有 **TBB 与 CUDA**；其余为 ROS 2 / Nav2 常规依赖。

## 参数速查

| 参数 | 含义 | 备注 |
| --- | --- | --- |
| `frequency` | 控制频率，`dt = 1/frequency` | 建议与 `controller_frequency` 一致 |
| `max_v` / `max_w` | 线/角速度限幅 | 超出会被 `limit_input` 截断 |
| `step_T` | 预测时域步数 | GPU 开销随步数近似线性 |
| `samples_K` | 每周期采样轨迹数 | CUDA 版可开到数百~上千 |
| `lambda` | 信息温度 | 权衡探索与收敛 |
| `gamma_dv/dw` | 控制量变化率成本 | 抑制抖动 |
| `gamma_v/w` | 控制量成本 | 抑制大油门/大转角 |
| `sigma_v/w` | 采样噪声方差（内部按 sqrt 转采样标准差） | 与探索范围直接相关 |
| `stage/terminal_cost_weight_*` | 跟踪代价权重（x/y/yaw） | `yaw` 权重不宜过大 |
| `obstacle_cost_weight` | 一般避障权重 | |
| `critical_weight` | 安全边距内渐增惩罚 | |
| `collision_cost` | 致命碰撞代价 | 过大易造成数值溢出报错 |
| `collision_margin_distance` | 安全边距 | |
| `near_goal_distance` | 接近目标阈值 | 每周期按与目标距离动态停用一般避障项 |
| `inflation_radius` / `cost_scaling_factor` | 膨胀层模型参数 | 用于由代价值反算距障碍距离 |

## 运行流程

1. `setPlan`：接收全局路径（`map` 系）→ 查询 `map → odom` 变换 → 逐点转到 `odom` 系，预计算弧长与切线朝向。
2. `computeVelocityCommands` 每个控制周期：
   - 快速定位最近路径点（维护 `prev_waypoints_idx_` 滑动窗口 + 弧长索引）；
   - 主机把路径点、代价地图、上一周期控制序列、`MPPIParams` 等上传到设备端 buffer；
   - 启动 `computeTrajectoryCostsKernel`：每个线程（独立 curand 状态）采样一条带噪轨迹、前向仿真并累计阶段/输入/避障代价，写回 `costs`；
   - CPU 端 `calc_weights` 按 `exp(-cost/lambda)` 归一化，`calc_control_seq` 加权合成控制增量；
   - 发布 `local_plan`（可视化）；`smooth_control_seq` 平滑、`shift_control_seq` 平移热启动。
3. 路径过短（`< 10` 个点）时输出零速指令等待新路径。

代价地图取 `costmap_ros_->getCostmap()`（`odom` 系、含膨胀层），障碍代价按 Nav2 膨胀代价模型由单元代价反算障碍距离后平滑惩罚。

## 目录结构

```
rm_mppi_controller/
├── CMakeLists.txt              # CUDA 语言开启、arch=sm_89、nvcc 路径
├── LICENSE
├── package.xml
├── plugin.xml                  # nav2_core controller 插件注册
├── include/rm_mppi_controller/
│   ├── eigen_types.h           # Eigen 常用类型别名
│   └── mppi_controller.hpp     # MPPIController 声明 + Options 参数结构
└── src/
    └── mppi_controller.cu      # 控制器实现 + CUDA kernel（~1070 行）
```

## License

Apache-2.0（见 `rm_mppi_controller/LICENSE`）。
