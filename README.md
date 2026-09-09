# rm_mppi_controller（mppi_opencl · OpenCL 版）

一个自研的 **MPPI（Model Predictive Path Integral，模型预测路径积分）** 局部控制器，以 ROS 2 Navigation2 `nav2_core::Controller` 插件的形式接入 `controller_server`，负责在跟踪全局路径的同时实时避障。

本分支（`mppi_opencl`）为 **OpenCL 加速版**：采样轨迹的前向仿真与代价计算内核（`__kernel calc_total_cost`）写在 `kernel/kernel_func.cl`，运行期由主机端读取源码并 `clBuildProgram` 动态编译到 **任意厂商的 OpenCL 设备**（Intel iGPU、AMD/NVIDIA GPU、CPU 设备等）。主机与设备通过一份**共享结构体 `MPPIParams`**（`mppi_params.h`，C++ 与 OpenCL C 双端包含、1 字节对齐）交换全部参数，路径点、代价地图与控制序列由主机定期上传到 device buffer。

> 本仓库早期根目录下有一个无扩展名的 `README` 文件，内容为本分支的 Nav2 调参配置笔记，已合并进本文件，原文件可删除。

## Demo

实机演示（录屏）：**哨兵（双轮腿机器人）** 搭载本 MPPI 控制器跟迹避障的效果。在 RViz2 中向机器人发布目标点指令后，哨兵沿全局路径实时跟踪，并对路径上的障碍自动绕行避让（动图见仓库根 `demo/sentry.gif`）。

![哨兵实机 MPPI 跟迹避障演示](demo/sentry.gif)

## 分支总览

仓库用三个分支实现了同一套 MPPI 算法，代价模型与控制器接口完全一致，区别仅在**采样并行化的计算后端**：

| 分支 | 计算后端 | 说明 |
| --- | --- | --- |
| `master` | CPU 多线程（`std::execution` + TBB + Eigen） | 无额外硬件要求，可移植性最好 |
| `mppi_cuda` | NVIDIA CUDA（`sm_89`，Ada 架构 GPU） | NVIDIA 专属，吞吐最高 |
| `mppi_opencl` | OpenCL（跨厂商 GPU / iGPU / CPU 设备） | 本分支，内核 `.cl` 运行期动态编译 |

## 特性

- **完整 MPPI 滚动时域优化**：对控制输入序列叠加高斯噪声采样 `K` 条轨迹，向前仿真 `T` 步，按代价加权平均得到最优控制序列。
- **OpenCL 并行采样**：`kernel_func.cl` 中 `__kernel calc_total_cost` 在设备端并行执行 `K` 条轨迹的前向仿真与阶段/输入/避障代价累计；噪声在设备端按工作项生成（xorshift 系，种子随工作项索引偏移，见 `utils` 风格注释）。
- **主机/设备共享参数**：`mppi_params.h` 定义 `MPPIParams`，C++ 侧 `#pragma pack(1)`、设备侧 `__attribute__((packed))`，并 `static_assert(sizeof(MPPIParams) == 41*4)` 保证两端口径一致；`clBuildProgram` 时通过 `-I` 指向该头文件所在目录，`kernel_func.cl` 内 `#include "mppi_params.h"`。
- **代价函数完备**，由四部分构成：
  - 跟踪代价：对参考路径的横向/纵向偏差（阶段项 + 终端项）；
  - 控制输入代价：速度/角速度及其变化率的惩罚（`gamma_*`）；
  - 避障代价：读取 Nav2 代价地图，对致命障碍重罚（`collision_cost`）、膨胀区域内按反算距离施加渐变惩罚（`critical_weight` / `collision_margin_distance`）；
  - 安全约束：`limit_input` 对速度/角速度限幅。
- **滚动时域热启动**：上一周期最优控制序列整体平移一位作为本轮均值（`shift_control_seq`）再叠加噪声采样。
- 发布 **`local_plan`** 话题，可在 RViz 中直接可视化最优轨迹。
- 全部控制/代价参数通过 ROS 2 参数系统暴露，可在线整定。

## 依赖

- ROS 2（Humble 或更新版本）及其 Navigation2（`nav2_core`、`nav2_costmap_2d`、`nav2_util`）
- **OpenCL**（`find_package(OpenCL)`，需要 ICD + 目标设备驱动，如 Intel/AMD/NVIDIA 的 OpenCL runtime）
- **glog**（`find_package(Glog)`，用于打印 kernel 路径等日志）
- oneTBB（`TBB::tbb`）
- Eigen3（含 unsupported 模块，主机端数据结构使用）

## 编译与安装

```bash
# 在 ROS 2 工作空间下
cd ws/src
git clone -b mppi_opencl <仓库地址> my_mppi
cd ..
colcon build --packages-select rm_mppi_controller
source install/setup.bash
```

构建仅需在编译机装有 OpenCL 头文件与 glog；**OpenCL 设备与驱动只需在运行机具备**（内核是运行期编译的）。产物为共享库 `rm_mppi_controller_plugin`，通过 `pluginlib_export_plugin_description_file(nav2_core plugin.xml)` 自动注册插件。

## 接入 Nav2（controller_server 配置示例）

`FollowPath` 插件名与 `master`/`mppi_cuda` 相同，仅**多出 3 个 OpenCL 专有参数**。下面数值为实车调试使用过的配置（`K=512`、`T=60`）：

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

    # ---- 自研 MPPI（OpenCL 版）----
    FollowPath:
      plugin: "rm_mppi_controller::MPPIController"
      frequency: 30.0            # 控制频率（Hz），内部 dt = 1/frequency
      max_v: 1.7                 # 最大线速度（m/s）
      max_w: 2.0                 # 最大角速度（rad/s）
      step_T: 60                 # 预测时域长度（步）
      samples_K: 512             # 采样轨迹数（设备端并行）
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
      near_goal_distance: 0.5        # 接近目标距离，此距离内停用一般避障项
      inflation_radius: 0.5
      cost_scaling_factor: 3.0
      # ---- OpenCL 专有参数（见下方说明，路径必须改成本机绝对路径）----
      kernel_file: /home/<user>/.../rm_mppi_controller/kernel/kernel_func.cl
      kernel_name: calc_total_cost    # 必须与 .cl 中 __kernel 函数名一致
      mppi_params_dir: /home/<user>/.../rm_mppi_controller/include/rm_mppi_controller
```

### OpenCL 专有参数（重要）

| 参数 | 含义 | 注意事项 |
| --- | --- | --- |
| `kernel_file` | `kernel_func.cl` 的**绝对路径** | 代码内默认值是作者机器的 `/home/lrj/RM/test/...` 路径，**不改必然加载失败**，请务必换成你本机该包的绝对路径 |
| `kernel_name` | 要启动的 `__kernel` 名 | 默认 `calc_total_cost`，必须与 `.cl` 中内核名一致 |
| `mppi_params_dir` | 包含 `mppi_params.h` 的目录 | 用于 `clBuildProgram` 的 `-I` 选项，让 OpenCL 编译器能找到该头文件 |

> 设备选择：`initOpenCL()` 负责枚举平台/设备、创建 context 与 command queue、读取并编译 `.cl` 源码；如需指定具体 OpenCL 平台/设备，请在该函数内修改选择逻辑。

## 参数速查

| 参数 | 含义 | 备注 |
| --- | --- | --- |
| `frequency` | 控制频率，`dt = 1/frequency` | 建议与 `controller_frequency` 一致 |
| `max_v` / `max_w` | 线/角速度限幅 | 超出会被 `limit_input` 截断 |
| `step_T` | 预测时域步数 | 设备端开销随步数近似线性 |
| `samples_K` | 每周期采样轨迹数 | 设备端并行，可开到数百~上千 |
| `lambda` | 信息温度 | 权衡探索与收敛 |
| `gamma_dv/dw` | 控制量变化率成本 | 抑制抖动 |
| `gamma_v/w` | 控制量成本 | 抑制大油门/大转角 |
| `sigma_v/w` | 采样噪声方差（内部按 sqrt 转采样标准差） | 与探索范围直接相关 |
| `stage/terminal_cost_weight_*` | 跟踪代价权重（x/y/yaw） | `yaw` 权重不宜过大 |
| `obstacle_cost_weight` | 一般避障权重 | |
| `critical_weight` | 安全边距内渐增惩罚 | |
| `collision_cost` | 致命碰撞代价 | 过大易造成数值溢出报错 |
| `collision_margin_distance` | 安全边距 | |
| `near_goal_distance` | 接近目标阈值 | 阈值内停用一般避障项（本分支在每周期按与目标距离动态切换） |
| `inflation_radius` / `cost_scaling_factor` | 膨胀层模型参数 | 用于由代价值反算距障碍距离 |

## 运行流程

1. `setPlan`：接收全局路径（`map` 系）→ 查询 `map → odom` 变换 → 逐点转到 `odom` 系，预计算弧长与切线朝向。
2. `activate`：`update_parameters()` 读取全部参数并填充 `MPPIParams` → `initOpenCL()` 建好 context/queue/program/kernel 与各 device buffer。
3. `computeVelocityCommands` 每个控制周期：
   - 快速定位最近路径点（维护 `prev_waypoints_idx_` 滑动窗口 + 弧长索引），并按与目标距离动态启停避障项；
   - `copyPathToDevice` / `copyCostmapToDevice` 上传路径与代价地图（代价地图变化时才刷新）；
   - 提交 `calc_total_cost` kernel：`K` 个 work-item 各采样一条带噪轨迹、前向仿真并累计代价，写回 device buffer；
   - 读回 `costs` 后 CPU 端 `calc_weights` 归一化、`calc_control_seq` 加权合成控制增量；
   - 发布 `local_plan`（可视化）；`smooth_control_seq` 平滑、`shift_control_seq` 平移热启动。
4. 路径过短（`< 10` 个点）时输出零速指令等待新路径；`deactivate` / 析构时 `releaseOpenCL()` 释放全部资源。

代价地图取 `costmap_ros_->getCostmap()`（`odom` 系、含膨胀层），障碍代价按 Nav2 膨胀代价模型由单元代价反算障碍距离后平滑惩罚。

## 目录结构

```
rm_mppi_controller/
├── CMakeLists.txt              # find_package(OpenCL/Glog)，链接 OpenCL + glog + TBB
├── LICENSE
├── package.xml
├── plugin.xml                  # nav2_core controller 插件注册
├── kernel/
│   └── kernel_func.cl          # OpenCL 内核源码（__kernel calc_total_cost 等）
├── include/rm_mppi_controller/
│   ├── eigen_types.h           # Eigen 常用类型别名
│   ├── mppi_controller.hpp     # MPPIController 声明 + Options 参数结构 + OpenCL 句柄
│   └── mppi_params.h           # 主机/设备共享 MPPIParams（packed、双端 include）
└── src/
    └── mppi_controller.cpp     # 控制器实现 + OpenCL host 代码（~710 行）
```

## License

Apache-2.0（见 `rm_mppi_controller/LICENSE`）。
