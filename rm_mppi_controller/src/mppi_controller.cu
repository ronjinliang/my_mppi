#include "rm_mppi_controller/mppi_controller.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include <algorithm>
#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <random>
#include <execution>
#include <cmath>

// CUDA 头文件
#include <cuda_runtime.h>
#include <curand_kernel.h>

namespace rm_mppi_controller {

// -------------------------------------------------------------------
// 设备端辅助函数
// -------------------------------------------------------------------

__device__ float calc_obstacle_cost_device(
  float x, float y,
  const unsigned char *costmap_data, int width, int height,
  float resolution, float origin_x, float origin_y,
  float inflation_radius, float cost_scaling_factor,
  float inscribed_radius,
  float obstacle_cost_weight, float critical_weight,
  float collision_cost, float collision_margin_distance){

  // 世界坐标转栅格索引
  int mx = (int)((x - origin_x) / resolution);
  int my = (int)((y - origin_y) / resolution);
  if (mx < 0 || mx >= width || my < 0 || my >= height) {
    return 0.0f; // 超出地图边界，无障碍
  }
  unsigned char cost = costmap_data[my * width + mx];
  if (cost <= 0) { // FREE_SPACE = 0
    return 0.0f;
  }
  if (cost >= 254) { // LETHAL_OBSTACLE
    return collision_cost;
  }
  // 使用膨胀层参数计算距离
  if (inflation_radius > 0.0f && cost_scaling_factor > 0.0f) {
    float dist = (logf(254.0f) - logf((float)cost)) / cost_scaling_factor + inscribed_radius;
    if (dist < inscribed_radius) dist = inscribed_radius;
    float max_penalty_dist = inflation_radius;
    float penalty = 0.0f;
    if (dist <= collision_margin_distance) {
      penalty = critical_weight * (collision_margin_distance - dist);
    } else if (dist < max_penalty_dist) {
      penalty = obstacle_cost_weight * (max_penalty_dist - dist);
    }
    return penalty;
  } else {
    // 无膨胀参数，直接线性缩放
    return obstacle_cost_weight * ((float)cost / 254.0f);
  }
}

__device__ float calc_stage_cost_device(
  float x, float y, float yaw, int prev_waypoints_idx, 
  const float *path_points, const float *arc_lengths, int num_points,
  float *stage_weight, float dt, float u_v, int t){
  // 1. 找欧氏最近点索引
  int nearest_idx = 0;
  float best_dist2 = 1e20f;

  // 基于 prev_waypoints_idx_ 或上一次的投影点，在局部窗口内搜索（前后各10个点）。
  int start = prev_waypoints_idx;
  if ( start > 10 ) start = prev_waypoints_idx - 10;
  else start = 0;
  int end = prev_waypoints_idx + 10;
  if ( end > num_points - 1 ) end = num_points - 1;
  for (int i = start; i < end; ++i) {
    float dx = path_points[i*3] - x;
    float dy = path_points[i*3+1] - y;
    float d2 = dx*dx + dy*dy;
    if (d2 < best_dist2) {
      best_dist2 = d2;
      nearest_idx = i;
    }
  }

  // 确保投影点不会小于上一个参考点（避免倒退）
  if (nearest_idx < prev_waypoints_idx) nearest_idx = prev_waypoints_idx;

  // 2. 计算前向投影距离
  float forward_dist = fabs(u_v) * dt * t;
  if (forward_dist < 0.5f) forward_dist = 0.5f;
  float target_arc = arc_lengths[nearest_idx] + forward_dist;

  // 3. 二分查找目标弧长对应的索引
  int low = 0, high = num_points - 1;
  int target_idx = high;  // 默认最后一个
  while (low <= high) {
    int mid = (low + high) / 2;
    if (arc_lengths[mid] >= target_arc) {
      target_idx = mid;
      high = mid - 1;
    } else {
      low = mid + 1;
    }
  }

  // 4. 获取参考点坐标和朝向
  float ref_x = path_points[target_idx*3];
  float ref_y = path_points[target_idx*3+1];
  float ref_yaw = path_points[target_idx*3+2];

  // 5. 计算偏差
  float dx = ref_x - x;
  float dy = ref_y - y;
  float dyaw = ref_yaw - yaw;
  if (dyaw < -1.5708f) dyaw += 3.14159f;
  if (dyaw >  1.5708f) dyaw -= 3.14159f;

  return stage_weight[0] * dx*dx + stage_weight[1] * dy*dy + stage_weight[2] * dyaw*dyaw;
}

__device__ float calc_terminal_cost_device(
  float x, float y, float yaw, int prev_waypoints_idx,
  const float *path_points, const float *arc_lengths, int num_points,
  float *terminal_weight, float dt, float u_v, float T){

  // 1. 找欧氏最近点索引
  int nearest_idx = 0;
  float best_dist2 = 1e20f;

  // 基于 prev_waypoints_idx_ 或上一次的投影点，在局部窗口内搜索（前后各10个点）。
  int start = prev_waypoints_idx;
  if ( start > 10 ) start = prev_waypoints_idx - 10;
  else start = 0;
  int end = prev_waypoints_idx + 10;
  if ( end > num_points - 1 ) end = num_points - 1;
  for (int i = start; i < end; ++i) {
    float dx = path_points[i*3] - x;
    float dy = path_points[i*3+1] - y;
    float d2 = dx*dx + dy*dy;
    if (d2 < best_dist2) {
      best_dist2 = d2;
      nearest_idx = i;
    }
  }

  // 确保投影点不会小于上一个参考点（避免倒退）
  if (nearest_idx < prev_waypoints_idx) nearest_idx = prev_waypoints_idx;

  // 2. 计算前向投影距离
  float forward_dist = fabs(u_v) * T * dt;
  if (forward_dist < 0.5f) forward_dist = 0.5f;
  float target_arc = arc_lengths[nearest_idx] + forward_dist;

  //   TODO 这个应该错了
  int target_idx = nearest_idx;
  for (int i = target_idx; i < num_points; ++i) {
    if (arc_lengths[i] >= target_arc) {
        target_idx = i;
        break;
    }
  }

  // // 3.二分查找目标弧长对应的索引
  // int low = 0, high = cl_mps->path_points_size - 1;
  // int target_idx = high;
  // while ( low <= high ) {
  //     int mid = ( low + high ) / 2;
  //     if ( cl_arc_lengths[mid] >= target_arc ) {
  //         target_idx = mid;
  //         high = mid - 1;
  //     } else {
  //         low = mid + 1;
  //     }
  // }

  float ref_x = path_points[target_idx*3];
  float ref_y = path_points[target_idx*3+1];
  float ref_yaw = path_points[target_idx*3+2];

  float dx = ref_x - x;
  float dy = ref_y - y;
  float dyaw = ref_yaw - yaw;
  if (dyaw < -1.5708f) dyaw += 3.14159f;
  if (dyaw >  1.5708f) dyaw -= 3.14159f;

  return terminal_weight[0] * dx*dx + terminal_weight[1] * dy*dy + terminal_weight[2] * dyaw*dyaw;
}

// -------------------------------------------------------------------
// CUDA Kernel：计算所有采样轨迹的总成本
// -------------------------------------------------------------------
__global__ void computeTrajectoryCostsKernel(
  float *d_costs,                     // 输出，长度为 K
  float *d_epsilon,           // 新增：输出噪声 (K*T*2)
  const float *d_u_prev,              // 控制序列 (T x 2)，行主序
  const float *d_path_points,         // 路径点 (N x 3)
  const float *d_arc_lengths,         // 弧长 (N)
  int prev_waypoints_idx,
  int num_points,
  const unsigned char *d_costmap_data,// 代价地图数据
  int costmap_width, int costmap_height,
  float costmap_resolution, float costmap_origin_x, float costmap_origin_y,
  float inflation_radius, float cost_scaling_factor, float inscribed_radius,
  float dt, int T, float max_v, float max_w,
  float gamma_dv, float gamma_dw, float gamma_v, float gamma_w,
  float std_v, float std_w,
  float sigma_0, float sigma_1,       // sigma 对角线元素
  float stage_weight_x, float stage_weight_y, float stage_weight_yaw,
  float terminal_weight_x, float terminal_weight_y, float terminal_weight_yaw,
  float obstacle_cost_weight, float critical_weight,
  float collision_cost, float collision_margin_distance,
  float start_x, float start_y, float start_yaw,
  bool stop_obstacle, int K){

  int k = blockIdx.x * blockDim.x + threadIdx.x;
  if (k >= K) return;

  // 初始化 curand 状态（每个线程独立）
  curandState state;
  curand_init(clock64() + k, 0, 0, &state);

  // 当前状态
  float x = start_x;
  float y = start_y;
  float yaw = start_yaw;

  float total_cost = 0.0f;
  float u_v_prev = d_u_prev[0];       // t=0 的控制输入（用于输入成本中的差分项）
  float u_w_prev = d_u_prev[1];

  float u_v = 0.f;
  float u_w = 0.f;

  for (int t = 0; t < T; ++t) {
    // 从 d_u_prev 读取当前时刻的控制输入（未加噪声）
    float u_v_nominal = d_u_prev[t * 2];
    float u_w_nominal = d_u_prev[t * 2 + 1];
    // 生成高斯噪声
    float noise_v = curand_normal(&state) * std_v;
    float noise_w = curand_normal(&state) * std_w;

    // 存储噪声到 d_epsilon
    d_epsilon[((k * T) + t) * 2 + 0] = noise_v;
    d_epsilon[((k * T) + t) * 2 + 1] = noise_w;

    u_v = u_v_nominal + noise_v;
    u_w = u_w_nominal + noise_w;
    // 限幅
    if (u_v > max_v) u_v = max_v;
    if (u_v < -max_v) u_v = -max_v;
    if (u_w > max_w) u_w = max_w;
    if (u_w < -max_w) u_w = -max_w;

    // 更新状态
    x += u_v * cosf(yaw) * dt;
    y += u_v * sinf(yaw) * dt;
    yaw += u_w * dt;
    // 规范化 yaw
    if (yaw > 3.14159f) yaw -= 2*3.14159f;
    if (yaw < -3.14159f) yaw += 2*3.14159f;

    // 计算阶段成本
    float stage_cost = calc_stage_cost_device(x, y, yaw, prev_waypoints_idx,
        d_path_points, d_arc_lengths, num_points,
        (float[]){stage_weight_x, stage_weight_y, stage_weight_yaw},
        dt, u_v, t);
    // 输入成本
    float du_v = u_v - u_v_prev;
    float du_w = u_w - u_w_prev;
    float input_cost = gamma_dv * du_v*du_v / sigma_0 +
                        gamma_dw * du_w*du_w / sigma_1 +
                        gamma_v * u_v*u_v / sigma_0 +
                        gamma_w * u_w*u_w / sigma_1;
    stage_cost += input_cost;

    if (!stop_obstacle) {
      float obs_cost = calc_obstacle_cost_device(x, y,
        d_costmap_data, costmap_width, costmap_height,
        costmap_resolution, costmap_origin_x, costmap_origin_y,
        inflation_radius, cost_scaling_factor, inscribed_radius,
        obstacle_cost_weight, critical_weight,
        collision_cost, collision_margin_distance);
      stage_cost += obs_cost;
    }

    total_cost += stage_cost;

    // 更新上一时刻控制量（用于下一步的差分）
    u_v_prev = u_v;
    u_w_prev = u_w;
  }

  // 终端成本
  float terminal_cost = calc_terminal_cost_device(x, y, yaw, prev_waypoints_idx, 
      d_path_points, d_arc_lengths, num_points,
      (float[]){terminal_weight_x, terminal_weight_y, terminal_weight_yaw},
      dt, u_v, T);
  total_cost += terminal_cost;

  d_costs[k] = total_cost;
}

// -------------------------------------------------------------------
// MPPIController 类成员函数实现（修改 calc_total_costs 及其他必要部分）
// -------------------------------------------------------------------

MPPIController::MPPIController(){
  // 初始化随机数生成器（CPU端仍保留，以备不时之需）
  gen_.seed(rd_());
}

MPPIController::~MPPIController(){
  freeDeviceMemory();
}

void MPPIController::allocateDeviceMemory(){
  // 分配 costs 数组
  cudaMalloc(&d_costs_, opts_.samples_K * sizeof(float));
  size_t epsilon_bytes = opts_.samples_K * opts_.step_T * opts_.dim_u * sizeof(float);
  cudaMalloc(&d_epsilon_, epsilon_bytes);
}

void MPPIController::freeDeviceMemory(){
  if (d_costs_) cudaFree(d_costs_);
  if (d_epsilon_) cudaFree(d_epsilon_);
  if (d_path_points_) cudaFree(d_path_points_);
  if (d_arc_lengths_) cudaFree(d_arc_lengths_);
  if (d_costmap_data_) cudaFree(d_costmap_data_);
  d_costs_ = nullptr;
  d_epsilon_ = nullptr;
  d_path_points_ = nullptr;
  d_arc_lengths_ = nullptr;
  d_costmap_data_ = nullptr;
}

void MPPIController::copyPathToDevice(){
  if (path_points_size_ == 0) return;
  // 分配设备内存（如果大小变化则重新分配）
  size_t points_bytes = path_points_size_ * 3 * sizeof(float);
  size_t arcs_bytes = path_points_size_ * sizeof(float);
  if (d_path_points_) cudaFree(d_path_points_);
  if (d_arc_lengths_) cudaFree(d_arc_lengths_);
  cudaMalloc(&d_path_points_, points_bytes);
  cudaMalloc(&d_arc_lengths_, arcs_bytes);
  // 拷贝数据
  cudaMemcpy(d_path_points_, path_points_.data(), points_bytes, cudaMemcpyHostToDevice);
  cudaMemcpy(d_arc_lengths_, path_arc_lengths_.data(), arcs_bytes, cudaMemcpyHostToDevice);
}

void MPPIController::copyCostmapToDevice(){
  if (!costmap_) return;
  unsigned int size_x = costmap_->getSizeInCellsX();
  unsigned int size_y = costmap_->getSizeInCellsY();
  size_t map_bytes = size_x * size_y * sizeof(unsigned char);
  // 如果大小改变或未分配，重新分配
  if (d_costmap_data_ == nullptr || d_costmap_size_x_ != (int)size_x || d_costmap_size_y_ != (int)size_y) {
    if (d_costmap_data_) cudaFree(d_costmap_data_);
    cudaMalloc(&d_costmap_data_, map_bytes);
    d_costmap_size_x_ = size_x;
    d_costmap_size_y_ = size_y;
  }
  // 拷贝代价地图数据
  cudaMemcpy(d_costmap_data_, costmap_->getCharMap(), map_bytes, cudaMemcpyHostToDevice);
  // 同时拷贝元数据（分辨率、原点等）
  d_costmap_resolution_ = costmap_->getResolution();
  d_costmap_origin_x_ = costmap_->getOriginX();
  d_costmap_origin_y_ = costmap_->getOriginY();
  // 获取内接圆半径（从 layered costmap 中获取）
  d_inscribed_radius_ = costmap_ros_->getLayeredCostmap()->getInscribedRadius();
  d_inflation_radius_ = opts_.inflation_radius;
  d_cost_scaling_factor_ = opts_.cost_scaling_factor;
}

void MPPIController::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) {
  // 初始化随机数发生器
  std::normal_distribution<float> dist(0.0, 1.0);

  node_ = parent.lock();
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  costmap_frame_id_ = costmap_ros_->getGlobalFrameID();

  tf_ = tf;
  plugin_name_ = name;
}

void MPPIController::cleanup() { RCLCPP_INFO(node_->get_logger(), "清理控制器：%s 类型为 rm_mppi_controller::MPPIController", plugin_name_.c_str()); }

void MPPIController::activate() {
  RCLCPP_INFO(node_->get_logger(), "激活控制器：%s 类型为 rm_mppi_controller::MPPIController", plugin_name_.c_str());

  // 创建局部路径发布者
  local_plan_pub_ = node_->create_publisher<nav_msgs::msg::Path>(
    "local_plan", rclcpp::SystemDefaultsQoS());

  // update 参数
  update_parameters();

  RCLCPP_INFO(node_->get_logger(),"update MPPI 控制器完成!!!!!!!!!!");
}

void MPPIController::deactivate() { RCLCPP_INFO(node_->get_logger(), "停用控制器：%s 类型为 rm_mppi_controller::MPPIController", plugin_name_.c_str());}

geometry_msgs::msg::TwistStamped MPPIController::computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped &pose,
    const geometry_msgs::msg::Twist &velocity,
    nav2_core::GoalChecker * goal_checker) {
  (void)velocity;
  (void)goal_checker;

  // 在 odom 系下计算，因为costmap的系就是odom，这样方便访问代价地图

  std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

  if ( path_points_size_ < 10 ) {
    // 清除控制量
    u_prev_.setZero();
    geometry_msgs::msg::TwistStamped cmd_vel;
    cmd_vel.header.stamp = node_->now();
    cmd_vel.header.frame_id = pose.header.frame_id; // odom
    cmd_vel.twist.linear.x = 0.0;
    cmd_vel.twist.angular.z = 0.0;
    return cmd_vel;
  }

  Vec3f x0( pose.pose.position.x, pose.pose.position.y,
    tf2::getYaw(pose.pose.orientation));

  float delta_t = (x0.head<2>() - goal_pt_.head<2>()).norm();
  if ( delta_t < opts_.near_goal_distance ) {
    stop_obstacle_ = true;
  } else {
    stop_obstacle_ = false;
  }

  // 寻找离当前车辆位置最近的参考路径点，并更新索引
  Vec2f search_pt = Vec2f( x0.x(), x0.y() );
  get_nearest_waypoint( search_pt, true );
  if ( prev_waypoints_idx_ >= path_points_size_ ) {
    throw nav2_core::PlannerException("无法找到有效的路径点");
  }

  // loop for 0 ~ K-1 samples  耗时较长  TODO
  calc_total_costs(x0);

  // 计算每个采样轨迹的权重（基于成本） weights_
  calc_weights();

  // calculate w_k * epsilon_k
  calc_control_seq();

  // 可视化最优轨迹
  publish_local_plan(x0);

  // 计算控制指令
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.stamp    = node_->now();
  cmd_vel.header.frame_id = pose.header.frame_id;
  cmd_vel.twist.linear.x  = u_prev_(0, 0);
  cmd_vel.twist.angular.z = u_prev_(0, 1);

  // 对控制序列平滑一下
  smooth_control_seq();
  
  // 控制序列移位（滚动时域热启动）
  shift_control_seq();
  std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
  auto time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
  RCLCPP_INFO( node_->get_logger(), "[computeVelocityCommands]:%.5fs", time_used.count() );
  return cmd_vel;
}

void MPPIController::setPlan(const nav_msgs::msg::Path &path) {
  // 代价地图是odom坐标系，global_plan是map系，为了后面方便处理，只能将global_plan转到odom系
  // 1.2ms左右
  global_plan_ = path;

  // 预计算弧长和路径点
  if (global_plan_.poses.empty()) return;

  // 目标点
  auto temp_pose = global_plan_.poses.back().pose;
  float goal_x = temp_pose.position.x;
  float goal_y = temp_pose.position.y;
  float goal_yaw = 2 * std::atan2(temp_pose.orientation.z, temp_pose.orientation.w); // 2d yaw = 2 * atan2(z, w)
  goal_pt_ = Vec3f(goal_x, goal_y, goal_yaw);

  path_points_size_ = global_plan_.poses.size();
  
  path_arc_lengths_ = Eigen::Tensor<float, 2, Eigen::RowMajor>(path_points_size_, 1); // 累积弧长数组
  path_points_      = Eigen::Tensor<float, 2, Eigen::RowMajor>(path_points_size_, 3);  // x, y, yaw
  float cum_len = 0.0;
  path_arc_lengths_(0) = cum_len;

  geometry_msgs::msg::TransformStamped transform;
  try {
      transform = tf_->lookupTransform(costmap_frame_id_, "map", tf2::TimePointZero);
  } catch (...) {
    RCLCPP_ERROR(node_->get_logger(), "[setPlan]:无法获取 map -> %s 的变换", costmap_frame_id_.c_str());
    return;
  }

  // 提取第一个点
  geometry_msgs::msg::PoseStamped pose_in_odom;
  tf2::doTransform(global_plan_.poses[0], pose_in_odom, transform);
  float x0 = pose_in_odom.pose.position.x;
  float y0 = pose_in_odom.pose.position.y;
  float yaw0 = tf2::getYaw(pose_in_odom.pose.orientation);
  path_points_(0, 0) = x0;   path_points_(0, 1) = y0;   path_points_(0, 2) = yaw0;

  for (size_t i = 1; i < path_points_size_; ++i) {
    // 转到odom系
    tf2::doTransform(global_plan_.poses[i], pose_in_odom, transform);
    float x1 = pose_in_odom.pose.position.x;   float y1 = pose_in_odom.pose.position.y;
    float dx = x1 - x0;                        float dy = y1 - y0;
    float seg_len = std::hypot(dx, dy);
    cum_len += seg_len;
    float yaw1 = std::atan2(dy, dx);  // 朝向无效，用切线方向代替
    path_arc_lengths_(i) = cum_len;
    path_points_(i, 0) = x1;   path_points_(i, 1) = y1;   path_points_(i, 2) = yaw1;
    x0 = x1;                   y0 = y1;
  }

  copyPathToDevice();
  
  RCLCPP_INFO(node_->get_logger(), "Path arc length: %.2f m", cum_len);
  RCLCPP_INFO(node_->get_logger(), "new global plan size: %zu", path_points_size_);
}

void MPPIController::setSpeedLimit(const double &speed_limit,
                                     const bool &percentage) {
  (void)percentage;
  (void)speed_limit;
}

void MPPIController::calc_total_costs( const Vec3f &start_state ){
  // 确保设备内存已分配
  if (d_costs_ == nullptr) {
      allocateDeviceMemory();
  }
  // 拷贝代价地图（每次控制周期都需要更新）
  copyCostmapToDevice();

  // 拷贝 u_prev_ 到设备（注意 u_prev_ 是 Eigen::Tensor，需要提取数据指针）
  // 注意：u_prev_ 尺寸为 (step_T, dim_u)
  float *h_u_prev = u_prev_.data(); // Eigen::Tensor 数据是连续的
  float *d_u_prev = nullptr;
  cudaMalloc(&d_u_prev, opts_.step_T * opts_.dim_u * sizeof(float));
  cudaMemcpy(d_u_prev, h_u_prev, opts_.step_T * opts_.dim_u * sizeof(float), cudaMemcpyHostToDevice);
  // RCLCPP_INFO(node_->get_logger(), "Before kernel: sigma_0=%.6f, sigma_1=%.6f", opts_.sigma(0,0), opts_.sigma(1,1));
  // 启动 kernel
  int threads = 256;
  int blocks = (opts_.samples_K + threads - 1) / threads;
  computeTrajectoryCostsKernel<<<blocks, threads>>>(
    d_costs_,
    d_epsilon_,
    d_u_prev,
    d_path_points_,
    d_arc_lengths_,
    prev_waypoints_idx_,
    path_points_size_,
    d_costmap_data_,
    d_costmap_size_x_, d_costmap_size_y_,
    d_costmap_resolution_, d_costmap_origin_x_, d_costmap_origin_y_,
    d_inflation_radius_, d_cost_scaling_factor_, d_inscribed_radius_,
    opts_.dt, opts_.step_T, opts_.max_v, opts_.max_w,
    opts_.gamma_dv, opts_.gamma_dw, opts_.gamma_v, opts_.gamma_w,
    opts_.std_v, opts_.std_w,
    opts_.sigma(0,0), opts_.sigma(1,1),
    opts_.stage_cost_weight.x(), opts_.stage_cost_weight.y(), opts_.stage_cost_weight.z(),
    opts_.terminal_cost_weight.x(), opts_.terminal_cost_weight.y(), opts_.terminal_cost_weight.z(),
    opts_.obstacle_cost_weight, opts_.critical_weight,
    opts_.collision_cost, opts_.collision_margin_distance,
    start_state.x(), start_state.y(), start_state.z(),
    stop_obstacle_, opts_.samples_K);

  cudaDeviceSynchronize();  // 等待 kernel 完成
  cudaError_t err = cudaGetLastError();
  if (err != cudaSuccess) {
      RCLCPP_ERROR(node_->get_logger(), "CUDA kernel error: %s", cudaGetErrorString(err));
  }

  // 将 costs 拷贝回主机
  cudaMemcpy(costs_.data(), d_costs_, opts_.samples_K * sizeof(float), cudaMemcpyDeviceToHost);

  // 拷贝 epsilon 回主机（epsilon_ 是 Eigen::Tensor，需要连续内存）
  cudaMemcpy(epsilon_.data(), d_epsilon_, opts_.samples_K * opts_.step_T * opts_.dim_u * sizeof(float), cudaMemcpyDeviceToHost);

  // 释放临时设备内存
  cudaFree(d_u_prev);

  // RCLCPP_INFO(node_->get_logger(), "epsilon[0,0,0]=%f, epsilon[0,0,1]=%f", epsilon_(0,0,0), epsilon_(0,0,1));
  // for ( size_t k = 0; k < opts_.samples_K; ++k ) {
  //   RCLCPP_INFO(node_->get_logger(), "c%d:%.2f", k, costs_[k]);
  // }
}

float MPPIController::calc_obstacle_cost( const Vec3f & state ){
  // 在此处读的是local_map，不是global_map，local_map的更新频率要高
  float x = state(0);
  float y = state(1);

  unsigned int mx, my;
  // 1. 检查点是否在地图内
  if (!costmap_->worldToMap(x, y, mx, my)) {
    return 0.0f; // 超出地图边界，假设无障碍
  }

  unsigned char cost = costmap_->getCost(mx, my);

  // 自由空间直接返回 0 代价
  // 否则可能导致 log(0) = -inf 进而 Velocity message contains NaNs or Infs! Ignoring as invalid!
  // 1. 自由空间（包括 cost 0 和 1? 通常 FREE_SPACE = 0）
  if (cost <= nav2_costmap_2d::FREE_SPACE) {  // FREE_SPACE 通常是 0
    return 0.0f;
  }
  
  // 2. 未知区域（可选处理）
  if (cost == nav2_costmap_2d::NO_INFORMATION) {
    return 0.0f;  // 或返回一个小惩罚，但不要返回 inf
  }
  
  // 3. 致命障碍物
  if (cost >= nav2_costmap_2d::LETHAL_OBSTACLE) {
    return opts_.collision_cost;
  }

  // 4. 此时 cost 范围应为 1~253
  // 如果没有膨胀层参数，则使用原始代价比例
  if (opts_.inflation_radius <= 0.0f || opts_.cost_scaling_factor <= 0.0f) {
    if (cost <= nav2_costmap_2d::FREE_SPACE) {
      return 0.0f;
    }
    // 将代价值 (0-254) 线性缩放到 [0,1] 区间
    return opts_.obstacle_cost_weight * (static_cast<float>(cost) / 254.0f);
  }

  // 5. 有膨胀层参数，计算到障碍物的真实距离
  // 膨胀层代价模型: cost = 254 * exp(-scale_factor * (dist - inscribed_radius))
  const float min_radius = costmap_ros_->getLayeredCostmap()->getInscribedRadius();
  const float scale_factor = opts_.cost_scaling_factor;
  
  // 解算距离公式: dist = (log(254) - log(cost)) / scale_factor + inscribed_radius
  // float cost_f = std::max(static_cast<float>(cost), 1.0f);
  float dist_to_obstacle = (std::log(254.0f) - std::log(static_cast<float>(cost))) / scale_factor + min_radius;

  // 如果代价小于最小内接圆半径内的代价，直接使用最小半径距离
  if (dist_to_obstacle < min_radius) {
    dist_to_obstacle = min_radius;
  }

  // 5. 计算惩罚项
  float penalty = 0.0f;
  const float max_penalty_dist = opts_.inflation_radius;
  if (dist_to_obstacle <= opts_.collision_margin_distance) {
    // 严重惩罚 (安全边距内)
    penalty += opts_.critical_weight * (opts_.collision_margin_distance - dist_to_obstacle);
  } else if (dist_to_obstacle < max_penalty_dist) {
    // 一般避障惩罚 (膨胀半径内)
    penalty += opts_.obstacle_cost_weight * (max_penalty_dist - dist_to_obstacle);
  }

  // if ( cost != 0 ) {
  //   RCLCPP_INFO(node_->get_logger(), "%u, %.2f, %.2f", cost, dist_to_obstacle, penalty);
  // }
  
  return penalty;
}

float MPPIController::calc_input_cost( const Vec2f & u, const size_t & t ){
  // u.transpose() * opts_.sigma.inverse() * u;
  float du_v = u.x() - u_prev_(t, 0);
  float du_w = u.y() - u_prev_(t, 1);
  float du_cost = 
      opts_.gamma_dv * du_v * du_v / opts_.sigma(0,0) + 
      opts_.gamma_dw * du_w * du_w / opts_.sigma(1,1);
  float u_cost  = 
      opts_.gamma_v  * u.x()*u.x() / opts_.sigma(0,0) + 
      opts_.gamma_w  * u.y()*u.y() / opts_.sigma(1,1);
  float total_cost = du_cost + u_cost;
  return total_cost;
}

float MPPIController::calc_stage_cost(const Vec3f &state, const size_t &t) {
  Vec2f search_pt(state[0], state[1]);
  size_t proj_idx = get_projected_waypoint(search_pt);

  // 根据当前速度预测前向弧长距离（也可以使用固定值）
  float forward_dist = std::max(0.5f, std::abs(u_prev_(0,0)) * opts_.dt * t);
  float target_arc = path_arc_lengths_(proj_idx) + forward_dist;

  // 手动实现二分查找：找到第一个弧长 >= target_arc 的索引 Olog(n)
  size_t low = 0;
  size_t high = path_points_size_ - 1;
  size_t target_idx = high;  // 默认最后一个（若所有弧长都小于 target_arc）

  while (low <= high) {
    size_t mid = low + (high - low) / 2;
    if (path_arc_lengths_(mid) >= target_arc) {
      target_idx = mid;
      if (mid == 0) break;   // 已经是第一个，无需继续
      high = mid - 1;        // 继续向左寻找更小的满足条件的索引
    } else {
      low = mid + 1;         // 向右搜索
    }
  }

  
  // 获取参考点坐标和朝向
  float ref_x = path_points_(target_idx, 0);
  float ref_y = path_points_(target_idx, 1);
  float ref_yaw = path_points_(target_idx, 2);
  
  float dx = ref_x - state[0];
  float dy = ref_y - state[1];
  float dyaw = ref_yaw - state[2];
  // if (dyaw < -M_PI) dyaw += M_2PI_;
  // if (dyaw >  M_PI) dyaw -= M_2PI_;
  if (dyaw < -M_PI_2) dyaw += M_PI;
  if (dyaw >  M_PI_2) dyaw -= M_PI;
  
  return opts_.stage_cost_weight[0] * dx*dx +
          opts_.stage_cost_weight[1] * dy*dy +
          opts_.stage_cost_weight[2] * dyaw*dyaw;
}

float MPPIController::calc_terminal_cost( const Vec3f & state ) {
  Vec2f search_pt(state[0], state[1]);
  size_t proj_idx = get_projected_waypoint(search_pt);
  float forward_dist = std::abs(u_prev_(0,0)) * opts_.step_T * opts_.dt;
  float target_arc = path_arc_lengths_(proj_idx) + forward_dist;

  // 找到目标弧长对应的路径点索引
  size_t target_idx = proj_idx;
  for (size_t i = proj_idx; i < path_points_size_; ++i) {
    if (path_arc_lengths_(i) >= target_arc) {
        target_idx = i;
        break;
    }
  }
  target_idx = std::min(target_idx, path_points_size_ - 1);
  
  // 获取参考点坐标和朝向
  float ref_x = path_points_(target_idx, 0);
  float ref_y = path_points_(target_idx, 1);
  float ref_yaw = path_points_(target_idx, 2);
  
  float dx = ref_x - state[0];
  float dy = ref_y - state[1];
  float dyaw = ref_yaw - state[2];
  // if (dyaw < -M_PI) dyaw += M_2PI_;
  // if (dyaw >  M_PI) dyaw -= M_2PI_;
  // 限制到-PI/2到-PI/2区间内可以倒着跑
  if (dyaw < -M_PI_2) dyaw += M_PI;
  if (dyaw >  M_PI_2) dyaw -= M_PI;

  float terminal_cost =  opts_.terminal_cost_weight[0] * dx * dx + 
                          opts_.terminal_cost_weight[1] * dy * dy + 
                          opts_.terminal_cost_weight[2] * dyaw * dyaw;
  RCLCPP_INFO(node_->get_logger(), "terminal cost: %.2f, dyaw:%.2f", terminal_cost, dyaw);
  return terminal_cost;
}

void MPPIController::calc_weights() {
  // Implementation for calculating weights
  // 找到最小的代价
  float rho = std::numeric_limits<float>().max();
  for (size_t k = 0; k < opts_.samples_K; ++k) {
    if ( costs_[k] < rho ) {
      rho = costs_[k];
    }
  }

  if ( std::isinf(rho) ) {
    RCLCPP_WARN(node_->get_logger(), "rho is Inf!!!!!");
    rho = min_cost_;
  }

  min_cost_ = rho;
  float eta = 0.f;
  float inv_temp = -1.f / opts_.lambda;

  // 可以考虑使用并发计算
  Eigen::ArrayXf buffer_weights(opts_.samples_K);

  std::for_each(std::execution::par_unseq, index_K_.begin(), index_K_.end(),
    [&](const size_t & k){ buffer_weights[k] = std::exp( inv_temp * ( costs_[k] - rho ) ); });
  
  eta = buffer_weights.sum();

  std::for_each(std::execution::par_unseq, index_K_.begin(), index_K_.end(),
    [&](const size_t & k) { weights_[k] = buffer_weights[k]  / eta;  });
}

void MPPIController::calc_control_seq(){
  std::for_each(std::execution::par_unseq, index_T_.begin(), index_T_.end(),
    [&](const size_t & t) {
      w_epsilon_(t,0) = 0.0;
      w_epsilon_(t,1) = 0.0;
      
      for ( size_t k = 0; k < opts_.samples_K; ++ k ) {
        w_epsilon_(t,0) += weights_[k] * epsilon_(k, t, 0);
        w_epsilon_(t,1) += weights_[k] * epsilon_(k, t, 1);
      }

      u_prev_(t, 0) += w_epsilon_(t,0);
      u_prev_(t, 1) += w_epsilon_(t,1);

      // 限幅  TODO
      Vec2f u(u_prev_(t, 0), u_prev_(t, 1));
      limit_input(u);
      u_prev_(t, 0) = u(0);
      u_prev_(t, 1) = u(1);
      
    });
}

/**
 * 根据当前状态和控制输入计算下一个状态
 * @param state 当前状态向量 [x, y, yaw]
 * @param input 控制输入向量 [v, omega]
 * @return 下一个状态向量 [x_next, y_next, yaw_next]
 */
Vec3f MPPIController::calc_next_state( const Vec3f & state, const Vec2f & input ){
  Vec3f next_state;
  next_state[0] = state[0] + input[0] * std::cos(state[2]) * opts_.dt;
  next_state[1] = state[1] + input[0] * std::sin(state[2]) * opts_.dt;
  next_state[2] = state[2] + input[1] * opts_.dt;
  return next_state;
}

void MPPIController::limit_input( Vec2f & input ) {
  input[0] = std::clamp(input[0], -opts_.max_v, opts_.max_v);
  input[1] = std::clamp(input[1], -opts_.max_w, opts_.max_w);
}

size_t MPPIController::get_projected_waypoint(const Vec2f &pt) {
  // 在局部窗口内找到真正的投影点（基于弧长）
  // 方法：找到路径上与 pt 最近的两个点，然后根据投影比例插值弧长
  size_t best_idx = 0;
  float best_dist = std::numeric_limits<float>::max();
  
  // 基于 prev_waypoints_idx_ 或上一次的投影点，在局部窗口内搜索（前后各10个点）。
  const size_t num = 10;
  size_t start = (prev_waypoints_idx_ > num) ? prev_waypoints_idx_ - num : 0;
  size_t end = std::min(prev_waypoints_idx_ + num, path_points_size_ - 1);
  for (size_t i = start; i <= end; ++i) {
      float dx = path_points_(i,0) - pt(0);
      float dy = path_points_(i,1) - pt(1);
      float dist = dx*dx + dy*dy;
      if (dist < best_dist) {
          best_dist = dist;
          best_idx = i;
      }
  }
  
  if (best_idx >= path_points_size_) {
    best_idx = path_points_size_ - 1;
  }
  // 确保投影点不会小于上一个参考点（避免倒退）
  if (best_idx < prev_waypoints_idx_) {
      best_idx = prev_waypoints_idx_;
  }
  return best_idx;
}

size_t MPPIController::get_nearest_waypoint( Vec2f & pt, bool update_prev_idx ) {
  // 搜索起点：上一个最近点（不允许回到更早的点）
  size_t start_idx = prev_waypoints_idx_;
  if (start_idx >= path_points_size_) start_idx = 0;
  
  // 搜索终点：限制窗口大小，避免全路径搜索（效率）
  size_t end_idx = std::min(start_idx + opts_.step_T, path_points_size_);
  
  float min_dist = std::numeric_limits<float>::max();
  size_t best_idx = start_idx;
  
  // 跳着搜索
  for (size_t i = start_idx; i < end_idx; i+=2) {
      float dx = path_points_(i, 0) - pt.x();
      float dy = path_points_(i, 1) - pt.y();
      float dist = dx*dx + dy*dy;  // 用平方距离，省去sqrt
      if (dist < min_dist) {
          min_dist = dist;
          best_idx = i;
      }
  }
  
  if (update_prev_idx) {
    prev_waypoints_idx_ = best_idx;
  }
  return best_idx;
}

void MPPIController::smooth_control_seq(){
  // 一定一定一定要滑动滤波一下，这东西突变非常大, 角速度能从-0.4调到正数
  // 三点均值平滑（注意边界）
  Eigen::Tensor<float, 2, Eigen::RowMajor> u_smooth(opts_.step_T, opts_.dim_u);
  u_smooth.setZero();
  u_smooth(0, 0) = u_prev_(0, 0);
  u_smooth(0, 1) = u_prev_(0, 1);
  size_t idx = opts_.step_T - 1;
  u_smooth(idx, 0) = u_prev_(idx, 0);
  u_smooth(idx, 1) = u_prev_(idx, 1);
  for (size_t t = 1; t < opts_.step_T - 1; ++t) {
      u_smooth(t, 0) = (u_prev_(t-1, 0) + u_prev_(t, 0) + u_prev_(t+1, 0)) / 3.0f;
      u_smooth(t, 1) = (u_prev_(t-1, 1) + u_prev_(t, 1) + u_prev_(t+1, 1)) / 3.0f;
  }
  u_prev_ = std::move(u_smooth);
}

void MPPIController::shift_control_seq() {
  // 将控制序列向前移动一位，丢弃第一个控制量，最后一个复制前一个
  for (size_t t = 0; t < opts_.step_T - 1; ++t) {
    u_prev_(t, 0) = u_prev_(t + 1, 0);
    u_prev_(t, 1) = u_prev_(t + 1, 1);
  }
  // 最后一个元素复制倒数第二个（保持序列长度不变）
  u_prev_(opts_.step_T - 1, 0) = u_prev_(opts_.step_T - 2, 0);
  u_prev_(opts_.step_T - 1, 1) = u_prev_(opts_.step_T - 2, 1);
}

void MPPIController::publish_local_plan(const Vec3f &start_state) {
  if (!local_plan_pub_ || local_plan_pub_->get_subscription_count() == 0) {
    return;  // 无人订阅，避免计算开销
  }

  std::thread( [this, start_state](){
    // 清空之前的路径
    nav_msgs::msg::Path local_plan;
    local_plan.header.stamp = node_->now();
    // 使用与costmap相同的 frame_id, 因为所有处理都在odom坐标系下进行
    local_plan.header.frame_id = costmap_frame_id_;  // "odom";

    // 从起始状态开始，根据控制序列积分得到轨迹点
    Vec3f state = start_state;
    // 添加起始点
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header = local_plan.header;
    pose_stamped.pose.position.x = state[0];
    pose_stamped.pose.position.y = state[1];
    pose_stamped.pose.position.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, state[2]);
    pose_stamped.pose.orientation = tf2::toMsg(q);
    local_plan.poses.push_back(pose_stamped);

    // 逐步积分
    for (size_t t = 0; t < opts_.step_T; ++t) {
      Vec2f u = Vec2f(u_prev_(t, 0), u_prev_(t, 1));
      state = calc_next_state(state, u);
      
      geometry_msgs::msg::PoseStamped pose;
      pose.header = local_plan.header;
      pose.pose.position.x = state[0];
      pose.pose.position.y = state[1];
      pose.pose.position.z = 0.0;
      tf2::Quaternion q_pose;
      q_pose.setRPY(0, 0, state[2]);
      pose.pose.orientation = tf2::toMsg(q_pose);
      local_plan.poses.push_back(pose);
    }

    // 发布
    local_plan_pub_->publish(local_plan);
  } ).detach();

}

void MPPIController::update_parameters(){
  float frequency;
  nav2_util::declare_parameter_if_not_declared(node_, plugin_name_ + ".frequency", rclcpp::ParameterValue(20.0));
  nav2_util::declare_parameter_if_not_declared(node_, plugin_name_ + ".max_v", rclcpp::ParameterValue(2.0));
  nav2_util::declare_parameter_if_not_declared(node_, plugin_name_ + ".max_w", rclcpp::ParameterValue(2.0));
  nav2_util::declare_parameter_if_not_declared(node_, plugin_name_ + ".step_T", rclcpp::ParameterValue(30));
  nav2_util::declare_parameter_if_not_declared(node_, plugin_name_ + ".samples_K", rclcpp::ParameterValue(100));
  nav2_util::declare_parameter_if_not_declared(node_, plugin_name_ + ".lambda", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(node_, plugin_name_ + ".gamma_dv", rclcpp::ParameterValue(0.001));
  nav2_util::declare_parameter_if_not_declared(node_, plugin_name_ + ".gamma_dw", rclcpp::ParameterValue(0.001));
  nav2_util::declare_parameter_if_not_declared(node_, plugin_name_ + ".gamma_v", rclcpp::ParameterValue(0.001));
  nav2_util::declare_parameter_if_not_declared(node_, plugin_name_ + ".gamma_w", rclcpp::ParameterValue(0.001));
  nav2_util::declare_parameter_if_not_declared(node_, plugin_name_ + ".sigma_v", rclcpp::ParameterValue(0.01));  // velocity
  nav2_util::declare_parameter_if_not_declared(node_, plugin_name_ + ".sigma_w", rclcpp::ParameterValue(0.01));  // omega
  nav2_util::declare_parameter_if_not_declared(node_, plugin_name_ + ".stage_cost_weight_x", rclcpp::ParameterValue(50.0));
  nav2_util::declare_parameter_if_not_declared(node_, plugin_name_ + ".stage_cost_weight_y", rclcpp::ParameterValue(50.0));
  nav2_util::declare_parameter_if_not_declared(node_, plugin_name_ + ".stage_cost_weight_yaw", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(node_, plugin_name_ + ".terminal_cost_weight_x", rclcpp::ParameterValue(50.0));
  nav2_util::declare_parameter_if_not_declared(node_, plugin_name_ + ".terminal_cost_weight_y", rclcpp::ParameterValue(50.0));
  nav2_util::declare_parameter_if_not_declared(node_, plugin_name_ + ".terminal_cost_weight_yaw", rclcpp::ParameterValue(1.0));
  // 避障参数
  nav2_util::declare_parameter_if_not_declared(node_, plugin_name_ + ".obstacle_cost_weight", rclcpp::ParameterValue(5.0)); // 一般避障权重
  nav2_util::declare_parameter_if_not_declared(node_, plugin_name_ + ".critical_weight", rclcpp::ParameterValue(100.0));    // 严重惩罚权重 (安全边距内)
  nav2_util::declare_parameter_if_not_declared(node_, plugin_name_ + ".collision_cost", rclcpp::ParameterValue(1000000.0)); // 严重惩罚权重 (安全边距内)
  nav2_util::declare_parameter_if_not_declared(node_, plugin_name_ + ".collision_margin_distance", rclcpp::ParameterValue(0.2)); // 安全边距 (米)
  nav2_util::declare_parameter_if_not_declared(node_, plugin_name_ + ".near_goal_distance", rclcpp::ParameterValue(0.5));        // 接近目标距离，此距离内停用一般避障项
  nav2_util::declare_parameter_if_not_declared(node_, plugin_name_ + ".inflation_radius", rclcpp::ParameterValue(0.55));         // 代价地图的膨胀半径
  nav2_util::declare_parameter_if_not_declared(node_, plugin_name_ + ".cost_scaling_factor", rclcpp::ParameterValue(3.0));       // 代价衰减系数：越大衰减越快（更敢贴边

  node_->get_parameter(plugin_name_ + ".frequency", frequency);
  opts_.dt = 1.0 / frequency;
  node_->get_parameter(plugin_name_ + ".max_v", opts_.max_v);
  node_->get_parameter(plugin_name_ + ".max_w", opts_.max_w);
  int T = 0;
  node_->get_parameter(plugin_name_ + ".step_T", T);
  opts_.step_T = static_cast<size_t>(T);
  int K = 0;
  node_->get_parameter(plugin_name_ + ".samples_K", K);
  opts_.samples_K = static_cast<size_t>(K);
  node_->get_parameter(plugin_name_ + ".lambda", opts_.lambda);
  node_->get_parameter(plugin_name_ + ".gamma_dv", opts_.gamma_dv);
  node_->get_parameter(plugin_name_ + ".gamma_dw", opts_.gamma_dw);
  node_->get_parameter(plugin_name_ + ".gamma_v", opts_.gamma_v);
  node_->get_parameter(plugin_name_ + ".gamma_w", opts_.gamma_w);
  float sigma_v, sigma_w;
  node_->get_parameter(plugin_name_ + ".sigma_v", sigma_v);
  node_->get_parameter(plugin_name_ + ".sigma_w", sigma_w);
  opts_.sigma.diagonal() << sigma_v, sigma_w;
  opts_.std_v = std::sqrt(opts_.sigma(0,0));
  opts_.std_w = std::sqrt(opts_.sigma(1,1));
  node_->get_parameter(plugin_name_ + ".stage_cost_weight_x", opts_.stage_cost_weight.x());
  node_->get_parameter(plugin_name_ + ".stage_cost_weight_y", opts_.stage_cost_weight.y());
  node_->get_parameter(plugin_name_ + ".stage_cost_weight_yaw", opts_.stage_cost_weight.z());
  node_->get_parameter(plugin_name_ + ".terminal_cost_weight_x", opts_.terminal_cost_weight.x());
  node_->get_parameter(plugin_name_ + ".terminal_cost_weight_y", opts_.terminal_cost_weight.y());
  node_->get_parameter(plugin_name_ + ".terminal_cost_weight_yaw", opts_.terminal_cost_weight.z());
  node_->get_parameter(plugin_name_ + ".obstacle_cost_weight", opts_.obstacle_cost_weight);
  node_->get_parameter(plugin_name_ + ".critical_weight", opts_.critical_weight);
  node_->get_parameter(plugin_name_ + ".collision_cost", opts_.collision_cost);
  node_->get_parameter(plugin_name_ + ".collision_margin_distance", opts_.collision_margin_distance);
  node_->get_parameter(plugin_name_ + ".near_goal_distance", opts_.near_goal_distance);
  node_->get_parameter(plugin_name_ + ".inflation_radius", opts_.inflation_radius);
  node_->get_parameter(plugin_name_ + ".cost_scaling_factor", opts_.cost_scaling_factor);

  u_prev_ = Eigen::Tensor<float, 2, Eigen::RowMajor>(opts_.step_T, opts_.dim_u);
  u_prev_.setZero();

  epsilon_ = Eigen::Tensor<float, 3, Eigen::RowMajor>(opts_.samples_K, opts_.step_T, opts_.dim_u);
  epsilon_.setZero();
  
  w_epsilon_ = Eigen::Tensor<float, 2, Eigen::RowMajor>(opts_.step_T, opts_.dim_u);
  w_epsilon_.setZero();

  costs_ = Eigen::ArrayXf(opts_.samples_K);
  costs_.setZero();
  
  weights_ = Eigen::ArrayXf(opts_.samples_K);
  weights_.setZero();

  index_K_ = std::vector<size_t>(opts_.samples_K, 0);
  for (size_t k = 0; k < opts_.samples_K; ++k ) {
    index_K_[k] = k;
  }

  index_T_ = std::vector<size_t>(opts_.step_T, 0);
  for (size_t t = 0; t < opts_.step_T; ++t ) {
    index_T_[t] = t;
  }

  freeDeviceMemory();
  // 分配 CUDA 内存
  allocateDeviceMemory();
}

} // namespace rm_mppi_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(rm_mppi_controller::MPPIController, nav2_core::Controller)