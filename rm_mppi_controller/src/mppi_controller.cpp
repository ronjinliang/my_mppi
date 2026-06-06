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
#include <execution>
#include <cmath>
#include <glog/logging.h>

namespace rm_mppi_controller {

MPPIController::~MPPIController(){
  if ( has_init_cl_ ) {
    releaseOpenCL();
  }
}

void MPPIController::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) {

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

  // OpenCL 相关变量
  initOpenCL();

  RCLCPP_INFO(node_->get_logger(),"update MPPI 控制器完成!!!!!!!!!!");
}

void MPPIController::deactivate() {
  releaseOpenCL();
  RCLCPP_INFO(node_->get_logger(), "停用控制器：%s 类型为 rm_mppi_controller::MPPIController", plugin_name_.c_str());
}

geometry_msgs::msg::TwistStamped MPPIController::computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped &pose,
    const geometry_msgs::msg::Twist &velocity,
    nav2_core::GoalChecker * goal_checker) {
  (void)velocity;
  (void)goal_checker;

  // 在 odom 系下计算，因为costmap的系就是odom，这样方便访问代价地图

  // std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

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
    opts_.use_obstacle_cost = false;
  } else {
    opts_.use_obstacle_cost = true;
  }

  // 寻找离当前车辆位置最近的参考路径点，并更新索引
  Vec2f search_pt = Vec2f( x0.x(), x0.y() );
  find_nearest_waypoint( search_pt );
  if ( prev_waypoints_idx_ >= path_points_size_ ) {
    throw nav2_core::PlannerException("无法找到有效的路径点");
  }

  // loop for 0 ~ K-1 samples  耗时较长  TODO
  std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
  calc_total_costs(x0);
  std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
  auto time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
  RCLCPP_INFO( node_->get_logger(), "[calc_total_costs]:%.5fs", time_used.count() );
  

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
  // std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
  // auto time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
  // RCLCPP_INFO( node_->get_logger(), "[computeVelocityCommands]:%.5fs", time_used.count() );
  
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

  /// OpenCL
  copyPathToDevice();
  
  // RCLCPP_INFO(node_->get_logger(), "Path arc length: %.2f m", cum_len);
  // RCLCPP_INFO(node_->get_logger(), "new global plan size: %zu", path_points_size_);
}

void MPPIController::setSpeedLimit(const double &speed_limit,
                                     const bool &percentage) {
  (void)percentage;
  (void)speed_limit;
}

void MPPIController::calc_total_costs( const Vec3f &start_state ){

  /// OpenCL
  copyCostmapToDevice();

  updateMPPIParams( start_state );

  // cl_u_prev_
  clEnqueueWriteBuffer( cl_queue_, cl_u_prev_, CL_TRUE, 0, sizeof(float) * opts_.step_T * opts_.dim_u, u_prev_.data(), 0, nullptr, nullptr );

  // 设置内核参数
  cl_ret_ = clSetKernelArg( cl_kernel_, 0, sizeof(cl_mem), &cl_u_prev_ );
  if ( cl_ret_ != CL_SUCCESS ) LOG(INFO) << "clSetKernelArg [cl_u_prev_] faild!" << cl_ret_;
  cl_ret_ = clSetKernelArg( cl_kernel_, 1, sizeof(cl_mem), &cl_costs_ );
  if ( cl_ret_ != CL_SUCCESS ) LOG(INFO) << "clSetKernelArg [cl_costs_] faild! " << cl_ret_;
  cl_ret_ = clSetKernelArg( cl_kernel_, 2, sizeof(cl_mem), &cl_epsilon_ );
  if ( cl_ret_ != CL_SUCCESS ) LOG(INFO) << "clSetKernelArg [cl_epsilon_] faild! " << cl_ret_;
  cl_ret_ = clSetKernelArg( cl_kernel_, 3, sizeof(cl_mem), &cl_path_points_ );
  if ( cl_ret_ != CL_SUCCESS ) LOG(INFO) << "clSetKernelArg [cl_path_points_] faild! " << cl_ret_;
  cl_ret_ = clSetKernelArg( cl_kernel_, 4, sizeof(cl_mem), &cl_arc_lengths_ );
  if ( cl_ret_ != CL_SUCCESS ) LOG(INFO) << "clSetKernelArg [cl_arc_lengths_] faild! " << cl_ret_;
  cl_ret_ = clSetKernelArg( cl_kernel_, 5, sizeof(cl_mem), &cl_costmap_ );
  if ( cl_ret_ != CL_SUCCESS ) LOG(INFO) << "clSetKernelArg [cl_costmap_] faild! " << cl_ret_;
  cl_ret_ = clSetKernelArg( cl_kernel_, 6, sizeof(cl_mem), &cl_mps_buf_ );
  if ( cl_ret_ != CL_SUCCESS ) LOG(INFO) << "clSetKernelArg [cl_mps_] faild! " << cl_ret_;
  uint global_seed = node_->get_clock()->now().nanoseconds();
  cl_ret_ = clSetKernelArg( cl_kernel_, 7, sizeof(uint), &global_seed );
  if ( cl_ret_ != CL_SUCCESS ) LOG(INFO) << "clSetKernelArg [global_seed] faild! " << cl_ret_;

  // 执行内核的核心函数, 调用是非阻塞，函数立即返回，内核在设备端异步运行。
  // command_queue	命令队列，内核将在此队列上执行
  // kernel	要执行的内核对象（由 clCreateKernel 创建）
  // work_dim	工作空间维度，1、2 或 3
  // global_work_offset	全局偏移量，通常设为 NULL（表示从 0 开始）
  // global_work_size	每个维度的工作项总数（如 {1024} 表示 1024 个线程）
  // local_work_size	每个工作组的工作项数，可为 NULL（由实现自动选择）
  // num_events_in_wait_list	等待事件数量，通常为 0
  // event_wait_list	等待的事件列表，通常为 NULL
  // event	返回的事件对象，用于同步，不需要时可为 NULL
  size_t global_size = opts_.samples_K;  // 线程数
  cl_ret_ = clEnqueueNDRangeKernel( cl_queue_, cl_kernel_, 1, nullptr, &global_size, nullptr, 0, nullptr, nullptr );
  if ( cl_ret_ != CL_SUCCESS ) LOG(INFO) << "clEnqueueNDRangeKernel faild! " << cl_ret_;

  // 等待执行
  clFinish( cl_queue_ );

  // 读取结果
  // queue,          // 命令队列
  // d_buffer,       // 设备端缓冲区对象
  // CL_TRUE,        // 阻塞标志（BLOCKING）
  // 0,              // 偏移量（字节）
  // size_bytes,     // 要读取的字节数
  // host_ptr,       // 主机端内存指针（必须已分配空间）
  // 0,              // 等待事件数量
  // NULL,           // 等待事件列表
  // NULL            // 返回的事件对象（不需要可为 NULL）
  cl_ret_ = clEnqueueReadBuffer( cl_queue_, 
    cl_costs_, CL_TRUE, 0,  opts_.samples_K * sizeof(float), costs_.data(), 0, nullptr, nullptr );
  if ( cl_ret_ != CL_SUCCESS ) LOG(INFO) << "clEnqueueReadBuffer costs_ failed! " << cl_ret_;
  cl_ret_ = clEnqueueReadBuffer( cl_queue_, 
    cl_epsilon_, CL_TRUE, 0, opts_.samples_K * opts_.step_T * opts_.dim_u * sizeof(float), epsilon_.data(), 0, nullptr, nullptr );
  if ( cl_ret_ != CL_SUCCESS ) LOG(INFO) << "clEnqueueReadBuffer epsilon_ failed! " << cl_ret_;
  // LOG(INFO) << costs_[0] << ", " << epsilon_(0,0,0) << ", " << epsilon_(0,0,1);
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
  next_state[0] = state[0] + input[0] * cosf(state[2]) * opts_.dt;
  next_state[1] = state[1] + input[0] * sinf(state[2]) * opts_.dt;
  next_state[2] = state[2] + input[1] * opts_.dt;
  return next_state;
}

void MPPIController::limit_input( Vec2f & input ) {
  input[0] = std::clamp(input[0], -opts_.max_v, opts_.max_v);
  input[1] = std::clamp(input[1], -opts_.max_w, opts_.max_w);
}

size_t MPPIController::find_nearest_waypoint( Vec2f & pt ) {
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
  
  prev_waypoints_idx_ = best_idx;
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
  nav2_util::declare_parameter_if_not_declared(node_, plugin_name_ + ".kernel_file", rclcpp::ParameterValue("/home/lrj/RM/test/MPPI_test/MPPI_opencl/rm_mppi_controller/kernel/kernel_func.cl"));
  nav2_util::declare_parameter_if_not_declared(node_, plugin_name_ + ".kernel_name", rclcpp::ParameterValue("calc_total_cost"));
  nav2_util::declare_parameter_if_not_declared(node_, plugin_name_ + ".mppi_params_dir", rclcpp::ParameterValue("/home/lrj/RM/test/MPPI_test/MPPI_opencl/rm_mppi_controller/include/rm_mppi_controller"));

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
  opts_.std_v = sqrtf(opts_.sigma(0,0));
  opts_.std_w = sqrtf(opts_.sigma(1,1));
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
  node_->get_parameter(plugin_name_ + ".kernel_file", cl_kernel_file_);
  node_->get_parameter(plugin_name_ + ".kernel_name", cl_kernel_name_);
  node_->get_parameter(plugin_name_ + ".mppi_params_dir", cl_mppi_params_dir_);

  LOG(INFO) << "cl_kernel_file_: " << cl_kernel_file_;
  LOG(INFO) << "cl_kernel_name_: " << cl_kernel_name_;
  LOG(INFO) << "cl_mppi_params_dir_: " << cl_mppi_params_dir_;

  cl_mps_.dim_x                     = opts_.dim_x;       // 系统状态向量维度
  cl_mps_.dim_u                     = opts_.dim_u;       // 控制输入向量维度
  cl_mps_.dt                        = opts_.dt;          // 离散时间步长
  cl_mps_.max_v                     = opts_.max_v;       // max velocity
  cl_mps_.max_w                     = opts_.max_w;       // max omega
  cl_mps_.step_T                    = opts_.step_T;      // 预测时域长度
  cl_mps_.samples_K                 = opts_.samples_K;   // 采样轨迹数量
  cl_mps_.lambda                    = opts_.lambda;      // MPPI的温度参数，影响权重分布
  cl_mps_.gamma_dv                  = opts_.gamma_dv;    // cost of velocity input
  cl_mps_.gamma_dw                  = opts_.gamma_dw;    // cost of omega input
  cl_mps_.gamma_v                   = opts_.gamma_v;     // cost of velocity input
  cl_mps_.gamma_w                   = opts_.gamma_w;     // cost of omega input
  cl_mps_.sigma_v                   = opts_.sigma(0,0);  // 噪声协方差矩阵
  cl_mps_.sigma_w                   = opts_.sigma(1,1);  // 噪声协方差矩阵
  cl_mps_.std_v                     = opts_.std_v;
  cl_mps_.std_w                     = opts_.std_w;
  cl_mps_.stage_cost_weight_x       = opts_.stage_cost_weight(0);     // 阶段成本权重  x
  cl_mps_.stage_cost_weight_y       = opts_.stage_cost_weight(1);     // 阶段成本权重  y
  cl_mps_.stage_cost_weight_yaw     = opts_.stage_cost_weight(2);     // 阶段成本权重  yaw
  cl_mps_.terminal_cost_weight_x    = opts_.terminal_cost_weight(0);  // 终端成本权重  x
  cl_mps_.terminal_cost_weight_y    = opts_.terminal_cost_weight(1);  // 终端成本权重  y
  cl_mps_.terminal_cost_weight_yaw  = opts_.terminal_cost_weight(2);  // 终端成本权重  yaw
  cl_mps_.obstacle_cost_weight      = opts_.obstacle_cost_weight;     // 一般避障权重
  cl_mps_.critical_weight           = opts_.critical_weight;          // 严重惩罚权重 (安全边距内)
  cl_mps_.collision_cost            = opts_.collision_cost;           // 碰撞代价
  cl_mps_.collision_margin_distance = opts_.collision_margin_distance;// 安全边距 (米)
  cl_mps_.near_goal_distance        = opts_.near_goal_distance;       // 接近目标距离，此距离内停用一般避障项 unused
  
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

  RCLCPP_INFO(node_->get_logger(), "更新参数：frequency=%.2f, max_v=%.2f, max_w=%.2f, step_T=%zu, samples_K=%zu, lambda=%.2f, sigma_v=%.4f, sigma_w=%.4f",
    frequency, opts_.max_v, opts_.max_w, opts_.step_T, opts_.samples_K, opts_.lambda, sigma_v, sigma_w);
}

/********* OpenCL ******/

char* MPPIController::read_kernel_file(const char* filename, size_t* length) {
    FILE* fp = fopen(filename, "rb");
    if (!fp) return NULL;
    fseek(fp, 0, SEEK_END);                  // fseek 跳到文件末尾
    *length = ftell(fp);                     // ftell 得到字节数 length
    rewind(fp);                              // rewind 回到开头
    char* src = (char*)malloc(*length + 1);  // malloc(length + 1) 多申请 1 字节用于存放字符串结束符 '\0'
    fread(src, 1, *length, fp);              // fread 将整个文件读入内存
    src[*length] = '\0';                     // src[*length] = '\0'，使内容成为合法的 C 字符串
    fclose(fp);                              // 返回指向内存块的指针
    return src;
}

void MPPIController::initOpenCL(){
  // 1. 获取平台与设备
  cl_ret_ = clGetPlatformIDs(1, &cl_platform_, nullptr);
  if ( cl_ret_ != CL_SUCCESS ) LOG(INFO) << "clGetPlatformIDs failed! " << cl_ret_;
  cl_ret_ = clGetDeviceIDs(cl_platform_, CL_DEVICE_TYPE_GPU, 1, &cl_device_, nullptr);
  if ( cl_ret_ != CL_SUCCESS ) LOG(INFO) << "clGetDeviceIDs failed! " << cl_ret_;

  // 2. 创建上下文和命令队列
  cl_context_ = clCreateContext(nullptr, 1, &cl_device_, nullptr, nullptr, &cl_ret_);
  if ( cl_ret_ != CL_SUCCESS ) LOG(INFO) << "clCreateContext failed! " << cl_ret_;
  cl_queue_ = clCreateCommandQueue(cl_context_, cl_device_, 0, &cl_ret_);
  if ( cl_ret_ != CL_SUCCESS ) LOG(INFO) << "clCreateCommandQueue failed! " << cl_ret_;

  // 3. 加载并编译内核
  size_t kernel_len;
  char * kernel_src = read_kernel_file(cl_kernel_file_.c_str(), &kernel_len);
  cl_program_ = clCreateProgramWithSource(cl_context_, 1, (const char **)&kernel_src, &kernel_len, &cl_ret_);
  if ( cl_ret_ != CL_SUCCESS ) LOG(INFO) << "clCreateProgramWithSource failed! " << cl_ret_;
  free(kernel_src);
  std::string options = "-I " + cl_mppi_params_dir_;
  cl_ret_ = clBuildProgram(cl_program_, 1, &cl_device_, options.c_str(), nullptr, nullptr);
  if ( cl_ret_ != CL_SUCCESS ) LOG(INFO) << "clBuildProgram failed! " << cl_ret_;

  // 4.创建内核对象
  cl_kernel_ = clCreateKernel(cl_program_, cl_kernel_name_.c_str(), &cl_ret_);
  if ( cl_ret_ != CL_SUCCESS ) LOG(INFO) << "clCreateKernel failed! " << cl_ret_;

  // 5.创建缓冲区
  cl_u_prev_  = clCreateBuffer(cl_context_,
    CL_MEM_READ_ONLY, sizeof(float) * opts_.step_T * opts_.dim_u, nullptr, &cl_ret_ );
  if ( cl_ret_ != CL_SUCCESS ) LOG(INFO) << "clCreateBuffer cl_u_prev_ failed! " << cl_ret_;
  cl_costs_   = clCreateBuffer(cl_context_, 
    CL_MEM_WRITE_ONLY, sizeof(float) * opts_.samples_K, nullptr, &cl_ret_);
  if ( cl_ret_ != CL_SUCCESS ) LOG(INFO) << "clCreateBuffer cl_costs_ failed! " << cl_ret_;
  cl_epsilon_ = clCreateBuffer(cl_context_, 
    CL_MEM_WRITE_ONLY, sizeof(float) * opts_.samples_K * opts_.step_T * opts_.dim_u, nullptr, &cl_ret_);
  if ( cl_ret_ != CL_SUCCESS ) LOG(INFO) << "clCreateBuffer cl_epsilon_ failed! " << cl_ret_;
  cl_mps_buf_ = clCreateBuffer(cl_context_,
    CL_MEM_READ_ONLY, sizeof(float) * 41, nullptr, &cl_ret_);
  if ( cl_ret_ != CL_SUCCESS ) LOG(INFO) << "clCreateBuffer cl_mps_buf_ failed! " << cl_ret_;
  has_init_cl_ = true;
}

void MPPIController::releaseOpenCL(){
  if (cl_u_prev_)      { clReleaseMemObject(cl_u_prev_);      cl_u_prev_ = nullptr; }
  if (cl_costs_)       { clReleaseMemObject(cl_costs_);       cl_costs_ = nullptr; }
  if (cl_epsilon_)     { clReleaseMemObject(cl_epsilon_);     cl_epsilon_ = nullptr; }
  if (cl_path_points_) { clReleaseMemObject(cl_path_points_); cl_path_points_ = nullptr; }
  if (cl_arc_lengths_) { clReleaseMemObject(cl_arc_lengths_); cl_arc_lengths_ = nullptr; }
  if (cl_costmap_)     { clReleaseMemObject(cl_costmap_);     cl_costmap_ = nullptr; }
  if (cl_mps_buf_)     { clReleaseMemObject(cl_mps_buf_);     cl_mps_buf_ = nullptr; }
  if (cl_kernel_)      { clReleaseKernel(cl_kernel_);         cl_kernel_ = nullptr; }
  if (cl_program_)     { clReleaseProgram(cl_program_);       cl_program_ = nullptr; }
  if (cl_queue_)       { clReleaseCommandQueue(cl_queue_);    cl_queue_ = nullptr; }
  if (cl_context_)     { clReleaseContext(cl_context_);       cl_context_ = nullptr; }
  if (cl_device_)      { clReleaseDevice(cl_device_);         cl_device_ = nullptr; }
  has_init_cl_ = false;
}

void MPPIController::copyPathToDevice(){
  if ( path_points_size_ == 0 ) return;
  if ( cl_path_points_ ) {
    clReleaseMemObject( cl_path_points_ );
    cl_path_points_ = nullptr;
  }
  if ( cl_arc_lengths_ ) {
    clReleaseMemObject( cl_arc_lengths_ );
    cl_arc_lengths_ = nullptr;
  }
  // 创建 buffer 同时拷贝数据
  cl_path_points_ = clCreateBuffer(cl_context_,
    CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR, sizeof(float) * path_points_size_ * 3, path_points_.data(), &cl_ret_ );
  if ( cl_ret_ != CL_SUCCESS ) LOG(INFO) << "clCreateBuffer cl_path_points_ failed! " << cl_ret_;
  cl_arc_lengths_ = clCreateBuffer(cl_context_,
    CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR, sizeof(float) * path_points_size_, path_arc_lengths_.data(), &cl_ret_ );
  if ( cl_ret_ != CL_SUCCESS ) LOG(INFO) << "clCreateBuffer cl_arc_lengths_ failed! " << cl_ret_;

  cl_mps_.path_points_size = path_points_size_;
}

void MPPIController::copyCostmapToDevice(){
  if (!costmap_) return;
  unsigned int size_x = costmap_->getSizeInCellsX();
  unsigned int size_y = costmap_->getSizeInCellsY();
  size_t map_bytes = size_x * size_y * sizeof(unsigned char);
  // 如果大小改变或未分配，重新分配
  if ( cl_costmap_ == nullptr || cl_mps_.costmap_size_x != (int)size_x || cl_mps_.costmap_size_y != (int)size_y ) {
    if ( cl_costmap_ ) clReleaseMemObject( cl_costmap_ );
    cl_costmap_ = nullptr;
    // 创建新的缓冲区并拷贝数据
    cl_costmap_ = clCreateBuffer(cl_context_,
      CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR, map_bytes, costmap_->getCharMap(), &cl_ret_);
    if ( cl_ret_ != CL_SUCCESS ) LOG(INFO) << "clCreateBuffer cl_costmap_ failed! " << cl_ret_;

    cl_mps_.costmap_size_x = (int)size_x;
    cl_mps_.costmap_size_y = (int)size_y;
  } else {
    // 拷贝数据
    cl_ret_ = clEnqueueWriteBuffer(cl_queue_, cl_costmap_, CL_TRUE, 0, map_bytes, costmap_->getCharMap(), 0, nullptr, nullptr);
    if ( cl_ret_ != CL_SUCCESS ) LOG(INFO) << "clEnqueueWriteBuffer costmap_ failed! " << cl_ret_;
  }
}


void MPPIController::updateMPPIParams( const Vec3f &start_state ){
  cl_mps_.use_obstacle_cost = static_cast<int>(opts_.use_obstacle_cost); // 1:true 0:false

  cl_mps_.prev_waypoints_idx = prev_waypoints_idx_;
  cl_mps_.start_x   = start_state.x();
  cl_mps_.start_y   = start_state.y();
  cl_mps_.start_yaw = start_state.z();

  // costmap 部分的参数
  cl_mps_.costmap_cost_scaling_factor = opts_.cost_scaling_factor;
  cl_mps_.costmap_inflation_radius    = opts_.inflation_radius;
  cl_mps_.costmap_inscribed_radius    = costmap_ros_->getLayeredCostmap()->getInscribedRadius();
  cl_mps_.costmap_origin_x            = costmap_->getOriginX();
  cl_mps_.costmap_origin_y            = costmap_->getOriginY();
  cl_mps_.costmap_resolution          = costmap_->getResolution();

  clEnqueueWriteBuffer(cl_queue_, cl_mps_buf_, CL_TRUE, 0, 4*41, &cl_mps_, 0, nullptr, nullptr);
}

} // namespace rm_mppi_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(rm_mppi_controller::MPPIController, nav2_core::Controller)
