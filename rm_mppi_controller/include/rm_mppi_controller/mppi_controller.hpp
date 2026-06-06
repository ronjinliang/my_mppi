#ifndef RM_MPPI_CONTROLLER_HPP__
#define RM_MPPI_CONTROLLER_HPP__

#include <memory>
#include <string>
#include <vector>

#include "rm_mppi_controller/eigen_types.h"

#include "nav2_core/controller.hpp"
#include "nav2_util/robot_utils.hpp"

#include <cuda_runtime.h>

namespace rm_mppi_controller {

class MPPIController : public nav2_core::Controller {
public:
  struct Options{
    Options() = default;
    size_t dim_x = 3;                                        // 系统状态向量维度
    size_t dim_u = 2;                                        // 控制输入向量维度
    float dt = 0.05;                                      // 离散时间步长
    float max_v = 2.0;                                   // max velocity
    float max_w = 2.0;                                   // max omega
    size_t step_T = 30;                                      // 预测时域长度
    size_t samples_K = 100;                                  // 采样轨迹数量
    float lambda = 50.0;                                  // MPPI的温度参数，影响权重分布
    float gamma_dv = 0.001;                               // cost of velocity input
    float gamma_dw = 0.001;                                // cost of omega input
    float gamma_v = 0.001;                               // cost of velocity input
    float gamma_w = 0.001;                                // cost of omega input
    Mat2f sigma = Mat2f::Identity() * 0.01;                // 噪声协方差矩阵
    float std_v = sqrtf(sigma(0,0));
    float std_w = sqrtf(sigma(1,1));
    Vec3f stage_cost_weight = Vec3f(50.0, 50.0, 1.0);     // 阶段成本权重  x y yaw
    Vec3f terminal_cost_weight = Vec3f(50.0, 50.0, 1.0);  // 终端成本权重  x y yaw
    // 避障代价参数
    bool use_obstacle_cost = true;             // 是否启用避障代价  unused
    float obstacle_cost_weight = 5.0f;         // 一般避障权重
    float critical_weight = 100.0f;            // 严重惩罚权重 (安全边距内)
    float collision_cost = 1000000.0f;         // 碰撞代价
    float collision_margin_distance = 0.2f;    // 安全边距 (米)
    float near_goal_distance = 0.5f;           // 接近目标距离，此距离内停用一般避障项 unused
    // 代价地图的参数
    float inflation_radius = 0.55f;
    float cost_scaling_factor = 3.0f;
  };

public:
  MPPIController();
  ~MPPIController() override;
  void configure(
      const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, std::string name,
      std::shared_ptr<tf2_ros::Buffer> tf,
      std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void cleanup() override;
  void activate() override;
  void deactivate() override;
  geometry_msgs::msg::TwistStamped
  computeVelocityCommands(const geometry_msgs::msg::PoseStamped &pose,
                          const geometry_msgs::msg::Twist &velocity,
                          nav2_core::GoalChecker * goal_checker) override;
  void setPlan(const nav_msgs::msg::Path &path) override;
  void setSpeedLimit(const double &speed_limit,
                     const bool &percentage) override;

private:
  void calc_total_costs( const Vec3f &start_state );
  float calc_obstacle_cost( const Vec3f & state );  // 计算单点避障代价
  float calc_input_cost( const Vec2f & u, const size_t & t );
  float calc_stage_cost( const Vec3f & state, const size_t & t );
  float calc_terminal_cost( const Vec3f & state );
  void calc_weights();
  void calc_control_seq();
  Vec3f calc_next_state( const Vec3f & state, const Vec2f & input );
  void limit_input( Vec2f & input );
  size_t get_projected_waypoint(const Vec2f &pt);
  size_t get_nearest_waypoint( Vec2f & pt, bool update_prev_idx = false );
  void smooth_control_seq();
  void shift_control_seq();
  void publish_local_plan(const Vec3f &start_state);
  void update_parameters();

      // ---------- CUDA 相关成员 ----------
  void allocateDeviceMemory();    // 分配设备端内存
  void freeDeviceMemory();        // 释放设备端内存
  void copyPathToDevice();        // 将 path_points_ 和 arc_lengths 拷贝到 GPU
  void copyCostmapToDevice();     // 将当前代价地图数据拷贝到 GPU
protected:
  Options opts_;

  std::random_device rd_;
  std::mt19937 gen_;
  std::normal_distribution<float> noise_dist_{0.0, 1.0};
  const float M_2PI_ = 2.0 * M_PI;

  bool stop_obstacle_ = false;
  Vec3f goal_pt_ = Vec3f::Zero();
  size_t prev_waypoints_idx_ = 0;       // 上一次最近的路径点索引
  float min_cost_ = 0.0f;
  Eigen::Tensor<float, 2, Eigen::RowMajor> u_prev_;     // 存储上一次的控制输入序列 step_T * dim_u
  Eigen::Tensor<float, 3, Eigen::RowMajor> epsilon_;    // sample_K * step_T * dim_u
  Eigen::Tensor<float, 2, Eigen::RowMajor> w_epsilon_;  // 存储加权后的噪声        samples_K * dim_u
  Eigen::ArrayXf costs_;               // 存储每条采样轨迹的总成本 samples_K * 1
  Eigen::ArrayXf weights_;             // 存储每条采样轨迹的权重 samples_K
  Eigen::Tensor<float, 2, Eigen::RowMajor> path_arc_lengths_; // 路径累积弧长数组（从起点开始沿路径的累积距离）
  Eigen::Tensor<float, 2, Eigen::RowMajor> path_points_;  // x, y, yaw  // 路径点数组（存储每个点的位置和朝向，便于快速访问）
  size_t path_points_size_ = 0;

  // 用于并发计算，暂时先用串行计算
  std::vector<size_t> index_K_;
  std::vector<size_t> index_T_;

  std::string plugin_name_;                                     // 存储插件名称
  std::shared_ptr<tf2_ros::Buffer> tf_;                         // 存储坐标变换缓存指针，可用于查询坐标关系
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;  // 存储代价地图
  nav2_util::LifecycleNode::SharedPtr node_;                    // 存储节点指针
  nav2_costmap_2d::Costmap2D *costmap_;                         // 存储全局代价地图             /odom
  std::string costmap_frame_id_;
  nav_msgs::msg::Path global_plan_;                             // 存储 setPlan 提供的全局路径  /map

  // publish local plan for visualization
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_plan_pub_;

  // ----- 设备端指针 -----
  float *d_costs_ = nullptr;                // 设备端 costs 数组 (samples_K)
  float *d_epsilon_ = nullptr;
  float *d_path_points_ = nullptr;          // 设备端路径点 (N x 3)
  float *d_arc_lengths_ = nullptr;          // 设备端弧长 (N)
  unsigned char *d_costmap_data_ = nullptr; // 设备端代价地图数据
  int d_costmap_size_x_ = 0, d_costmap_size_y_ = 0;
  float d_costmap_resolution_ = 0.0;
  float d_costmap_origin_x_ = 0.0, d_costmap_origin_y_ = 0.0;
  float d_inflation_radius_ = 0.0;
  float d_cost_scaling_factor_ = 0.0;
  float d_inscribed_radius_ = 0.0;
};

} // namespace rm_mppi_controller

#endif // rm_mppi_controller