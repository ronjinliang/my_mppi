#include "mppi_params.h"
// #include "/home/lrj/RM/test/MPPI_test/MPPI_opencl/rm_mppi_controller/include/rm_mppi_controller/mppi_params.h"

uint xorshift32(uint* state) {
    uint x = *state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    *state = x;
    return x;
}

// 种子公式 uint seed = global_seed + get_global_id(0) * 1664525u + iteration_counter * 1013904223u;
// global_seed：主机传入的固定种子（例如时间戳）.
// get_global_id(0)：当前工作项的索引，保证不同工作项种子不同。
// iteration_counter：MPPI 迭代轮次（可选，每轮变化）。

// 浮点数版本 [-1, 1)
// float rand_float(uint* state) {
//     return (xorshift32(state) & 0xFFFFFF) / 8388608.f - 1.f;  // 24位精度 
// }

// 浮点数版本 [0, 1)
float rand_float(uint* state) {
    return (xorshift32(state) & 0xFFFFFF) / 16777216.0f;  // 24位精度
}

// 返回标准正态分布 N(0,1) 的随机数
float rand_gaussian(uint* state) {
    // 生成两个独立的 [0,1) 均匀分布随机数
    float u1 = rand_float(state);
    float u2 = rand_float(state);
    
    // [0,1)
    if (u1 < 0.01f) u1 = 0.01f;  // 避免 log(0)
    
    float radius = sqrt(-2.0f * log(u1));
    float angle = 2.0f * M_PI * u2;
    return radius * cos(angle);   // 返回一个标准正态变量
}

float calc_obstacle_cost(
    __global unsigned char* cl_costmap,
    __constant MPPIParams* cl_mps,
    float x_t, float y_t){
    
    // world to grid index
    int mx = (int)( ( x_t - cl_mps->costmap_origin_x ) / cl_mps->costmap_resolution );
    int my = (int)( ( y_t - cl_mps->costmap_origin_y ) / cl_mps->costmap_resolution );
    if ( mx < 0 || mx >= cl_mps->costmap_size_x || 
         my < 0 || my >= cl_mps->costmap_size_y ) {
        return 0.f; // 超出地图边界，无障碍
    }

    unsigned char cost = cl_costmap[ my * cl_mps->costmap_size_x + mx ];
    if ( cost <= 0 ) {
        return 0.f;   // FREE_SPACE = 0
    } else if ( cost >= 254 ) {
        return cl_mps->collision_cost;
    }

    // 使用膨胀层参数计算距离
    if ( cl_mps->costmap_inflation_radius > 0.0f && cl_mps->costmap_cost_scaling_factor > 0.0f) {
        float dist = ( log( 254.0f ) - log( (float)cost ) ) / cl_mps->costmap_cost_scaling_factor + cl_mps->costmap_inscribed_radius;
        if ( dist < cl_mps->costmap_inscribed_radius ) dist = cl_mps->costmap_inflation_radius;
        float max_penalty_dist = cl_mps->costmap_inflation_radius;
        float penalty = 0.f;
        if ( dist <= cl_mps->collision_margin_distance ) {
            penalty = cl_mps->critical_weight * ( cl_mps->collision_margin_distance - dist );
        } else if ( dist < max_penalty_dist ) {
            penalty = cl_mps->obstacle_cost_weight * ( max_penalty_dist - dist );
        }
        return penalty;
    } else {
        // 无膨胀参数，直接线性缩放
        return cl_mps->obstacle_cost_weight * ( ( float )cost / 254.0f );
    }
}

float calc_path_align_cost(
    __global float* cl_path_points,
    __global float* cl_arc_lengths,
    __constant MPPIParams* cl_mps,
    float x_t, float y_t, float yaw_t,
    float u_v, int t){
    
    // 1. 寻找欧氏最近点
    int nearest_idx = 0;
    float best_dist2 = 1e10f;

    // 基于 prev_waypoints_idx_ 或上一次的投影点，在局部窗口内搜索（前后各10个点）。
    int start = cl_mps->prev_waypoints_idx;
    if ( start > 10 ) start = cl_mps->prev_waypoints_idx - 10;
    else start = 0;
    int end = cl_mps->prev_waypoints_idx + 10;
    if ( end >= cl_mps->path_points_size ) end = cl_mps->path_points_size - 1;

    for ( int i = start; i < end; ++i ) {
        int idx = 3*i;
        float dx = cl_path_points[ idx ] - x_t;
        float dy = cl_path_points[ idx + 1 ] - y_t;
        float d2 = dx*dx + dy*dy;
        if ( d2 < best_dist2 ) {
            best_dist2 = d2;
            nearest_idx = i;
        }
    }

    // 确保投影点不会小于上一个参考点（避免倒退）
    if (nearest_idx < cl_mps->prev_waypoints_idx) {
        nearest_idx = cl_mps->prev_waypoints_idx;
    }

    // 2.计算前向投影距离
    float forward_dist = fabs(u_v) * cl_mps->dt * t;
    if ( forward_dist < 0.5f ) forward_dist = 0.5f;
    float target_arc = cl_arc_lengths[ nearest_idx ] + forward_dist;

    // 3.二分查找目标弧长对应的索引
    int low = 0, high = cl_mps->path_points_size - 1;
    int target_idx = high;
    while ( low <= high ) {
        int mid = ( low + high ) / 2;
        if ( cl_arc_lengths[mid] >= target_arc ) {
            target_idx = mid;
            high = mid - 1;
        } else {
            low = mid + 1;
        }
    }

    // 4. 获取参考点坐标和朝向
    int idx = target_idx * 3;
    float ref_x = cl_path_points[ idx ];
    float ref_y = cl_path_points[ idx + 1 ];
    float ref_yaw = cl_path_points[ idx + 2 ];

    // 5. 计算偏差
    float dx = ref_x - x_t;
    float dy = ref_y - y_t;
    float dyaw = ref_yaw - yaw_t;
    // if ( dyaw < -M_PI / 2.0f ) dyaw += M_PI;
    // if ( dyaw > M_PI / 2.0f ) dyaw -= M_PI;
    if ( dyaw < -M_PI ) dyaw += 2.f * M_PI;
    if ( dyaw > M_PI) dyaw -= 2.f * M_PI;

    float stage_cost = cl_mps->stage_cost_weight_x * dx * dx +
                        cl_mps->stage_cost_weight_y * dy * dy +
                        cl_mps->stage_cost_weight_yaw * dyaw * dyaw;
    return stage_cost;
}

float calc_path_follow_cost(
    __global float* cl_path_points,
    __global float* cl_arc_lengths,
    __constant MPPIParams* cl_mps,
    float x_t, float y_t, float yaw_t,
    float u_v, int t) {

}

float calc_path_angle_cost(
    __global float* cl_path_points,
    __global float* cl_arc_lengths,
    __constant MPPIParams* cl_mps,
    float x_t, float y_t, float yaw_t,
    float u_v, int t) {

}

float calc_terminal_cost(
    __global float* cl_path_points,
    __global float* cl_arc_lengths,
    __constant MPPIParams* cl_mps,
    float x_t, float y_t, float yaw_t,
    float u_v){
    
    // 1. 寻找欧氏最近点
    int nearest_idx = 0;
    float best_dist2 = 1e10f;

    // 基于 prev_waypoints_idx_ 或上一次的投影点，在局部窗口内搜索（前后各10个点）。
    int start = cl_mps->prev_waypoints_idx;
    if ( start > 10 ) start = cl_mps->prev_waypoints_idx - 10;
    else start = 0;
    int end = cl_mps->prev_waypoints_idx + 10;
    if ( end >= cl_mps->path_points_size ) end = cl_mps->path_points_size - 1;

    for ( int i = start; i < end; ++i ) {
        int idx = 3*i;
        float dx = cl_path_points[ idx ] - x_t;
        float dy = cl_path_points[ idx + 1 ] - y_t;
        float d2 = dx*dx + dy*dy;
        if ( d2 < best_dist2 ) {
            best_dist2 = d2;
            nearest_idx = i;
        }
    }

    // 确保投影点不会小于上一个参考点（避免倒退）
    if (nearest_idx < cl_mps->prev_waypoints_idx) {
        nearest_idx = cl_mps->prev_waypoints_idx;
    }

    // 2.计算前向投影距离
    float forward_dist = fabs(u_v) * cl_mps->step_T * cl_mps->dt;
    if ( forward_dist < 0.5f ) forward_dist = 0.5f;
    float target_arc = cl_arc_lengths[nearest_idx] + forward_dist;

    // 3.二分查找目标弧长对应的索引
    int low = 0, high = cl_mps->path_points_size - 1;
    int target_idx = high;
    while ( low <= high ) {
        int mid = ( low + high ) / 2;
        if ( cl_arc_lengths[mid] >= target_arc ) {
            target_idx = mid;
            high = mid - 1;
        } else {
            low = mid + 1;
        }
    }

    // 4. 获取参考点坐标和朝向
    int idx = target_idx * 3;
    float ref_x = cl_path_points[ idx ];
    float ref_y = cl_path_points[ idx + 1 ];
    float ref_yaw = cl_path_points[ idx + 2 ];

    // 5. 计算偏差
    float dx = ref_x - x_t;
    float dy = ref_y - y_t;
    float dyaw = ref_yaw - yaw_t;
    // if ( dyaw < -M_PI / 2.0f ) dyaw += M_PI;
    // if ( dyaw > M_PI / 2.0f ) dyaw -= M_PI;
    if ( dyaw < -M_PI ) dyaw += 2.f * M_PI;
    if ( dyaw > M_PI ) dyaw -= 2.f * M_PI;

    float terminal_cost = cl_mps->terminal_cost_weight_x * dx * dx +
                        cl_mps->terminal_cost_weight_y * dy * dy +
                        cl_mps->terminal_cost_weight_yaw * dyaw * dyaw;
    return terminal_cost;
}

__kernel void calc_total_cost(
    __global float* cl_u_prev,
    __global float* cl_costs,
    __global float* cl_epsilon,
    __global float* cl_path_points,
    __global float* cl_arc_lengths,
    __global unsigned char* cl_costmap,
    __constant MPPIParams* cl_mps,
    uint global_seed){
    
    int k = get_global_id(0);
    if ( k >= cl_mps->samples_K ) return;

    // 为这个样本生成初始种子（每个样本不同，时间步独立更新）
    uint seed = global_seed + k * 1664525u;

    float x = cl_mps->start_x;
    float y = cl_mps->start_y;
    float yaw = cl_mps->start_yaw;

    float total_cost = 0.f;

    // t = 0 时的控制输入
    float u_v_prev = cl_u_prev[0];
    float u_w_prev = cl_u_prev[1];

    float u_v = 0.f;
    float u_w = 0.f;

    for ( int t = 0; t < cl_mps->step_T; ++t ) {
        // 为每个时间步更新种子（保证不同步随机性）
        uint step_seed = seed + t * 104729u;
        
        int idx = 2 * t;
        float u_v_nominal = cl_u_prev[ idx ];
        float u_w_nominal = cl_u_prev[ idx + 1 ];
        // 生成噪声
        // float noise_v = rand_float(&step_seed) * cl_mps->std_v;
        // float noise_w = rand_float(&step_seed) * cl_mps->std_w;
        float noise_v = rand_gaussian(&step_seed) * cl_mps->std_v;
        float noise_w = rand_gaussian(&step_seed) * cl_mps->std_w;
        
        // 存储噪声
        idx = (( k * cl_mps->step_T ) + t) * 2;
        cl_epsilon[ idx ] = noise_v;
        cl_epsilon[ idx + 1 ] = noise_w;
        // 添加噪声
        u_v = u_v_nominal + noise_v;
        u_w = u_w_nominal + noise_w;
        // 限幅
        if      ( u_v >   cl_mps->max_v ) u_v =   cl_mps->max_v;
        else if ( u_v < - cl_mps->max_v ) u_v = - cl_mps->max_v;
        if      ( u_w >   cl_mps->max_w ) u_w =   cl_mps->max_w;
        else if ( u_w < - cl_mps->max_w ) u_w = - cl_mps->max_w;
        // 更新状态
        x += u_v * cos(yaw) * cl_mps->dt;
        y += u_v * sin(yaw) * cl_mps->dt;
        yaw += u_w * cl_mps->dt;

        if      ( yaw >   M_PI ) yaw -= 2.0f * M_PI;
        else if ( yaw < - M_PI ) yaw += 2.0f * M_PI;

        // 计算阶段成本
        float stage_cost = calc_path_align_cost( cl_path_points, cl_arc_lengths, cl_mps, x, y, yaw, u_v, t );
        // 计算输入成本
        float du_v = u_v - u_v_prev;
        float du_w = u_w - u_w_prev;
        float input_cost =  cl_mps->gamma_dv * du_v * du_v / cl_mps->sigma_v +
                            cl_mps->gamma_dw * du_w * du_w / cl_mps->sigma_w + 
                            cl_mps->gamma_v  * u_v  * u_v  / cl_mps->sigma_v + 
                            cl_mps->gamma_w  * u_w  * u_w  / cl_mps->sigma_w;
        stage_cost += input_cost;

        if ( cl_mps->use_obstacle_cost ) {
            float obs_cost = calc_obstacle_cost( cl_costmap, cl_mps, x, y );
            stage_cost += obs_cost;
        }

        total_cost += stage_cost;

        // 更新上一时刻控制量（用于下一步的差分）
        u_v_prev = u_v;
        u_w_prev = u_w;
    }

    // 终端成本
    float terminal_cost = calc_terminal_cost( cl_path_points, cl_arc_lengths, cl_mps, x, y, yaw, u_v );
    total_cost += terminal_cost;

    cl_costs[k] = total_cost;
}
