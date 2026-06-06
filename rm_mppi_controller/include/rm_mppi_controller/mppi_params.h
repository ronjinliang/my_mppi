// mppi_params.h
#ifndef MPPI_PARAMS_H
#define MPPI_PARAMS_H

// 为了避免对齐问题，强制使用1字节对齐（可选，根据你的数据布局决定）
// 在 C++ 端使用 pragma pack，在 OpenCL 端使用 __attribute__((packed))
#ifdef __OPENCL_VERSION__
    // __OPENCL_VERSION__ 是 OpenCL 编译器（如 clBuildProgram 编译 .cl 文件时）预定义的宏。
    // ========== OpenCL 环境 ==========
    // OpenCL 中定义为 __attribute__((packed))，
    // 告诉编译器取消结构体的内存对齐填充，按实际成员大小紧密排列。
    #define PACKED __attribute__((packed))
#else
    // ========== C++ 环境 ==========
    // C++ 中定义为空，因为 C++ 端使用 #pragma pack(push, 1) 
    // 来实现 1 字节对齐，不需要额外属性。
    #include <cstdint>
    #define PACKED
    #pragma pack(push, 1)  // 将当前对齐值压栈，并设置新的对齐值为 1 字节。
#endif

// 编译器默认会对结构体成员进行对齐（例如 4 字节对齐），以提高内存访问效率。
// 但这会导致主机端和设备端的内存布局不一致，因为 GPU 可能采用不同的对齐规则。
// 后果：主机端写入的 MPPIParams 结构体，内核读到的成员位置错位，所有数据都变成错误值。
// 解决方案：强制双方使用 1 字节对齐（即取消填充），保证布局唯一且紧凑。

// 成员类型：全部使用 4 字节的类型（float 和 int）。
// 这样即使 1 字节对齐，每个成员也从 4 的倍数偏移开始（因为自身大小是 4），
// 实际上不会产生内部填充。但加上 PACKED 可以防止结构体末尾填充，确保总大小正好是成员大小之和。
// 为什么不用 bool：C++ 中 bool 是 1 字节，而 OpenCL C 中 bool 可能不是标准类型，统一用 int 最安全。
// 为什么不用 size_t：size_t 在 64 位主机上是 8 字节，在 GPU 上可能仍是 32 位（取决于设备架构），导致大小不一致。因此用固定宽度的 int

// 结构体定义内部不能设置成员默认值
typedef struct PACKED {
    int dim_x;        // 系统状态向量维度
    int dim_u;        // 控制输入向量维度
    float dt;         // 离散时间步长
    float max_v;      // max velocity
    float max_w;      // max omega
    int step_T;       // 预测时域长度
    int samples_K;    // 采样轨迹数量
    float lambda;     // MPPI的温度参数，影响权重分布
    float gamma_dv;   // cost of velocity input
    float gamma_dw;   // cost of omega input
    float gamma_v;    // cost of velocity input
    float gamma_w;    // cost of omega input
    float sigma_v;    // 噪声协方差矩阵
    float sigma_w;    // 噪声协方差矩阵
    float std_v;
    float std_w;
    float stage_cost_weight_x;       // 阶段成本权重  x y yaw
    float stage_cost_weight_y;       // 阶段成本权重  x y yaw
    float stage_cost_weight_yaw;     // 阶段成本权重  x y yaw
    float terminal_cost_weight_x;    // 终端成本权重  x y yaw
    float terminal_cost_weight_y;    // 终端成本权重  x y yaw
    float terminal_cost_weight_yaw;  // 终端成本权重  x y yaw
    // 避障代价参数
    int use_obstacle_cost;            // 1:true 0:false
    float obstacle_cost_weight;       // 一般避障权重
    float critical_weight;            // 严重惩罚权重 (安全边距内)
    float collision_cost;             // 碰撞代价
    float collision_margin_distance;  // 安全边距 (米)
    float near_goal_distance;         // 接近目标距离，此距离内停用一般避障项 unused
    // 代价地图的参数
    int costmap_size_x;    // 在 copyCostmapToDevice 单独更新
    int costmap_size_y;
    float costmap_resolution;
    float costmap_origin_x;
    float costmap_origin_y;
    float costmap_inflation_radius;
    float costmap_cost_scaling_factor;
    float costmap_inscribed_radius;

    int path_points_size;   // 路径点数量 在 copyPathToDevice 单独更新
    int prev_waypoints_idx;
    float start_x;
    float start_y;
    float start_yaw;
} MPPIParams;

#ifdef __OPENCL_VERSION__
    // OpenCL 环境不需要额外结束
#else
    // C++ 环境恢复对齐设置
    #pragma pack(pop)  // 恢复之前的对齐值，不影响其他代码。
    // 编译期检查大小，确保与设备端一致
    // 计算结构体的大小，如果与预期值不符，编译时报错。
    static_assert(sizeof(MPPIParams) == 41*4, "MPPIParams size mismatch");
#endif

// 使用时的注意事项
// 包含路径：在 clBuildProgram 时需添加 -I 选项，让 OpenCL 编译器能找到这个头文件。
//     在 clBuildProgram 之前，设置编译选项
//     const char* options = "-I /path/to/header/dir";
//     ret = clBuildProgram(program, 1, &device, options, NULL, NULL);
// 内核文件中的包含：在 .cl 文件开头写 #include "mppi_params.h"。
// 主机端包含：在 .cpp 文件中直接 #include "mppi_params.h"。
// 缓冲区创建：主机端分配 MPPIParams 结构体后，
// 用 clCreateBuffer(..., sizeof(MPPIParams), &params, ...) 创建常量缓冲区，
// 内核参数声明为 __constant MPPIParams* params。

#endif // MPPI_PARAMS_H