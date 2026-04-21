#ifndef __UTILS_H
#define __UTILS_H

#include <cmath>

unsigned int xorshift32(unsigned int* state) {
    unsigned int x = *state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    *state = x;
    return x;
}

// 种子公式 uint seed = global_seed + k * 1664525u + iteration_counter * 1013904223u;
// global_seed：主机传入的固定种子（例如时间戳）.
// k：当前工作项的索引，保证不同工作项种子不同。
// iteration_counter：MPPI 迭代轮次（可选，每轮变化）。

// 浮点数版本 [-1, 1)
// float rand_float(uint* state) {
//     return (xorshift32(state) & 0xFFFFFF) / 8388608.f - 1.f;  // 24位精度 
// }

// 浮点数版本 [0, 1)
float rand_float(unsigned int* state) {
    return (xorshift32(state) & 0xFFFFFF) / 16777216.0f;  // 24位精度
}

// 返回标准正态分布 N(0,1) 的随机数
float rand_gaussian(unsigned int* state) {
    // 生成两个独立的 [0,1) 均匀分布随机数
    float u1 = rand_float(state);
    float u2 = rand_float(state);
    
    // [0,1)
    if (u1 < 0.01f) u1 = 0.01f;  // 避免 log(0)
    
    float radius = sqrtf(-2.0f * logf(u1));
    float angle = 2.0f * M_PI * u2;
    return radius * cosf(angle);   // 返回一个标准正态变量
}


#endif
