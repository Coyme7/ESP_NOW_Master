#pragma once

#include "master/config/types/master_haptic_wall_types.h"

struct MasterAxisInputConfig {
    float center_deg;             // 机械中位绝对角度，单位 deg。
    int8_t axis_sign;             // 输入坐标方向符号。
    int16_t norm_deadband_counts; // 协议坐标中心死区，单位 counts。
};

struct MasterAxisRangeConfig {
    float min_deg; // 低端虚拟边界，单位 deg。
    float max_deg; // 高端虚拟边界，单位 deg。
};

struct MasterAxisCurrentConfig {
    float limit_a;         // 力反馈目标电流限幅，单位 A。
    float ramp_a_per_s;    // 目标电流爬升斜率，单位 A/s。
    float release_a_per_s; // 离墙目标电流释放斜率，单位 A/s。
};

struct MasterAxisConfig {
    MasterAxisInputConfig input;     // 编码器中位、方向和协议死区。
    MasterAxisRangeConfig range;     // 旋钮角度虚拟边界。
    MasterAxisCurrentConfig current; // 力反馈目标电流限制。
    MasterHapticAxisConfig haptic;   // 纸面墙和中心阻尼参数。
};
