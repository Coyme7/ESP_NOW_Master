#pragma once

#include <stdint.h>

struct MasterHapticWallConfig {
    float lpf_tf_s;                 // 墙位置低通滤波时间常数，单位 s。
    float start_mm;                 // 纸面虚拟墙开始距离，单位 mm。
    float hard_limit_mm;            // 纸面虚拟墙硬限位，单位 mm。
    float safety_cut_mm;            // raw 纸面位置安全切断距离，单位 mm。
    float release_hyst_mm;          // 虚拟墙释放迟滞距离，单位 mm。
    float min_current_a;            // 入墙最小目标电流，单位 A。
    float damping_gain_a_per_deg_s; // 墙内速度阻尼增益，单位 A/(deg/s)。
    float damping_limit_a;          // 墙内速度阻尼电流限幅，单位 A。
    int8_t direction_sign;          // 虚拟墙输出方向符号。
};

struct MasterCenterDampingConfig {
    bool enabled;              // 中心阻尼开关。
    int8_t direction_sign;     // 中心阻尼输出方向符号。
    float gain_a_per_deg_s;    // 中心阻尼速度增益，单位 A/(deg/s)。
    float velocity_lpf_tf_s;   // 速度估计滤波时间常数，单位 s。
    float still_lpf_tf_s;      // 静止检测滤波时间常数，单位 s。
    float deadband_deg_s;      // 阻尼死区速度，单位 deg/s。
    float full_speed_deg_s;    // 阻尼满量程速度，单位 deg/s。
    float coulomb_a;           // 低速库仑阻尼项，单位 A。
    float vel_scale_deg_s;     // 库仑项速度缩放，单位 deg/s。
    float limit_a;             // 中心阻尼电流限幅，单位 A。
};

struct MasterHapticAxisConfig {
    MasterHapticWallConfig wall;      // 纸面虚拟墙参数。
    MasterCenterDampingConfig center; // 中心阻尼参数。
};
