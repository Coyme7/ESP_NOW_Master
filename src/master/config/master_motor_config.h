#pragma once

#include "master/config/master_build_options.h"
#include "master/config/master_current_sense_config.h"
#include "master/config/master_haptic_config.h"

// DengFoc / BLDCDriver3PWM EN 脚极性。
// 1：高电平使能驱动。
// 0：低电平使能驱动。
// 若板子实测 EN 低电平有效，必须先改该值再开启真实电机输出。
#ifndef MASTER_DRIVER_ENABLE_ACTIVE_HIGH
#define MASTER_DRIVER_ENABLE_ACTIVE_HIGH 1
#endif

// 主机 2804 无刷电机极对数。
// SimpleFOC 需要该值把机械角转换成电角度；填错会导致 FOC 角度错误、力矩异常。
#ifndef MASTER_MOTOR_POLE_PAIRS
#define MASTER_MOTOR_POLE_PAIRS 7
#endif

// 主机 SimpleFOC 电机/电流环配置。
// 该结构在 master_config.cpp 中实例化为 kMasterMotorFoc，硬件初始化时读取。
struct MasterMotorFocConfig {
    // 驱动板母线/电源电压，单位 V；用于 SimpleFOC 计算输出限制。
    float supply_voltage_v;     // 驱动板母线/电源电压，单位 V。
    // SimpleFOC 输出电压上限，单位 V；限制电流环最多能给多少电压去追目标电流。
    float voltage_limit_v;      // SimpleFOC 输出电压上限，单位 V。
    // initFOC 对齐/检测使用电压，单位 V；过高会让启动对齐动作过猛。
    float align_voltage_v;      // initFOC 对齐/检测使用电压，单位 V。
    // q 轴电流环 P 增益；Iq 是主要产生力矩和手感的电流分量。
    float current_q_pid_p;      // q 轴电流环 P 增益。
    // q 轴电流环 I 增益；当前 P-only 阶段保持 0，避免 windup。
    float current_q_pid_i;      // q 轴电流环 I 增益；当前保持 0 防 windup。
    // q 轴电流环 D 增益；当前保持 0，避免放大电流采样噪声。
    float current_q_pid_d;      // q 轴电流环 D 增益；当前保持 0。
    // q 轴 PID 输出斜率限制；限制电流环电压输出变化速度。
    float current_q_pid_ramp;   // q 轴 PID 输出斜率限制。
    // d 轴电流环 P 增益；用于抑制 Id，使磁场尽量保持在 q 轴出力。
    float current_d_pid_p;      // d 轴电流环 P 增益。
    // d 轴电流环 I 增益；当前关闭，避免 Vd 积分积累。
    float current_d_pid_i;      // d 轴电流环 I 增益；当前保持 0 防 windup。
    // d 轴电流环 D 增益；当前关闭。
    float current_d_pid_d;      // d 轴电流环 D 增益；当前保持 0。
    // d 轴 PID 输出斜率限制；通常与 q 轴保持一致。
    float current_d_pid_ramp;   // d 轴 PID 输出斜率限制。
    // q/d 电流低通滤波时间常数，单位 s；影响电流反馈噪声和响应速度。
    float current_lpf_tf;       // q/d 电流低通滤波时间常数，单位 s。
};

// 主机电机/电流环配置实例；定义在 master_config.cpp。
extern const MasterMotorFocConfig kMasterMotorFoc;

