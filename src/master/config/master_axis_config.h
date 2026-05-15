#pragma once

#include <Arduino.h>
#include <board/board_pins_master.h>

// 旋钮坐标方向。
//  1：保持机械相对角原符号。
// -1：翻转机械相对角符号。
// 用途：让“顺时针/逆时针”与协议坐标方向一致；不用于修正电机 FOC 方向。
#ifndef MASTER_KNOB_AXIS_SIGN
#define MASTER_KNOB_AXIS_SIGN -1
#endif

// MT6701 机械中位对应的绝对角，单位 deg。
// MT6701 输出 0..360deg 绝对角，控制层需要以旋钮机械中位为 0 的相对角。
// 标定方法：把旋钮放在机械中位，读取 raw，再用 raw * 360 / 16384 计算该值。
#ifndef MASTER_KNOB_CENTER_DEG
#define MASTER_KNOB_CENTER_DEG 180.0f
#endif

// 临时正交编码器 A/B 相输入脚。
// 只在 MASTER_DEMO_QUADRATURE_ENABLED=1 时使用；MT6701 正常路径不会访问这两个脚。
static constexpr int MASTER_DEMO_ENCODER_PIN_A = board_pins_master::UNUSED_DPI_1;
static constexpr int MASTER_DEMO_ENCODER_PIN_B = board_pins_master::UNUSED_DPI_2;

// 主机单轴配置。
// 当前阶段只实例化 X 轴；后续双旋钮时可以增加 kMasterYAxis。
struct MasterAxisConfig {
    // 机械中位对应的 MT6701 绝对角，单位 deg；用于绝对角 -> 相对角转换。
    float center_deg;                   // 机械中位对应的 MT6701 绝对角，单位 deg。
    // 控制/力反馈低端虚拟边界，单位 deg；低于该角度进入低端墙或越界保护。
    float min_deg;                      // 控制/力反馈低端虚拟边界，单位 deg。
    // 控制/力反馈高端虚拟边界，单位 deg；高于该角度进入高端墙或越界保护。
    float max_deg;                      // 控制/力反馈高端虚拟边界，单位 deg。
    // 边界内侧虚拟墙渐硬区宽度，单位 deg；值越大，越早开始变硬，入口越平滑。
    float boundary_soft_zone_deg;       // 边界内侧墙接触区宽度，单位 deg。
    // 墙/越界回推最大 q 轴目标电流，单位 A；决定墙的最大硬度。
    float haptic_current_limit_a;       // 墙/越界回推最大 q 轴目标电流，单位 A。
    // 目标电流斜率限制，单位 A/s；限制手感阶跃，降低边界细碎振动。
    float haptic_current_ramp_a_per_s;  // 目标电流斜率限制，单位 A/s。
};

// 主机 X 轴配置实例；定义在 master_config.cpp。
extern const MasterAxisConfig kMasterXAxis;

