#pragma once

#include <stdint.h>

#include "master/config/core/master_axis_config.h"

// 力反馈预计算参数：把配置派生常量从周期计算中移出。
struct MasterHapticPrecomputedConfig {
    float paper_half_range_mm;
    float current_limit_a;
    float wall_hard_limit_mm;
    float wall_start_mm;
    float wall_safety_cut_mm;
    float wall_release_hyst_mm;
    float wall_inv_zone_mm;
    float wall_min_factor;
    float wall_direction_sign;
    float center_direction_sign;
    float center_deadband_deg_s;
    float center_inv_speed_span;
    float center_inv_vel_scale;
};

// 力反馈内部状态：墙位置滤波和墙接触迟滞需要跨周期保存。
struct MasterHapticEngineState {
    // 用于墙深度计算的轻微滤波纸面轴位置，单位 mm。
    float filtered_wall_axis_mm;
    // 纸面墙滤波器是否完成首帧初始化。
    bool has_filtered_wall_axis;
    // 墙接触方向：1 高端墙，-1 低端墙，0 未接触。
    int8_t wall_contact_side;
    // 预计算参数是否有效。
    bool has_precomputed_config;
    // 当前轴配置和纸面半幅派生出的热路径常量。
    MasterHapticPrecomputedConfig precomputed_config;
};

// 力反馈输入：控制角、滤波速度、轴坐标、轴纸面半幅和本周期 dt。
struct MasterHapticEngineInput {
    // 当前控制角，单位 deg。
    float control_angle_deg;
    // 当前滤波速度，单位 deg/s。
    float filtered_velocity_deg_s;
    // 当前轴归一化坐标，范围 -1..+1。
    float axis_unit;
    // 当前纸面轴位置，单位 mm。
    float axis_mm;
    // 当前轴纸面半幅，单位 mm；X/Y 轴分别传入自己的可用半幅，避免 Y 轴复用 X 轴墙宽。
    float paper_half_range_mm;
    // 当前控制周期实际 dt，单位 s。
    float dt_s;
};

// 力反馈输出：目标电流和边界状态，后续交给 current_command 做最终限速。
struct MasterHapticEngineOutput {
    // 力反馈算法输出的目标电流，单位 A。
    float target_current_a;
    // 是否处于墙接触、越界回推或安全切断状态。
    bool boundary_active;
    // 是否触发越界安全切断。
    bool boundary_safety_cut;
};

MasterHapticEngineOutput computeMasterHapticCommand(MasterHapticEngineState &state,
                                                    const MasterHapticEngineInput &input,
                                                    const MasterAxisConfig &config);
