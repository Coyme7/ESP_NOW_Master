#pragma once

#include <stdint.h>

// 力反馈内部状态：墙角度滤波和墙接触迟滞需要跨周期保存。
struct MasterHapticEngineState {
    // 用于墙深度计算的轻微滤波纸面 X 位置，单位 mm。
    float filtered_wall_x_mm;
    // 纸面墙滤波器是否完成首帧初始化。
    bool has_filtered_wall_x;
    // 墙接触方向：1 高端墙，-1 低端墙，0 未接触。
    int8_t wall_contact_side;
};

// 力反馈输入：控制角、滤波速度和本周期 dt。
struct MasterHapticEngineInput {
    // 当前控制角，单位 deg。
    float control_angle_deg;
    // 当前滤波速度，单位 deg/s。
    float filtered_velocity_deg_s;
    // 当前 X 归一化坐标，范围 -1..+1。
    float x_unit;
    // 当前纸面 X 位置，单位 mm。
    float x_mm;
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
                                                    const MasterHapticEngineInput &input);
