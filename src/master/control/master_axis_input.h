#pragma once

#include <stdint.h>

struct MasterAxisInputState {
    // 上一次控制角，用于角速度差分。
    float previous_angle_deg;
    // 低通后的角速度，单位 deg/s。
    // 当前滤波速度，单位 deg/s。
    float filtered_velocity_deg_s;
    // 是否已经有上一帧角度，防止首帧速度异常。
    bool has_previous_angle;
};

struct MasterAxisInputSample {
    // 当前控制角，单位 deg。
    float control_angle_deg;
    float clamped_angle_deg;
    float filtered_velocity_deg_s;
    // 当前控制角映射到协议归一化 X 坐标。
    int16_t x_norm;
    // 当前控制角映射到协议归一化 Y 坐标。默认 SingleX 下固定为 0。
    int16_t y_norm;
};

int16_t masterAxisAngleDegToNorm(float angle_deg);
MasterAxisInputSample updateMasterAxisInput(MasterAxisInputState &state,
                                            float control_angle_deg,
                                            float dt_s);
