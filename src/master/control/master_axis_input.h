#pragma once

#include <stdint.h>

struct MasterAxisInputState {
    // X 上一次控制角，用于角速度差分。
    float previous_angle_deg;
    // X 当前滤波速度，单位 deg/s。
    float filtered_velocity_deg_s;
    // X 是否已经有上一帧角度，防止首帧速度异常。
    bool has_previous_angle;
    // Y 上一次控制角，用于角速度差分。
    float previous_y_angle_deg;
    // Y 当前滤波速度，单位 deg/s。
    float filtered_y_velocity_deg_s;
    // Y 是否已经有上一帧角度，防止首帧速度异常。
    bool has_previous_y_angle;
};

struct MasterAxisInputSample {
    // 当前主显示轴控制角，默认代表 X，单位 deg。
    float control_angle_deg;
    float clamped_angle_deg;
    float filtered_velocity_deg_s;
    // X/Y 独立控制角与速度，用于双旋钮力反馈。
    float x_control_angle_deg;
    float x_clamped_angle_deg;
    float x_filtered_velocity_deg_s;
    float y_control_angle_deg;
    float y_clamped_angle_deg;
    float y_filtered_velocity_deg_s;
    // 当前协议归一化坐标。SingleY 轴模式使用主机 Y 旋钮写入 y_norm。
    int16_t x_norm;
    int16_t y_norm;
};

int16_t masterAxisAngleDegToNorm(float angle_deg);
MasterAxisInputSample updateMasterAxisInput(MasterAxisInputState &state,
                                            float control_angle_deg,
                                            float y_control_angle_deg,
                                            float dt_s);
