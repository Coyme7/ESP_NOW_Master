#pragma once

#include "master/config/build/master_bringup_config.h"
#include "master/config/types/master_axis_types.h"

// X 轴输入标定：编码器中位、输入方向和协议死区。
static constexpr MasterAxisInputConfig kMasterXAxisInput = {
    165.0f, // X 机械中位绝对角度，单位 deg。
    -1,     // X 输入坐标方向符号。
    10,      // X 协议坐标中心死区，单位 counts。
};

// Y 轴输入标定：编码器中位、输入方向和协议死区。
static constexpr MasterAxisInputConfig kMasterYAxisInput = {
    165.0f, // Y 机械中位绝对角度，单位 deg。
    -1,     // Y 输入坐标方向符号。
    10,      // Y 协议坐标中心死区，单位 counts。
};

static constexpr MasterAxisCurrentConfig kMasterXAxisCurrent = {
#if MASTER_ENABLE_CURRENT_SENSE
#if MASTER_ENABLE_STRONG_TORQUE_TEST
    0.50f, // X 强力矩目标电流限幅，单位 A。
    50.0f, // X 强力矩目标电流爬升斜率，单位 A/s。
#else
    0.08f, // X 常规目标电流限幅，单位 A。
    20.0f, // X 常规目标电流爬升斜率，单位 A/s。
#endif
#else
    0.020f, // X 电压模式等效目标限幅，单位 A。
    2.4f,   // X 电压模式等效爬升斜率，单位 A/s。
#endif
    120.0f, // X 目标电流释放斜率，单位 A/s。
};

static constexpr MasterAxisCurrentConfig kMasterYAxisCurrent = {
#if MASTER_ENABLE_CURRENT_SENSE
#if MASTER_ENABLE_STRONG_TORQUE_TEST
    0.50f, // Y 强力矩目标电流限幅，单位 A。
    50.0f, // Y 强力矩目标电流爬升斜率，单位 A/s。
#else
    0.08f, // Y 常规目标电流限幅，单位 A。
    20.0f, // Y 常规目标电流爬升斜率，单位 A/s。
#endif
#else
    0.020f, // Y 电压模式等效目标限幅，单位 A。
    2.4f,   // Y 电压模式等效爬升斜率，单位 A/s。
#endif
    120.0f, // Y 目标电流释放斜率，单位 A/s。
};

extern const MasterAxisConfig kMasterXAxis;
extern const MasterAxisConfig kMasterYAxis;
