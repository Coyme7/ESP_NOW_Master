#pragma once

#include "master/config/build/master_bringup_config.h"
#include "master/config/types/master_current_sense_types.h"

// 电流采样硬件换算配置。
// shunt/gain 必须与 DengFoc 驱动板实物一致，ADC 满量程按 DB_12 约 3.10V 计算。
static constexpr MasterCurrentSenseHardwareConfig kMasterCurrentSenseHardware = {
    0.01f,          // 分流电阻，单位 ohm。
    50.0f,          // 电流采样放大倍数。
    3.10f,          // ADC 满量程电压，单位 V。
    4095.0f,        // ADC raw 最大值。
    3.10f / 4095.0f, // ADC raw 到电压换算系数，单位 V/count。
    true,           // 是否跳过 SimpleFOC driverAlign。
};

// 连续四个完整 A/B 采样周期失败后禁用对应轴；主机 5kHz 下约为 0.8ms。
static constexpr uint16_t kMasterCurrentSenseAdcConsecutiveErrorLimit = 4U;

// X/Y 轴电流采样方向符号。
// Y 默认复用已验证 X 符号；实机 bring-up 后可单独改 kMasterYCurrentSenseAxis。
static constexpr MasterCurrentSenseAxisConfig kMasterXCurrentSenseAxis = {
    1,  // X A 相采样方向符号。
    -1, // X B 相采样方向符号。
};

static constexpr MasterCurrentSenseAxisConfig kMasterYCurrentSenseAxis = {
    kMasterXCurrentSenseAxis.gain_sign_a, // Y A 相采样方向符号。
    kMasterXCurrentSenseAxis.gain_sign_b, // Y B 相采样方向符号。
};

// 电流采样诊断和 offset 校准参数。
// 这些延时只在启动诊断/校准路径使用，不进入控制热路径。
static constexpr MasterCurrentSenseDiagConfig kMasterCurrentSenseDiag = {
    0.30f, // 诊断注入电压，单位 V。
    5U,    // 早期采样等待时间，单位 ms。
    80U,   // 稳定采样等待时间，单位 ms。
    8U,    // offset 校准前 ADC 预读次数。
    1000U, // offset 校准平均次数。
    80U,   // offset 校准前等待时间，单位 ms。
};
