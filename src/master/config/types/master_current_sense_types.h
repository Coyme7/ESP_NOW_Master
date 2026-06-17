#pragma once

#include <stdint.h>

struct MasterCurrentSenseHardwareConfig {
    float shunt_ohm;             // 分流电阻，单位 ohm。
    float gain;                  // 电流采样放大倍数。
    float adc_full_scale_v;      // ADC 满量程电压，单位 V。
    float adc_raw_max;           // ADC raw 最大值。
    float adc_raw_to_voltage_v;  // ADC raw 到电压换算系数，单位 V/count。
    bool skip_align;             // 是否跳过 SimpleFOC driverAlign。
};

struct MasterCurrentSenseAxisConfig {
    int8_t gain_sign_a;          // A 相采样方向符号。
    int8_t gain_sign_b;          // B 相采样方向符号。
};
