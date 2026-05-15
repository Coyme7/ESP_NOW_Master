#pragma once

// SimpleFOC 电流采样 driverAlign 开关。
// 1：跳过自动相位校验，只做 offset 校准。
// 当前 ESP32-S3 + DengFoc bring-up 中自动校验可能失败，因此默认跳过。
// 跳过后必须通过 current_probe 和 0A 烟测确认采样符号。
#ifndef MASTER_CURRENT_SENSE_SKIP_ALIGN
#define MASTER_CURRENT_SENSE_SKIP_ALIGN 1
#endif

// A 相电流采样增益方向。
//  1：保持库默认方向。
// -1：翻转该相采样方向。
// 如果 U/V/W 注入时 ia/ib 响应方向与预期相反，需要调整该符号。
#ifndef MASTER_CURRENT_SENSE_GAIN_SIGN_A
#define MASTER_CURRENT_SENSE_GAIN_SIGN_A 1
#endif

// B 相电流采样增益方向。
//  1：保持库默认方向。
// -1：翻转该相采样方向。
// 当前配置为 -1，表示手动校准后 B 相需要翻转。
#ifndef MASTER_CURRENT_SENSE_GAIN_SIGN_B
#define MASTER_CURRENT_SENSE_GAIN_SIGN_B -1
#endif

// 电流采样分流电阻，单位 ohm。
// 必须与驱动板实际 shunt 一致；值填错会让电流换算成比例偏差。
#ifndef MASTER_CURRENT_SHUNT_OHM
#define MASTER_CURRENT_SHUNT_OHM 0.01f
#endif

// 电流采样放大倍数。
// 必须与驱动板运放/电流采样放大器增益一致。
// 电流换算关系大致为：phase_current = voltage_delta / (shunt * gain)。
#ifndef MASTER_CURRENT_SENSE_GAIN
#define MASTER_CURRENT_SENSE_GAIN 50.0f
#endif

// ESP32-S3 ADC 满量程电压，单位 V。
// ADC_ATTEN_DB_12 下可测范围约 0..3100mV，这里用 3.10V 参与 raw->voltage 换算。
#ifndef MASTER_CURRENT_SENSE_ADC_FULL_SCALE_V
#define MASTER_CURRENT_SENSE_ADC_FULL_SCALE_V 3.10f
#endif

// ESP32-S3 ADC 原始最大值。
// 12-bit ADC 对应 0..4095。
#ifndef MASTER_CURRENT_SENSE_ADC_RAW_MAX
#define MASTER_CURRENT_SENSE_ADC_RAW_MAX 4095.0f
#endif

// ADC raw 到电压的换算系数，单位 V/count。
// 该宏由满量程电压和 raw 最大值计算，供 ADC1 电流采样实现使用。
#ifndef MASTER_CURRENT_SENSE_ADC_RAW_TO_VOLTAGE_V
#define MASTER_CURRENT_SENSE_ADC_RAW_TO_VOLTAGE_V \
    (MASTER_CURRENT_SENSE_ADC_FULL_SCALE_V / MASTER_CURRENT_SENSE_ADC_RAW_MAX)
#endif

// 电流采样诊断注入电压，单位 V。
// 仅用于 MASTER_CURRENT_SENSE_DIAG_ONLY 模式下的 U/V/W 短促注入。
// 值太大有发热和误动作风险，bring-up 阶段保持较小。
#ifndef MASTER_CURRENT_SENSE_DIAG_VOLTAGE_V
#define MASTER_CURRENT_SENSE_DIAG_VOLTAGE_V 0.30f
#endif

// 诊断注入后的早期采样等待时间，单位 ms。
// 用于观察注入刚开始时的电流响应。
#ifndef MASTER_CURRENT_SENSE_DIAG_EARLY_MS
#define MASTER_CURRENT_SENSE_DIAG_EARLY_MS 5
#endif

// 诊断注入后的稳定采样等待时间，单位 ms。
// 用于等待电流/ADC 读数稳定后再采样。
#ifndef MASTER_CURRENT_SENSE_DIAG_SETTLE_MS
#define MASTER_CURRENT_SENSE_DIAG_SETTLE_MS 80
#endif

// offset 校准前丢弃的 ADC 预读次数。
// 作用是让 ADC 和采样链路先稳定，避免刚启动的瞬态值进入 offset 平均。
#ifndef MASTER_CURRENT_SENSE_ADC_PRIME_READS
#define MASTER_CURRENT_SENSE_ADC_PRIME_READS 8
#endif

// 电流采样 offset 校准平均次数。
// 数值越大 offset 越稳，但启动越慢；只影响启动校准，不进入热路径。
#ifndef MASTER_CURRENT_SENSE_OFFSET_CALIBRATION_READS
#define MASTER_CURRENT_SENSE_OFFSET_CALIBRATION_READS 1000
#endif

// offset 校准前等待时间，单位 ms。
// 默认复用诊断稳定等待时间，让驱动、电源和 ADC 有时间稳定。
#ifndef MASTER_CURRENT_SENSE_OFFSET_SETTLE_MS
#define MASTER_CURRENT_SENSE_OFFSET_SETTLE_MS MASTER_CURRENT_SENSE_DIAG_SETTLE_MS
#endif

