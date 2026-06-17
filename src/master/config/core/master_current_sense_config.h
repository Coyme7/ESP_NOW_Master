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

// 连续四个控制周期没有任何有效帧时判定无帧故障；主机 5kHz 下约为 0.8ms。
// 已有有效帧后的 stale 判定放宽到 10ms，避开启动任务对齐和日志阶段的短暂调度空洞。
static constexpr uint16_t kMasterCurrentSenseAdcConsecutiveErrorLimit = 4U;
static constexpr uint32_t kMasterCurrentSenseAdcStaleFaultUs = 10000U;

#ifndef MASTER_ADC_DMA_SINGLE_POOL_FRAMES
#define MASTER_ADC_DMA_SINGLE_POOL_FRAMES 8U
#endif

#ifndef MASTER_ADC_DMA_DUAL_POOL_FRAMES
#define MASTER_ADC_DMA_DUAL_POOL_FRAMES 128U
#endif

#ifndef MASTER_ADC_DMA_STARTUP_WARMUP_FRAMES
#define MASTER_ADC_DMA_STARTUP_WARMUP_FRAMES 4U
#endif

#ifndef MASTER_ADC_DMA_STARTUP_GRACE_CONTROL_CYCLES
#define MASTER_ADC_DMA_STARTUP_GRACE_CONTROL_CYCLES 8U
#endif

#ifndef MASTER_ADC_DMA_RUNTIME_FAULT_LATCH_ENABLED
#define MASTER_ADC_DMA_RUNTIME_FAULT_LATCH_ENABLED 1
#endif

// ADC DMA 内部池深度和启动对齐参数。
// DualXY 5kHz 一帧固定约 200us；这里的 128 是池深，不改变单帧 conv_frame_size。
static constexpr uint32_t kMasterCurrentSenseAdcSinglePoolFrames =
    MASTER_ADC_DMA_SINGLE_POOL_FRAMES;
static constexpr uint32_t kMasterCurrentSenseAdcDualPoolFrames =
    MASTER_ADC_DMA_DUAL_POOL_FRAMES;
static constexpr uint8_t kMasterCurrentSenseAdcStartupWarmupFrames =
    MASTER_ADC_DMA_STARTUP_WARMUP_FRAMES;
static constexpr uint16_t kMasterCurrentSenseAdcStartupGraceControlCycles =
    MASTER_ADC_DMA_STARTUP_GRACE_CONTROL_CYCLES;
static constexpr bool kMasterCurrentSenseAdcRuntimeFaultLatchEnabled =
    MASTER_ADC_DMA_RUNTIME_FAULT_LATCH_ENABLED != 0;

// X/Y 轴电流采样方向符号。
// Y 默认复用已验证 X 符号；实机 bring-up 后可单独改 kMasterYCurrentSenseAxis。
static constexpr MasterCurrentSenseAxisConfig kMasterXCurrentSenseAxis = {
    1,  // X A 相采样方向符号。
    -1, // X B 相采样方向符号。
};

static constexpr MasterCurrentSenseAxisConfig kMasterYCurrentSenseAxis = {
    1, // Y A 相采样方向符号。
    -1, // Y B 相采样方向符号。
};
