#pragma once

#include <stdint.h>

#include "master/config/build/master_bringup_config.h"

// == 控制周期 =================================================================

// 未手动覆盖时，控制周期由 MASTER_RUN_MODE 派生。
// SingleX/SingleY_10kHz 为 100us；DualXY_5kHz 为 200us。
#ifndef MASTER_CONTROL_LOOP_PERIOD_US_CONFIG
#if MASTER_RUN_MODE == MASTER_MODE_SINGLE_X_10KHZ_ID || \
    MASTER_RUN_MODE == MASTER_MODE_SINGLE_Y_10KHZ_ID
#define MASTER_CONTROL_LOOP_PERIOD_US_CONFIG 100UL
#else
#define MASTER_CONTROL_LOOP_PERIOD_US_CONFIG 200UL
#endif
#endif

static constexpr uint32_t MASTER_CONTROL_LOOP_PERIOD_US =
    MASTER_CONTROL_LOOP_PERIOD_US_CONFIG;

// 外层控制周期：haptic、速度估计、current command 和 SimpleFOC move() 默认降到 1kHz。
#ifndef MASTER_OUTER_LOOP_PERIOD_US
#define MASTER_OUTER_LOOP_PERIOD_US 1000UL
#endif

static constexpr uint32_t MASTER_OUTER_LOOP_EVERY_N_STEPS =
    MASTER_OUTER_LOOP_PERIOD_US / MASTER_CONTROL_LOOP_PERIOD_US;

// 外环若连续错过该窗口，快环改用 0A 并强制刷新 SimpleFOC 目标。
#ifndef MASTER_OUTER_LOOP_STALE_TIMEOUT_US
#define MASTER_OUTER_LOOP_STALE_TIMEOUT_US (MASTER_OUTER_LOOP_PERIOD_US * 4UL)
#endif

static constexpr uint32_t MASTER_OUTER_LOOP_STALE_EVERY_N_STEPS =
    MASTER_OUTER_LOOP_STALE_TIMEOUT_US / MASTER_CONTROL_LOOP_PERIOD_US;

// 控制热路径状态快照发布分频。
// 0/1：每步发布；>1：每 N 步发布一次，默认 10。
#ifndef MASTER_CONTROL_STATUS_PUBLISH_DIV
#define MASTER_CONTROL_STATUS_PUBLISH_DIV 10
#endif

// 实验路径：直接更新 SimpleFOC torque 模式的 target/current_sp，绕过 move()。
// 默认关闭，先保守使用 1kHz move() 保持 SimpleFOC 状态更新语义。
#ifndef MASTER_DIRECT_CURRENT_SETPOINT_ENABLED
#define MASTER_DIRECT_CURRENT_SETPOINT_ENABLED 0
#endif
