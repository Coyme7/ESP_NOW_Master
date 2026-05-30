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

// 控制热路径状态快照发布分频。
// 0/1：每步发布；>1：每 N 步发布一次，默认 10。
#ifndef MASTER_CONTROL_STATUS_PUBLISH_DIV
#define MASTER_CONTROL_STATUS_PUBLISH_DIV 10
#endif
