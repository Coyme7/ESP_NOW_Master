#pragma once

#include "master/config/build/master_bringup_config.h"
#include "master/config/types/master_haptic_types.h"

static constexpr MasterHapticDiagnosticConfig kMasterHapticDiagnostic = {
#if MASTER_ENABLE_STRONG_TORQUE_TEST
    0.30f, // 固定电流测试目标值，单位 A。
#else
    0.20f, // 固定电流测试目标值，单位 A。
#endif
    50U,   // boundary_hit 状态保持时间，单位 ms。
};
