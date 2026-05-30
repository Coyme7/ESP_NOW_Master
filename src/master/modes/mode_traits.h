#pragma once

#include <stdint.h>

#include "common/math/axis_math.h"
#include "master/config/build/master_bringup_config.h"

// run mode 是主机唯一硬件路径选择来源。
// *_HW_ENABLED 只表示编译期是否允许对象存在，不能单独决定初始化路径。
constexpr bool masterRunModeHasLogicalAxis(AxisId axis) {
    return axis == AXIS_X
               ? (MASTER_RUN_MODE == MASTER_MODE_SINGLE_X_10KHZ_ID ||
                  MASTER_RUN_MODE == MASTER_MODE_DUAL_XY_5KHZ_ID)
               : (MASTER_RUN_MODE == MASTER_MODE_SINGLE_Y_10KHZ_ID ||
                  MASTER_RUN_MODE == MASTER_MODE_DUAL_XY_5KHZ_ID);
}

constexpr bool masterRunModeNeedsEncoderHardware(AxisId axis) {
    return masterRunModeHasLogicalAxis(axis);
}

constexpr bool masterRunModeNeedsMotorHardware(AxisId axis) {
    return masterRunModeHasLogicalAxis(axis);
}

constexpr bool masterRunModeIsDualXYLogic() {
    return MASTER_RUN_MODE == MASTER_MODE_DUAL_XY_5KHZ_ID;
}

constexpr uint32_t masterRunModeNominalPeriodUs() {
    return (MASTER_RUN_MODE == MASTER_MODE_SINGLE_X_10KHZ_ID ||
            MASTER_RUN_MODE == MASTER_MODE_SINGLE_Y_10KHZ_ID)
               ? 100UL
               : 200UL;
}
