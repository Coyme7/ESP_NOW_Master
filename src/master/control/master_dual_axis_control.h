#pragma once

#include <stdint.h>

#include "common/math/axis_math.h"

// 主机单轴运行态。当前默认只用 X 旋钮，Y 运行态固定为 0，后续第二旋钮接入时复用该结构。
struct MasterAxisRuntime {
    AxisId axis;
    float position_percent;
    int16_t norm;
    bool hardware_enabled;
};
