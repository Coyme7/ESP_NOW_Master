#pragma once

#include <stdint.h>

struct MasterHapticDiagnosticConfig {
    float fixed_current_test_a; // 固定电流测试目标值，单位 A。
    uint16_t boundary_hold_ms;  // boundary_hit 状态保持时间，单位 ms。
};
