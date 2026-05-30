#pragma once

#include <stdint.h>

// 主机业务模式只描述用户正在做什么，不描述本次构建启用了哪些硬件。
// 硬件 bring-up 仍由 master/config 中的编译期开关约束。
enum MasterRuntimeMode : uint8_t {
    MASTER_RUNTIME_MODE_MANUAL_DRAW = 0,
    MASTER_RUNTIME_MODE_AUTO_DRAW = 1,
    MASTER_RUNTIME_MODE_BLUETOOTH = 2,
    MASTER_RUNTIME_MODE_DIAGNOSTICS = 3,
};

enum MasterRuntimeModeButton : uint8_t {
    MASTER_RUNTIME_MODE_BUTTON_NONE = 0,
    MASTER_RUNTIME_MODE_BUTTON_MANUAL_DRAW = 1,
    MASTER_RUNTIME_MODE_BUTTON_AUTO_DRAW = 2,
    MASTER_RUNTIME_MODE_BUTTON_BLUETOOTH = 3,
};

struct MasterRuntimeModeSnapshot {
    uint8_t active_mode;
    uint8_t requested_mode;
    uint8_t last_button;
    uint8_t request_accepted;
    uint8_t request_rejected;
    uint8_t manual_button_down;
    uint8_t auto_button_down;
    uint8_t bluetooth_button_down;
    uint32_t request_count;
    uint32_t last_change_ms;
};
