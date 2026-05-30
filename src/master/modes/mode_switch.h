#pragma once

#include <stdint.h>

// 只负责模式切换输入的消抖和事件生成，不决定业务模式。
struct MasterModeSwitchEvents {
    bool manual_down;
    bool auto_down;
    bool bluetooth_down;
    bool manual_pressed;
    bool auto_pressed;
    bool bluetooth_pressed;
};

MasterModeSwitchEvents updateMasterModeSwitches(uint32_t now_ms);
