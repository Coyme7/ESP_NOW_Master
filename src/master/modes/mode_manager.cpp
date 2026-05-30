#include "master/modes/mode_manager.h"

#include <Arduino.h>

#include "master/config/build/master_bringup_config.h"

namespace {

portMUX_TYPE runtimeModeMux = portMUX_INITIALIZER_UNLOCKED;

uint8_t defaultRuntimeModeFromStartupMode() {
    if (MASTER_STARTUP_APP_MODE == MASTER_STARTUP_APP_AUTO_DRAW_ID) {
        return MASTER_ENABLE_AUTO_DRAW
                   ? static_cast<uint8_t>(MASTER_RUNTIME_MODE_AUTO_DRAW)
                   : static_cast<uint8_t>(MASTER_RUNTIME_MODE_MANUAL_DRAW);
    }
    return (MASTER_STARTUP_APP_MODE == MASTER_STARTUP_APP_DIAGNOSTICS_ID)
               ? static_cast<uint8_t>(MASTER_RUNTIME_MODE_DIAGNOSTICS)
               : static_cast<uint8_t>(MASTER_RUNTIME_MODE_MANUAL_DRAW);
}

MasterRuntimeModeSnapshot runtimeModeState = {
    defaultRuntimeModeFromStartupMode(),
    defaultRuntimeModeFromStartupMode(),
    MASTER_RUNTIME_MODE_BUTTON_NONE,
    1U,
    0U,
    0U,
    0U,
    0U,
    0UL,
    0UL,
};

bool runtimeModeAvailableInternal(uint8_t mode) {
    switch (mode) {
        case MASTER_RUNTIME_MODE_MANUAL_DRAW:
        case MASTER_RUNTIME_MODE_DIAGNOSTICS:
            return true;
        case MASTER_RUNTIME_MODE_AUTO_DRAW:
            return MASTER_ENABLE_AUTO_DRAW != 0;
        case MASTER_RUNTIME_MODE_BLUETOOTH:
            return MASTER_ENABLE_BLE != 0;
        default:
            return false;
    }
}

void applyRuntimeModeRequest(uint8_t requested_mode,
                             uint8_t button,
                             bool accepted,
                             uint32_t now_ms) {
    portENTER_CRITICAL(&runtimeModeMux);
    runtimeModeState.requested_mode = requested_mode;
    runtimeModeState.last_button = button;
    runtimeModeState.request_accepted = accepted ? 1U : 0U;
    runtimeModeState.request_rejected = accepted ? 0U : 1U;
    runtimeModeState.active_mode =
        accepted ? requested_mode : static_cast<uint8_t>(MASTER_RUNTIME_MODE_MANUAL_DRAW);
    runtimeModeState.request_count++;
    runtimeModeState.last_change_ms = now_ms;
    portEXIT_CRITICAL(&runtimeModeMux);
}

}  // namespace

void updateMasterModeFromSwitches(const MasterModeSwitchEvents &events, uint32_t now_ms) {
    portENTER_CRITICAL(&runtimeModeMux);
    runtimeModeState.manual_button_down = events.manual_down ? 1U : 0U;
    runtimeModeState.auto_button_down = events.auto_down ? 1U : 0U;
    runtimeModeState.bluetooth_button_down = events.bluetooth_down ? 1U : 0U;
    portEXIT_CRITICAL(&runtimeModeMux);

    if (events.manual_pressed) {
        applyRuntimeModeRequest(MASTER_RUNTIME_MODE_MANUAL_DRAW,
                                MASTER_RUNTIME_MODE_BUTTON_MANUAL_DRAW,
                                true,
                                now_ms);
    } else if (events.auto_pressed) {
        // 自动绘图当前只发 dry-run/trajectory 语义，真实 UV 仍由从机安全联锁决定。
        applyRuntimeModeRequest(MASTER_RUNTIME_MODE_AUTO_DRAW,
                                MASTER_RUNTIME_MODE_BUTTON_AUTO_DRAW,
                                runtimeModeAvailableInternal(MASTER_RUNTIME_MODE_AUTO_DRAW),
                                now_ms);
    } else if (events.bluetooth_pressed) {
        // BLE 入口受 MASTER_ENABLE_BLE 保护；关闭时只记录拒绝请求并回到手动绘图。
        applyRuntimeModeRequest(MASTER_RUNTIME_MODE_BLUETOOTH,
                                MASTER_RUNTIME_MODE_BUTTON_BLUETOOTH,
                                runtimeModeAvailableInternal(MASTER_RUNTIME_MODE_BLUETOOTH),
                                now_ms);
    }
}

uint8_t masterDefaultRuntimeMode() {
    return defaultRuntimeModeFromStartupMode();
}

bool masterRuntimeModeAvailable(uint8_t mode) {
    return runtimeModeAvailableInternal(mode);
}

MasterRuntimeModeSnapshot getMasterRuntimeModeSnapshot() {
    MasterRuntimeModeSnapshot snapshot = {};
    portENTER_CRITICAL(&runtimeModeMux);
    snapshot = runtimeModeState;
    portEXIT_CRITICAL(&runtimeModeMux);
    return snapshot;
}

const char *masterRuntimeModeName(uint8_t mode) {
    switch (mode) {
        case MASTER_RUNTIME_MODE_MANUAL_DRAW:
            return "ManualDraw";
        case MASTER_RUNTIME_MODE_AUTO_DRAW:
            return "AutoDraw";
        case MASTER_RUNTIME_MODE_BLUETOOTH:
            return "Bluetooth";
        case MASTER_RUNTIME_MODE_DIAGNOSTICS:
            return "Diagnostics";
        default:
            return "Unknown";
    }
}

const char *masterRuntimeButtonName(uint8_t button) {
    switch (button) {
        case MASTER_RUNTIME_MODE_BUTTON_MANUAL_DRAW:
            return "G42Manual";
        case MASTER_RUNTIME_MODE_BUTTON_AUTO_DRAW:
            return "G41Auto";
        case MASTER_RUNTIME_MODE_BUTTON_BLUETOOTH:
            return "G40Bluetooth";
        case MASTER_RUNTIME_MODE_BUTTON_NONE:
        default:
            return "None";
    }
}
