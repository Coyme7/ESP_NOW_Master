#include "master/modes/mode_switch.h"

#include "master/config/build/master_bringup_config.h"
#include "master/hardware/master_board_io.h"

namespace {

struct DebouncedButton {
    bool raw_down;
    bool stable_down;
    uint32_t raw_change_ms;
};

DebouncedButton manualDrawButton = {};
DebouncedButton autoDrawButton = {};
DebouncedButton bluetoothButton = {};

constexpr uint32_t kButtonDebounceMs = 30;

bool updateDebouncedButton(DebouncedButton &button, bool raw_down, uint32_t now_ms) {
    if (raw_down != button.raw_down) {
        button.raw_down = raw_down;
        button.raw_change_ms = now_ms;
    }

    if (button.stable_down == button.raw_down) {
        return false;
    }

    if (static_cast<uint32_t>(now_ms - button.raw_change_ms) < kButtonDebounceMs) {
        return false;
    }

    button.stable_down = button.raw_down;
    return button.stable_down;
}

}  // namespace

MasterModeSwitchEvents updateMasterModeSwitches(uint32_t now_ms) {
    const bool manual_down = readMasterManualDrawButtonDown();
#if MASTER_ENABLE_AUTO_DRAW
    const bool auto_down = readMasterAutoDrawButtonDown();
#else
    const bool auto_down = false;
#endif
#if MASTER_ENABLE_BLE
    const bool bluetooth_down = readMasterBluetoothButtonDown();
#else
    const bool bluetooth_down = false;
#endif

    MasterModeSwitchEvents events = {};
    events.manual_pressed = updateDebouncedButton(manualDrawButton, manual_down, now_ms);
    events.auto_pressed = updateDebouncedButton(autoDrawButton, auto_down, now_ms);
    events.bluetooth_pressed = updateDebouncedButton(bluetoothButton, bluetooth_down, now_ms);
    events.manual_down = manualDrawButton.stable_down;
    events.auto_down = autoDrawButton.stable_down;
    events.bluetooth_down = bluetoothButton.stable_down;
    return events;
}
