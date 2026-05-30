#include "master/modes/mode_switch.h"

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
    const bool auto_down = readMasterAutoDrawButtonDown();
    const bool bluetooth_down = readMasterBluetoothButtonDown();

    MasterModeSwitchEvents events = {};
    events.manual_pressed = updateDebouncedButton(manualDrawButton, manual_down, now_ms);
    events.auto_pressed = updateDebouncedButton(autoDrawButton, auto_down, now_ms);
    events.bluetooth_pressed = updateDebouncedButton(bluetoothButton, bluetooth_down, now_ms);
    events.manual_down = manualDrawButton.stable_down;
    events.auto_down = autoDrawButton.stable_down;
    events.bluetooth_down = bluetoothButton.stable_down;
    return events;
}
