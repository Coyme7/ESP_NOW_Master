#include "master/modes/mode_table.h"

#include "master/config/build/master_bringup_config.h"
#include "master/config/core/master_control_config.h"
#include "master/modes/auto_draw/auto_draw_mode.h"
#include "master/modes/ble/ble_mode.h"
#include "master/modes/manual_draw/manual_draw_mode.h"
#include "master/modes/mode_manager.h"
#include "master/modes/mode_traits.h"

const char *masterStartupAppModeName() {
    switch (MASTER_STARTUP_APP_MODE) {
        case MASTER_STARTUP_APP_MANUAL_DRAW_ID:
            return "ManualDraw";
        case MASTER_STARTUP_APP_AUTO_DRAW_ID:
            return "AutoDraw";
        case MASTER_STARTUP_APP_DIAGNOSTICS_ID:
            return "Diagnostics";
        default:
            return "Unknown";
    }
}

const char *masterRunModeName() {
    switch (MASTER_RUN_MODE) {
        case MASTER_MODE_SINGLE_X_10KHZ_ID:
            return "SingleX_10kHz";
        case MASTER_MODE_SINGLE_Y_10KHZ_ID:
            return "SingleY_10kHz";
        case MASTER_MODE_DUAL_XY_5KHZ_ID:
            return "DualXY_5kHz";
        default:
            return "Unknown";
    }
}

const char *masterRunPathName() {
    if (masterRunModeIsDualXYLogic()) {
        return "dual_xy_hardware";
    }
    return masterRunModeHasLogicalAxis(AXIS_Y) ? "single_y_hardware" : "single_x_hardware";
}

ModeCapability masterModeCapabilityForRuntime(uint8_t runtime_mode) {
    switch (runtime_mode) {
        case MASTER_RUNTIME_MODE_MANUAL_DRAW:
            return masterManualDrawCapability();
        case MASTER_RUNTIME_MODE_AUTO_DRAW:
            return masterRuntimeModeAvailable(runtime_mode) ? masterAutoDrawCapability() : ModeCapability{};
        case MASTER_RUNTIME_MODE_BLUETOOTH:
            return masterRuntimeModeAvailable(runtime_mode) ? masterBleCapability() : ModeCapability{};
        case MASTER_RUNTIME_MODE_DIAGNOSTICS: {
            ModeCapability capability = {};
            capability.flags = MODE_CAP_NONE;
            capability.control_rate_hz =
                static_cast<uint16_t>(1000000UL / MASTER_CONTROL_LOOP_PERIOD_US);
            capability.outer_rate_hz = 333;
            return capability;
        }
        default:
            return ModeCapability{};
    }
}

ModeCapability masterModeCapability() {
    const MasterRuntimeModeSnapshot runtime = getMasterRuntimeModeSnapshot();
    return masterModeCapabilityForRuntime(runtime.active_mode);
}
