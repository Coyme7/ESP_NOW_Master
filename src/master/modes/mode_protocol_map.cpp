#include "master/modes/mode_protocol_map.h"

#include "common/protocol/protocol_types.h"
#include "master/modes/auto_draw/auto_draw_mode.h"
#include "master/modes/manual_draw/manual_draw_mode.h"
#include "master/modes/mode_manager.h"

uint8_t masterProtocolModeForRuntime(uint8_t runtime_mode) {
    switch (runtime_mode) {
        case MASTER_RUNTIME_MODE_MANUAL_DRAW:
            return masterManualDrawProtocolMode();
        case MASTER_RUNTIME_MODE_AUTO_DRAW:
            return masterAutoDrawProtocolMode();
        case MASTER_RUNTIME_MODE_BLUETOOTH:
            return MODE_BLE_MEDIA;
        case MASTER_RUNTIME_MODE_DIAGNOSTICS:
            return MODE_DUALXY_DRY_RUN;
        default:
            return masterManualDrawProtocolMode();
    }
}

uint16_t masterCommandFlagsForRuntime(uint8_t runtime_mode) {
    switch (runtime_mode) {
        case MASTER_RUNTIME_MODE_MANUAL_DRAW:
            return masterManualDrawCommandFlags();
        case MASTER_RUNTIME_MODE_AUTO_DRAW:
            return masterAutoDrawCommandFlags();
        case MASTER_RUNTIME_MODE_BLUETOOTH:
            return PACKET_FLAG_NONE;
        case MASTER_RUNTIME_MODE_DIAGNOSTICS:
            return PACKET_FLAG_DRY_RUN;
        default:
            return masterManualDrawCommandFlags();
    }
}

uint8_t masterProtocolMode() {
    const MasterRuntimeModeSnapshot runtime = getMasterRuntimeModeSnapshot();
    return masterProtocolModeForRuntime(runtime.active_mode);
}

uint16_t masterCommandFlags() {
    const MasterRuntimeModeSnapshot runtime = getMasterRuntimeModeSnapshot();
    return masterCommandFlagsForRuntime(runtime.active_mode);
}
