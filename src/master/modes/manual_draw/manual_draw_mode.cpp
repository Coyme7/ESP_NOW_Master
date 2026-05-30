#include "master/modes/manual_draw/manual_draw_mode.h"

#include "common/protocol/protocol_types.h"
#include "common/timing/link_timing.h"
#include "master/config/build/master_bringup_config.h"
#include "master/config/core/master_comm_config.h"
#include "master/config/core/master_control_config.h"
#include "master/modes/mode_traits.h"

ModeCapability masterManualDrawCapability() {
    ModeCapability capability = {};
    capability.control_rate_hz =
        static_cast<uint16_t>(1000000UL / MASTER_CONTROL_LOOP_PERIOD_US);
    capability.outer_rate_hz =
        static_cast<uint16_t>(1000UL / MASTER_COMMAND_PERIOD_MS);
    const uint16_t remote_flags =
        MASTER_ENABLE_ESPNOW ? (MODE_CAP_REMOTE_COMMAND | MODE_CAP_PEN) : MODE_CAP_NONE;
    capability.flags = remote_flags;
    if (masterRunModeHasLogicalAxis(AXIS_X) && MASTER_ENABLE_X_ENCODER_HW) {
        capability.flags |= MODE_CAP_X_SENSOR;
        if (MASTER_ENABLE_X_MOTOR_HW) {
            capability.flags |= MODE_CAP_X_MOTOR;
        }
    }
    if (masterRunModeHasLogicalAxis(AXIS_Y) && MASTER_ENABLE_Y_ENCODER_HW) {
        capability.flags |= MODE_CAP_Y_SENSOR;
        if (MASTER_ENABLE_Y_MOTOR_HW) {
            capability.flags |= MODE_CAP_Y_MOTOR;
        }
    }
    return capability;
}

uint8_t masterManualDrawProtocolMode() {
    return MODE_COLLAB_DRAW;
}

uint16_t masterManualDrawCommandFlags() {
    return PACKET_FLAG_NONE;
}
