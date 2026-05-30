#include "master/modes/auto_draw/auto_draw_mode.h"

#include "common/protocol/protocol_types.h"
#include "common/timing/link_timing.h"
#include "master/config/core/master_control_config.h"

ModeCapability masterAutoDrawCapability() {
    ModeCapability capability = {};
    capability.flags = MODE_CAP_REMOTE_COMMAND | MODE_CAP_TRAJECTORY |
                       MODE_CAP_PEN | MODE_CAP_DRY_RUN;
    capability.control_rate_hz =
        static_cast<uint16_t>(1000000UL / MASTER_CONTROL_LOOP_PERIOD_US);
    capability.outer_rate_hz =
        static_cast<uint16_t>(1000UL / MASTER_COMMAND_PERIOD_MS);
    return capability;
}

uint8_t masterAutoDrawProtocolMode() {
    // 当前自动绘图仍处于干跑命令阶段；真实 UV 只允许由从机安全联锁最终决定。
    return MODE_DUALXY_DRAW_DRY_RUN;
}

uint16_t masterAutoDrawCommandFlags() {
    return PACKET_FLAG_DRY_RUN | PACKET_FLAG_TRAJECTORY_ACTIVE;
}

uint16_t masterAutoDrawTaskId() {
    return masterAutoDrawTrajectoryTaskId();
}

uint8_t masterAutoDrawSegmentCount() {
    return masterAutoDrawTrajectorySegmentCount();
}

const char *masterAutoDrawPresetName() {
    return masterAutoDrawTrajectoryPresetName();
}

bool masterAutoDrawSegmentAt(uint8_t index, MasterAutoDrawSegmentSpec &segment) {
    return masterAutoDrawTrajectorySegmentAt(index, segment);
}
