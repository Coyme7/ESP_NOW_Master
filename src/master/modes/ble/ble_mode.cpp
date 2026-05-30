#include "master/modes/ble/ble_mode.h"

#include "master/config/build/master_bringup_config.h"
#include "master/config/core/master_control_config.h"
#include "master/modes/mode_traits.h"

ModeCapability masterBleCapability() {
    ModeCapability capability = {};
    // BLE 是主机单机人机模式，不依赖从机、不发送绘图命令、不允许 UV。
    capability.flags = MODE_CAP_NONE;
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
    capability.control_rate_hz =
        static_cast<uint16_t>(1000000UL / MASTER_CONTROL_LOOP_PERIOD_US);
    capability.outer_rate_hz = 0;
    return capability;
}
