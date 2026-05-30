#include "master/config/core/master_axis_config.h"

#include "common/protocol/protocol_units.h"
#include "master/config/haptic/master_haptic_config.h"

const MasterAxisConfig kMasterXAxis = {
    kMasterXAxisInput,
    {
        -MASTER_KNOB_HALF_RANGE_DEG, // X 低端虚拟边界，单位 deg。
        MASTER_KNOB_HALF_RANGE_DEG,  // X 高端虚拟边界，单位 deg。
    },
    kMasterXAxisCurrent,
    kMasterXHaptic,
};

const MasterAxisConfig kMasterYAxis = {
    kMasterYAxisInput,
    {
        -MASTER_KNOB_HALF_RANGE_DEG, // Y 低端虚拟边界，单位 deg。
        MASTER_KNOB_HALF_RANGE_DEG,  // Y 高端虚拟边界，单位 deg。
    },
    kMasterYAxisCurrent,
    kMasterYHaptic,
};
