#pragma once

#include "master/modes/mode_traits.h"

constexpr bool masterRunModeRunsAxis(AxisId axis) {
    return masterRunModeHasLogicalAxis(axis);
}

constexpr bool masterRunModeReadsEncoder(AxisId axis) {
    return masterRunModeNeedsEncoderHardware(axis);
}

constexpr bool masterRunModeDrivesAxis(AxisId axis) {
    return masterRunModeNeedsMotorHardware(axis);
}
