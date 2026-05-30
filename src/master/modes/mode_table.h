#pragma once

#include <stdint.h>

#include "common/app/mode_capability.h"

const char *masterStartupAppModeName();
const char *masterRunModeName();
const char *masterRunPathName();
ModeCapability masterModeCapabilityForRuntime(uint8_t runtime_mode);
ModeCapability masterModeCapability();
