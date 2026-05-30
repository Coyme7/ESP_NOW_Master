#pragma once

#include <stdint.h>

#include "common/app/mode_capability.h"

ModeCapability masterManualDrawCapability();
uint8_t masterManualDrawProtocolMode();
uint16_t masterManualDrawCommandFlags();
