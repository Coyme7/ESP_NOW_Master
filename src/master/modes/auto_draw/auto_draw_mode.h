#pragma once

#include <stdint.h>

#include "common/app/mode_capability.h"
#include "master/modes/auto_draw/trajectory/auto_draw_trajectory.h"

ModeCapability masterAutoDrawCapability();
uint8_t masterAutoDrawProtocolMode();
uint16_t masterAutoDrawCommandFlags();
uint16_t masterAutoDrawTaskId();
uint8_t masterAutoDrawSegmentCount();
const char *masterAutoDrawPresetName();
bool masterAutoDrawSegmentAt(uint8_t index, MasterAutoDrawSegmentSpec &segment);
