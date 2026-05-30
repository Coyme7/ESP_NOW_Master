#pragma once

#include <stdint.h>

uint8_t masterProtocolModeForRuntime(uint8_t runtime_mode);
uint16_t masterCommandFlagsForRuntime(uint8_t runtime_mode);
uint8_t masterProtocolMode();
uint16_t masterCommandFlags();
