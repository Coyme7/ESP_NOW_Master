#pragma once

#include <stdint.h>

int masterDriverDisabledLevel();
void configureMasterSafeOutputs();
bool readMasterPenButtonDown();
int32_t readMasterDemoEncoderCount();

