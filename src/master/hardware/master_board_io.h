#pragma once

#include <stdint.h>

int masterDriverDisabledLevel();
void configureMasterSafeOutputs();
bool readMasterPenButtonDown();
bool readMasterManualDrawButtonDown();
bool readMasterAutoDrawButtonDown();
bool readMasterBluetoothButtonDown();
