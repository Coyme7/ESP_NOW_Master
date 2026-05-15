#pragma once

bool setupMasterMotorHardware();
const char *getMasterMotorHardwareStatus();
void resetMasterMotorCurrentPid();
void runMasterMotorOutput(float target_current_a);

