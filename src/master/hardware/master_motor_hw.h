#pragma once

bool setupMasterMotorHardware();
const char *getMasterMotorHardwareStatus();
void resetMasterMotorCurrentPid();
void runMasterMotorOutput(float x_target_current_a, float y_target_current_a);

inline void runMasterMotorOutput(float target_current_a) {
    runMasterMotorOutput(target_current_a, 0.0f);
}
