#pragma once

#include "master/diagnostics/master_diagnostics.h"

// 在驱动 EN/PWM 已进入运行态偏置后校准 ADC offset，保持原始电流采样公式不变。
bool calibrateMasterCurrentSenseOffsets(MasterMotorDiagnosticsContext &context);
