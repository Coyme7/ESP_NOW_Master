#pragma once

#include "master/diagnostics/master_diagnostics.h"

// 在驱动 EN/PWM 已进入运行态偏置后校准 ADC offset，保持原始电流采样公式不变。
void calibrateMasterCurrentSenseOffsets(MasterMotorDiagnosticsContext &context);

// U/V/W 单相注入诊断，只在 MASTER_CURRENT_SENSE_DIAG_ONLY 下由启动流程调用。
void runMasterCurrentSenseProbe(MasterMotorDiagnosticsContext &context);
