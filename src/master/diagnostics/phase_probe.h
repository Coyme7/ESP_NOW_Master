#pragma once

#include "master/diagnostics/master_diagnostics.h"

// 开环电角度扫描，用于确认相序、方向和编码器读数变化。
void runMasterPhaseProbe(MasterMotorDiagnosticsContext &context);
