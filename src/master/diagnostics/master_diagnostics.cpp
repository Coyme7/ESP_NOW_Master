#include "master/diagnostics/master_diagnostics.h"

#include "master/config/master_config.h"
#include "master/diagnostics/current_probe.h"
#include "master/diagnostics/phase_probe.h"

// initFOC 前诊断：主要用于电流采样通道和符号检查。
bool runMasterDiagnosticsBeforeFoc(MasterMotorDiagnosticsContext &context) {
#if MASTER_USE_CURRENT_SENSE && MASTER_CURRENT_SENSE_DIAG_ONLY
    runMasterCurrentSenseProbe(context);
    return true;
#else
    (void)context;
    return false;
#endif
}

// motor.init 后、initFOC 前诊断：可执行开环相位扫描。
void runMasterDiagnosticsAfterMotorInit(MasterMotorDiagnosticsContext &context) {
#if MASTER_MOTOR_PHASE_PROBE_ENABLED
    runMasterPhaseProbe(context);
#else
    (void)context;
#endif
}
