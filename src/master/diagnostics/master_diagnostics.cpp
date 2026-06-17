#include "master/diagnostics/master_diagnostics.h"

#include "master/config/master_config.h"
#include "master/diagnostics/phase_probe.h"

// motor.init 后、initFOC 前诊断：可执行开环相位扫描。
void runMasterDiagnosticsAfterMotorInit(MasterMotorDiagnosticsContext &context) {
#if MASTER_ENABLE_PHASE_SCAN_TEST
    runMasterPhaseProbe(context);
#else
    (void)context;
#endif
}
