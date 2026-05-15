#pragma once

#include <SimpleFOC.h>

#include "master/hardware/master_current_sense_adc1.h"
#include "master/hardware/master_encoder_hw.h"

// 主机诊断模块共享的硬件上下文。
// 诊断代码只在启动/显式诊断路径运行，不进入 125us / 8kHz 控制热路径。
// 诊断上下文用引用持有硬件对象，避免诊断函数依赖全局变量过多。
struct MasterMotorDiagnosticsContext {
    BLDCMotor &motor;
    BLDCDriver3PWM &driver;
    MasterAdc1CurrentSense &current_sense;
    MasterMt6701Sensor &sensor;
};

// current_sense diag_only 会在 initFOC 前占用启动流程，返回 true 表示调用方应保持电机禁用。
bool runMasterDiagnosticsBeforeFoc(MasterMotorDiagnosticsContext &context);

// 开环相位扫描需要在 motor.init() 后、initFOC() 前运行。
void runMasterDiagnosticsAfterMotorInit(MasterMotorDiagnosticsContext &context);
