#include "master/diagnostics/phase_probe.h"

#include <Arduino.h>

#include "common/math/angle_math.h"
#include "common/math/clamp.h"
#include "master/config/master_config.h"

#if MASTER_MOTOR_HW_ENABLED && MASTER_MOTOR_PHASE_PROBE_ENABLED
namespace {

// 读取当前机械角度并转成度，用于判断开环电压是否带动电机。
float readKnobMechanicalAngleDeg(MasterMotorDiagnosticsContext &context) {
    context.sensor.update();
    return radToDeg(context.sensor.getMechanicalAngle());
}

}  // namespace
#endif

// 低压开环扫相诊断，只在启动诊断路径运行，不能进入控制热路径。
void runMasterPhaseProbe(MasterMotorDiagnosticsContext &context) {
#if MASTER_MOTOR_HW_ENABLED && MASTER_MOTOR_PHASE_PROBE_ENABLED
    const float probe_voltage_v = clampFloat(kMasterMotorFoc.align_voltage_v,
                                             0.0f,
                                             kMasterMotorFoc.voltage_limit_v);
    const float start_angle_deg = readKnobMechanicalAngleDeg(context);
    Serial.printf("[Master] motor_probe start voltage=%.2fV raw=%u angle=%.2fdeg\n",
                  probe_voltage_v,
                  static_cast<unsigned int>(context.sensor.rawAngle()),
                  start_angle_deg);

    const float probe_angles_rad[] = {
        degToRad(270.0f),
        degToRad(330.0f),
        degToRad(30.0f),
        degToRad(90.0f),
        degToRad(150.0f),
        degToRad(210.0f),
        degToRad(270.0f),
    };

    for (size_t i = 0; i < sizeof(probe_angles_rad) / sizeof(probe_angles_rad[0]); ++i) {
        context.motor.setPhaseVoltage(probe_voltage_v, 0.0f, probe_angles_rad[i]);
        delay(180);
        const float angle_deg = readKnobMechanicalAngleDeg(context);
        const float delta_deg = absoluteAngleDegToSignedAngleDeg(angle_deg, start_angle_deg);
        Serial.printf("[Master] motor_probe step=%u elec=%.1fdeg raw=%u angle=%.2fdeg delta=%.2fdeg\n",
                      static_cast<unsigned int>(i),
                      radToDeg(probe_angles_rad[i]),
                      static_cast<unsigned int>(context.sensor.rawAngle()),
                      angle_deg,
                      delta_deg);
    }

    context.motor.setPhaseVoltage(0.0f, 0.0f, 0.0f);
    delay(100);
#else
    (void)context;
#endif
}
