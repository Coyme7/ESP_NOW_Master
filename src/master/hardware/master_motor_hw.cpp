#include "master/hardware/master_motor_hw.h"

#include <Arduino.h>
#include <board/board_pins_master.h>

#include "common/math/clamp.h"
#include "common/state/system_state.h"
#include "master/config/master_config.h"
#include "master/diagnostics/current_probe.h"
#include "master/diagnostics/master_diagnostics.h"
#include "master/hardware/master_board_io.h"
#include "master/hardware/master_current_sense_adc1.h"
#include "master/hardware/master_encoder_hw.h"
#include "master/tasks/master_tasks.h"

#if MASTER_MOTOR_HW_ENABLED
#include <SimpleFOC.h>
#endif

namespace {

#if MASTER_MOTOR_HW_ENABLED
bool knobMotorReady = false;
const char *knobHardwareStatus = "not_started";

BLDCMotor knobMotor = BLDCMotor(MASTER_MOTOR_POLE_PAIRS);
BLDCDriver3PWM knobDriver = BLDCDriver3PWM(
    board_pins_master::MOTOR1_PWM_U,
    board_pins_master::MOTOR1_PWM_V,
    board_pins_master::MOTOR1_PWM_W,
    board_pins_master::MOTOR1_EN);

MasterAdc1CurrentSense knobCurrentSense = MasterAdc1CurrentSense(
    MASTER_CURRENT_SHUNT_OHM,
    MASTER_CURRENT_SENSE_GAIN,
    board_pins_master::MOTOR1_CURRENT_A,
    board_pins_master::MOTOR1_CURRENT_B);

// 把当前硬件对象打包给诊断模块，避免诊断模块直接访问匿名命名空间变量。
MasterMotorDiagnosticsContext makeMasterDiagnosticsContext() {
    return {
        knobMotor,
        knobDriver,
        knobCurrentSense,
        masterKnobSensor(),
    };
}
#endif

}  // namespace

// 主机电机硬件初始化总流程：编码器、驱动、电流采样、motor.init、initFOC。
bool setupMasterMotorHardware() {
#if MASTER_MOTOR_HW_ENABLED
#if MASTER_SIMPLEFOC_DEBUG_ENABLED
    SimpleFOCDebug::enable(&Serial);
#endif

    knobHardwareStatus = "spi_sensor_init";
    setupMasterEncoderHardware();
    MasterMotorDiagnosticsContext diagnostics_context = makeMasterDiagnosticsContext();
    knobHardwareStatus = "sensor_ready";

    knobHardwareStatus = "driver_init";
    knobDriver.enable_active_high = MASTER_DRIVER_ENABLE_ACTIVE_HIGH != 0;
    knobDriver.voltage_power_supply = kMasterMotorFoc.supply_voltage_v;
    knobDriver.voltage_limit = kMasterMotorFoc.voltage_limit_v;
    Serial.printf("[Master] motor_diag driver en_active_high=%u supply=%.2fV vlim=%.2fV align=%.2fV pp=%u\n",
                  MASTER_DRIVER_ENABLE_ACTIVE_HIGH ? 1 : 0,
                  kMasterMotorFoc.supply_voltage_v,
                  kMasterMotorFoc.voltage_limit_v,
                  kMasterMotorFoc.align_voltage_v,
                  static_cast<unsigned int>(MASTER_MOTOR_POLE_PAIRS));
    if (!knobDriver.init()) {
        digitalWrite(board_pins_master::MOTOR1_EN, masterDriverDisabledLevel());
        knobHardwareStatus = "driver_init_failed";
        addLocalFault(FAULT_MOTOR_OUTPUT_DISABLED);
        return false;
    }
    // 驱动 init 成功后仍先 disable，直到 initFOC 完成并确认安全后才允许控制输出。
    knobDriver.disable();
    knobHardwareStatus = "driver_ready";

    knobMotor.linkSensor(&masterKnobSensor());
    knobMotor.linkDriver(&knobDriver);
    knobMotor.voltage_sensor_align = kMasterMotorFoc.align_voltage_v;
    knobMotor.voltage_limit = kMasterMotorFoc.voltage_limit_v;
    knobMotor.current_limit = kMasterXAxis.haptic_current_limit_a;
#if MASTER_USE_CURRENT_SENSE
#if MASTER_CURRENT_ZERO_CURRENT_SMOKE_TEST && MASTER_CURRENT_ZERO_SMOKE_USE_DC_CURRENT
    knobMotor.torque_controller = TorqueControlType::dc_current;
    const char *torque_controller_name = "dc_current";
#else
    knobMotor.torque_controller = TorqueControlType::foc_current;
    const char *torque_controller_name = "foc_current";
#endif
#else
    knobMotor.torque_controller = TorqueControlType::voltage;
    const char *torque_controller_name = "voltage";
#endif
    knobMotor.controller = MotionControlType::torque;
    Serial.printf("[Master] motor_diag torque_controller=%s\n", torque_controller_name);

    // 写入 P-only 电流环参数；I/D 当前保持 0，优先保证力反馈稳定和可预测。
    knobMotor.PID_current_q.P = kMasterMotorFoc.current_q_pid_p;
    knobMotor.PID_current_q.I = kMasterMotorFoc.current_q_pid_i;
    knobMotor.PID_current_q.D = kMasterMotorFoc.current_q_pid_d;
    knobMotor.PID_current_q.output_ramp = kMasterMotorFoc.current_q_pid_ramp;
    knobMotor.PID_current_q.limit = kMasterMotorFoc.voltage_limit_v;
    knobMotor.PID_current_d.P = kMasterMotorFoc.current_d_pid_p;
    knobMotor.PID_current_d.I = kMasterMotorFoc.current_d_pid_i;
    knobMotor.PID_current_d.D = kMasterMotorFoc.current_d_pid_d;
    knobMotor.PID_current_d.output_ramp = kMasterMotorFoc.current_d_pid_ramp;
    knobMotor.PID_current_d.limit = kMasterMotorFoc.voltage_limit_v;
    knobMotor.LPF_current_q.Tf = kMasterMotorFoc.current_lpf_tf;
    knobMotor.LPF_current_d.Tf = kMasterMotorFoc.current_lpf_tf;

#if MASTER_USE_CURRENT_SENSE
    knobHardwareStatus = "current_sense_init";
    knobCurrentSense.linkDriver(&knobDriver);
    knobCurrentSense.skip_align = MASTER_CURRENT_SENSE_SKIP_ALIGN != 0;
    Serial.printf("[Master] motor_diag current_sense adc1 shunt=%.4fohm gain=%.2f pins=%d,%d raw_to_v=%.9f skip_align=%u\n",
                  MASTER_CURRENT_SHUNT_OHM,
                  MASTER_CURRENT_SENSE_GAIN,
                  board_pins_master::MOTOR1_CURRENT_A,
                  board_pins_master::MOTOR1_CURRENT_B,
                  static_cast<float>(MASTER_CURRENT_SENSE_ADC_RAW_TO_VOLTAGE_V),
                  MASTER_CURRENT_SENSE_SKIP_ALIGN ? 1 : 0);
    if (!knobCurrentSense.init()) {
        knobDriver.disable();
        knobHardwareStatus = "current_sense_init_failed";
        addLocalFault(FAULT_MOTOR_OUTPUT_DISABLED);
        return false;
    }
    calibrateMasterCurrentSenseOffsets(diagnostics_context);
    knobCurrentSense.gain_a *= (MASTER_CURRENT_SENSE_GAIN_SIGN_A < 0) ? -1.0f : 1.0f;
    knobCurrentSense.gain_b *= (MASTER_CURRENT_SENSE_GAIN_SIGN_B < 0) ? -1.0f : 1.0f;
    Serial.printf("[Master] motor_diag current_sense offsets ia=%.3fV ib=%.3fV gain_a=%.2f gain_b=%.2f sign_a=%d sign_b=%d\n",
                  knobCurrentSense.offset_ia,
                  knobCurrentSense.offset_ib,
                  knobCurrentSense.gain_a,
                  knobCurrentSense.gain_b,
                  MASTER_CURRENT_SENSE_GAIN_SIGN_A,
                  MASTER_CURRENT_SENSE_GAIN_SIGN_B);
    knobMotor.linkCurrentSense(&knobCurrentSense);
    knobHardwareStatus = "current_sense_ready";
#if MASTER_CURRENT_SENSE_DIAG_ONLY
    knobHardwareStatus = "current_sense_diag_only";
    (void)runMasterDiagnosticsBeforeFoc(diagnostics_context);
    Serial.println("[Master] motor_diag current_sense diag_only; keep motor disabled");
    addLocalFault(FAULT_MOTOR_OUTPUT_DISABLED);
    return false;
#endif
#else
    knobHardwareStatus = "current_sense_skipped";
    Serial.println("[Master] motor_diag current_sense skipped for voltage torque bring-up");
#endif

    knobHardwareStatus = "motor_init";
    knobMotor.init();

    runMasterDiagnosticsAfterMotorInit(diagnostics_context);
    knobHardwareStatus = "init_foc";
    if (!knobMotor.initFOC()) {
        knobDriver.disable();
        knobHardwareStatus = "init_foc_failed";
        Serial.println("[Master] motor_diag initFOC failed; keep motor disabled");
        addLocalFault(FAULT_MOTOR_OUTPUT_DISABLED);
        return false;
    }
    knobMotorReady = true;
    setMasterKnobMotorReadyForEncoder(true);
    knobHardwareStatus = "ready";
    return true;
#else
    addLocalFault(FAULT_MOTOR_OUTPUT_DISABLED);
    return false;
#endif
}

// 返回当前硬件初始化状态字符串，用于启动阶段和状态行诊断。
const char *getMasterMotorHardwareStatus() {
#if MASTER_MOTOR_HW_ENABLED
    return knobHardwareStatus;
#else
    return "compiled_off";
#endif
}

// 清除 SimpleFOC q/d 电流环 PID 状态，避免模式切换或安全切断后的残余输出。
void resetMasterMotorCurrentPid() {
#if MASTER_MOTOR_HW_ENABLED
    if (!knobMotorReady) {
        return;
    }
    knobMotor.PID_current_q.reset();
    knobMotor.PID_current_d.reset();
#endif
}

// 控制热路径硬件输出：限幅目标电流，执行 move() 和 loopFOC()，并记录耗时。
void runMasterMotorOutput(float target_current_a) {
#if MASTER_MOTOR_HW_ENABLED
    if (!knobMotorReady) {
        (void)target_current_a;
        return;
    }
#if MASTER_CONTROL_TIMING_DIAG_ENABLED
    const uint32_t motor_start_us = micros();
#endif
    static uint8_t publish_div = 0;
    // 硬件输出层再次限幅，防止上层算法异常时越过配置电流上限。
    const float safe_current_a = clampFloat(target_current_a,
                                           -kMasterXAxis.haptic_current_limit_a,
                                           kMasterXAxis.haptic_current_limit_a);
#if MASTER_CONTROL_TIMING_DIAG_ENABLED
    const uint32_t move_start_us = micros();
#endif
    knobMotor.move(safe_current_a);
#if MASTER_CONTROL_TIMING_DIAG_ENABLED
    const uint32_t after_move_us = micros();
#endif
    knobMotor.loopFOC();
#if MASTER_CONTROL_TIMING_DIAG_ENABLED
    const uint32_t after_loop_foc_us = micros();
#endif
#if MASTER_CONTROL_STATUS_PUBLISH_DIV <= 1
    const bool publish_now = true;
#else
    publish_div++;
    const bool publish_now = publish_div >= MASTER_CONTROL_STATUS_PUBLISH_DIV;
    if (publish_now) {
        publish_div = 0;
    }
#endif
    if (publish_now) {
        sysData.master.current_q_a = knobMotor.current.q;
        sysData.master.current_d_a = knobMotor.current.d;
        sysData.master.voltage_q_v = knobMotor.voltage.q;
        sysData.master.voltage_d_v = knobMotor.voltage.d;
    }
#if MASTER_CONTROL_TIMING_DIAG_ENABLED
    recordMasterTimingMotorUs(after_loop_foc_us - motor_start_us,
                              after_move_us - move_start_us,
                              after_loop_foc_us - after_move_us,
                              masterKnobSensor().lastReadDurationUs());
#endif
#else
    (void)target_current_a;
#endif
}
