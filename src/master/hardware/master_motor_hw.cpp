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
#include "master/modes/mode_traits.h"
#include "master/tasks/master_tasks.h"

#if MASTER_ENABLE_X_MOTOR_HW || MASTER_ENABLE_Y_MOTOR_HW
#include <SimpleFOC.h>
#endif

namespace {

#if MASTER_ENABLE_X_ENCODER_HW || MASTER_ENABLE_Y_ENCODER_HW || MASTER_ENABLE_X_MOTOR_HW || MASTER_ENABLE_Y_MOTOR_HW
const char *knobHardwareStatus = "not_started";
#endif

#if MASTER_ENABLE_X_MOTOR_HW
bool xKnobMotorReady = false;

BLDCMotor xKnobMotor = BLDCMotor(MASTER_MOTOR_POLE_PAIRS);
BLDCDriver3PWM xKnobDriver = BLDCDriver3PWM(
    board_pins_master::MOTOR1_PWM_U,
    board_pins_master::MOTOR1_PWM_V,
    board_pins_master::MOTOR1_PWM_W,
    board_pins_master::MOTOR_DRIVER_EN);

MasterAdc1CurrentSense xKnobCurrentSense = MasterAdc1CurrentSense(
    kMasterCurrentSenseHardware.shunt_ohm,
    kMasterCurrentSenseHardware.gain,
    board_pins_master::MOTOR1_CURRENT_A,
    board_pins_master::MOTOR1_CURRENT_B);

// 把 X 硬件对象打包给诊断模块，避免诊断模块直接访问匿名命名空间变量。
MasterMotorDiagnosticsContext makeMasterXDiagnosticsContext() {
    return {
        AXIS_X,
        xKnobMotor,
        xKnobDriver,
        xKnobCurrentSense,
        masterKnobSensor(),
        kMasterXMotorFoc,
    };
}
#endif

#if MASTER_ENABLE_Y_MOTOR_HW
bool yKnobMotorReady = false;

BLDCMotor yKnobMotor = BLDCMotor(MASTER_MOTOR_POLE_PAIRS);
BLDCDriver3PWM yKnobDriver = BLDCDriver3PWM(
    board_pins_master::MOTOR2_PWM_U,
    board_pins_master::MOTOR2_PWM_V,
    board_pins_master::MOTOR2_PWM_W,
    board_pins_master::MOTOR_DRIVER_EN);

MasterAdc1CurrentSense yKnobCurrentSense = MasterAdc1CurrentSense(
    kMasterCurrentSenseHardware.shunt_ohm,
    kMasterCurrentSenseHardware.gain,
    board_pins_master::MOTOR2_CURRENT_A,
    board_pins_master::MOTOR2_CURRENT_B);

MasterMotorDiagnosticsContext makeMasterYDiagnosticsContext() {
    return {
        AXIS_Y,
        yKnobMotor,
        yKnobDriver,
        yKnobCurrentSense,
        masterYKnobSensor(),
        kMasterYMotorFoc,
    };
}
#endif

#if MASTER_ENABLE_X_MOTOR_HW || MASTER_ENABLE_Y_MOTOR_HW
void applyMasterMotorFocConfig(BLDCMotor &motor,
                               const MasterAxisConfig &axis_config,
                               const MasterMotorFocConfig &motor_config) {
    motor.voltage_sensor_align = motor_config.align_voltage_v;
    motor.voltage_limit = motor_config.voltage_limit_v;
    motor.current_limit = axis_config.current.limit_a;
#if MASTER_ENABLE_CURRENT_SENSE
#if MASTER_ENABLE_ZERO_CURRENT_TEST && MASTER_ENABLE_ZERO_CURRENT_DC_TEST
    motor.torque_controller = TorqueControlType::dc_current;
#else
    motor.torque_controller = TorqueControlType::foc_current;
#endif
#else
    motor.torque_controller = TorqueControlType::voltage;
#endif
    motor.controller = MotionControlType::torque;

    // X/Y 使用独立只读 FOC 参数对象；每个 motor 自己保存 PID/LPF 状态。
    motor.PID_current_q.P = motor_config.current_loop.q.p;
    motor.PID_current_q.I = motor_config.current_loop.q.i;
    motor.PID_current_q.D = motor_config.current_loop.q.d;
    motor.PID_current_q.output_ramp = motor_config.current_loop.q.output_ramp;
    motor.PID_current_q.limit = motor_config.voltage_limit_v;
    motor.PID_current_d.P = motor_config.current_loop.d.p;
    motor.PID_current_d.I = motor_config.current_loop.d.i;
    motor.PID_current_d.D = motor_config.current_loop.d.d;
    motor.PID_current_d.output_ramp = motor_config.current_loop.d.output_ramp;
    motor.PID_current_d.limit = motor_config.voltage_limit_v;
    motor.LPF_current_q.Tf = motor_config.current_loop.lpf_tf;
    motor.LPF_current_d.Tf = motor_config.current_loop.lpf_tf;
}

const char *masterTorqueControllerName() {
#if MASTER_ENABLE_CURRENT_SENSE
#if MASTER_ENABLE_ZERO_CURRENT_TEST && MASTER_ENABLE_ZERO_CURRENT_DC_TEST
    return "dc_current";
#else
    return "foc_current";
#endif
#else
    return "voltage";
#endif
}

float clampMasterAxisCurrent(float target_current_a, const MasterAxisConfig &axis_config) {
    return clampFloat(target_current_a,
                      -axis_config.current.limit_a,
                      axis_config.current.limit_a);
}

struct MasterMotorAxisStatusNames {
    const char *driver_init;
    const char *driver_init_failed;
    const char *driver_ready;
    const char *current_sense_init;
    const char *current_sense_init_failed;
    const char *current_sense_ready;
    const char *current_sense_skipped;
    const char *motor_init;
    const char *init_foc;
    const char *init_foc_failed;
};

struct MasterMotorAxisInitContext {
    const char *axis_name;
    BLDCMotor &motor;
    BLDCDriver3PWM &driver;
    MasterAdc1CurrentSense &current_sense;
    MasterMt6701Sensor &sensor;
    const MasterAxisConfig &axis_config;
    const MasterMotorFocConfig &motor_config;
    int current_pin_a;
    int current_pin_b;
    int current_gain_sign_a;
    int current_gain_sign_b;
    bool &ready_flag;
    void (*set_encoder_ready)(bool);
    MasterMotorDiagnosticsContext diagnostics;
    MasterMotorAxisStatusNames status;
};

bool initMasterMotorAxis(MasterMotorAxisInitContext &axis) {
    knobHardwareStatus = axis.status.driver_init;
    axis.driver.enable_active_high = MASTER_DRIVER_ENABLE_ACTIVE_HIGH != 0;
    axis.driver.voltage_power_supply = axis.motor_config.supply_voltage_v;
    axis.driver.voltage_limit = axis.motor_config.voltage_limit_v;
    Serial.printf("[Master] motor_diag axis=%s driver en_active_high=%u supply=%.2fV vlim=%.2fV align=%.2fV pp=%u\n",
                  axis.axis_name,
                  MASTER_DRIVER_ENABLE_ACTIVE_HIGH ? 1 : 0,
                  axis.motor_config.supply_voltage_v,
                  axis.motor_config.voltage_limit_v,
                  axis.motor_config.align_voltage_v,
                  static_cast<unsigned int>(MASTER_MOTOR_POLE_PAIRS));
    if (!axis.driver.init()) {
        digitalWrite(board_pins_master::MOTOR_DRIVER_EN, masterDriverDisabledLevel());
        knobHardwareStatus = axis.status.driver_init_failed;
        addLocalFault(FAULT_MOTOR_OUTPUT_DISABLED);
        return false;
    }
    axis.driver.disable();
    knobHardwareStatus = axis.status.driver_ready;

    axis.motor.linkSensor(&axis.sensor);
    axis.motor.linkDriver(&axis.driver);
    applyMasterMotorFocConfig(axis.motor, axis.axis_config, axis.motor_config);
    Serial.printf("[Master] motor_diag axis=%s torque_controller=%s\n",
                  axis.axis_name,
                  masterTorqueControllerName());

#if MASTER_ENABLE_CURRENT_SENSE
    knobHardwareStatus = axis.status.current_sense_init;
    axis.current_sense.linkDriver(&axis.driver);
    axis.current_sense.skip_align = kMasterCurrentSenseHardware.skip_align;
    Serial.printf("[Master] motor_diag axis=%s current_sense adc1 shunt=%.4fohm gain=%.2f pins=%d,%d raw_to_v=%.9f skip_align=%u\n",
                  axis.axis_name,
                  kMasterCurrentSenseHardware.shunt_ohm,
                  kMasterCurrentSenseHardware.gain,
                  axis.current_pin_a,
                  axis.current_pin_b,
                  kMasterCurrentSenseHardware.adc_raw_to_voltage_v,
                  kMasterCurrentSenseHardware.skip_align ? 1 : 0);
    if (!axis.current_sense.init()) {
        axis.driver.disable();
        knobHardwareStatus = axis.status.current_sense_init_failed;
        addLocalFault(FAULT_MOTOR_OUTPUT_DISABLED);
        return false;
    }
    if (!calibrateMasterCurrentSenseOffsets(axis.diagnostics)) {
        axis.driver.disable();
        knobHardwareStatus = axis.status.current_sense_init_failed;
        addLocalFault(FAULT_MOTOR_OUTPUT_DISABLED);
        return false;
    }
    axis.current_sense.gain_a *= (axis.current_gain_sign_a < 0) ? -1.0f : 1.0f;
    axis.current_sense.gain_b *= (axis.current_gain_sign_b < 0) ? -1.0f : 1.0f;
    Serial.printf("[Master] motor_diag axis=%s current_sense offsets ia=%.3fV ib=%.3fV gain_a=%.2f gain_b=%.2f sign_a=%d sign_b=%d\n",
                  axis.axis_name,
                  axis.current_sense.offset_ia,
                  axis.current_sense.offset_ib,
                  axis.current_sense.gain_a,
                  axis.current_sense.gain_b,
                  axis.current_gain_sign_a,
                  axis.current_gain_sign_b);
    axis.motor.linkCurrentSense(&axis.current_sense);
    knobHardwareStatus = axis.status.current_sense_ready;
#else
    knobHardwareStatus = axis.status.current_sense_skipped;
    Serial.printf("[Master] motor_diag axis=%s current_sense skipped for voltage torque bring-up\n",
                  axis.axis_name);
#endif

    knobHardwareStatus = axis.status.motor_init;
    axis.motor.init();

    runMasterDiagnosticsAfterMotorInit(axis.diagnostics);
    knobHardwareStatus = axis.status.init_foc;
    if (!axis.motor.initFOC()) {
        axis.driver.disable();
        knobHardwareStatus = axis.status.init_foc_failed;
        Serial.printf("[Master] motor_diag axis=%s initFOC failed; keep motor disabled\n",
                      axis.axis_name);
        addLocalFault(FAULT_MOTOR_OUTPUT_DISABLED);
        return false;
    }
    axis.ready_flag = true;
    axis.set_encoder_ready(true);
    return true;
}
#endif

}  // namespace

// 主机电机硬件初始化总流程：
// 轴级编码器 -> 轴级驱动 -> 轴级电流采样 -> motor.init -> initFOC。
// 编码器和电机开关分层，便于只读 X/Y 编码器、只测单轴电机或双轴共同工作。
bool setupMasterMotorHardware() {
#if MASTER_ENABLE_X_ENCODER_HW || MASTER_ENABLE_Y_ENCODER_HW || MASTER_ENABLE_X_MOTOR_HW || MASTER_ENABLE_Y_MOTOR_HW
#if (MASTER_ENABLE_X_MOTOR_HW || MASTER_ENABLE_Y_MOTOR_HW) && MASTER_SIMPLEFOC_DEBUG_ENABLED
    SimpleFOCDebug::enable(&Serial);
#endif

    if (masterRunModeNeedsEncoderHardware(AXIS_X) && !MASTER_ENABLE_X_ENCODER_HW) {
        knobHardwareStatus = "x_encoder_compiled_off";
        addLocalFault(FAULT_MOTOR_OUTPUT_DISABLED);
        return false;
    }
    if (masterRunModeNeedsEncoderHardware(AXIS_Y) && !MASTER_ENABLE_Y_ENCODER_HW) {
        knobHardwareStatus = "y_encoder_compiled_off";
        addLocalFault(FAULT_MOTOR_OUTPUT_DISABLED);
        return false;
    }
    if (masterRunModeNeedsMotorHardware(AXIS_X) && !MASTER_ENABLE_X_MOTOR_HW) {
        knobHardwareStatus = "x_motor_compiled_off";
        addLocalFault(FAULT_MOTOR_OUTPUT_DISABLED);
        return false;
    }
    if (masterRunModeNeedsMotorHardware(AXIS_Y) && !MASTER_ENABLE_Y_MOTOR_HW) {
        knobHardwareStatus = "y_motor_compiled_off";
        addLocalFault(FAULT_MOTOR_OUTPUT_DISABLED);
        return false;
    }

#if MASTER_ENABLE_X_ENCODER_HW || MASTER_ENABLE_Y_ENCODER_HW
    knobHardwareStatus = "spi_sensor_init";
    setupMasterEncoderHardware();
    knobHardwareStatus = "sensor_ready";
#endif

#if MASTER_ENABLE_X_MOTOR_HW || MASTER_ENABLE_Y_MOTOR_HW
    bool any_motor_ready = false;
#if MASTER_ENABLE_X_MOTOR_HW
    if (masterRunModeNeedsMotorHardware(AXIS_X)) {
        MasterMotorDiagnosticsContext x_diagnostics_context = makeMasterXDiagnosticsContext();
        MasterMotorAxisInitContext x_axis = {
            "X",
            xKnobMotor,
            xKnobDriver,
            xKnobCurrentSense,
            masterKnobSensor(),
            kMasterXAxis,
            kMasterXMotorFoc,
            board_pins_master::MOTOR1_CURRENT_A,
            board_pins_master::MOTOR1_CURRENT_B,
            kMasterXCurrentSenseAxis.gain_sign_a,
            kMasterXCurrentSenseAxis.gain_sign_b,
            xKnobMotorReady,
            setMasterKnobMotorReadyForEncoder,
            x_diagnostics_context,
            {
                "driver_init",
                "driver_init_failed",
                "driver_ready",
                "current_sense_init",
                "current_sense_init_failed",
                "current_sense_ready",
                "current_sense_skipped",
                "motor_init",
                "init_foc",
                "init_foc_failed",
            },
        };
        if (!initMasterMotorAxis(x_axis)) {
            return false;
        }
        any_motor_ready = true;
    }
#endif

#if MASTER_ENABLE_Y_MOTOR_HW
    if (masterRunModeNeedsMotorHardware(AXIS_Y)) {
        MasterMotorDiagnosticsContext y_diagnostics_context = makeMasterYDiagnosticsContext();
        MasterMotorAxisInitContext y_axis = {
            "Y",
            yKnobMotor,
            yKnobDriver,
            yKnobCurrentSense,
            masterYKnobSensor(),
            kMasterYAxis,
            kMasterYMotorFoc,
            board_pins_master::MOTOR2_CURRENT_A,
            board_pins_master::MOTOR2_CURRENT_B,
            kMasterYCurrentSenseAxis.gain_sign_a,
            kMasterYCurrentSenseAxis.gain_sign_b,
            yKnobMotorReady,
            setMasterYKnobMotorReadyForEncoder,
            y_diagnostics_context,
            {
                "y_driver_init",
                "y_driver_init_failed",
                "y_driver_ready",
                "y_current_sense_init",
                "y_current_sense_init_failed",
                "y_current_sense_ready",
                "y_current_sense_skipped",
                "y_motor_init",
                "y_init_foc",
                "y_init_foc_failed",
            },
        };
        if (!initMasterMotorAxis(y_axis)) {
            return false;
        }
        any_motor_ready = true;
    }
#endif

    if (!any_motor_ready) {
        knobHardwareStatus = "motor_not_required";
        addLocalFault(FAULT_MOTOR_OUTPUT_DISABLED);
        return false;
    }
    knobHardwareStatus = "ready";
    return true;
#else
    knobHardwareStatus = "motor_compiled_off";
    addLocalFault(FAULT_MOTOR_OUTPUT_DISABLED);
    return false;
#endif
#else
    addLocalFault(FAULT_MOTOR_OUTPUT_DISABLED);
    return false;
#endif
}

// 返回当前硬件初始化状态字符串，用于启动阶段和状态行诊断。
const char *getMasterMotorHardwareStatus() {
#if MASTER_ENABLE_X_ENCODER_HW || MASTER_ENABLE_Y_ENCODER_HW || MASTER_ENABLE_X_MOTOR_HW || MASTER_ENABLE_Y_MOTOR_HW
    return knobHardwareStatus;
#else
    return "compiled_off";
#endif
}

// 清除 SimpleFOC q/d 电流环 PID 状态，避免模式切换或安全切断后的残余输出。
void resetMasterMotorCurrentPid() {
#if MASTER_ENABLE_X_MOTOR_HW
    if (xKnobMotorReady) {
        xKnobMotor.PID_current_q.reset();
        xKnobMotor.PID_current_d.reset();
    }
#endif
#if MASTER_ENABLE_Y_MOTOR_HW
    if (yKnobMotorReady) {
        yKnobMotor.PID_current_q.reset();
        yKnobMotor.PID_current_d.reset();
    }
#endif
}

// 控制热路径硬件输出：
// X/Y 目标电流 -> 每轴限幅 -> 低频目标更新 -> 每 tick motor.loopFOC() -> 低频诊断快照。
// 该函数运行在控制任务内，禁止串口、无线、动态内存和阻塞等待。
void disableMasterMotorOutputsForAdcFault() {
#if MASTER_ENABLE_X_MOTOR_HW || MASTER_ENABLE_Y_MOTOR_HW
    bool disabled_any = false;
#if MASTER_ENABLE_X_MOTOR_HW
    if (xKnobMotorReady) {
        xKnobMotor.target = 0.0f;
        xKnobMotor.current_sp = 0.0f;
        xKnobMotor.PID_current_q.reset();
        xKnobMotor.PID_current_d.reset();
        xKnobDriver.disable();
        xKnobMotorReady = false;
        setMasterKnobMotorReadyForEncoder(false);
        disabled_any = true;
    }
#endif
#if MASTER_ENABLE_Y_MOTOR_HW
    if (yKnobMotorReady) {
        yKnobMotor.target = 0.0f;
        yKnobMotor.current_sp = 0.0f;
        yKnobMotor.PID_current_q.reset();
        yKnobMotor.PID_current_d.reset();
        yKnobDriver.disable();
        yKnobMotorReady = false;
        setMasterYKnobMotorReadyForEncoder(false);
        disabled_any = true;
    }
#endif
    if (disabled_any) {
        knobHardwareStatus = "adc_dma_fault";
    }
    addLocalFault(FAULT_MOTOR_OUTPUT_DISABLED);
#else
    addLocalFault(FAULT_MOTOR_OUTPUT_DISABLED);
#endif
}

void runMasterMotorOutput(float x_target_current_a,
                          float y_target_current_a,
                          bool update_motion_target) {
#if MASTER_ENABLE_X_MOTOR_HW || MASTER_ENABLE_Y_MOTOR_HW
#if MASTER_TIMING_DETAIL_DIAG_ENABLED
    const uint32_t motor_start_us = micros();
#endif
#if MASTER_CONTROL_STATUS_PUBLISH_DIV > 1
    static uint16_t publish_div = 0;
#endif
#if MASTER_TIMING_DETAIL_DIAG_ENABLED
    uint32_t move_total_us = 0;
    uint32_t loop_foc_total_us = 0;
    uint32_t sensor_total_us = 0;
#endif

#if MASTER_ENABLE_X_MOTOR_HW
    if (xKnobMotorReady) {
        // 硬件输出层再次限幅，防止上层算法异常时越过配置电流上限。
        const float safe_current_a = clampMasterAxisCurrent(x_target_current_a, kMasterXAxis);
#if MASTER_TIMING_DETAIL_DIAG_ENABLED
        const uint32_t move_start_us = micros();
        uint32_t after_move_us = move_start_us;
#endif
        if (update_motion_target) {
#if MASTER_DIRECT_CURRENT_SETPOINT_ENABLED
            xKnobMotor.target = safe_current_a;
            xKnobMotor.current_sp = safe_current_a;
#else
            xKnobMotor.move(safe_current_a);
#endif
        }
#if MASTER_TIMING_DETAIL_DIAG_ENABLED
        if (update_motion_target) {
            after_move_us = micros();
        }
#endif
        xKnobMotor.loopFOC();
        if (xKnobCurrentSense.readFaulted()) {
            disableMasterMotorOutputsForAdcFault();
        }
#if MASTER_TIMING_DETAIL_DIAG_ENABLED
        const uint32_t after_loop_foc_us = micros();
        move_total_us += after_move_us - move_start_us;
        loop_foc_total_us += after_loop_foc_us - after_move_us;
        sensor_total_us += masterKnobSensor().lastReadDurationUs();
#endif
    }
#else
    (void)x_target_current_a;
#endif

#if MASTER_ENABLE_Y_MOTOR_HW
    if (yKnobMotorReady) {
        const float safe_current_a = clampMasterAxisCurrent(y_target_current_a, kMasterYAxis);
#if MASTER_TIMING_DETAIL_DIAG_ENABLED
        const uint32_t move_start_us = micros();
        uint32_t after_move_us = move_start_us;
#endif
        if (update_motion_target) {
#if MASTER_DIRECT_CURRENT_SETPOINT_ENABLED
            yKnobMotor.target = safe_current_a;
            yKnobMotor.current_sp = safe_current_a;
#else
            yKnobMotor.move(safe_current_a);
#endif
        }
#if MASTER_TIMING_DETAIL_DIAG_ENABLED
        if (update_motion_target) {
            after_move_us = micros();
        }
#endif
        yKnobMotor.loopFOC();
        if (yKnobCurrentSense.readFaulted()) {
            disableMasterMotorOutputsForAdcFault();
        }
#if MASTER_TIMING_DETAIL_DIAG_ENABLED
        const uint32_t after_loop_foc_us = micros();
        move_total_us += after_move_us - move_start_us;
        loop_foc_total_us += after_loop_foc_us - after_move_us;
        sensor_total_us += masterYKnobSensor().lastReadDurationUs();
#endif
    }
#else
    (void)y_target_current_a;
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
#if MASTER_ENABLE_X_MOTOR_HW
        if (xKnobMotorReady) {
            sysData.master.current_q_a = xKnobMotor.current.q;
            sysData.master.current_d_a = xKnobMotor.current.d;
            sysData.master.voltage_q_v = xKnobMotor.voltage.q;
            sysData.master.voltage_d_v = xKnobMotor.voltage.d;
        }
#endif
#if MASTER_ENABLE_Y_MOTOR_HW
        if (yKnobMotorReady) {
            sysData.master.y_current_q_a = yKnobMotor.current.q;
            sysData.master.y_current_d_a = yKnobMotor.current.d;
            sysData.master.y_voltage_q_v = yKnobMotor.voltage.q;
            sysData.master.y_voltage_d_v = yKnobMotor.voltage.d;
        }
#endif
    }
#if MASTER_TIMING_DETAIL_DIAG_ENABLED
    recordMasterTimingMotorUs(micros() - motor_start_us,
                              move_total_us,
                              loop_foc_total_us,
                              sensor_total_us);
#endif
#else
    (void)x_target_current_a;
    (void)y_target_current_a;
    (void)update_motion_target;
#endif
}
