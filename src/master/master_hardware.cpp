#include "master/master_hardware.h"

#include <Arduino.h>
#include <board/board_pins_master.h>

#include "common/mt6701_ssi_sensor.h"
#include "common/system_state.h"
#include "master/master_config.h"

#if MASTER_MOTOR_HW_ENABLED
#include <SimpleFOC.h>
#endif

// 主机硬件适配层。
// 这里集中处理 GPIO、SimpleFOC 对象和按钮输入，控制算法不直接碰板级引脚。
// 这样后续换驱动板或临时关闭硬件时，只需要收敛在本文件和 master_config。

namespace {

// 临时正交编码器计数。仅在 MASTER_DEMO_QUADRATURE_ENABLED=1 时由中断更新。
// 默认真实方案是 MT6701 + SimpleFOC，因此该变量通常保持 0。
volatile int32_t encoder_count = 0;

#if MASTER_MOTOR_HW_ENABLED
bool knobMotorReady = false;
bool knobSensorReady = false;
const char *knobHardwareStatus = "not_started";

#ifndef MASTER_MOTOR_POLE_PAIRS
// 主机 2804 旋钮电机极对数，属于硬件属性；当前电机为 7 极对。
#define MASTER_MOTOR_POLE_PAIRS 7
#endif

BLDCMotor knobMotor = BLDCMotor(MASTER_MOTOR_POLE_PAIRS);
BLDCDriver3PWM knobDriver = BLDCDriver3PWM(
    board_pins_master::MOTOR1_PWM_U,
    board_pins_master::MOTOR1_PWM_V,
    board_pins_master::MOTOR1_PWM_W,
    board_pins_master::MOTOR1_EN);
Mt6701SsiSensor knobSensor = Mt6701SsiSensor(board_pins_master::ENCODER1_CS);

InlineCurrentSense knobCurrentSense = InlineCurrentSense(
    MASTER_CURRENT_SHUNT_OHM,
    MASTER_CURRENT_SENSE_GAIN,
    board_pins_master::MOTOR1_CURRENT_A,
    board_pins_master::MOTOR1_CURRENT_B);

int masterDriverDisabledLevel() {
    return MASTER_DRIVER_ENABLE_ACTIVE_HIGH ? LOW : HIGH;
}

#if MASTER_MOTOR_PHASE_PROBE_ENABLED
float readKnobMechanicalAngleDeg() {
    knobSensor.update();
    return radToDeg(knobSensor.getMechanicalAngle());
}

void runMasterPhaseProbe() {
    const float probe_voltage_v = clampFloat(kMasterMotorFoc.align_voltage_v,
                                             0.0f,
                                             kMasterMotorFoc.voltage_limit_v);
    const float start_angle_deg = readKnobMechanicalAngleDeg();
    Serial.printf("[Master] motor_probe start voltage=%.2fV raw=%u angle=%.2fdeg\n",
                  probe_voltage_v,
                  static_cast<unsigned int>(knobSensor.rawAngle()),
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
        knobMotor.setPhaseVoltage(probe_voltage_v, 0.0f, probe_angles_rad[i]);
        delay(180);
        const float angle_deg = readKnobMechanicalAngleDeg();
        const float delta_deg = absoluteAngleDegToSignedAngleDeg(angle_deg, start_angle_deg);
        Serial.printf("[Master] motor_probe step=%u elec=%.1fdeg raw=%u angle=%.2fdeg delta=%.2fdeg\n",
                      static_cast<unsigned int>(i),
                      radToDeg(probe_angles_rad[i]),
                      static_cast<unsigned int>(knobSensor.rawAngle()),
                      angle_deg,
                      delta_deg);
    }

    knobMotor.setPhaseVoltage(0.0f, 0.0f, 0.0f);
    delay(100);
}
#endif

#if MASTER_MOTOR_HW_ENABLED && MASTER_USE_CURRENT_SENSE
PhaseCurrent_s sampleMasterCurrentSenseVoltagesOnce() {
    const float saved_offset_ia = knobCurrentSense.offset_ia;
    const float saved_offset_ib = knobCurrentSense.offset_ib;
    const float saved_offset_ic = knobCurrentSense.offset_ic;
    const float saved_gain_a = knobCurrentSense.gain_a;
    const float saved_gain_b = knobCurrentSense.gain_b;
    const float saved_gain_c = knobCurrentSense.gain_c;

    knobCurrentSense.offset_ia = 0.0f;
    knobCurrentSense.offset_ib = 0.0f;
    knobCurrentSense.offset_ic = 0.0f;
    knobCurrentSense.gain_a = 1.0f;
    knobCurrentSense.gain_b = 1.0f;
    knobCurrentSense.gain_c = 1.0f;

    const PhaseCurrent_s sample = knobCurrentSense.getPhaseCurrents();

    knobCurrentSense.offset_ia = saved_offset_ia;
    knobCurrentSense.offset_ib = saved_offset_ib;
    knobCurrentSense.offset_ic = saved_offset_ic;
    knobCurrentSense.gain_a = saved_gain_a;
    knobCurrentSense.gain_b = saved_gain_b;
    knobCurrentSense.gain_c = saved_gain_c;

    return sample;
}

void primeMasterCurrentSenseAdc() {
    for (uint8_t i = 0; i < MASTER_CURRENT_SENSE_ADC_PRIME_READS; ++i) {
        (void)analogRead(board_pins_master::MOTOR1_CURRENT_A);
        (void)analogRead(board_pins_master::MOTOR1_CURRENT_B);
    }
}

void calibrateMasterCurrentSenseOffsets() {
    const float saved_gain_a = knobCurrentSense.gain_a;
    const float saved_gain_b = knobCurrentSense.gain_b;
    const float saved_gain_c = knobCurrentSense.gain_c;
    const float saved_offset_ic = knobCurrentSense.offset_ic;
    const uint32_t calibration_reads =
        (MASTER_CURRENT_SENSE_OFFSET_CALIBRATION_READS > 0)
            ? MASTER_CURRENT_SENSE_OFFSET_CALIBRATION_READS
            : 1UL;

    // DengFoc 在 EN/PWM 活动后 ADC 零点会漂到运行态偏置，offset 必须在同状态下测。
    knobDriver.enable();
    knobDriver.setPwm(0.0f, 0.0f, 0.0f);
    delay(MASTER_CURRENT_SENSE_OFFSET_SETTLE_MS);
    primeMasterCurrentSenseAdc();

    knobCurrentSense.offset_ia = 0.0f;
    knobCurrentSense.offset_ib = 0.0f;
    knobCurrentSense.offset_ic = 0.0f;
    knobCurrentSense.gain_a = 1.0f;
    knobCurrentSense.gain_b = 1.0f;
    knobCurrentSense.gain_c = 1.0f;

    float sum_a_v = 0.0f;
    float sum_b_v = 0.0f;
    for (uint32_t i = 0; i < calibration_reads; ++i) {
        const PhaseCurrent_s voltage = sampleMasterCurrentSenseVoltagesOnce();
        sum_a_v += voltage.a;
        sum_b_v += voltage.b;
        delay(1);
    }

    knobCurrentSense.offset_ia = sum_a_v / static_cast<float>(calibration_reads);
    knobCurrentSense.offset_ib = sum_b_v / static_cast<float>(calibration_reads);
    knobCurrentSense.offset_ic = saved_offset_ic;
    knobCurrentSense.gain_a = saved_gain_a;
    knobCurrentSense.gain_b = saved_gain_b;
    knobCurrentSense.gain_c = saved_gain_c;

    const int runtime_raw_adc_a = analogRead(board_pins_master::MOTOR1_CURRENT_A);
    const int runtime_raw_adc_b = analogRead(board_pins_master::MOTOR1_CURRENT_B);
    knobDriver.setPwm(0.0f, 0.0f, 0.0f);
    knobDriver.disable();

    Serial.printf("[Master] motor_diag current_sense offset_cal mode=driver_enabled_pwm0 settle=%ums samples=%lu ia=%.4fV ib=%.4fV raw_adc=%d,%d\n",
                  static_cast<unsigned int>(MASTER_CURRENT_SENSE_OFFSET_SETTLE_MS),
                  static_cast<unsigned long>(calibration_reads),
                  knobCurrentSense.offset_ia,
                  knobCurrentSense.offset_ib,
                  runtime_raw_adc_a,
                  runtime_raw_adc_b);
}

#if MASTER_CURRENT_SENSE_DIAG_ONLY
void logMasterCurrentProbeSample(const char *label,
                                 const char *stage,
                                 float phase_u_v,
                                 float phase_v_v,
                                 float phase_w_v) {
    const PhaseCurrent_s current = knobCurrentSense.getPhaseCurrents();
    const PhaseCurrent_s voltage = sampleMasterCurrentSenseVoltagesOnce();
    const int raw_adc_a = analogRead(board_pins_master::MOTOR1_CURRENT_A);
    const int raw_adc_b = analogRead(board_pins_master::MOTOR1_CURRENT_B);
    const float delta_mv_a = (voltage.a - knobCurrentSense.offset_ia) * 1000.0f;
    const float delta_mv_b = (voltage.b - knobCurrentSense.offset_ib) * 1000.0f;
    knobSensor.update();
    Serial.printf("[Master] current_probe %s %s pwm=%.2f,%.2f,%.2f va=%.4fV vb=%.4fV dmv_a=%.1f dmv_b=%.1f ia=%.4fA ib=%.4fA raw_adc=%d,%d raw=%u angle=%.2fdeg\n",
                  label,
                  stage,
                  phase_u_v,
                  phase_v_v,
                  phase_w_v,
                  voltage.a,
                  voltage.b,
                  delta_mv_a,
                  delta_mv_b,
                  current.a,
                  current.b,
                  raw_adc_a,
                  raw_adc_b,
                  static_cast<unsigned int>(knobSensor.rawAngle()),
                  radToDeg(knobSensor.getMechanicalAngle()));
}

void runMasterCurrentProbePoint(const char *label, float phase_u_v, float phase_v_v, float phase_w_v) {
    knobDriver.setPwm(phase_u_v, phase_v_v, phase_w_v);
    delay(MASTER_CURRENT_SENSE_DIAG_EARLY_MS);
    logMasterCurrentProbeSample(label, "early", phase_u_v, phase_v_v, phase_w_v);
    const uint32_t remaining_settle_ms =
        (MASTER_CURRENT_SENSE_DIAG_SETTLE_MS > MASTER_CURRENT_SENSE_DIAG_EARLY_MS)
            ? (MASTER_CURRENT_SENSE_DIAG_SETTLE_MS - MASTER_CURRENT_SENSE_DIAG_EARLY_MS)
            : 0;
    if (remaining_settle_ms > 0) {
        delay(remaining_settle_ms);
    }
    logMasterCurrentProbeSample(label, "settled", phase_u_v, phase_v_v, phase_w_v);
}

void runMasterCurrentSenseProbe() {
    const float probe_voltage_v = clampFloat(MASTER_CURRENT_SENSE_DIAG_VOLTAGE_V,
                                             0.0f,
                                             kMasterMotorFoc.voltage_limit_v);
    Serial.printf("[Master] current_probe start voltage=%.2fV settle=%ums\n",
                  probe_voltage_v,
                  static_cast<unsigned int>(MASTER_CURRENT_SENSE_DIAG_SETTLE_MS));
    knobDriver.enable();
    knobDriver.setPwm(0.0f, 0.0f, 0.0f);
    delay(MASTER_CURRENT_SENSE_DIAG_SETTLE_MS);
    logMasterCurrentProbeSample("idle0", "settled", 0.0f, 0.0f, 0.0f);
    runMasterCurrentProbePoint("phase_u", probe_voltage_v, 0.0f, 0.0f);
    runMasterCurrentProbePoint("phase_v", 0.0f, probe_voltage_v, 0.0f);
    runMasterCurrentProbePoint("phase_w", 0.0f, 0.0f, probe_voltage_v);
    knobDriver.setPwm(0.0f, 0.0f, 0.0f);
    delay(MASTER_CURRENT_SENSE_DIAG_SETTLE_MS);
    logMasterCurrentProbeSample("idle1", "settled", 0.0f, 0.0f, 0.0f);
    knobDriver.disable();
}
#endif
#endif

#endif  // MASTER_MOTOR_HW_ENABLED

#if !MASTER_MOTOR_HW_ENABLED
int masterDriverDisabledLevel() {
    return MASTER_DRIVER_ENABLE_ACTIVE_HIGH ? LOW : HIGH;
}
#endif

#if MASTER_DEMO_QUADRATURE_ENABLED
// 临时正交编码器中断入口。
// IRAM_ATTR 避免中断期间从不可用存储取代码；中断内只做计数，不打印、不分配。
void IRAM_ATTR encoderISR() {
    if (digitalRead(MASTER_DEMO_ENCODER_PIN_A) == digitalRead(MASTER_DEMO_ENCODER_PIN_B)) {
        encoder_count++;
    } else {
        encoder_count--;
    }
}
#endif

}  // namespace

void configureMasterSafeOutputs() {
    // 启动安全：任何任务启动前先强制关闭两路电机使能。
    const int disabled_level = masterDriverDisabledLevel();
    pinMode(board_pins_master::MOTOR1_EN, OUTPUT);
    pinMode(board_pins_master::MOTOR2_EN, OUTPUT);
    digitalWrite(board_pins_master::MOTOR1_EN, disabled_level);
    digitalWrite(board_pins_master::MOTOR2_EN, disabled_level);
    pinMode(MASTER_DEMO_ENCODER_PIN_A, INPUT_PULLUP);
    pinMode(MASTER_DEMO_ENCODER_PIN_B, INPUT_PULLUP);
    pinMode(board_pins_master::MAIN_BUTTON, INPUT_PULLUP);
#if MASTER_DEMO_QUADRATURE_ENABLED
    // 只有显式启用临时演示输入时才挂中断，避免默认占用未确认引脚。
    attachInterrupt(digitalPinToInterrupt(MASTER_DEMO_ENCODER_PIN_A), encoderISR, FALLING);
#endif
}

bool setupMasterMotorHardware() {
#if MASTER_MOTOR_HW_ENABLED
    // 输入：真实 MT6701 + 2804 硬件；输出：SimpleFOC 电流力矩模式就绪。
    // 故障行为：任一初始化失败都会关闭驱动并锁存 MOTOR_OUTPUT_DISABLED。
#if MASTER_SIMPLEFOC_DEBUG_ENABLED
    SimpleFOCDebug::enable(&Serial);
#endif

    knobHardwareStatus = "spi_sensor_init";
    SPI.begin(board_pins_master::ENCODER1_CLK,
              board_pins_master::ENCODER1_DO,
              -1,
              board_pins_master::ENCODER1_CS);
    knobSensor.init(&SPI);
    knobSensorReady = true;
    knobHardwareStatus = "sensor_ready";
    Serial.printf("[Master] motor_diag sensor raw=%u angle=%.2fdeg stat=0x%01x frame=0x%06lx spi_mode=%u\n",
                  static_cast<unsigned int>(knobSensor.rawAngle()),
                  radToDeg(knobSensor.getMechanicalAngle()),
                  static_cast<unsigned int>(knobSensor.magneticStatus()),
                  static_cast<unsigned long>(knobSensor.lastFrame()),
                  static_cast<unsigned int>(MT6701_SSI_SPI_MODE));

    // 驱动供电和电压限制先使用保守值。主机最终目标是电流力矩控制，
    // 但电压上限仍限制对齐和异常时的最大输出。
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
    knobDriver.disable();
    knobHardwareStatus = "driver_ready";

    knobMotor.linkSensor(&knobSensor);
    knobMotor.linkDriver(&knobDriver);

    // bring-up 阶段可先用电压力矩模式跳过电流采样对齐；
    // 确认 IA/IB、shunt/gain 和采样相位后，再切回 foc_current。
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

    // 主机力反馈只给 Iq 目标；Id 轴保持目标 0，默认不积分，避免 D 轴误差累积。
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
    // 电流采样必须绑定驱动后初始化；失败时立即禁用驱动。
    // 当前 shunt 和 gain 只是临时默认，真实上电前要按 DengFoc 板实测确认。
    knobHardwareStatus = "current_sense_init";
    knobCurrentSense.linkDriver(&knobDriver);
    knobCurrentSense.skip_align = MASTER_CURRENT_SENSE_SKIP_ALIGN != 0;
    Serial.printf("[Master] motor_diag current_sense inline shunt=%.4fohm gain=%.2f pins=%d,%d skip_align=%u\n",
                  MASTER_CURRENT_SHUNT_OHM,
                  MASTER_CURRENT_SENSE_GAIN,
                  board_pins_master::MOTOR1_CURRENT_A,
                  board_pins_master::MOTOR1_CURRENT_B,
                  MASTER_CURRENT_SENSE_SKIP_ALIGN ? 1 : 0);
    if (!knobCurrentSense.init()) {
        knobDriver.disable();
        knobHardwareStatus = "current_sense_init_failed";
        addLocalFault(FAULT_MOTOR_OUTPUT_DISABLED);
        return false;
    }
    calibrateMasterCurrentSenseOffsets();
    // 自动 driverAlign 在当前硬件上会失败，因此采样方向用 MASTER_CURRENT_SENSE_GAIN_SIGN_A/B
    // 手动指定。若符号错，foc_current 会把电流误差放大成正反馈，上电前必须用诊断确认。
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
    runMasterCurrentSenseProbe();
    Serial.println("[Master] motor_diag current_sense diag_only; keep motor disabled");
    addLocalFault(FAULT_MOTOR_OUTPUT_DISABLED);
    return false;
#endif
#else
    knobHardwareStatus = "current_sense_skipped";
    Serial.println("[Master] motor_diag current_sense skipped for voltage torque bring-up");
#endif

    // initFOC 会做传感器/相序对齐。失败时保持输出禁用，不让后续控制步移动电机。
    knobHardwareStatus = "motor_init";
    knobMotor.init();

#if MASTER_MOTOR_PHASE_PROBE_ENABLED
    runMasterPhaseProbe();
#endif
    knobHardwareStatus = "init_foc";
    if (!knobMotor.initFOC()) {
        knobDriver.disable();
        knobHardwareStatus = "init_foc_failed";
        Serial.println("[Master] motor_diag initFOC failed; keep motor disabled");
        addLocalFault(FAULT_MOTOR_OUTPUT_DISABLED);
        return false;
    }
    knobMotorReady = true;
    knobHardwareStatus = "ready";
    return true;
#else
    addLocalFault(FAULT_MOTOR_OUTPUT_DISABLED);
    return false;
#endif
}

const char *getMasterMotorHardwareStatus() {
#if MASTER_MOTOR_HW_ENABLED
    return knobHardwareStatus;
#else
    return "compiled_off";
#endif
}

void getMasterEncoderDiagnostics(uint32_t &frame, uint16_t &raw_angle, uint8_t &magnetic_status) {
#if MASTER_MOTOR_HW_ENABLED
    frame = knobSensor.lastFrame();
    raw_angle = knobSensor.rawAngle();
    magnetic_status = knobSensor.magneticStatus();
#else
    frame = 0;
    raw_angle = 0;
    magnetic_status = 0;
#endif
}

float readMasterKnobAngleDeg() {
#if MASTER_MOTOR_HW_ENABLED
    // 真实硬件路径：优先读 MT6701 机械角。即使 initFOC 失败，也允许串口显示编码器角度，
    // 方便区分“编码器没通”和“电流采样/FOC 对齐失败”。只有输出电流时才要求 motorReady。
    if (!knobSensorReady) {
        return 0.0f;
    }
    if (!knobMotorReady) {
        // SimpleFOC 的 Sensor::getAngle() 返回的是 update() 缓存值。
        // 当驱动、电流采样或 initFOC 失败时，runMasterMotorOutput() 不会调用 loopFOC()，
        // 也就不会间接刷新传感器。这里主动 update，让“只测编码器”的主机固件仍能
        // 持续刷新 angle/raw/frame，避免独立 MT6701 测试通过但主机状态停在启动值。
        knobSensor.update();
    }
    const float absolute_angle_deg = radToDeg(knobSensor.getMechanicalAngle());
    const float signed_angle_deg =
        absoluteAngleDegToSignedAngleDeg(absolute_angle_deg, kMasterXAxis.center_deg);
    return static_cast<float>(MASTER_KNOB_AXIS_SIGN) * signed_angle_deg;
#elif MASTER_DEMO_QUADRATURE_ENABLED
    // 临时演示路径：每个计数粗略当作 1 deg，只用于无 MT6701 时跑通数据流。
    return static_cast<float>(MASTER_KNOB_AXIS_SIGN) * static_cast<float>(encoder_count);
#else
    // 默认安全路径：没有真实输入时固定中心，主从通信仍能验证。
    return 0.0f;
#endif
}

bool readMasterPenButtonDown() {
    // 按钮使用 INPUT_PULLUP，按下时读到 LOW。该值会进入 MasterCommandPacket.pen_down。
    return digitalRead(board_pins_master::MAIN_BUTTON) == LOW;
}

void resetMasterMotorCurrentPid() {
#if MASTER_MOTOR_HW_ENABLED
    if (!knobMotorReady) {
        return;
    }
    knobMotor.PID_current_q.reset();
    knobMotor.PID_current_d.reset();
#endif
}

void runMasterMotorOutput(float target_current_a) {
#if MASTER_MOTOR_HW_ENABLED
    // 控制热路径中的真实输出：先写入本周期力矩目标，再更新 FOC。
    // SimpleFOC 的 current/foc_current 模式在 loopFOC() 中消费 current_sp；
    // 先 move(target) 可避免虚拟墙/阻尼目标落后一整个控制子步。
    // 注意这里不做 Serial 和无线操作，避免扰动 10 kHz 节拍。
    if (!knobMotorReady) {
        (void)target_current_a;
        return;
    }
    const float safe_current_a = clampFloat(target_current_a,
                                           -kMasterXAxis.haptic_current_limit_a,
                                           kMasterXAxis.haptic_current_limit_a);
    knobMotor.move(safe_current_a);
    knobMotor.loopFOC();
    sysData.master_current_q_a = knobMotor.current.q;
    sysData.master_current_d_a = knobMotor.current.d;
    sysData.master_voltage_q_v = knobMotor.voltage.q;
    sysData.master_voltage_d_v = knobMotor.voltage.d;
#else
    // 硬件关闭时保留参数消费，避免编译警告；算法和状态更新仍保持可测试。
    (void)target_current_a;
#endif
}
