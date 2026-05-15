#include "master/hardware/master_encoder_hw.h"

#include <Arduino.h>
#include <SPI.h>
#include <board/board_pins_master.h>

#include "common/math/angle_math.h"
#include "master/config/master_config.h"
#include "master/hardware/master_board_io.h"

MasterMt6701Sensor::MasterMt6701Sensor(int cs_pin) : reader_(cs_pin) {}

// 初始化底层 MT6701 快速读取器。
void MasterMt6701Sensor::init(SPIClass *spi) {
    reader_.init(spi);
    Sensor::init();
}

// 返回最近一帧 SSI 原始数据，用于状态打印。
uint32_t MasterMt6701Sensor::lastFrame() const {
    return reader_.lastFrame();
}

// 返回最近一次 MT6701 14-bit 原始角。
uint16_t MasterMt6701Sensor::rawAngle() const {
    return reader_.rawAngle();
}

// 返回最近一次磁场状态。
uint8_t MasterMt6701Sensor::magneticStatus() const {
    return reader_.magneticStatus();
}

// 返回最近一次 SSI 读取耗时，用于确认重构后没有拖慢热路径。
uint32_t MasterMt6701Sensor::lastReadDurationUs() const {
    return reader_.lastReadDurationUs();
}

// SimpleFOC Sensor 接口：返回弧度角，供 loopFOC 读取电角度。
float MasterMt6701Sensor::getSensorAngle() {
    const Mt6701SsiFrame frame = reader_.readFrame();
    return static_cast<float>(frame.raw_angle) * (_2PI / 16384.0f);
}

namespace {

MasterMt6701Sensor knobSensor(board_pins_master::ENCODER1_CS);
bool knobSensorReady = false;
bool knobMotorReadyForEncoder = false;

}  // namespace

// 初始化 SPI 总线和主机旋钮 MT6701。
bool setupMasterEncoderHardware() {
    SPI.begin(board_pins_master::ENCODER1_CLK,
              board_pins_master::ENCODER1_DO,
              -1,
              board_pins_master::ENCODER1_CS);
    knobSensor.init(&SPI);
    knobSensorReady = true;
    Serial.printf("[Master] motor_diag sensor raw=%u angle=%.2fdeg stat=0x%01x frame=0x%06lx spi_mode=%u\n",
                  static_cast<unsigned int>(knobSensor.rawAngle()),
                  radToDeg(knobSensor.getMechanicalAngle()),
                  static_cast<unsigned int>(knobSensor.magneticStatus()),
                  static_cast<unsigned long>(knobSensor.lastFrame()),
                  static_cast<unsigned int>(MT6701_SSI_SPI_MODE));
    return true;
}

// 标记电机是否完成 initFOC；编码器可据此决定诊断状态。
void setMasterKnobMotorReadyForEncoder(bool ready) {
    knobMotorReadyForEncoder = ready;
}

// 返回全局主机旋钮 Sensor 对象，供 SimpleFOC 链接。
MasterMt6701Sensor &masterKnobSensor() {
    return knobSensor;
}

// 获取编码器诊断快照：原始帧、原始角和磁场状态。
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

// 控制层读取入口：把编码器绝对角转成主机控制角度。
float readMasterKnobAngleDeg() {
#if MASTER_MOTOR_HW_ENABLED
    if (!knobSensorReady) {
        return 0.0f;
    }
    if (!knobMotorReadyForEncoder) {
        // initFOC 失败或硬件未 ready 时主动刷新，便于独立观察编码器。
        // 电机 ready 后不在这里额外读 SSI，保持原热路径读数开销。
        knobSensor.update();
    }
    const float absolute_angle_deg = radToDeg(knobSensor.getMechanicalAngle());
    const float signed_angle_deg =
        absoluteAngleDegToSignedAngleDeg(absolute_angle_deg, kMasterXAxis.center_deg);
    return static_cast<float>(MASTER_KNOB_AXIS_SIGN) * signed_angle_deg;
#elif MASTER_DEMO_QUADRATURE_ENABLED
    return static_cast<float>(MASTER_KNOB_AXIS_SIGN) *
           static_cast<float>(readMasterDemoEncoderCount());
#else
    return 0.0f;
#endif
}
