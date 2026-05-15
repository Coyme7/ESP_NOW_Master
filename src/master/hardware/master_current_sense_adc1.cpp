#include "master/hardware/master_current_sense_adc1.h"

#include "current_sense/hardware_specific/esp32/esp32_adc_driver.h"
#include "master/config/master_config.h"

#if MASTER_CONTROL_TIMING_DIAG_ENABLED
extern "C" void recordMasterTimingCurrentSenseUs(uint32_t duration_us) __attribute__((weak));
#endif

// 构造函数只保存引脚和换算系数，不做硬件初始化，避免全局对象阶段访问外设。
MasterAdc1CurrentSense::MasterAdc1CurrentSense(float shunt_resistor,
                                               float amp_gain,
                                               int pinA,
                                               int pinB,
                                               int pinC)
    : InlineCurrentSense(shunt_resistor, amp_gain, pinA, pinB, pinC),
      pin_a_(pinA),
      pin_b_(pinB),
      has_phase_c_(pinC != NOT_SET),
      raw_to_voltage_v_(MASTER_CURRENT_SENSE_ADC_RAW_TO_VOLTAGE_V) {
    offset_ia = 0.0f;
    offset_ib = 0.0f;
    offset_ic = 0.0f;
}

// ESP32-S3 ADC1 GPIO 到通道号的映射检查；不支持的 GPIO 直接初始化失败。
bool MasterAdc1CurrentSense::gpioToAdc1Channel(int pin, uint8_t &channel) {
    switch (pin) {
    case 1:
        channel = 0;
        return true;
    case 2:
        channel = 1;
        return true;
    case 3:
        channel = 2;
        return true;
    case 4:
        channel = 3;
        return true;
    case 5:
        channel = 4;
        return true;
    case 6:
        channel = 5;
        return true;
    case 7:
        channel = 6;
        return true;
    case 8:
        channel = 7;
        return true;
    case 9:
        channel = 8;
        return true;
    case 10:
        channel = 9;
        return true;
    default:
        return false;
    }
}

// 热路径原始 ADC 读取封装，保持单次 adcRead，不加锁、不打印。
int MasterAdc1CurrentSense::readFastRaw(int pin) {
    return static_cast<int>(adcRead(static_cast<uint8_t>(pin)));
}

// 初始化 ADC 采样对象：当前只支持 A/B 两相高侧采样。
int MasterAdc1CurrentSense::init() {
    if (has_phase_c_) {
        initialized = false;
        return 0;
    }

    if (!gpioToAdc1Channel(pin_a_, chan_a_) || !gpioToAdc1Channel(pin_b_, chan_b_)) {
        initialized = false;
        return 0;
    }

    pinMode(pin_a_, INPUT);
    pinMode(pin_b_, INPUT);
    (void)readFastRaw(pin_a_);
    (void)readFastRaw(pin_b_);

    initialized = true;
    return 1;
}

// 跳过 SimpleFOC 自动相位对齐，采样方向由手动诊断参数决定。
int MasterAdc1CurrentSense::driverAlign(float align_voltage) {
    (void)align_voltage;
    return 1;
}

// SimpleFOC 每次 loopFOC 会调用这里读取相电流，是控制热路径的一部分。
PhaseCurrent_s MasterAdc1CurrentSense::getPhaseCurrents() {
#if MASTER_CONTROL_TIMING_DIAG_ENABLED
    const uint32_t read_start_us = micros();
#endif
    // 热路径只做两次 ADC 原始采样和线性换算，不做 offset 重算或诊断打印。
    const int raw_a = readFastRaw(pin_a_);
    const int raw_b = readFastRaw(pin_b_);
    const float voltage_a = static_cast<float>(raw_a) * raw_to_voltage_v_;
    const float voltage_b = static_cast<float>(raw_b) * raw_to_voltage_v_;

    PhaseCurrent_s current;
    current.a = (voltage_a - offset_ia) * gain_a;
    current.b = (voltage_b - offset_ib) * gain_b;
    current.c = 0.0f;
#if MASTER_CONTROL_TIMING_DIAG_ENABLED
    if (recordMasterTimingCurrentSenseUs) {
        recordMasterTimingCurrentSenseUs(micros() - read_start_us);
    }
#endif
    return current;
}

// 诊断读取 A 相原始 ADC 值。
int MasterAdc1CurrentSense::readRawA() const {
    if (!initialized) {
        return 0;
    }
    return readFastRaw(pin_a_);
}

// 诊断读取 B 相原始 ADC 值。
int MasterAdc1CurrentSense::readRawB() const {
    if (!initialized) {
        return 0;
    }
    return readFastRaw(pin_b_);
}
