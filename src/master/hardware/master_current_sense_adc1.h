#pragma once

#include <Arduino.h>
#include <SimpleFOC.h>

// 电流采样适配类：用 ESP32-S3 ADC1 两相采样对接 SimpleFOC InlineCurrentSense 接口。
class MasterAdc1CurrentSense : public InlineCurrentSense {
public:
    MasterAdc1CurrentSense(float shunt_resistor,
                           float amp_gain,
                           int pinA,
                           int pinB,
                           int pinC = NOT_SET);

    int init() override;
    int driverAlign(float align_voltage) override;
    PhaseCurrent_s getPhaseCurrents() override;

    int readRawA() const;
    int readRawB() const;

private:
    static bool gpioToAdc1Channel(int pin, uint8_t &channel);
    static int readFastRaw(int pin);

    int pin_a_ = NOT_SET;
    int pin_b_ = NOT_SET;
    bool has_phase_c_ = false;
    uint8_t chan_a_ = 0;
    uint8_t chan_b_ = 0;
    float raw_to_voltage_v_ = 0.0f;
};
