#pragma once

#include <Arduino.h>
#include <SimpleFOC.h>

#include "master/config/types/master_current_sense_types.h"
#include "master/hardware/master_adc1_dma_sampler.h"

class MasterAdc1CurrentSense : public InlineCurrentSense {
public:
    MasterAdc1CurrentSense(float shunt_resistor,
                           float amp_gain,
                           int pinA,
                           int pinB,
                           int pinC = NOT_SET);

    int init() override;
    int driverAlign(float align_voltage, bool modulation_centered = false) override;
    PhaseCurrent_s getPhaseCurrents() override;

    int readRawA() const;
    int readRawB() const;
    bool waitNextRawPair(uint32_t &last_sequence,
                         uint32_t timeout_ms,
                         int &raw_a,
                         int &raw_b) const;
    uint32_t readErrorCount() const;
    uint16_t consecutiveReadErrors() const;
    bool readFaulted() const;

private:
    int pin_a_ = NOT_SET;
    int pin_b_ = NOT_SET;
    bool has_phase_c_ = false;
    MasterAdc1DmaSlot slot_a_ = MASTER_ADC1_DMA_SLOT_X_A;
    MasterAdc1DmaSlot slot_b_ = MASTER_ADC1_DMA_SLOT_X_B;
    float raw_to_voltage_v_ = 0.0f;
    mutable int last_raw_a_ = 0;
    mutable int last_raw_b_ = 0;
};
