#include "master/hardware/master_current_sense_adc1.h"

#include <Arduino.h>
#include <algorithm>

#include "master/config/master_config.h"
#include "master/hardware/master_adc1_dma_sampler.h"

extern "C" void recordMasterTimingCurrentSenseUs(uint32_t duration_us);

MasterAdc1CurrentSense::MasterAdc1CurrentSense(float shunt_resistor,
                                               float amp_gain,
                                               int pinA,
                                               int pinB,
                                               int pinC)
    : InlineCurrentSense(shunt_resistor, amp_gain, pinA, pinB, pinC),
      pin_a_(pinA),
      pin_b_(pinB),
      has_phase_c_(pinC != NOT_SET),
      raw_to_voltage_v_(kMasterCurrentSenseHardware.adc_raw_to_voltage_v) {
    offset_ia = 0.0f;
    offset_ib = 0.0f;
    offset_ic = 0.0f;
}

int MasterAdc1CurrentSense::init() {
    if (has_phase_c_ || !masterAdc1DmaSamplerRequired()) {
        initialized = false;
        return 0;
    }

    if (!masterAdc1DmaSlotForPin(pin_a_, slot_a_) ||
        !masterAdc1DmaSlotForPin(pin_b_, slot_b_)) {
        initialized = false;
        return 0;
    }

    int raw_a = 0;
    int raw_b = 0;
    if (!masterAdc1DmaReadLatestRaw(slot_a_, raw_a) ||
        !masterAdc1DmaReadLatestRaw(slot_b_, raw_b)) {
        initialized = false;
        return 0;
    }

    last_raw_a_ = raw_a;
    last_raw_b_ = raw_b;
    initialized = true;
    return 1;
}

int MasterAdc1CurrentSense::driverAlign(float align_voltage, bool modulation_centered) {
    (void)align_voltage;
    (void)modulation_centered;
    return 1;
}

PhaseCurrent_s MasterAdc1CurrentSense::getPhaseCurrents() {
#if MASTER_TIMING_DETAIL_DIAG_ENABLED
    const uint32_t sample_start_us = micros();
#endif
    int raw_a = last_raw_a_;
    int raw_b = last_raw_b_;
    int sampled_a = 0;
    int sampled_b = 0;
    if (masterAdc1DmaReadControlRaw(slot_a_, sampled_a) &&
        masterAdc1DmaReadControlRaw(slot_b_, sampled_b)) {
        raw_a = sampled_a;
        raw_b = sampled_b;
        last_raw_a_ = raw_a;
        last_raw_b_ = raw_b;
    }

    PhaseCurrent_s current = {};
    current.a = (static_cast<float>(raw_a) * raw_to_voltage_v_ - offset_ia) * gain_a;
    current.b = (static_cast<float>(raw_b) * raw_to_voltage_v_ - offset_ib) * gain_b;
    current.c = 0.0f;
#if MASTER_TIMING_DETAIL_DIAG_ENABLED
    recordMasterTimingCurrentSenseUs(micros() - sample_start_us);
#endif
    return current;
}

int MasterAdc1CurrentSense::readRawA() const {
    if (!initialized) {
        return 0;
    }
    int raw = last_raw_a_;
    if (masterAdc1DmaReadLatestRaw(slot_a_, raw)) {
        last_raw_a_ = raw;
    }
    return raw;
}

int MasterAdc1CurrentSense::readRawB() const {
    if (!initialized) {
        return 0;
    }
    int raw = last_raw_b_;
    if (masterAdc1DmaReadLatestRaw(slot_b_, raw)) {
        last_raw_b_ = raw;
    }
    return raw;
}

bool MasterAdc1CurrentSense::waitNextRawPair(uint32_t &last_sequence,
                                             uint32_t timeout_ms,
                                             int &raw_a,
                                             int &raw_b) const {
    if (!initialized) {
        return false;
    }
    if (!waitForMasterAdc1DmaRawPair(slot_a_,
                                     slot_b_,
                                     last_sequence,
                                     timeout_ms,
                                     raw_a,
                                     raw_b)) {
        return false;
    }
    last_raw_a_ = raw_a;
    last_raw_b_ = raw_b;
    return true;
}

uint32_t MasterAdc1CurrentSense::readErrorCount() const {
    const MasterAdc1DmaHealthSnapshot health = snapshotMasterAdc1DmaHealth();
    return health.invalid_frames + health.invalid_samples +
           health.read_errors + health.pool_overflows;
}

uint16_t MasterAdc1CurrentSense::consecutiveReadErrors() const {
    const MasterAdc1DmaHealthSnapshot health = snapshotMasterAdc1DmaHealth();
    return static_cast<uint16_t>(
        std::min<uint32_t>(health.stale_control_cycles, UINT16_MAX));
}

bool MasterAdc1CurrentSense::readFaulted() const {
    return masterAdc1DmaFaultLatched();
}
