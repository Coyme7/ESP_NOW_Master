#include "master/diagnostics/current_probe.h"

#include <Arduino.h>

#include "master/config/master_config.h"

#if MASTER_ENABLE_MOTOR_HW && MASTER_ENABLE_CURRENT_SENSE
namespace {

constexpr uint32_t kMasterCurrentSenseOffsetFrameTimeoutMs = 20U;
constexpr uint32_t kMasterCurrentSenseOffsetSettleMs = 80U;
constexpr uint16_t kMasterCurrentSenseOffsetReads = 1000U;

}  // namespace

bool calibrateMasterCurrentSenseOffsets(MasterMotorDiagnosticsContext &context) {
    context.driver.enable();
    context.driver.setPwm(0.0f, 0.0f, 0.0f);
    delay(kMasterCurrentSenseOffsetSettleMs);

    uint32_t last_sequence = 0;
    uint16_t valid_reads = 0;
    uint32_t adc_errors = 0;
    uint32_t sum_raw_a = 0;
    uint32_t sum_raw_b = 0;

    for (uint16_t i = 0; i < kMasterCurrentSenseOffsetReads; ++i) {
        int raw_a = 0;
        int raw_b = 0;
        if (!context.current_sense.waitNextRawPair(last_sequence,
                                                   kMasterCurrentSenseOffsetFrameTimeoutMs,
                                                   raw_a,
                                                   raw_b)) {
            adc_errors++;
            continue;
        }

        sum_raw_a += static_cast<uint32_t>(raw_a);
        sum_raw_b += static_cast<uint32_t>(raw_b);
        valid_reads++;
    }

    if (valid_reads == 0) {
        context.driver.setPwm(0.0f, 0.0f, 0.0f);
        context.driver.disable();
        Serial.printf("[Master] motor_diag current_sense offset_cal failed samples=%u adc_errors=%lu\n",
                      static_cast<unsigned int>(kMasterCurrentSenseOffsetReads),
                      static_cast<unsigned long>(adc_errors));
        return false;
    }

    const int avg_raw_a = static_cast<int>(sum_raw_a / valid_reads);
    const int avg_raw_b = static_cast<int>(sum_raw_b / valid_reads);
    context.current_sense.offset_ia =
        static_cast<float>(avg_raw_a) * kMasterCurrentSenseHardware.adc_raw_to_voltage_v;
    context.current_sense.offset_ib =
        static_cast<float>(avg_raw_b) * kMasterCurrentSenseHardware.adc_raw_to_voltage_v;

    context.driver.setPwm(0.0f, 0.0f, 0.0f);
    context.driver.disable();

    Serial.printf("[Master] motor_diag current_sense offset_cal mode=driver_enabled_pwm0 settle=%ums samples=%u ia=%.4fV ib=%.4fV raw_adc=%d,%d adc_errors=%lu\n",
                  static_cast<unsigned int>(kMasterCurrentSenseOffsetSettleMs),
                  static_cast<unsigned int>(kMasterCurrentSenseOffsetReads),
                  context.current_sense.offset_ia,
                  context.current_sense.offset_ib,
                  avg_raw_a,
                  avg_raw_b,
                  static_cast<unsigned long>(adc_errors));
    return true;
}

#else

bool calibrateMasterCurrentSenseOffsets(MasterMotorDiagnosticsContext &context) {
    (void)context;
    return true;
}

#endif
