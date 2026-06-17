#include "master/hardware/master_adc1_dma_sampler.h"

#include <Arduino.h>
#include <atomic>
#include <board/board_pins_master.h>
#include <esp_adc/adc_continuous.h>
#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <soc/soc_caps.h>

#include "master/config/master_config.h"
#include "master/modes/mode_traits.h"

namespace {

static constexpr uint32_t kMasterAdc1DmaSampleRateHz = 80000UL;
static constexpr uint32_t kMasterAdc1DmaDualFrameBytes = 64UL;
static constexpr uint32_t kMasterAdc1DmaSingleFrameBytes = 32UL;
static constexpr uint32_t kMasterAdc1DmaFrameBytes =
    masterRunModeIsDualXYLogic() ? kMasterAdc1DmaDualFrameBytes
                                 : kMasterAdc1DmaSingleFrameBytes;
static constexpr uint8_t kMasterAdc1DmaPatternCount =
    masterRunModeIsDualXYLogic() ? 4U : 2U;
static constexpr uint32_t kMasterAdc1DmaSamplesPerFrame =
    kMasterAdc1DmaFrameBytes / SOC_ADC_DIGI_RESULT_BYTES;
static constexpr uint8_t kMasterAdc1DmaSamplesPerActiveChannel =
    static_cast<uint8_t>(kMasterAdc1DmaSamplesPerFrame /
                         kMasterAdc1DmaPatternCount);
static constexpr uint32_t kMasterAdc1DmaPoolFrames =
    masterRunModeIsDualXYLogic() ? kMasterCurrentSenseAdcDualPoolFrames
                                 : kMasterCurrentSenseAdcSinglePoolFrames;
static constexpr uint32_t kMasterAdc1DmaPoolBytes =
    kMasterAdc1DmaFrameBytes * kMasterAdc1DmaPoolFrames;
static constexpr uint32_t kMasterAdc1DmaReadNotifyBit = 1UL << 0;
static constexpr uint32_t kMasterAdc1DmaReadTimeoutMs = 0UL;

static_assert((kMasterAdc1DmaFrameBytes % SOC_ADC_DIGI_RESULT_BYTES) == 0,
              "ADC DMA frame must fit whole result items");
static_assert((kMasterAdc1DmaFrameBytes % SOC_ADC_DIGI_DATA_BYTES_PER_CONV) == 0,
              "ADC DMA frame must match IDF conversion alignment");
static_assert((kMasterAdc1DmaSamplesPerFrame % kMasterAdc1DmaPatternCount) == 0,
              "ADC DMA frame must contain balanced active channels");
static_assert((kMasterAdc1DmaSampleRateHz % kMasterAdc1DmaSamplesPerFrame) == 0,
              "ADC DMA sample rate must produce integral frame rate");
static_assert(kMasterAdc1DmaPoolFrames >= 2U,
              "ADC DMA pool must hold at least two frames");

TaskHandle_t adcConsumerTaskHandle = nullptr;
adc_continuous_handle_t adcContinuousHandle = nullptr;
adc_digi_pattern_config_t adcPattern[kMasterAdc1DmaMaxSlots] = {};
MasterAdc1DmaParserConfig parserConfig = {};
MasterAdc1DmaFrameSnapshot publishedFrames[2] = {};
MasterAdc1DmaFrameSnapshot controlFrame = {};
alignas(4) uint8_t adcReadBuffer[kMasterAdc1DmaFrameBytes] = {};
std::atomic<uint8_t> publishedFrameIndex{0};
std::atomic<bool> adcStarted{false};
std::atomic<bool> adcFirstFrameReady{false};
std::atomic<bool> adcFaultLatched{false};
std::atomic<uint32_t> invalidFrames{0};
std::atomic<uint32_t> invalidSamples{0};
std::atomic<uint32_t> readErrors{0};
std::atomic<uint32_t> readEmptyCount{0};
std::atomic<uint32_t> poolOverflows{0};
std::atomic<uint32_t> staleControlCycles{0};
std::atomic<uint32_t> consumerLastUs{0};
std::atomic<uint32_t> consumerMaxUs{0};
std::atomic<uint32_t> latestAgeMaxUs{0};
std::atomic<uint32_t> faultAgeUs{0};
std::atomic<uint16_t> startupGraceControlCycles{0};
std::atomic<uint8_t> faultReason{MASTER_ADC1_DMA_FAULT_NONE};

constexpr bool adcDmaNeedsXAxis() {
    return MASTER_ENABLE_CURRENT_SENSE &&
           masterRunModeNeedsMotorHardware(AXIS_X);
}

constexpr bool adcDmaNeedsYAxis() {
    return MASTER_ENABLE_CURRENT_SENSE &&
           masterRunModeNeedsMotorHardware(AXIS_Y);
}

uint8_t expectedSamplesPerActiveChannel() {
    return kMasterAdc1DmaSamplesPerActiveChannel;
}

bool slotActive(MasterAdc1DmaSlot slot) {
    switch (slot) {
        case MASTER_ADC1_DMA_SLOT_X_A:
        case MASTER_ADC1_DMA_SLOT_X_B:
            return adcDmaNeedsXAxis();
        case MASTER_ADC1_DMA_SLOT_Y_A:
        case MASTER_ADC1_DMA_SLOT_Y_B:
            return adcDmaNeedsYAxis();
        default:
            return false;
    }
}

bool slotPin(MasterAdc1DmaSlot slot, int &pin) {
    switch (slot) {
        case MASTER_ADC1_DMA_SLOT_X_A:
            pin = board_pins_master::MOTOR1_CURRENT_A;
            return true;
        case MASTER_ADC1_DMA_SLOT_X_B:
            pin = board_pins_master::MOTOR1_CURRENT_B;
            return true;
        case MASTER_ADC1_DMA_SLOT_Y_A:
            pin = board_pins_master::MOTOR2_CURRENT_A;
            return true;
        case MASTER_ADC1_DMA_SLOT_Y_B:
            pin = board_pins_master::MOTOR2_CURRENT_B;
            return true;
        default:
            return false;
    }
}

void publishParsedFrame(const MasterAdc1DmaParserResult &result) {
    const uint8_t active_index =
        publishedFrameIndex.load(std::memory_order_acquire);
    const uint8_t inactive_index = static_cast<uint8_t>(active_index ^ 1U);
    MasterAdc1DmaFrameSnapshot &frame = publishedFrames[inactive_index];
    frame.valid = true;
    frame.sequence = publishedFrames[active_index].sequence + 1U;
    if (frame.sequence == 0U) {
        frame.sequence = 1U;
    }
    frame.published_us = micros();
    frame.slot_mask = result.present_slot_mask;
    for (uint8_t slot = 0; slot < kMasterAdc1DmaMaxSlots; ++slot) {
        frame.raw[slot] = result.average[slot];
        frame.count[slot] = result.count[slot];
    }
    publishedFrameIndex.store(inactive_index, std::memory_order_release);
    adcFirstFrameReady.store(true, std::memory_order_release);
}

MasterAdc1DmaFrameSnapshot latestPublishedFrame() {
    const uint8_t index =
        publishedFrameIndex.load(std::memory_order_acquire);
    return publishedFrames[index];
}

void updateMaxAtomic(std::atomic<uint32_t> &target, uint32_t value) {
    uint32_t observed = target.load(std::memory_order_relaxed);
    while (value > observed &&
           !target.compare_exchange_weak(observed,
                                         value,
                                         std::memory_order_relaxed,
                                         std::memory_order_relaxed)) {
    }
}

void latchAdcFault(uint8_t reason, uint32_t age_us) {
    uint8_t expected = MASTER_ADC1_DMA_FAULT_NONE;
    if (faultReason.compare_exchange_strong(expected,
                                            reason,
                                            std::memory_order_release,
                                            std::memory_order_relaxed)) {
        faultAgeUs.store(age_us, std::memory_order_relaxed);
    }
    adcFaultLatched.store(true, std::memory_order_release);
}

bool readRawFromFrame(const MasterAdc1DmaFrameSnapshot &frame,
                      MasterAdc1DmaSlot slot,
                      int &raw) {
    const uint8_t slot_index = static_cast<uint8_t>(slot);
    if (!frame.valid || slot_index >= kMasterAdc1DmaMaxSlots ||
        (frame.slot_mask & (1U << slot_index)) == 0U) {
        return false;
    }
    raw = frame.raw[slot_index];
    return true;
}

bool configureParserAndPattern() {
    parserConfig = {};
    parserConfig.slot_count = kMasterAdc1DmaMaxSlots;
    parserConfig.raw_max =
        static_cast<uint16_t>(kMasterCurrentSenseHardware.adc_raw_max);
    for (uint8_t slot = 0; slot < kMasterAdc1DmaMaxSlots; ++slot) {
        parserConfig.channel_for_slot[slot] = kMasterAdc1DmaInvalidChannel;
        parserConfig.expected_count[slot] = 0U;
    }
    for (uint8_t channel = 0; channel < kMasterAdc1DmaParserChannelMapSize; ++channel) {
        parserConfig.slot_for_channel[channel] = kMasterAdc1DmaMaxSlots;
    }

    uint8_t pattern_count = 0U;
    const uint8_t expected_count = expectedSamplesPerActiveChannel();
    for (uint8_t slot = 0; slot < kMasterAdc1DmaMaxSlots; ++slot) {
        const MasterAdc1DmaSlot dma_slot = static_cast<MasterAdc1DmaSlot>(slot);
        if (!slotActive(dma_slot)) {
            continue;
        }

        int pin = -1;
        if (!slotPin(dma_slot, pin)) {
            return false;
        }

        adc_unit_t unit = ADC_UNIT_1;
        adc_channel_t channel = ADC_CHANNEL_0;
        if (adc_continuous_io_to_channel(pin, &unit, &channel) != ESP_OK ||
            unit != ADC_UNIT_1) {
            return false;
        }

        parserConfig.channel_for_slot[slot] = static_cast<uint8_t>(channel);
        if (static_cast<uint8_t>(channel) >= kMasterAdc1DmaParserChannelMapSize) {
            return false;
        }
        parserConfig.slot_for_channel[static_cast<uint8_t>(channel)] = slot;
        parserConfig.expected_count[slot] = expected_count;
        parserConfig.required_slot_mask =
            static_cast<uint16_t>(parserConfig.required_slot_mask | (1U << slot));

        adcPattern[pattern_count].atten = ADC_ATTEN_DB_12;
        adcPattern[pattern_count].channel = static_cast<uint8_t>(channel);
        adcPattern[pattern_count].unit = ADC_UNIT_1;
        adcPattern[pattern_count].bit_width = ADC_BITWIDTH_12;
        pattern_count++;
    }
    parserConfig.channel_map_valid = true;
    return pattern_count == kMasterAdc1DmaPatternCount;
}

bool parseReadBuffer(uint32_t out_length) {
    if (out_length != kMasterAdc1DmaFrameBytes ||
        (out_length % SOC_ADC_DIGI_RESULT_BYTES) != 0U) {
        invalidFrames.fetch_add(1U, std::memory_order_relaxed);
        return false;
    }

    MasterAdc1DmaParserResult result = {};
    resetMasterAdc1DmaParserResult(result);
    const size_t sample_count = out_length / SOC_ADC_DIGI_RESULT_BYTES;
    for (size_t i = 0; i < sample_count; ++i) {
        const adc_digi_output_data_t *data =
            reinterpret_cast<const adc_digi_output_data_t *>(
                &adcReadBuffer[i * SOC_ADC_DIGI_RESULT_BYTES]);
        const MasterAdc1DmaParserSample sample = {
            static_cast<uint8_t>(data->type2.unit),
            static_cast<uint8_t>(data->type2.channel),
            static_cast<uint16_t>(data->type2.data),
            data->type2.channel < SOC_ADC_CHANNEL_NUM(0) &&
                data->type2.data <= kMasterCurrentSenseHardware.adc_raw_max,
        };
        (void)appendMasterAdc1DmaParserSample(parserConfig, sample, result);
    }

    if (!finalizeMasterAdc1DmaParserResult(parserConfig, result)) {
        invalidFrames.fetch_add(1U, std::memory_order_relaxed);
        invalidSamples.fetch_add(result.invalid_samples, std::memory_order_relaxed);
        return false;
    }
    publishParsedFrame(result);
    return true;
}

bool IRAM_ATTR onAdcFrameDone(adc_continuous_handle_t handle,
                              const adc_continuous_evt_data_t *edata,
                              void *user_data) {
    (void)handle;
    (void)edata;
    (void)user_data;

    BaseType_t high_task_woken = pdFALSE;
    TaskHandle_t task = adcConsumerTaskHandle;
    if (task != nullptr) {
        xTaskNotifyFromISR(task,
                           kMasterAdc1DmaReadNotifyBit,
                           eSetBits,
                           &high_task_woken);
    }
    return high_task_woken == pdTRUE;
}

bool IRAM_ATTR onAdcPoolOverflow(adc_continuous_handle_t handle,
                                 const adc_continuous_evt_data_t *edata,
                                 void *user_data) {
    (void)handle;
    (void)edata;
    (void)user_data;

    poolOverflows.fetch_add(1U, std::memory_order_relaxed);

    BaseType_t high_task_woken = pdFALSE;
    TaskHandle_t task = adcConsumerTaskHandle;
    if (task != nullptr) {
        xTaskNotifyFromISR(task,
                           kMasterAdc1DmaReadNotifyBit,
                           eSetBits,
                           &high_task_woken);
    }
    return high_task_woken == pdTRUE;
}

void taskAdcDmaConsumer(void *pvParameters) {
    (void)pvParameters;

    while (true) {
        uint32_t notified = 0;
        xTaskNotifyWait(0U, UINT32_MAX, &notified, portMAX_DELAY);

        while (adcContinuousHandle != nullptr) {
            const uint32_t start_us = micros();
            uint32_t out_length = 0U;
            const esp_err_t err = adc_continuous_read(adcContinuousHandle,
                                                      adcReadBuffer,
                                                      sizeof(adcReadBuffer),
                                                      &out_length,
                                                      kMasterAdc1DmaReadTimeoutMs);
            if (err == ESP_ERR_TIMEOUT) {
                readEmptyCount.fetch_add(1U, std::memory_order_relaxed);
                break;
            }
            if (err != ESP_OK) {
                readErrors.fetch_add(1U, std::memory_order_relaxed);
                break;
            }
            (void)parseReadBuffer(out_length);
            const uint32_t elapsed_us = micros() - start_us;
            consumerLastUs.store(elapsed_us, std::memory_order_relaxed);
            if (elapsed_us > consumerMaxUs.load(std::memory_order_relaxed)) {
                consumerMaxUs.store(elapsed_us, std::memory_order_relaxed);
            }
        }
    }
}

}  // namespace

bool masterAdc1DmaSamplerRequired() {
    return adcDmaNeedsXAxis() || adcDmaNeedsYAxis();
}

bool masterAdc1DmaSlotForPin(int pin, MasterAdc1DmaSlot &slot) {
    for (uint8_t i = 0; i < kMasterAdc1DmaMaxSlots; ++i) {
        int candidate_pin = -1;
        if (slotPin(static_cast<MasterAdc1DmaSlot>(i), candidate_pin) &&
            candidate_pin == pin) {
            slot = static_cast<MasterAdc1DmaSlot>(i);
            return true;
        }
    }
    return false;
}

bool startMasterAdc1DmaSampler() {
    if (!masterAdc1DmaSamplerRequired()) {
        return true;
    }
    if (adcStarted.load(std::memory_order_acquire)) {
        return true;
    }
    const BaseType_t current_core = xPortGetCoreID();
    configASSERT(current_core == MASTER_IO_CORE);
    if (current_core != MASTER_IO_CORE) {
        latchAdcFault(MASTER_ADC1_DMA_FAULT_WRONG_CORE, 0U);
        return false;
    }
    if (!configureParserAndPattern()) {
        latchAdcFault(MASTER_ADC1_DMA_FAULT_CONFIG, 0U);
        return false;
    }

    if (adcConsumerTaskHandle == nullptr) {
        const BaseType_t created =
            xTaskCreatePinnedToCore(taskAdcDmaConsumer,
                                    "MasterAdcDma",
                                    MASTER_ADC_DMA_TASK_STACK_BYTES,
                                    nullptr,
                                    MASTER_ADC_DMA_TASK_PRIORITY,
                                    &adcConsumerTaskHandle,
                                    MASTER_IO_CORE);
        if (created != pdPASS) {
            latchAdcFault(MASTER_ADC1_DMA_FAULT_TASK_CREATE, 0U);
            return false;
        }
    }

    adc_continuous_handle_cfg_t handle_config = {};
    handle_config.max_store_buf_size = kMasterAdc1DmaPoolBytes;
    handle_config.conv_frame_size = kMasterAdc1DmaFrameBytes;
    handle_config.flags.flush_pool = 1U;
    if (adc_continuous_new_handle(&handle_config, &adcContinuousHandle) != ESP_OK) {
        latchAdcFault(MASTER_ADC1_DMA_FAULT_NEW_HANDLE, 0U);
        return false;
    }

    adc_continuous_config_t adc_config = {};
    adc_config.pattern_num = kMasterAdc1DmaPatternCount;
    adc_config.adc_pattern = adcPattern;
    adc_config.sample_freq_hz = kMasterAdc1DmaSampleRateHz;
    adc_config.conv_mode = ADC_CONV_SINGLE_UNIT_1;
    adc_config.format = ADC_DIGI_OUTPUT_FORMAT_TYPE2;
    if (adc_continuous_config(adcContinuousHandle, &adc_config) != ESP_OK) {
        latchAdcFault(MASTER_ADC1_DMA_FAULT_ADC_CONFIG, 0U);
        return false;
    }

    adc_continuous_evt_cbs_t callbacks = {};
    callbacks.on_conv_done = onAdcFrameDone;
    callbacks.on_pool_ovf = onAdcPoolOverflow;
    if (adc_continuous_register_event_callbacks(adcContinuousHandle,
                                                &callbacks,
                                                nullptr) != ESP_OK) {
        latchAdcFault(MASTER_ADC1_DMA_FAULT_CALLBACK, 0U);
        return false;
    }

    if (adc_continuous_start(adcContinuousHandle) != ESP_OK) {
        latchAdcFault(MASTER_ADC1_DMA_FAULT_START, 0U);
        return false;
    }
    adcStarted.store(true, std::memory_order_release);
    return true;
}

bool waitForMasterAdc1DmaFirstFrame(uint32_t timeout_ms) {
    if (!masterAdc1DmaSamplerRequired()) {
        return true;
    }
    const uint32_t start_ms = millis();
    uint32_t last_sequence = 0U;
    uint8_t warmup_frames = 0U;
    while (!adcFaultLatched.load(std::memory_order_acquire)) {
        const MasterAdc1DmaFrameSnapshot latest = latestPublishedFrame();
        if (latest.valid && latest.sequence != last_sequence) {
            last_sequence = latest.sequence;
            if (warmup_frames < UINT8_MAX) {
                warmup_frames++;
            }
            if (warmup_frames >= kMasterCurrentSenseAdcStartupWarmupFrames) {
                staleControlCycles.store(0U, std::memory_order_relaxed);
                return true;
            }
        }
        if ((millis() - start_ms) >= timeout_ms) {
            latchAdcFault(MASTER_ADC1_DMA_FAULT_FIRST_FRAME_TIMEOUT, 0U);
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    return false;
}

void armMasterAdc1DmaControlStartupGrace() {
    if (!masterAdc1DmaSamplerRequired()) {
        return;
    }
    startupGraceControlCycles.store(kMasterCurrentSenseAdcStartupGraceControlCycles,
                                    std::memory_order_release);
}

bool latchMasterAdc1DmaControlSnapshot() {
    if (!masterAdc1DmaSamplerRequired()) {
        return true;
    }
    const MasterAdc1DmaFrameSnapshot latest = latestPublishedFrame();
    uint32_t stale = staleControlCycles.load(std::memory_order_relaxed);
    if (!latest.valid) {
        if (stale != UINT32_MAX) {
            stale = stale + 1U;
            staleControlCycles.store(stale, std::memory_order_relaxed);
        }
    } else if (latest.sequence != controlFrame.sequence) {
        controlFrame = latest;
        stale = 0U;
        staleControlCycles.store(stale, std::memory_order_relaxed);
    } else if (stale != UINT32_MAX) {
        stale = stale + 1U;
        staleControlCycles.store(stale, std::memory_order_relaxed);
    }

    const uint32_t now_us = micros();
    const uint32_t latest_age_us =
        latest.valid ? (now_us - latest.published_us) : UINT32_MAX;
    if (latest.valid) {
        updateMaxAtomic(latestAgeMaxUs, latest_age_us);
    }
    const bool latest_timed_out =
        latest.valid && latest_age_us >= kMasterCurrentSenseAdcStaleFaultUs;
    const bool no_frame_after_start =
        !latest.valid && stale >= kMasterCurrentSenseAdcConsecutiveErrorLimit;
    uint16_t grace_cycles =
        startupGraceControlCycles.load(std::memory_order_acquire);
    const bool startup_grace_active = grace_cycles > 0U;
    if (startup_grace_active) {
        startupGraceControlCycles.store(static_cast<uint16_t>(grace_cycles - 1U),
                                        std::memory_order_release);
    }

    if (kMasterCurrentSenseAdcRuntimeFaultLatchEnabled &&
        !startup_grace_active && latest_timed_out) {
        latchAdcFault(MASTER_ADC1_DMA_FAULT_STALE_TIMEOUT, latest_age_us);
    } else if (kMasterCurrentSenseAdcRuntimeFaultLatchEnabled &&
               !startup_grace_active && no_frame_after_start) {
        latchAdcFault(MASTER_ADC1_DMA_FAULT_NO_VALID_FRAME, 0U);
    }
    return !adcFaultLatched.load(std::memory_order_acquire);
}

bool masterAdc1DmaReadControlRaw(MasterAdc1DmaSlot slot, int &raw) {
    return readRawFromFrame(controlFrame, slot, raw);
}

bool masterAdc1DmaReadLatestRaw(MasterAdc1DmaSlot slot, int &raw) {
    const MasterAdc1DmaFrameSnapshot latest = latestPublishedFrame();
    return readRawFromFrame(latest, slot, raw);
}

bool waitForMasterAdc1DmaRawPair(MasterAdc1DmaSlot slot_a,
                                 MasterAdc1DmaSlot slot_b,
                                 uint32_t &last_sequence,
                                 uint32_t timeout_ms,
                                 int &raw_a,
                                 int &raw_b) {
    if (!masterAdc1DmaSamplerRequired()) {
        return false;
    }
    const uint32_t start_ms = millis();
    while (!adcFaultLatched.load(std::memory_order_acquire)) {
        const MasterAdc1DmaFrameSnapshot latest = latestPublishedFrame();
        if (latest.valid && latest.sequence != last_sequence &&
            readRawFromFrame(latest, slot_a, raw_a) &&
            readRawFromFrame(latest, slot_b, raw_b)) {
            last_sequence = latest.sequence;
            return true;
        }
        if ((millis() - start_ms) >= timeout_ms) {
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    return false;
}

bool masterAdc1DmaFaultLatched() {
    return adcFaultLatched.load(std::memory_order_acquire);
}

MasterAdc1DmaHealthSnapshot snapshotMasterAdc1DmaHealth() {
    MasterAdc1DmaHealthSnapshot snapshot = {};
    const MasterAdc1DmaFrameSnapshot latest = latestPublishedFrame();
    snapshot.required = masterAdc1DmaSamplerRequired();
    snapshot.started = adcStarted.load(std::memory_order_acquire);
    snapshot.first_frame_ready = adcFirstFrameReady.load(std::memory_order_acquire);
    snapshot.fault_latched = adcFaultLatched.load(std::memory_order_acquire);
    snapshot.runtime_fault_latch_enabled =
        kMasterCurrentSenseAdcRuntimeFaultLatchEnabled;
    snapshot.fault_reason = faultReason.load(std::memory_order_acquire);
    snapshot.frame_sequence = latest.sequence;
    snapshot.latest_age_us = latest.valid ? (micros() - latest.published_us) : UINT32_MAX;
    snapshot.latest_age_max_us = latestAgeMaxUs.load(std::memory_order_relaxed);
    snapshot.fault_age_us = faultAgeUs.load(std::memory_order_relaxed);
    snapshot.stale_fault_us = kMasterCurrentSenseAdcStaleFaultUs;
    snapshot.frame_bytes = kMasterAdc1DmaFrameBytes;
    snapshot.pool_frames = kMasterAdc1DmaPoolFrames;
    snapshot.pool_bytes = kMasterAdc1DmaPoolBytes;
    snapshot.samples_per_active_channel = kMasterAdc1DmaSamplesPerActiveChannel;
    snapshot.invalid_frames = invalidFrames.load(std::memory_order_relaxed);
    snapshot.invalid_samples = invalidSamples.load(std::memory_order_relaxed);
    snapshot.read_errors = readErrors.load(std::memory_order_relaxed);
    snapshot.read_empty_count = readEmptyCount.load(std::memory_order_relaxed);
    snapshot.pool_overflows = poolOverflows.load(std::memory_order_relaxed);
    snapshot.stale_control_cycles = staleControlCycles.load(std::memory_order_relaxed);
    snapshot.consumer_last_us = consumerLastUs.load(std::memory_order_relaxed);
    snapshot.consumer_max_us = consumerMaxUs.load(std::memory_order_relaxed);
    snapshot.required_slot_mask = parserConfig.required_slot_mask;
    for (uint8_t slot = 0; slot < kMasterAdc1DmaMaxSlots; ++slot) {
        snapshot.expected_count[slot] = parserConfig.expected_count[slot];
    }
    return snapshot;
}
