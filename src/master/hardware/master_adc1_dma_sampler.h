#pragma once

#include <stdint.h>

#include "master/hardware/master_adc1_dma_parser.h"

enum MasterAdc1DmaSlot : uint8_t {
    MASTER_ADC1_DMA_SLOT_X_A = 0,
    MASTER_ADC1_DMA_SLOT_X_B = 1,
    MASTER_ADC1_DMA_SLOT_Y_A = 2,
    MASTER_ADC1_DMA_SLOT_Y_B = 3,
};

enum MasterAdc1DmaFaultReason : uint8_t {
    MASTER_ADC1_DMA_FAULT_NONE = 0,
    MASTER_ADC1_DMA_FAULT_WRONG_CORE = 1,
    MASTER_ADC1_DMA_FAULT_CONFIG = 2,
    MASTER_ADC1_DMA_FAULT_TASK_CREATE = 3,
    MASTER_ADC1_DMA_FAULT_NEW_HANDLE = 4,
    MASTER_ADC1_DMA_FAULT_ADC_CONFIG = 5,
    MASTER_ADC1_DMA_FAULT_CALLBACK = 6,
    MASTER_ADC1_DMA_FAULT_START = 7,
    MASTER_ADC1_DMA_FAULT_FIRST_FRAME_TIMEOUT = 8,
    MASTER_ADC1_DMA_FAULT_STALE_TIMEOUT = 9,
    MASTER_ADC1_DMA_FAULT_NO_VALID_FRAME = 10,
};

struct MasterAdc1DmaFrameSnapshot {
    bool valid;
    uint32_t sequence;
    uint32_t published_us;
    uint16_t slot_mask;
    int raw[kMasterAdc1DmaMaxSlots];
    uint8_t count[kMasterAdc1DmaMaxSlots];
};

struct MasterAdc1DmaHealthSnapshot {
    bool required;
    bool started;
    bool first_frame_ready;
    bool fault_latched;
    bool runtime_fault_latch_enabled;
    uint8_t fault_reason;
    uint32_t frame_sequence;
    uint32_t latest_age_us;
    uint32_t latest_age_max_us;
    uint32_t fault_age_us;
    uint32_t stale_fault_us;
    uint32_t frame_bytes;
    uint32_t pool_frames;
    uint32_t pool_bytes;
    uint8_t samples_per_active_channel;
    uint32_t invalid_frames;
    uint32_t invalid_samples;
    uint32_t read_errors;
    uint32_t read_empty_count;
    uint32_t pool_overflows;
    uint32_t stale_control_cycles;
    uint32_t consumer_last_us;
    uint32_t consumer_max_us;
    uint16_t required_slot_mask;
    uint8_t expected_count[kMasterAdc1DmaMaxSlots];
};

bool masterAdc1DmaSamplerRequired();
bool masterAdc1DmaSlotForPin(int pin, MasterAdc1DmaSlot &slot);

bool startMasterAdc1DmaSampler();
bool waitForMasterAdc1DmaFirstFrame(uint32_t timeout_ms);
void armMasterAdc1DmaControlStartupGrace();
bool latchMasterAdc1DmaControlSnapshot();
bool masterAdc1DmaReadControlRaw(MasterAdc1DmaSlot slot, int &raw);
bool masterAdc1DmaReadLatestRaw(MasterAdc1DmaSlot slot, int &raw);
bool waitForMasterAdc1DmaRawPair(MasterAdc1DmaSlot slot_a,
                                 MasterAdc1DmaSlot slot_b,
                                 uint32_t &last_sequence,
                                 uint32_t timeout_ms,
                                 int &raw_a,
                                 int &raw_b);
bool masterAdc1DmaFaultLatched();
MasterAdc1DmaHealthSnapshot snapshotMasterAdc1DmaHealth();
