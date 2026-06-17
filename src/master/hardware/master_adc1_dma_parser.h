#pragma once

#include <stddef.h>
#include <stdint.h>

static constexpr uint8_t kMasterAdc1DmaMaxSlots = 4U;
static constexpr uint8_t kMasterAdc1DmaParserChannelMapSize = 16U;
static constexpr uint8_t kMasterAdc1DmaInvalidChannel = 0xffU;
static constexpr uint8_t kMasterAdc1DmaUnitAdc1 = 0U;

struct MasterAdc1DmaParserConfig {
    uint8_t slot_count;
    uint8_t channel_for_slot[kMasterAdc1DmaMaxSlots];
    uint8_t slot_for_channel[kMasterAdc1DmaParserChannelMapSize];
    uint8_t expected_count[kMasterAdc1DmaMaxSlots];
    uint16_t required_slot_mask;
    uint16_t raw_max;
    bool channel_map_valid;
};

struct MasterAdc1DmaParserSample {
    uint8_t unit;
    uint8_t channel;
    uint16_t raw;
    bool valid;
};

struct MasterAdc1DmaParserResult {
    uint16_t present_slot_mask;
    uint32_t invalid_samples;
    uint32_t sum[kMasterAdc1DmaMaxSlots];
    uint8_t count[kMasterAdc1DmaMaxSlots];
    int average[kMasterAdc1DmaMaxSlots];
};

void resetMasterAdc1DmaParserResult(MasterAdc1DmaParserResult &result);

bool appendMasterAdc1DmaParserSample(const MasterAdc1DmaParserConfig &config,
                                     const MasterAdc1DmaParserSample &sample,
                                     MasterAdc1DmaParserResult &result);

bool finalizeMasterAdc1DmaParserResult(const MasterAdc1DmaParserConfig &config,
                                       MasterAdc1DmaParserResult &result);

bool parseMasterAdc1DmaSamples(const MasterAdc1DmaParserConfig &config,
                               const MasterAdc1DmaParserSample *samples,
                               size_t sample_count,
                               MasterAdc1DmaParserResult &result);
