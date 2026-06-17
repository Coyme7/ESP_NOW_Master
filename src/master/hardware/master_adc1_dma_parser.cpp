#include "master/hardware/master_adc1_dma_parser.h"

namespace {

uint8_t slotForChannel(const MasterAdc1DmaParserConfig &config, uint8_t channel) {
    if (config.channel_map_valid &&
        channel < kMasterAdc1DmaParserChannelMapSize) {
        const uint8_t mapped_slot = config.slot_for_channel[channel];
        if (mapped_slot < config.slot_count &&
            mapped_slot < kMasterAdc1DmaMaxSlots) {
            return mapped_slot;
        }
    }

    for (uint8_t slot = 0; slot < config.slot_count && slot < kMasterAdc1DmaMaxSlots; ++slot) {
        if (config.channel_for_slot[slot] == channel) {
            return slot;
        }
    }
    return kMasterAdc1DmaMaxSlots;
}

bool requiredSlotComplete(const MasterAdc1DmaParserConfig &config,
                          const MasterAdc1DmaParserResult &result,
                          uint8_t slot) {
    const uint16_t slot_mask = static_cast<uint16_t>(1U << slot);
    if ((config.required_slot_mask & slot_mask) == 0U) {
        return true;
    }
    return result.count[slot] == config.expected_count[slot] &&
           config.expected_count[slot] != 0U;
}

}  // namespace

void resetMasterAdc1DmaParserResult(MasterAdc1DmaParserResult &result) {
    result.present_slot_mask = 0U;
    result.invalid_samples = 0U;
    for (uint8_t i = 0; i < kMasterAdc1DmaMaxSlots; ++i) {
        result.sum[i] = 0U;
        result.count[i] = 0U;
        result.average[i] = 0;
    }
}

bool appendMasterAdc1DmaParserSample(const MasterAdc1DmaParserConfig &config,
                                     const MasterAdc1DmaParserSample &sample,
                                     MasterAdc1DmaParserResult &result) {
    if (config.slot_count == 0U ||
        config.slot_count > kMasterAdc1DmaMaxSlots) {
        result.invalid_samples++;
        return false;
    }

    const uint8_t slot = slotForChannel(config, sample.channel);
    if (!sample.valid ||
        sample.unit != kMasterAdc1DmaUnitAdc1 ||
        sample.raw > config.raw_max ||
        slot >= kMasterAdc1DmaMaxSlots) {
        result.invalid_samples++;
        return false;
    }

    result.sum[slot] += sample.raw;
    if (result.count[slot] != UINT8_MAX) {
        result.count[slot]++;
    }
    result.present_slot_mask =
        static_cast<uint16_t>(result.present_slot_mask | (1U << slot));
    return true;
}

bool finalizeMasterAdc1DmaParserResult(const MasterAdc1DmaParserConfig &config,
                                       MasterAdc1DmaParserResult &result) {
    if (result.invalid_samples != 0U) {
        return false;
    }

    for (uint8_t slot = 0; slot < config.slot_count; ++slot) {
        if (!requiredSlotComplete(config, result, slot)) {
            return false;
        }
        if (result.count[slot] != 0U) {
            result.average[slot] =
                static_cast<int>((result.sum[slot] + (result.count[slot] / 2U)) /
                                 result.count[slot]);
        }
    }
    return true;
}

bool parseMasterAdc1DmaSamples(const MasterAdc1DmaParserConfig &config,
                               const MasterAdc1DmaParserSample *samples,
                               size_t sample_count,
                               MasterAdc1DmaParserResult &result) {
    resetMasterAdc1DmaParserResult(result);
    if (samples == nullptr) {
        result.invalid_samples++;
        return false;
    }

    for (size_t i = 0; i < sample_count; ++i) {
        (void)appendMasterAdc1DmaParserSample(config, samples[i], result);
    }
    return finalizeMasterAdc1DmaParserResult(config, result);
}
