#include <Arduino.h>
#include <atomic>
#include <unity.h>

#include "master/hardware/master_adc1_dma_parser.h"
#include "../../src/master/hardware/master_adc1_dma_parser.cpp"

namespace {

MasterAdc1DmaParserConfig dualAxisConfig(uint8_t expected_count) {
    MasterAdc1DmaParserConfig config = {};
    config.slot_count = kMasterAdc1DmaMaxSlots;
    config.required_slot_mask = 0x0fU;
    config.raw_max = 4095U;
    for (uint8_t channel = 0; channel < kMasterAdc1DmaParserChannelMapSize; ++channel) {
        config.slot_for_channel[channel] = kMasterAdc1DmaMaxSlots;
    }
    for (uint8_t slot = 0; slot < kMasterAdc1DmaMaxSlots; ++slot) {
        const uint8_t channel = static_cast<uint8_t>(3U + slot);
        config.channel_for_slot[slot] = channel;
        config.slot_for_channel[channel] = slot;
        config.expected_count[slot] = expected_count;
    }
    config.channel_map_valid = true;
    return config;
}

MasterAdc1DmaParserSample sample(uint8_t channel, uint16_t raw) {
    MasterAdc1DmaParserSample value = {};
    value.unit = kMasterAdc1DmaUnitAdc1;
    value.channel = channel;
    value.raw = raw;
    value.valid = true;
    return value;
}

struct TestPublishedFrame {
    bool valid;
    uint32_t sequence;
    uint16_t slot_mask;
    int raw[kMasterAdc1DmaMaxSlots];
};

void test_parser_averages_out_of_order_scans() {
    const MasterAdc1DmaParserConfig config = dualAxisConfig(2U);
    const MasterAdc1DmaParserSample samples[] = {
        sample(5U, 300U),
        sample(3U, 100U),
        sample(6U, 400U),
        sample(4U, 200U),
        sample(4U, 220U),
        sample(6U, 420U),
        sample(3U, 120U),
        sample(5U, 320U),
    };

    MasterAdc1DmaParserResult result = {};
    TEST_ASSERT_TRUE(parseMasterAdc1DmaSamples(config,
                                              samples,
                                              sizeof(samples) / sizeof(samples[0]),
                                              result));
    TEST_ASSERT_EQUAL_UINT16(0x0fU, result.present_slot_mask);
    TEST_ASSERT_EQUAL_UINT8(2U, result.count[0]);
    TEST_ASSERT_EQUAL_INT(110, result.average[0]);
    TEST_ASSERT_EQUAL_INT(210, result.average[1]);
    TEST_ASSERT_EQUAL_INT(310, result.average[2]);
    TEST_ASSERT_EQUAL_INT(410, result.average[3]);
}

void test_parser_rejects_missing_required_channel() {
    const MasterAdc1DmaParserConfig config = dualAxisConfig(2U);
    const MasterAdc1DmaParserSample samples[] = {
        sample(3U, 100U),
        sample(4U, 200U),
        sample(5U, 300U),
        sample(6U, 400U),
        sample(3U, 120U),
        sample(4U, 220U),
        sample(5U, 320U),
    };

    MasterAdc1DmaParserResult result = {};
    TEST_ASSERT_FALSE(parseMasterAdc1DmaSamples(config,
                                               samples,
                                               sizeof(samples) / sizeof(samples[0]),
                                               result));
    TEST_ASSERT_EQUAL_UINT8(1U, result.count[3]);
}

void test_parser_rejects_invalid_result() {
    const MasterAdc1DmaParserConfig config = dualAxisConfig(1U);
    MasterAdc1DmaParserSample samples[] = {
        sample(3U, 100U),
        sample(4U, 200U),
        sample(5U, 5000U),
        sample(6U, 400U),
    };

    MasterAdc1DmaParserResult result = {};
    TEST_ASSERT_FALSE(parseMasterAdc1DmaSamples(config,
                                               samples,
                                               sizeof(samples) / sizeof(samples[0]),
                                               result));
    TEST_ASSERT_EQUAL_UINT32(1U, result.invalid_samples);
}

void test_parser_result_is_reset_for_latest_frame() {
    const MasterAdc1DmaParserConfig config = dualAxisConfig(1U);
    const MasterAdc1DmaParserSample first[] = {
        sample(3U, 100U),
        sample(4U, 200U),
        sample(5U, 300U),
        sample(6U, 400U),
    };
    const MasterAdc1DmaParserSample second[] = {
        sample(3U, 900U),
        sample(4U, 800U),
        sample(5U, 700U),
        sample(6U, 600U),
    };

    MasterAdc1DmaParserResult result = {};
    TEST_ASSERT_TRUE(parseMasterAdc1DmaSamples(config,
                                              first,
                                              sizeof(first) / sizeof(first[0]),
                                              result));
    TEST_ASSERT_TRUE(parseMasterAdc1DmaSamples(config,
                                              second,
                                              sizeof(second) / sizeof(second[0]),
                                              result));
    TEST_ASSERT_EQUAL_INT(900, result.average[0]);
    TEST_ASSERT_EQUAL_INT(800, result.average[1]);
    TEST_ASSERT_EQUAL_INT(700, result.average[2]);
    TEST_ASSERT_EQUAL_INT(600, result.average[3]);
}

void test_incremental_parser_uses_channel_map_without_sample_array() {
    MasterAdc1DmaParserConfig config = dualAxisConfig(2U);
    for (uint8_t slot = 0; slot < kMasterAdc1DmaMaxSlots; ++slot) {
        config.channel_for_slot[slot] = kMasterAdc1DmaInvalidChannel;
    }

    MasterAdc1DmaParserResult result = {};
    resetMasterAdc1DmaParserResult(result);

    TEST_ASSERT_TRUE(appendMasterAdc1DmaParserSample(config, sample(6U, 600U), result));
    TEST_ASSERT_TRUE(appendMasterAdc1DmaParserSample(config, sample(3U, 300U), result));
    TEST_ASSERT_TRUE(appendMasterAdc1DmaParserSample(config, sample(5U, 500U), result));
    TEST_ASSERT_TRUE(appendMasterAdc1DmaParserSample(config, sample(4U, 400U), result));
    TEST_ASSERT_TRUE(appendMasterAdc1DmaParserSample(config, sample(3U, 320U), result));
    TEST_ASSERT_TRUE(appendMasterAdc1DmaParserSample(config, sample(4U, 420U), result));
    TEST_ASSERT_TRUE(appendMasterAdc1DmaParserSample(config, sample(5U, 520U), result));
    TEST_ASSERT_TRUE(appendMasterAdc1DmaParserSample(config, sample(6U, 620U), result));

    TEST_ASSERT_TRUE(finalizeMasterAdc1DmaParserResult(config, result));
    TEST_ASSERT_EQUAL_UINT16(0x0fU, result.present_slot_mask);
    TEST_ASSERT_EQUAL_INT(310, result.average[0]);
    TEST_ASSERT_EQUAL_INT(410, result.average[1]);
    TEST_ASSERT_EQUAL_INT(510, result.average[2]);
    TEST_ASSERT_EQUAL_INT(610, result.average[3]);
}

void test_incremental_parser_rejects_invalid_before_finalize() {
    const MasterAdc1DmaParserConfig config = dualAxisConfig(1U);
    MasterAdc1DmaParserResult result = {};
    resetMasterAdc1DmaParserResult(result);

    TEST_ASSERT_TRUE(appendMasterAdc1DmaParserSample(config, sample(3U, 100U), result));
    TEST_ASSERT_FALSE(appendMasterAdc1DmaParserSample(config, sample(15U, 200U), result));
    TEST_ASSERT_FALSE(finalizeMasterAdc1DmaParserResult(config, result));
    TEST_ASSERT_EQUAL_UINT32(1U, result.invalid_samples);
}

void test_release_acquire_index_publishes_complete_frame() {
    TestPublishedFrame frames[2] = {};
    std::atomic<uint8_t> published_index{0U};
    frames[0].valid = true;
    frames[0].sequence = 1U;
    frames[0].slot_mask = 0x0fU;

    const uint8_t active =
        published_index.load(std::memory_order_acquire);
    const uint8_t inactive = static_cast<uint8_t>(active ^ 1U);
    frames[inactive].valid = true;
    frames[inactive].sequence = frames[active].sequence + 1U;
    frames[inactive].slot_mask = 0x0fU;
    for (uint8_t slot = 0; slot < kMasterAdc1DmaMaxSlots; ++slot) {
        frames[inactive].raw[slot] = static_cast<int>(100 + slot);
    }
    published_index.store(inactive, std::memory_order_release);

    const uint8_t read_index =
        published_index.load(std::memory_order_acquire);
    const TestPublishedFrame frame = frames[read_index];
    TEST_ASSERT_TRUE(frame.valid);
    TEST_ASSERT_EQUAL_UINT32(2U, frame.sequence);
    TEST_ASSERT_EQUAL_UINT16(0x0fU, frame.slot_mask);
    TEST_ASSERT_EQUAL_INT(100, frame.raw[0]);
    TEST_ASSERT_EQUAL_INT(103, frame.raw[3]);
}

}  // namespace

void setUp() {}

void tearDown() {}

extern "C" void app_main() {
    initArduino();
    delay(2000);
    UNITY_BEGIN();
    RUN_TEST(test_parser_averages_out_of_order_scans);
    RUN_TEST(test_parser_rejects_missing_required_channel);
    RUN_TEST(test_parser_rejects_invalid_result);
    RUN_TEST(test_parser_result_is_reset_for_latest_frame);
    RUN_TEST(test_incremental_parser_uses_channel_map_without_sample_array);
    RUN_TEST(test_incremental_parser_rejects_invalid_before_finalize);
    RUN_TEST(test_release_acquire_index_publishes_complete_frame);
    UNITY_END();
}
