#include "master/hardware/master_current_sense_adc1.h"

#include <esp_adc/adc_oneshot.h>

#include "master/config/master_config.h"

#if MASTER_TIMING_DETAIL_DIAG_ENABLED
extern "C" void recordMasterTimingCurrentSenseUs(uint32_t duration_us) __attribute__((weak));
#endif

namespace {

adc_oneshot_unit_handle_t masterAdc1Handle = nullptr;
uint16_t masterAdc1ConfiguredChannels = 0;

bool ensureMasterAdc1Unit() {
    if (masterAdc1Handle != nullptr) {
        return true;
    }

    adc_oneshot_unit_init_cfg_t unit_config = {};
    unit_config.unit_id = ADC_UNIT_1;
    unit_config.clk_src = ADC_RTC_CLK_SRC_DEFAULT;
    unit_config.ulp_mode = ADC_ULP_MODE_DISABLE;
    return adc_oneshot_new_unit(&unit_config, &masterAdc1Handle) == ESP_OK;
}

bool configureMasterAdc1Channel(uint8_t channel) {
    const uint16_t channel_mask = static_cast<uint16_t>(1U << channel);
    if ((masterAdc1ConfiguredChannels & channel_mask) != 0U) {
        return true;
    }

    adc_oneshot_chan_cfg_t channel_config = {};
    channel_config.atten = ADC_ATTEN_DB_12;
    channel_config.bitwidth = ADC_BITWIDTH_12;
    if (adc_oneshot_config_channel(masterAdc1Handle,
                                   static_cast<adc_channel_t>(channel),
                                   &channel_config) != ESP_OK) {
        return false;
    }

    masterAdc1ConfiguredChannels =
        static_cast<uint16_t>(masterAdc1ConfiguredChannels | channel_mask);
    return true;
}

}  // namespace

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
      raw_to_voltage_v_(kMasterCurrentSenseHardware.adc_raw_to_voltage_v) {
    offset_ia = 0.0f;
    offset_ib = 0.0f;
    offset_ic = 0.0f;
}

// 使用 ESP-IDF 当前版本的 GPIO 映射，避免在业务代码中维护芯片通道表。
bool MasterAdc1CurrentSense::gpioToAdc1Channel(int pin, uint8_t &channel) {
    adc_unit_t unit = ADC_UNIT_1;
    adc_channel_t mapped_channel = ADC_CHANNEL_0;
    if (adc_oneshot_io_to_channel(pin, &unit, &mapped_channel) != ESP_OK ||
        unit != ADC_UNIT_1) {
        return false;
    }
    channel = static_cast<uint8_t>(mapped_channel);
    return true;
}

// IDF oneshot 会返回明确错误；失败时由调用方沿用上一组完整有效样本。
bool MasterAdc1CurrentSense::readChannel(uint8_t channel, int &raw) const {
    int sample = 0;
    const esp_err_t result =
        adc_oneshot_read(masterAdc1Handle, static_cast<adc_channel_t>(channel), &sample);
    const int max_raw = static_cast<int>(kMasterCurrentSenseHardware.adc_raw_max);
    if (result != ESP_OK || sample < 0 || sample > max_raw) {
        if (read_error_count_ != UINT32_MAX) {
            read_error_count_ = read_error_count_ + 1U;
        }
        return false;
    }
    raw = sample;
    return true;
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

    if (!ensureMasterAdc1Unit() ||
        !configureMasterAdc1Channel(chan_a_) ||
        !configureMasterAdc1Channel(chan_b_)) {
        initialized = false;
        return 0;
    }

    int raw_a = 0;
    int raw_b = 0;
    if (!readChannel(chan_a_, raw_a) || !readChannel(chan_b_, raw_b)) {
        initialized = false;
        return 0;
    }
    last_raw_a_ = raw_a;
    last_raw_b_ = raw_b;
    consecutive_read_errors_ = 0;

    initialized = true;
    return 1;
}

// 跳过 SimpleFOC 自动相位对齐，采样方向由手动诊断参数决定。
int MasterAdc1CurrentSense::driverAlign(float align_voltage, bool modulation_centered) {
    (void)align_voltage;
    (void)modulation_centered;
    return 1;
}

// SimpleFOC 每次 loopFOC 会调用这里读取相电流，是控制热路径的一部分。
PhaseCurrent_s MasterAdc1CurrentSense::getPhaseCurrents() {
#if MASTER_TIMING_DETAIL_DIAG_ENABLED
    const uint32_t read_start_us = micros();
#endif
    // 热路径只做两次 ADC 原始采样和线性换算，不做 offset 重算或诊断打印。
    int raw_a = last_raw_a_;
    int raw_b = last_raw_b_;
    int sampled_a = 0;
    int sampled_b = 0;
    const bool read_a_ok = readChannel(chan_a_, sampled_a);
    const bool read_b_ok = readChannel(chan_b_, sampled_b);
    const bool read_ok = read_a_ok && read_b_ok;
    if (read_ok) {
        raw_a = sampled_a;
        raw_b = sampled_b;
        last_raw_a_ = raw_a;
        last_raw_b_ = raw_b;
        consecutive_read_errors_ = 0;
    } else if (consecutive_read_errors_ != UINT16_MAX) {
        consecutive_read_errors_ =
            static_cast<uint16_t>(consecutive_read_errors_ + 1U);
    }
    const float voltage_a = static_cast<float>(raw_a) * raw_to_voltage_v_;
    const float voltage_b = static_cast<float>(raw_b) * raw_to_voltage_v_;

    PhaseCurrent_s current;
    current.a = (voltage_a - offset_ia) * gain_a;
    current.b = (voltage_b - offset_ib) * gain_b;
    current.c = 0.0f;
#if MASTER_TIMING_DETAIL_DIAG_ENABLED
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
    int raw = last_raw_a_;
    if (readChannel(chan_a_, raw)) {
        last_raw_a_ = raw;
    }
    return raw;
}

// 诊断读取 B 相原始 ADC 值。
int MasterAdc1CurrentSense::readRawB() const {
    if (!initialized) {
        return 0;
    }
    int raw = last_raw_b_;
    if (readChannel(chan_b_, raw)) {
        last_raw_b_ = raw;
    }
    return raw;
}

uint32_t MasterAdc1CurrentSense::readErrorCount() const {
    return read_error_count_;
}

uint16_t MasterAdc1CurrentSense::consecutiveReadErrors() const {
    return consecutive_read_errors_;
}

bool MasterAdc1CurrentSense::readFaulted() const {
    return consecutive_read_errors_ >=
           kMasterCurrentSenseAdcConsecutiveErrorLimit;
}
