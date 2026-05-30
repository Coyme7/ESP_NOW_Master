#include "master/config/core/master_motor_config.h"

namespace {

constexpr MasterMotorFocConfig kMasterCurrentSenseNormalPreset = {
    12.0f, // 驱动板母线/电源电压，单位 V。
    0.8f,  // SimpleFOC 输出电压上限，单位 V。
    1.0f,  // initFOC 对齐/检测使用电压，单位 V。

    {
        {
            0.5f,   // q 轴电流环 P 增益。
            0.0f,   // q 轴电流环 I 增益。
            0.0f,   // q 轴电流环 D 增益。
            100.0f, // q 轴电流环输出变化斜率限制。
        },

        {
            0.5f,   // d 轴电流环 P 增益。
            0.0f,   // d 轴电流环 I 增益。
            0.0f,   // d 轴电流环 D 增益。
            100.0f, // d 轴电流环输出变化斜率限制。
        },

        0.001f, // q/d 电流低通滤波时间常数，单位 s。
    },
};

constexpr MasterMotorFocConfig kMasterCurrentSenseStrongPreset = {
    12.0f, // 驱动板母线/电源电压，单位 V。
    3.20f, // SimpleFOC 输出电压上限，单位 V。
    1.0f,  // initFOC 对齐/检测使用电压，单位 V。

    {
        {
            3.40f,  // q 轴电流环 P 增益。
            0.0f,   // q 轴电流环 I 增益。
            0.0f,   // q 轴电流环 D 增益。
            500.0f, // q 轴电流环输出变化斜率限制。
        },

        {
            3.40f,  // d 轴电流环 P 增益。
            0.0f,   // d 轴电流环 I 增益。
            0.0f,   // d 轴电流环 D 增益。
            500.0f, // d 轴电流环输出变化斜率限制。
        },

        0.001f, // q/d 电流低通滤波时间常数，单位 s。
    },
};

constexpr MasterMotorFocConfig kMasterVoltageFallbackPreset = {
    12.0f, // 驱动板母线/电源电压，单位 V。
    0.8f,  // SimpleFOC 输出电压上限，单位 V。
    1.0f,  // initFOC 对齐/检测使用电压，单位 V。

    {
        {
            0.5f,   // q 轴 PID P 增益；电压 fallback 下作为保守占位参数。
            0.0f,   // q 轴 PID I 增益；电压 fallback 下不启用积分。
            0.0f,   // q 轴 PID D 增益；电压 fallback 下不启用微分。
            100.0f, // q 轴输出变化斜率限制。
        },

        {
            0.5f,   // d 轴 PID P 增益；电压 fallback 下作为保守占位参数。
            0.0f,   // d 轴 PID I 增益；电压 fallback 下不启用积分。
            0.0f,   // d 轴 PID D 增益；电压 fallback 下不启用微分。
            100.0f, // d 轴输出变化斜率限制。
        },

        0.01f, // q/d 低通滤波时间常数，单位 s；fallback 下更保守。
    },
};

}  // namespace

#if MASTER_ENABLE_CURRENT_SENSE
#if MASTER_ENABLE_STRONG_TORQUE_TEST
const MasterMotorFocConfig kMasterXMotorFoc = kMasterCurrentSenseStrongPreset;
#else
const MasterMotorFocConfig kMasterXMotorFoc = kMasterCurrentSenseNormalPreset;
#endif
#else
const MasterMotorFocConfig kMasterXMotorFoc = kMasterVoltageFallbackPreset;
#endif

// Y 轴保留独立对象名，便于后续单独调低电压、PID 或滤波。
const MasterMotorFocConfig kMasterYMotorFoc = kMasterXMotorFoc;