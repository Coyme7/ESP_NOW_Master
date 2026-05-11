#pragma once

#include <Arduino.h>
#include <board/board_pins_master.h>

#include "shared_types.h"

// ========================================================================
//  上板调试快速调参区
//
// 这里集中放现场最常改的编译期选项。所有宏都用 #ifndef 包裹，仍允许
// platformio.ini/build_flags 临时覆盖；后文不要再次定义同名默认值。

// 真实主机电机输出总开关：0=只跑算法/通信状态，1=初始化驱动和 SimpleFOC。
#ifndef MASTER_MOTOR_HW_ENABLED
#define MASTER_MOTOR_HW_ENABLED 1
#endif

// 力矩控制模式选择：1=foc_current 电流闭环，0=voltage 电压力矩模式。
#ifndef MASTER_USE_CURRENT_SENSE
#define MASTER_USE_CURRENT_SENSE 1
#endif

// 0A 冒烟测试：1=真实电流目标固定为 0A，便于观察 iq/id 和 vq/vd。
#ifndef MASTER_CURRENT_ZERO_CURRENT_SMOKE_TEST
#define MASTER_CURRENT_ZERO_CURRENT_SMOKE_TEST 0
#endif

// 0A 烟测时的电流控制类型：1=dc_current，0=foc_current。
// 正常力反馈使用 foc_current，只有排查电流环时才临时改成 1。
#ifndef MASTER_CURRENT_ZERO_SMOKE_USE_DC_CURRENT
#define MASTER_CURRENT_ZERO_SMOKE_USE_DC_CURRENT 0
#endif

// 电流采样诊断模式：1=只做短促 U/V/W 注入采样，不进入运行态 FOC。
#ifndef MASTER_CURRENT_SENSE_DIAG_ONLY
#define MASTER_CURRENT_SENSE_DIAG_ONLY 0
#endif

// 固定小电流测试：1=绕过边界/阻尼逻辑，直接输出 MASTER_FIXED_CURRENT_TEST_A。
#ifndef MASTER_FIXED_CURRENT_TEST_ENABLED
#define MASTER_FIXED_CURRENT_TEST_ENABLED 0
#endif
// 固定小电流测试目标值，单位 A；正值为 +Iq，负值为 -Iq。
#ifndef MASTER_FIXED_CURRENT_TEST_A
#define MASTER_FIXED_CURRENT_TEST_A 0.005f
#endif

// SimpleFOC current sense driverAlign：1=跳过自动对齐，只做 offset 校准。
// 当前 DengFoc/ESP32-S3 bring-up 中 SimpleFOC 的自动相位校验总失败，因此默认跳过。
// 跳过后必须用 current_probe/0A 烟测确认 A/B 采样符号，否则 foc_current 可能正反馈。
#ifndef MASTER_CURRENT_SENSE_SKIP_ALIGN
#define MASTER_CURRENT_SENSE_SKIP_ALIGN 1
#endif
// A/B 相电流采样增益符号：1=保持库默认方向，-1=翻转该相采样方向。
// 因为自动相位校验已跳过，这两个符号就是手动校准结果；改动前先看 current_probe
// 中 U/V/W 注入时 ia/ib 的符号和 0A 烟测时 iq/id 是否稳定。
#ifndef MASTER_CURRENT_SENSE_GAIN_SIGN_A
#define MASTER_CURRENT_SENSE_GAIN_SIGN_A  -1
#endif
#ifndef MASTER_CURRENT_SENSE_GAIN_SIGN_B
#define MASTER_CURRENT_SENSE_GAIN_SIGN_B  -1
#endif

// 旋钮坐标方向：1=机械相对角原符号，-1=反向；不影响 FOC sensor_direction。
#ifndef MASTER_KNOB_AXIS_SIGN
#define MASTER_KNOB_AXIS_SIGN -1
#endif

// 墙/越界回推电流方向：1=当前符号，-1=只翻转墙和越界回推；不翻转中心阻尼。
#ifndef MASTER_HAPTIC_DIRECTION_SIGN
#define MASTER_HAPTIC_DIRECTION_SIGN 1
#endif

// 中心阻尼电流方向：1=当前符号，-1=单独翻转中心区阻尼。
// 如果中间区域出现“越转越被助推”，只改这个符号，不要同时改墙方向。
#ifndef MASTER_CENTER_DAMPING_DIRECTION_SIGN
#define MASTER_CENTER_DAMPING_DIRECTION_SIGN 1
#endif

// 安全切断距离，单位 deg：超过 min/max 外侧该角度后目标电流立即归零。
#ifndef MASTER_HAPTIC_OVERRUN_CUT_DEG
#define MASTER_HAPTIC_OVERRUN_CUT_DEG 10.0f
#endif
// boundary_hit 状态保持时间，单位 ms；只影响状态显示，不保持墙电流。
#ifndef MASTER_HAPTIC_BLOCK_HOLD_MS
#define MASTER_HAPTIC_BLOCK_HOLD_MS 50
#endif

// 中心主动阻尼开关：1=启用 -89..+89 deg 中心阻尼，0=中心区输出 0A。
#ifndef MASTER_CENTER_DAMPING_ENABLED
#define MASTER_CENTER_DAMPING_ENABLED 1
#endif
// 粘滞阻尼增益，单位 A/(deg/s)：速度越大，阻尼电流线性增大。
// 当前为增强档，让 -89..+89 deg 中间区能明显感觉到速度阻尼。
#ifndef MASTER_CENTER_DAMPING_GAIN_A_PER_DEG_S
#define MASTER_CENTER_DAMPING_GAIN_A_PER_DEG_S 0.00018f
#endif
// 库仑阻尼电流，单位 A：低速时通过 tanh 平滑接近该摩擦感。
// 静止速度为 0 时仍输出 0A，不会持续推着旋钮走。
#ifndef MASTER_CENTER_DAMPING_COULOMB_A
#define MASTER_CENTER_DAMPING_COULOMB_A 0.008f
#endif
// 库仑阻尼 tanh 速度尺度，单位 deg/s：越小越快达到库仑阻尼平台。
// 12 deg/s 让低速阻尼上升更平滑，减少突然卡住的感觉。
#ifndef MASTER_CENTER_DAMPING_VEL_SCALE_DEG_S
#define MASTER_CENTER_DAMPING_VEL_SCALE_DEG_S 12.0f
#endif
// 中心阻尼电流限幅，单位 A；墙区叠加阻尼后仍受 haptic_current_limit_a 限制。
#ifndef MASTER_CENTER_DAMPING_LIMIT_A
#define MASTER_CENTER_DAMPING_LIMIT_A 0.030f
#endif

// ========================================================================

// master_config
// 职责：集中主机侧任务调度、临时硬件测试和力反馈轴参数。
//
// 这个头文件是“调参入口”，不是运行逻辑入口。控制代码只读取这里的常量，
// 不把任务优先级、核心绑定、电流上限或软边界宽度硬编码在算法中。
//
// 运行约束：这里只放编译期或只读配置，不做任何外设访问。
// 后续阶段：自动配对后替换固定 MAC；加入 Y 轴时新增第二个 MasterAxisConfig。

// 临时正交编码器演示输入：1=使用下面两个 GPIO 的中断计数代替 MT6701。
#ifndef MASTER_DEMO_QUADRATURE_ENABLED
#define MASTER_DEMO_QUADRATURE_ENABLED 0
#endif

// 当前阶段没有接紫光灯和主机按钮，为了验证主从“落笔命令 -> 从机联锁/遥测”
// 这条通信路径，默认强制发送 pen_down=1。
// 后续接入真实按钮或紫光灯前，应在 build_flags 中设为 0，改回由 MAIN_BUTTON 控制。
#ifndef MASTER_FORCE_PEN_DOWN_FOR_TEST
#define MASTER_FORCE_PEN_DOWN_FOR_TEST 1
#endif

// DengFoc/BLDCDriver3PWM EN 脚极性：1=高电平使能，0=低电平使能。
// 如果实测 EN 低电平有效，必须在 build_flags 中设为 0 后再开真实电机输出。
#ifndef MASTER_DRIVER_ENABLE_ACTIVE_HIGH
#define MASTER_DRIVER_ENABLE_ACTIVE_HIGH 1
#endif

// SimpleFOC 启动诊断日志开关：1=输出 init/initFOC 阶段日志。
// 这些日志只在 init/initFOC 阶段输出，不进入 10 kHz 控制热路径。
#ifndef MASTER_SIMPLEFOC_DEBUG_ENABLED
#define MASTER_SIMPLEFOC_DEBUG_ENABLED 1
#endif

// initFOC 前开环相位扫描诊断：1=短时低电压扫相，只用于带电排查。
// 只在启动阶段短时输出对齐电压，用于区分“驱动没带动电机”和“编码器没跟着变”。
#ifndef MASTER_MOTOR_PHASE_PROBE_ENABLED
#define MASTER_MOTOR_PHASE_PROBE_ENABLED 0
#endif

// 电流采样分流电阻，单位 ohm；必须匹配 DengFoc 板实际 shunt。
#ifndef MASTER_CURRENT_SHUNT_OHM
#define MASTER_CURRENT_SHUNT_OHM 0.01f
#endif

// 电流采样放大倍数；必须匹配驱动板电流采样放大器。
#ifndef MASTER_CURRENT_SENSE_GAIN
#define MASTER_CURRENT_SENSE_GAIN 50.0f
#endif

// 电流采样诊断注入电压，单位 V；仅用于 MASTER_CURRENT_SENSE_DIAG_ONLY。
#ifndef MASTER_CURRENT_SENSE_DIAG_VOLTAGE_V
#define MASTER_CURRENT_SENSE_DIAG_VOLTAGE_V 0.30f
#endif

// 诊断注入后的早期采样等待时间，单位 ms。
#ifndef MASTER_CURRENT_SENSE_DIAG_EARLY_MS
#define MASTER_CURRENT_SENSE_DIAG_EARLY_MS 5
#endif

// 诊断注入后的稳定采样等待时间，单位 ms。
#ifndef MASTER_CURRENT_SENSE_DIAG_SETTLE_MS
#define MASTER_CURRENT_SENSE_DIAG_SETTLE_MS 80
#endif

// offset 校准前丢弃的 ADC 预读次数，用于让 ADC/采样路径稳定。
#ifndef MASTER_CURRENT_SENSE_ADC_PRIME_READS
#define MASTER_CURRENT_SENSE_ADC_PRIME_READS 8
#endif

// 电流采样 offset 校准平均次数；越大越稳但启动越慢。
#ifndef MASTER_CURRENT_SENSE_OFFSET_CALIBRATION_READS
#define MASTER_CURRENT_SENSE_OFFSET_CALIBRATION_READS 1000
#endif

// offset 校准前等待时间，单位 ms；默认复用诊断稳定等待时间。
#ifndef MASTER_CURRENT_SENSE_OFFSET_SETTLE_MS
#define MASTER_CURRENT_SENSE_OFFSET_SETTLE_MS MASTER_CURRENT_SENSE_DIAG_SETTLE_MS
#endif

// MT6701 输出的是 0..360 deg 的绝对角。主机控制需要的是以旋钮机械中位为 0 的
// -180..+180 deg 相对角。读取路径会先减去中心，再 wrap 到 [-180, +180] deg，
// 最后按配置的 min/max 做安全夹紧。
// 默认中心放在 180 deg：raw 0..8191 会显示为负角，raw 8192 附近为 0，
// raw 8193..16383 会显示为正角。机械中位不在 raw=8192 时再按实测精调。
// 精调公式：MASTER_KNOB_CENTER_DEG = 机械中位 raw * 360 / 16384。
#ifndef MASTER_KNOB_CENTER_DEG
#define MASTER_KNOB_CENTER_DEG 180.0f
#endif

// 控制任务绑核：Core 1 专门跑 10 kHz 控制热路径。
static constexpr BaseType_t MASTER_CONTROL_CORE = 1;
// IO 任务绑核：Core 0 放通信和串口，避免无线/日志扰动控制。
static constexpr BaseType_t MASTER_IO_CORE = 0;

// 控制任务优先级：最高优先级，保证 FOC/力反馈节拍优先。
static constexpr UBaseType_t MASTER_CONTROL_TASK_PRIORITY = configMAX_PRIORITIES - 1;
// 通信任务优先级：ESP-NOW 发送/接收和包快照处理，低于控制任务。
static constexpr UBaseType_t MASTER_COMM_TASK_PRIORITY = 3;
// 状态任务优先级：串口状态打印，最低，避免影响控制和通信。
static constexpr UBaseType_t MASTER_STATUS_TASK_PRIORITY = 1;

// 控制任务栈大小，单位 byte；包含 SimpleFOC 调用和控制状态。
static constexpr uint32_t MASTER_CONTROL_TASK_STACK_BYTES = 8192;
// 通信任务栈大小，单位 byte；包含 Wi-Fi/ESP-NOW 包处理。
static constexpr uint32_t MASTER_COMM_TASK_STACK_BYTES = 4096;
// 状态任务栈大小，单位 byte；包含串口格式化输出。
static constexpr uint32_t MASTER_STATUS_TASK_STACK_BYTES = 4096;

// 每个 1 ms FreeRTOS tick 内执行的控制子步数；当前目标约 10 kHz。
static constexpr uint32_t MASTER_CONTROL_STEPS_PER_TICK = 1000UL / CONTROL_LOOP_PERIOD_US;

// 临时正交编码器 A 相输入脚，仅在 MASTER_DEMO_QUADRATURE_ENABLED=1 时使用。
static constexpr int MASTER_DEMO_ENCODER_PIN_A = board_pins_master::UNUSED_DPI_1;
// 临时正交编码器 B 相输入脚，仅在 MASTER_DEMO_QUADRATURE_ENABLED=1 时使用。
static constexpr int MASTER_DEMO_ENCODER_PIN_B = board_pins_master::UNUSED_DPI_2;

// 主机 X 轴配置：
// center_deg 是机械中心偏移；min/max 是可映射到完整协议坐标的旋钮行程；
// boundary_soft_zone_deg 是虚拟硬墙内侧的接触区宽度；当前 1 deg 表示墙区为
// (+89,+90] 和 [-90,-89)，而 [-89,+89] 只允许中心阻尼。
// haptic_current_limit_a 是力反馈电流上限；haptic_current_ramp_a_per_s 限制目标电流渐变。
struct MasterAxisConfig {
    float center_deg;                   // 机械中位对应的 MT6701 绝对角，单位 deg。
    float min_deg;                      // 控制/力反馈低端虚拟边界，单位 deg。
    float max_deg;                      // 控制/力反馈高端虚拟边界，单位 deg。
    float boundary_soft_zone_deg;       // 边界内侧墙接触区宽度，单位 deg。
    float haptic_current_limit_a;       // 墙/越界回推最大 q 轴目标电流，单位 A。
    float haptic_current_ramp_a_per_s;  // 目标电流斜率限制，单位 A/s。
};

extern const MasterAxisConfig kMasterXAxis;

// 主机真实电流闭环调参入口。
// 这些参数只在 MASTER_MOTOR_HW_ENABLED=1 时作用于 SimpleFOC；
// 默认值偏保守，目的是先确认闭环方向和稳定性，再逐步提高响应。
struct MasterMotorFocConfig {
    float supply_voltage_v;     // 驱动板母线/电源电压，单位 V。
    float voltage_limit_v;      // SimpleFOC 输出电压上限，单位 V。
    float align_voltage_v;      // initFOC 对齐/检测使用电压，单位 V。
    float current_q_pid_p;      // q 轴电流环 P 增益。
    float current_q_pid_i;      // q 轴电流环 I 增益；当前保持 0 防 windup。
    float current_q_pid_d;      // q 轴电流环 D 增益；当前保持 0。
    float current_q_pid_ramp;   // q 轴 PID 输出斜率限制。
    float current_d_pid_p;      // d 轴电流环 P 增益。
    float current_d_pid_i;      // d 轴电流环 I 增益；当前保持 0 防 windup。
    float current_d_pid_d;      // d 轴电流环 D 增益；当前保持 0。
    float current_d_pid_ramp;   // d 轴 PID 输出斜率限制。
    float current_lpf_tf;       // q/d 电流低通滤波时间常数，单位 s。
};

extern const MasterMotorFocConfig kMasterMotorFoc;

// 当前单轴联调使用固定从机 MAC。加入配对流程前，不在运行时自动扫描对端。
extern const uint8_t kMasterPeerSlaveAddress[6];
