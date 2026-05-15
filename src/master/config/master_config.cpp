#include "master/config/master_config.h"

// ============================================================================
// 主机 X 轴配置
// ----------------------------------------------------------------------------
// 当前单轴阶段只配置 X 轴。后续双旋钮时，可以按同样结构新增 kMasterYAxis。
//
// center_deg：
//   用于把 MT6701 的绝对角转换为以机械中位为 0 的控制角。
// min_deg / max_deg：
//   定义主机旋钮可映射到从机坐标的控制范围，同时也是虚拟墙边界。
// boundary_soft_zone_deg：
//   定义边界内侧渐硬区宽度。强力矩档使用更宽墙区，减小 89..90deg 附近的抖动敏感度。
// haptic_current_limit_a：
//   墙和越界回推最大 q 轴目标电流，决定最大墙硬度。
// haptic_current_ramp_a_per_s：
//   目标电流斜率限制，避免目标电流阶跃造成手感突变。
// ============================================================================
#if MASTER_USE_CURRENT_SENSE
const MasterAxisConfig kMasterXAxis = {
    MASTER_KNOB_CENTER_DEG, // center_deg: 机械中位对应的 MT6701 绝对角，单位 deg。
    -90.00f,               // min_deg: 主机控制/力反馈低端虚拟边界，单位 deg。
    90.00f,                // max_deg: 主机控制/力反馈高端虚拟边界，单位 deg。
#if MASTER_STRONG_TORQUE_TEST_ENABLED
    MASTER_STRONG_TORQUE_BOUNDARY_SOFT_ZONE_DEG, // 边界软墙区宽度；强力矩档下更早进入渐硬区。
    MASTER_STRONG_TORQUE_CURRENT_LIMIT_A,        // 力反馈电流上限；决定墙/越界回推最大力度。
    MASTER_STRONG_TORQUE_CURRENT_RAMP_A_PER_S,   // 力反馈目标电流斜率限制；限制入墙电流变化速度。
#else
    1.0f,                  // boundary_soft_zone_deg: 保守档墙接触区宽度，1 deg -> 89..90 deg 快速变硬。
    0.25f,                // haptic_current_limit_a: 保守档墙/回推最大 q 轴目标电流，单位 A。
    30.0f,                 // haptic_current_ramp_a_per_s: 保守档目标电流斜率限制。
#endif
};
#else
// 电压模式联调配置。
// 该分支用于不启用电流采样时的低风险测试，输出目标不再是真实 q 轴电流。
// 边界范围更宽、输出上限更小，目的是先确认电机方向和基本控制链路。
const MasterAxisConfig kMasterXAxis = {
    MASTER_KNOB_CENTER_DEG,       // center_deg: 机械中位对应的 MT6701 绝对角，单位 deg。
    -MASTER_KNOB_HALF_RANGE_DEG,  // min_deg: 电压模式联调低端边界，单位 deg。
    MASTER_KNOB_HALF_RANGE_DEG,   // max_deg: 电压模式联调高端边界，单位 deg。
    30.0f,                       // boundary_soft_zone_deg: 电压模式宽软墙，便于低风险上板联调。
    0.020f,                      // haptic_current_limit_a: 输出目标上限；电压模式仍复用为安全限幅。
    2.4f,                        // haptic_current_ramp_a_per_s: 目标输出斜率限制。
};
#endif

// ============================================================================
// 主机 SimpleFOC / 电流环配置
// ----------------------------------------------------------------------------
// 当前主机力反馈采用 P-only 电流闭环：
// - q 轴 Iq 产生主要力矩；
// - d 轴 Id 理想上接近 0；
// - I/D 均保持 0，避免积分饱和和噪声放大。
//
// 强力矩档会提高 voltage_limit、current_pid_p、pid_ramp 和电流 LPF 响应速度，
// 以换取更明显的边界墙感；保守档则用于低风险上板确认闭环方向和稳定性。
// ============================================================================
const MasterMotorFocConfig kMasterMotorFoc = {
    12.0f,  // supply_voltage_v: 驱动板母线/电源电压，单位 V。
#if MASTER_STRONG_TORQUE_TEST_ENABLED
    MASTER_STRONG_TORQUE_VOLTAGE_LIMIT_V, // voltage_limit_v: 强力矩档输出电压上限，给电流环留跟随余量。
#else
    0.8f,   // voltage_limit_v: 保守档 SimpleFOC 输出电压上限，单位 V。
#endif
    1.0f,   // align_voltage_v: initFOC 对齐/检测使用电压，单位 V。
#if MASTER_USE_CURRENT_SENSE
#if MASTER_STRONG_TORQUE_TEST_ENABLED
    MASTER_STRONG_TORQUE_CURRENT_PID_P,     // current_q_pid_p: 强力矩档 q 轴电流环 P 增益。
    0.0f,                                  // current_q_pid_i: q 轴积分关闭，避免按住边界时 windup。
    0.0f,                                  // current_q_pid_d: q 轴微分关闭，避免放大采样噪声。
    MASTER_STRONG_TORQUE_CURRENT_PID_RAMP, // current_q_pid_ramp: q 轴 PID 输出斜率限制。
    MASTER_STRONG_TORQUE_CURRENT_PID_P,     // current_d_pid_p: d 轴电流环 P 增益，与 q 轴保持一致。
    0.0f,                                  // current_d_pid_i: d 轴积分关闭。
    0.0f,                                  // current_d_pid_d: d 轴微分关闭。
    MASTER_STRONG_TORQUE_CURRENT_PID_RAMP, // current_d_pid_ramp: d 轴 PID 输出斜率限制。
    MASTER_STRONG_TORQUE_CURRENT_LPF_TF,   // current_lpf_tf: q/d 电流低通滤波时间常数。
#else
    0.5f,    // current_q_pid_p: 保守档 q 轴电流环 P，只用纯 P 控制。
    0.0f,    // current_q_pid_i: q 轴积分关闭，避免 Vq 积分饱和。
    0.0f,    // current_q_pid_d: q 轴微分关闭。
    100.0f,  // current_q_pid_ramp: q 轴 PID 输出斜率限制。
    0.5f,    // current_d_pid_p: d 轴电流环 P，用于压住 Id。
    0.0f,    // current_d_pid_i: d 轴积分关闭，避免 Vd 积分饱和。
    0.0f,    // current_d_pid_d: d 轴微分关闭。
    100.0f,  // current_d_pid_ramp: d 轴 PID 输出斜率限制。
    0.001f,  // current_lpf_tf: q/d 电流低通滤波时间常数，单位 s。
#endif
#else
    // 未启用电流采样时，这些 PID 参数不参与真实电流闭环，只保留结构字段一致。
    0.5f,    // current_q_pid_p: 电压模式下不参与闭环，仅保留结构一致。
    0.0f,    // current_q_pid_i: 电压模式下不参与闭环。
    0.0f,    // current_q_pid_d: 电压模式下不参与闭环。
    100.0f,  // current_q_pid_ramp: 电压模式下不参与闭环。
    0.5f,    // current_d_pid_p: 电压模式下不参与闭环。
    0.0f,    // current_d_pid_i: 电压模式下不参与闭环。
    0.0f,    // current_d_pid_d: 电压模式下不参与闭环。
    100.0f,  // current_d_pid_ramp: 电压模式下不参与闭环。
    0.01f,   // current_lpf_tf: 电压模式保留默认值，不影响输出。
#endif
};

// ============================================================================
// 固定从机 ESP-NOW MAC
// ----------------------------------------------------------------------------
// 当前阶段采用固定 peer MAC，不做自动扫描和配对。
// 字节顺序就是 ESP-NOW 添加 peer 和发送时使用的 6 字节地址顺序。
// 如果更换从机开发板，需要把这里替换为新从机的 Wi-Fi STA MAC。
// ============================================================================
const uint8_t kMasterPeerSlaveAddress[6] = {
    0x24, // MAC 第 0 字节
    0x58, // MAC 第 1 字节
    0x7c, // MAC 第 2 字节
    0xd0, // MAC 第 3 字节
    0xab, // MAC 第 4 字节
    0x2c, // MAC 第 5 字节
};
