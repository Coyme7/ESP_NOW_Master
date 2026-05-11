#include "master/master_config.h"

// 主机 X 轴默认调参值。
// 当前旋钮机械中心由 MASTER_KNOB_CENTER_DEG 标定；bring-up 先把虚拟边界收到 +/-90 deg。
// 虚拟墙接触区只保留 1 deg：[-89,+89] 是纯阻尼中心区，
// 89..90 deg 用快速渐硬曲线升到满墙电流，越过 90 deg 保持强回推。
// 电流闭环模式下 haptic_current_limit_a 是真实 q 轴电流目标上限；当前增强档为 0.080 A，
// 用于让 +/-90 deg 附近的虚拟墙和越界回推更明显；
// 目标输出通过 ramp 渐入，避免从无反馈直接阶跃到端点力矩；
// 中间区使用速度主动阻尼；虚拟硬墙只在接近边界和越界时输出。
#if MASTER_USE_CURRENT_SENSE
const MasterAxisConfig kMasterXAxis = {
    MASTER_KNOB_CENTER_DEG, // center_deg: 机械中位对应的 MT6701 绝对角，单位 deg。
    -90.00f,               // min_deg: 主机控制/力反馈低端虚拟边界，单位 deg。
    90.00f,                // max_deg: 主机控制/力反馈高端虚拟边界，单位 deg。
    1.0f,                  // boundary_soft_zone_deg: 墙接触区宽度，1 deg -> 89..90 deg 快速变硬。
    0.080f,                // haptic_current_limit_a: 墙/回推最大 q 轴目标电流，单位 A。
    30.0f,                 // haptic_current_ramp_a_per_s: 目标电流斜率限制，约 2.7 ms 到 0.08 A。
};
#else
const MasterAxisConfig kMasterXAxis = {
    MASTER_KNOB_CENTER_DEG,       // center_deg: 机械中位对应的 MT6701 绝对角，单位 deg。
    -MASTER_KNOB_HALF_RANGE_DEG,  // min_deg: 电压模式联调低端边界，单位 deg。
    MASTER_KNOB_HALF_RANGE_DEG,   // max_deg: 电压模式联调高端边界，单位 deg。
    30.0f,                       // boundary_soft_zone_deg: 电压模式宽软墙，便于低风险 bring-up。
    0.020f,                      // haptic_current_limit_a: 输出目标上限；电压模式仍复用为安全限幅。
    2.4f,                        // haptic_current_ramp_a_per_s: 目标输出斜率限制。
};
#endif

// 主机电流闭环默认值。
// voltage_limit_v 当前提高到 1.2 V，给增强墙感和中心阻尼留出电流环输出余量；
// align_voltage_v 仍保持 0.5 V，避免首次 initFOC 对齐时输出过猛；
// 若 Failed to notice movement，应优先查 EN/供电/三相线，而不是继续调 PID。
// bring-up 阶段使用 P-only 电流环；Iq/Id 都关闭积分，避免按住边界时 windup。
const MasterMotorFocConfig kMasterMotorFoc = {
    12.0f,  // supply_voltage_v: 驱动板母线/电源电压，单位 V。
    1.2f,   // voltage_limit_v: SimpleFOC 输出电压上限，单位 V。
    0.5f,   // align_voltage_v: initFOC 对齐/检测使用电压，单位 V。
#if MASTER_USE_CURRENT_SENSE
    0.5f,    // current_q_pid_p: q 轴电流环 P，只用 P-only。
    0.0f,    // current_q_pid_i: q 轴积分关闭，避免 Vq windup。
    0.0f,    // current_q_pid_d: q 轴微分关闭。
    100.0f,  // current_q_pid_ramp: q 轴 PID 输出斜率限制。
    0.5f,    // current_d_pid_p: d 轴电流环 P，用于压住 Id。
    0.0f,    // current_d_pid_i: d 轴积分关闭，避免 Vd windup。
    0.0f,    // current_d_pid_d: d 轴微分关闭。
    100.0f,  // current_d_pid_ramp: d 轴 PID 输出斜率限制。
    0.001f,  // current_lpf_tf: q/d 电流低通滤波时间常数，单位 s。
#else
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

// 固定从机 ESP-NOW MAC，字节顺序为无线层发送时使用的 6 字节地址。
// 后续加入自动配对后，再用配对结果替换这个常量。
const uint8_t kMasterPeerSlaveAddress[6] = {
    0x24, // MAC byte 0
    0x58, // MAC byte 1
    0x7c, // MAC byte 2
    0xd0, // MAC byte 3
    0xab, // MAC byte 4
    0x2c, // MAC byte 5
};
