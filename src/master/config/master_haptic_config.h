#pragma once

#include "master/config/master_build_options.h"

// 强力矩档虚拟墙软区宽度，单位 deg。
// 例如 max=90deg、soft_zone=1.5deg，则高端墙从 88.5deg 开始渐硬。
// 值越小：墙更突然、更硬，但更容易把角度抖动放大成振动。
// 值越大：入口更平滑，但墙感会更早出现。
#ifndef MASTER_STRONG_TORQUE_BOUNDARY_SOFT_ZONE_DEG
#define MASTER_STRONG_TORQUE_BOUNDARY_SOFT_ZONE_DEG 1.5f
#endif

// 强力矩档力反馈电流上限，单位 A。
// 这是墙/越界回推最终允许的最大 q 轴目标电流，决定最大墙硬度。
// 手感振动明显时，不一定先降这个值，可先调斜率、LPF、P 增益。
#ifndef MASTER_STRONG_TORQUE_CURRENT_LIMIT_A
#define MASTER_STRONG_TORQUE_CURRENT_LIMIT_A 0.50f
#endif

// 强力矩档目标电流爬升斜率，单位 A/s。
// 控制“电流从当前值变到目标值”的最快速度。
// 值越大：墙响应更快；值越小：更柔和、更不容易抖。
#ifndef MASTER_STRONG_TORQUE_CURRENT_RAMP_A_PER_S
#define MASTER_STRONG_TORQUE_CURRENT_RAMP_A_PER_S 50.0f
#endif

// 固定电流测试在强力矩档下的默认目标值，单位 A。
// 只在 MASTER_FIXED_CURRENT_TEST_ENABLED=1 时使用，用于验证电流方向和力矩方向。
#ifndef MASTER_STRONG_TORQUE_FIXED_CURRENT_A
#define MASTER_STRONG_TORQUE_FIXED_CURRENT_A 0.30f
#endif

// 强力矩档 SimpleFOC 输出电压上限，单位 V。
// 电流环需要足够电压余量才能跟随目标电流；但过高会增加噪声、发热和风险。
#ifndef MASTER_STRONG_TORQUE_VOLTAGE_LIMIT_V
#define MASTER_STRONG_TORQUE_VOLTAGE_LIMIT_V 3.20f
#endif

// 强力矩档 q/d 轴电流环 P 增益。
// 当前阶段是 P-only 电流环，I/D 保持 0。
// 值越大：电流跟随更快，但更容易把目标电流纹波转成手感嗡振。
#ifndef MASTER_STRONG_TORQUE_CURRENT_PID_P
#define MASTER_STRONG_TORQUE_CURRENT_PID_P 3.40f
#endif

// 强力矩档 PID 输出斜率限制。
// 这是 SimpleFOC 内部 PID 输出变化限制，不等同于力反馈目标电流斜率。
// 用途：限制电流环电压输出变化速度，避免过激励。
#ifndef MASTER_STRONG_TORQUE_CURRENT_PID_RAMP
#define MASTER_STRONG_TORQUE_CURRENT_PID_RAMP 500.0f
#endif

// 强力矩档电流采样低通滤波时间常数，单位 s。
// 值越小：电流反馈更快，但通过更多采样噪声。
// 值越大：电流更平滑，但相位延迟增加，手感可能变软。
#ifndef MASTER_STRONG_TORQUE_CURRENT_LPF_TF
#define MASTER_STRONG_TORQUE_CURRENT_LPF_TF 0.001f
#endif

// 虚拟墙角度低通时间常数，单位 s。
// 只用于墙深度/墙接触判断的轻微滤波，不用于安全切断。
// 作用：抑制边界处角度噪声导致的墙电流细碎抖动。
#ifndef MASTER_HAPTIC_WALL_ANGLE_LPF_TF_S
#define MASTER_HAPTIC_WALL_ANGLE_LPF_TF_S 0.002f
#endif

// 虚拟墙释放迟滞，单位 deg。
// 进入墙区后，必须退出到更内侧一点才释放墙接触状态。
// 作用：避免角度在墙边界附近来回穿越时反复开关墙电流。
#ifndef MASTER_HAPTIC_WALL_RELEASE_HYST_DEG
#define MASTER_HAPTIC_WALL_RELEASE_HYST_DEG 0.20f
#endif

// 墙内最小贴墙电流，单位 A。
// 进入墙接触状态后，即使墙深度接近 0，也保持小电流，避免 0/非0 来回跳。
#ifndef MASTER_HAPTIC_WALL_MIN_CURRENT_A
#define MASTER_HAPTIC_WALL_MIN_CURRENT_A 0.03f
#endif

// 入墙阻尼增益，单位 A/(deg/s)。
// 旋钮继续往墙里推时，根据速度额外增加回推电流。
// 用途：快速撞墙时更硬，慢速贴墙时不至于过冲。
#ifndef MASTER_HAPTIC_WALL_DAMPING_GAIN_A_PER_DEG_S
#define MASTER_HAPTIC_WALL_DAMPING_GAIN_A_PER_DEG_S 0.001f
#endif

// 入墙阻尼电流限幅，单位 A。
// 防止高速撞墙时阻尼项过大，超过电流环舒适范围。
#ifndef MASTER_HAPTIC_WALL_DAMPING_LIMIT_A
#define MASTER_HAPTIC_WALL_DAMPING_LIMIT_A 0.08f
#endif

// 离开虚拟墙或目标电流反向时的快速卸载斜率，单位 A/s。
// 它通常比入墙斜率大，用于快速去掉残余回推力，避免释放时继续推手。
#ifndef MASTER_HAPTIC_CURRENT_RELEASE_RAMP_A_PER_S
#define MASTER_HAPTIC_CURRENT_RELEASE_RAMP_A_PER_S 500.0f
#endif

// 固定电流测试目标值，单位 A。
// 强力矩档开启时复用 MASTER_STRONG_TORQUE_FIXED_CURRENT_A；否则使用保守 0.20A。
// 正值表示 +Iq，负值表示 -Iq，具体手感方向还会受电机安装和方向符号影响。
#ifndef MASTER_FIXED_CURRENT_TEST_A
#if MASTER_STRONG_TORQUE_TEST_ENABLED
#define MASTER_FIXED_CURRENT_TEST_A MASTER_STRONG_TORQUE_FIXED_CURRENT_A
#else
#define MASTER_FIXED_CURRENT_TEST_A 0.20f
#endif
#endif

// 墙/越界回推电流方向符号。
//  1：保持当前计算方向。
// -1：只翻转虚拟墙和越界回推方向。
// 注意：如果墙变成“吸向边界”，才改这个；不要用它修正中心阻尼问题。
#ifndef MASTER_HAPTIC_DIRECTION_SIGN
#define MASTER_HAPTIC_DIRECTION_SIGN -1
#endif

// 中心阻尼电流方向符号。
//  1：保持当前中心阻尼方向。
// -1：只翻转中心区阻尼方向。
// 如果中心区域出现“越转越被助推”，只改这个，不要同时改墙方向。
#ifndef MASTER_CENTER_DAMPING_DIRECTION_SIGN
#define MASTER_CENTER_DAMPING_DIRECTION_SIGN -1
#endif

// 安全切断距离，单位 deg。
// 当角度超过 min/max 外侧该距离后，目标电流立即归零，避免持续越界输出。
// 这是安全保护，不是正常墙感的一部分。
#ifndef MASTER_HAPTIC_OVERRUN_CUT_DEG
#define MASTER_HAPTIC_OVERRUN_CUT_DEG 10.0f
#endif

// boundary_hit 状态保持时间，单位 ms。
// 只影响状态显示/遥测保持，不保持真实墙电流。
// 用途：让低频状态输出能看到刚刚触碰过边界。
#ifndef MASTER_HAPTIC_BLOCK_HOLD_MS
#define MASTER_HAPTIC_BLOCK_HOLD_MS 50
#endif

// 中心主动阻尼开关。
// 1：在非墙区根据旋钮速度输出阻尼电流，让旋钮有“粘滞阻尼”手感。
// 0：中心区输出 0A，只保留边界虚拟墙。
#ifndef MASTER_CENTER_DAMPING_ENABLED
#define MASTER_CENTER_DAMPING_ENABLED 1
#endif

// 中心粘滞阻尼增益，单位 A/(deg/s)。
// 速度越大，阻尼电流越大；用于模拟旋钮在油脂中的阻尼感。
#ifndef MASTER_CENTER_DAMPING_GAIN_A_PER_DEG_S
#define MASTER_CENTER_DAMPING_GAIN_A_PER_DEG_S 0.0005f
#endif

// 中心阻尼速度低通时间常数，单位 s。
// 值越小：速度估计更灵敏；值越大：速度更平滑但响应更慢。
#ifndef MASTER_CENTER_DAMPING_VELOCITY_LPF_TF_S
#define MASTER_CENTER_DAMPING_VELOCITY_LPF_TF_S 0.005f
#endif

// 静止附近的速度快速衰减时间常数，单位 s。
// 当原始速度接近 0 时，让滤波速度更快回到 0，避免松手后仍残留阻尼电流。
#ifndef MASTER_CENTER_DAMPING_STILL_LPF_TF_S
#define MASTER_CENTER_DAMPING_STILL_LPF_TF_S 0.0005f
#endif

// 中心阻尼速度死区，单位 deg/s。
// 低于该速度不输出中心阻尼，防止编码器微小抖动变成手感嗡振。
#ifndef MASTER_CENTER_DAMPING_DEADBAND_DEG_S
#define MASTER_CENTER_DAMPING_DEADBAND_DEG_S 4.0f
#endif

// 中心阻尼完全介入速度，单位 deg/s。
// 在 deadband 和 full_speed 之间渐入阻尼，避免刚起转时电流阶跃。
#ifndef MASTER_CENTER_DAMPING_FULL_SPEED_DEG_S
#define MASTER_CENTER_DAMPING_FULL_SPEED_DEG_S 20.0f
#endif

// 库仑阻尼电流，单位 A。
// 低速时通过 tanh 平滑接近该摩擦感，用于提供轻微静摩擦/干摩擦手感。
#ifndef MASTER_CENTER_DAMPING_COULOMB_A
#define MASTER_CENTER_DAMPING_COULOMB_A 0.008f
#endif

// 库仑阻尼 tanh 速度尺度，单位 deg/s。
// 值越小，越低速度就接近库仑阻尼平台；值越大，低速手感更柔和。
#ifndef MASTER_CENTER_DAMPING_VEL_SCALE_DEG_S
#define MASTER_CENTER_DAMPING_VEL_SCALE_DEG_S 12.0f
#endif

// 中心阻尼电流限幅，单位 A。
// 防止中心阻尼过大抢过虚拟墙手感，也降低持续转动时的发热。
#ifndef MASTER_CENTER_DAMPING_LIMIT_A
#define MASTER_CENTER_DAMPING_LIMIT_A 0.050f
#endif

