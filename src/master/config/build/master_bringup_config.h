#pragma once

#include <stdint.h>

// == 通信功能 =================================================================

// 启用 ESP-NOW 主机通信。
// 0：不初始化 ESP-NOW；1：初始化 ESP-NOW 并发送主机命令。
#ifndef MASTER_ENABLE_ESPNOW
#define MASTER_ENABLE_ESPNOW 1
#endif

// 启用后续 BLE 模式入口。
// 0：BLE 模式不可用；1：允许 BLE 模式参与运行时切换。
#ifndef MASTER_ENABLE_BLE
#define MASTER_ENABLE_BLE 0
#endif

// 启用自动绘图模式。
// 0：AutoDraw 不可进入；1：允许 AutoDraw 发送轨迹命令。
#ifndef MASTER_ENABLE_AUTO_DRAW
#define MASTER_ENABLE_AUTO_DRAW MASTER_ENABLE_ESPNOW
#endif

// == 运行能力模式 =============================================================
//
// 说明：
// - MASTER_RUN_MODE 是主机唯一硬件路径选择入口，同时派生默认控制周期。
// - *_HW 开关只表示编译期允许对象存在，不能单独决定初始化哪条轴路径。
// - 模式名中的频率必须与 MASTER_CONTROL_LOOP_PERIOD_US 保持一致。

// SingleX_10kHz：单 X 轴旋钮压测，初始化 X 编码器 + X 电机，控制周期 100us。
#define MASTER_MODE_SINGLE_X_10KHZ_ID 0
// SingleY_10kHz：单 Y 轴旋钮压测，初始化 Y 编码器 + Y 电机，控制周期 100us。
#define MASTER_MODE_SINGLE_Y_10KHZ_ID 1
// DualXY_5kHz：双轴手动/回驱验收路径，初始化 X/Y 编码器 + 电机，控制周期 200us。
#define MASTER_MODE_DUAL_XY_5KHZ_ID 2

// 功能说明：选择主机 run mode、硬件初始化路径和默认控制周期。
// 默认：SingleX_10kHz；切 SingleY/DualXY 时需同步打开对应 Y 轴硬件开关。
#ifndef MASTER_RUN_MODE
#define MASTER_RUN_MODE MASTER_MODE_DUAL_XY_5KHZ_ID
#endif

// == 轴硬件开关 ===============================================================

// 启用 X 轴旋钮电机硬件输出。
// 0：不编译/初始化 X 电机驱动；1：允许 X 电机对象和输出路径存在。
#ifndef MASTER_ENABLE_X_MOTOR_HW
#define MASTER_ENABLE_X_MOTOR_HW 1
#endif

// 启用 Y 轴旋钮电机硬件输出。
// 0：不编译/初始化 Y 电机驱动；1：允许 Y 电机对象和输出路径存在。
#ifndef MASTER_ENABLE_Y_MOTOR_HW
#define MASTER_ENABLE_Y_MOTOR_HW 1
#endif

// 启用 X 轴 MT6701 编码器。
// 0：不编译/初始化 X 编码器；1：允许 X 编码器对象和读数路径存在。
#ifndef MASTER_ENABLE_X_ENCODER_HW
#define MASTER_ENABLE_X_ENCODER_HW 1
#endif

// 启用 Y 轴 MT6701 编码器。
// 0：不编译/初始化 Y 编码器；1：允许 Y 编码器对象和读数路径存在。
#ifndef MASTER_ENABLE_Y_ENCODER_HW
#define MASTER_ENABLE_Y_ENCODER_HW 1
#endif

// 派生的任意电机硬件开关。
// 0：无电机硬件输出；1：至少一个电机硬件输出启用。
#ifndef MASTER_ENABLE_MOTOR_HW
#define MASTER_ENABLE_MOTOR_HW (MASTER_ENABLE_X_MOTOR_HW || MASTER_ENABLE_Y_MOTOR_HW)
#endif

// 双轴 initFOC 初始化顺序。
// 0：X initFOC first，Y initFOC second；1：Y initFOC first，X initFOC second。
#ifndef MASTER_INIT_FOC_Y_FIRST
#define MASTER_INIT_FOC_Y_FIRST 1
#endif

// == 触觉与电流环 =============================================================

// 启用 DengFoc 高侧电流采样。
// 0：使用电压模式 fallback；1：使用电流采样和电流环。
#ifndef MASTER_ENABLE_CURRENT_SENSE
#define MASTER_ENABLE_CURRENT_SENSE 1
#endif

// 启用主机旋钮力反馈计算。
// 0：不输出触觉目标电流；1：运行触觉算法并输出限幅电流。
#ifndef MASTER_ENABLE_FORCE_FEEDBACK
#define MASTER_ENABLE_FORCE_FEEDBACK 1
#endif

// 启用中心区速度阻尼。
// 0：中心阻尼不产生电流；1：按配置产生速度阻尼电流。
#ifndef MASTER_ENABLE_CENTER_DAMPING
#define MASTER_ENABLE_CENTER_DAMPING 1
#endif

// 启用纸面边界墙触觉。
// 0：墙区不输出回推电流；1：墙区按深度和阻尼输出回推电流。
#ifndef MASTER_ENABLE_PAPER_WALL_HAPTIC
#define MASTER_ENABLE_PAPER_WALL_HAPTIC 1
#endif

// == 上电功能测试 =============================================================

// 固定电流输出测试。
// 0：关闭固定电流测试；1：忽略正常触觉算法并输出固定测试电流。
#ifndef MASTER_ENABLE_FIXED_CURRENT_TEST
#define MASTER_ENABLE_FIXED_CURRENT_TEST 0
#endif

// 强力矩参数测试。
// 0：使用常规电流限幅；1：使用强力矩 preset，必须依赖电流采样。
#ifndef MASTER_ENABLE_STRONG_TORQUE_TEST
#define MASTER_ENABLE_STRONG_TORQUE_TEST 1
#endif

// 电机相序扫描测试。
// 0：关闭相序扫描；1：进入相序扫描诊断路径。
#ifndef MASTER_ENABLE_PHASE_SCAN_TEST
#define MASTER_ENABLE_PHASE_SCAN_TEST 0
#endif

// 零电流输出链路测试。
// 0：关闭零电流测试；1：强制目标电流为 0A。
#ifndef MASTER_ENABLE_ZERO_CURRENT_TEST
#define MASTER_ENABLE_ZERO_CURRENT_TEST 0
#endif

// 零电流 DC 诊断测试。
// 0：关闭 DC 诊断；1：仅在零电流测试打开时允许执行。
#ifndef MASTER_ENABLE_ZERO_CURRENT_DC_TEST
#define MASTER_ENABLE_ZERO_CURRENT_DC_TEST 0
#endif

// 强制落笔命令测试。
// 0：由模式逻辑控制落笔；1：启动后强制发送落笔语义。
#ifndef MASTER_ENABLE_FORCE_PEN_DOWN_TEST
#define MASTER_ENABLE_FORCE_PEN_DOWN_TEST 0
#endif

// == 诊断日志 =================================================================

// 启用低频状态日志。
// 0：不输出状态日志；1：输出主机状态和通信摘要。
#ifndef MASTER_ENABLE_STATUS_LOG
#define MASTER_ENABLE_STATUS_LOG 1
#endif

// 启用控制周期耗时日志。
// 0：不记录耗时细节；1：记录控制环和电机环耗时。
#ifndef MASTER_ENABLE_TIMING_LOG
#define MASTER_ENABLE_TIMING_LOG 0
#endif

// == 启动应用模式 =============================================================
//
// 说明：
// - MASTER_STARTUP_APP_MODE 只决定上电后的默认业务入口，不决定硬件路径。
// - 硬件路径只由 MASTER_RUN_MODE 决定；运行时切换仍受对应 ENABLE 开关限制。

// ManualDraw：读取当前 run mode 允许的旋钮轴，发送 MODE_COLLAB_DRAW。
#define MASTER_STARTUP_APP_MANUAL_DRAW_ID 0
// AutoDraw：发送 dry-run 轨迹命令，要求 MASTER_ENABLE_AUTO_DRAW=1 和 ESP-NOW。
#define MASTER_STARTUP_APP_AUTO_DRAW_ID 1
// Diagnostics：进入诊断/干跑语义，不作为常规绘图验收入口。
#define MASTER_STARTUP_APP_DIAGNOSTICS_ID 2

// 功能说明：选择上电后的默认应用模式。
// 0：手动绘图；1：自动绘图 dry-run；2：诊断模式。
#ifndef MASTER_STARTUP_APP_MODE
#define MASTER_STARTUP_APP_MODE MASTER_STARTUP_APP_MANUAL_DRAW_ID
#endif
