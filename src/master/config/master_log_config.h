#pragma once

#include <stdint.h>

// master_log_config
// 职责：集中管理主机日志、状态输出和 timing 诊断。
// 注意：本文件只控制“观察与诊断”，原则上不改变主机控制语义。
// 真正会改变运行行为的开关，应放在 master_build_options.h 或对应硬件配置文件中。
//
// 配置分层原则：
//   MASTER_TIMING_DIAG_LEVEL 控制“采不采 timing、采多细”，会影响 5kHz 热路径开销。
//   MASTER_STATUS_*_LOG_ENABLED 控制“打印哪些已有状态行”，主要影响 Core 0 串口输出。

// 主机启动配置日志开关。
// 默认值：1，上电时打印模式、硬件开关、ESP-NOW 和控制周期。
// 用途：现场确认主机是否处于 SingleX、force feedback、ESP-NOW 等预期状态。
// 风险：只在启动路径打印；关闭后不影响控制和通信。
#ifndef MASTER_BOOT_LOG_ENABLED
#define MASTER_BOOT_LOG_ENABLED 1
#endif

// 主机 ESP-NOW 身份日志开关。
// 默认值：跟随 MASTER_BOOT_LOG_ENABLED，上电时打印本机 STA MAC、对端 MAC 和固定信道。
// 用途：确认两块板是否烧录反、MAC 是否写错、主从信道是否一致。
// 风险：只在启动路径打印；关闭后不会影响 ESP-NOW 初始化和收发。
#ifndef MASTER_ESPNOW_IDENTITY_LOG_ENABLED
#define MASTER_ESPNOW_IDENTITY_LOG_ENABLED MASTER_BOOT_LOG_ENABLED
#endif

// 主机控制定时器启动日志开关。
// 默认值：跟随 MASTER_BOOT_LOG_ENABLED，成功启动时打印周期和 dispatch 模式。
// 用途：确认 200us 控制定时器是否按预期启用。
// 风险：只控制成功启动提示；创建失败和启动失败仍会打印错误。
#ifndef MASTER_CONTROL_TIMER_LOG_ENABLED
#define MASTER_CONTROL_TIMER_LOG_ENABLED MASTER_BOOT_LOG_ENABLED
#endif

// 主机低频状态日志总开关。
// 默认值：1，创建状态打印任务。
// 用途：低频观察命令、遥测、同步误差和 fault。
// 风险：串口输出只能在 Core 0 状态任务中运行，不能进入控制热路径。
#ifndef MASTER_STATUS_LOG_ENABLED
#define MASTER_STATUS_LOG_ENABLED 1
#endif

// 主机状态任务打印周期。
// 默认值：500ms，约 2Hz。
// 用途：统一控制主机运行期 Serial.printf 频率，不影响 ESP-NOW 命令发送周期。
// 风险：设得过小会增加 Core 0 串口格式化压力，可能间接影响 ESP-NOW 通信稳定性。
#ifndef MASTER_STATUS_LOOP_PERIOD_MS
#define MASTER_STATUS_LOOP_PERIOD_MS 500UL
#endif

static_assert(MASTER_STATUS_LOOP_PERIOD_MS > 0,
              "MASTER_STATUS_LOOP_PERIOD_MS must be greater than 0");

// 主机摘要状态行开关。
// 默认值：1，输出模式、tx/ack、主机位置、pen、链路计数和 fault。
// 用途：常规联调时保留最短可读摘要。
#ifndef MASTER_STATUS_SUMMARY_LOG_ENABLED
#define MASTER_STATUS_SUMMARY_LOG_ENABLED 1
#endif

// 主机同步误差详情行开关。
// 默认值：1，输出编码器诊断、协议归一化坐标、从机遥测位置和 XY 同步误差。
// 用途：联调 X/Y 同步时观察实际误差；性能收口时可临时关闭，只保留摘要行。
#ifndef MASTER_STATUS_SYNC_LOG_ENABLED
#define MASTER_STATUS_SYNC_LOG_ENABLED 1
#endif

// 主机 timing 诊断等级。
// level 0：关闭 timing 采样和统计，用于观察最真实控制负载。
// level 1：只保留整步统计：step_us / step_max / over_period / over_75pct / over_50pct。
// level 2：完整分段诊断：logic / motor / move / loopFOC / current / sensor。
// 默认值：1。日常 5kHz 联调用 level 1；定位热路径开销时短时间切 level 2。
// 把默认诊断误提升到 level 2。需要改变诊断强度时，统一配置 MASTER_TIMING_DIAG_LEVEL。
#ifndef MASTER_TIMING_DIAG_LEVEL
#define MASTER_TIMING_DIAG_LEVEL 1
#endif

static_assert(MASTER_TIMING_DIAG_LEVEL >= 0 && MASTER_TIMING_DIAG_LEVEL <= 2,
              "MASTER_TIMING_DIAG_LEVEL must be 0, 1, or 2");

// 整步健康统计：只统计 control step 总耗时和 over 阈值。
#define MASTER_TIMING_STEP_DIAG_ENABLED (MASTER_TIMING_DIAG_LEVEL >= 1)

// 分段 timing 统计：记录 logic / motor / move / loopFOC / current 等细项。
#define MASTER_TIMING_DETAIL_DIAG_ENABLED (MASTER_TIMING_DIAG_LEVEL >= 2)

// 兼容旧开关：MASTER_TIMING_DIAG_ENABLED。
// 新代码统一用 MASTER_TIMING_DIAG_LEVEL；旧代码只读取该派生宏，不再用它配置 level。
#undef MASTER_TIMING_DIAG_ENABLED
#define MASTER_TIMING_DIAG_ENABLED (MASTER_TIMING_DIAG_LEVEL > 0)

// 兼容旧代码中的控制 timing 宏。
// 后续应逐步把代码里的 MASTER_CONTROL_TIMING_DIAG_ENABLED 替换成 MASTER_TIMING_DETAIL_DIAG_ENABLED。
#undef MASTER_CONTROL_TIMING_DIAG_ENABLED
#define MASTER_CONTROL_TIMING_DIAG_ENABLED MASTER_TIMING_DETAIL_DIAG_ENABLED

// 兼容旧代码中的控制健康宏。
// 后续应逐步把代码里的 MASTER_CONTROL_HEALTH_DIAG_ENABLED 替换成 MASTER_TIMING_STEP_DIAG_ENABLED。
#undef MASTER_CONTROL_HEALTH_DIAG_ENABLED
#define MASTER_CONTROL_HEALTH_DIAG_ENABLED MASTER_TIMING_STEP_DIAG_ENABLED

// 主机 timing 健康行开关。
// 默认值：跟随 MASTER_TIMING_STEP_DIAG_ENABLED。
// 用途：只在 level 1/2 有整步 timing 数据时默认打印 ctrl_dt / step_us / over / miss。
// 注意：设为 0 只关闭串口输出，不关闭 timing 采样；关闭采样应设置 MASTER_TIMING_DIAG_LEVEL=0。
#ifndef MASTER_STATUS_TIMING_LOG_ENABLED
#define MASTER_STATUS_TIMING_LOG_ENABLED MASTER_TIMING_STEP_DIAG_ENABLED
#endif

// 主机 timing 分段详情行开关。
// 默认值：0，避免默认串口输出过长。
// 用途：配合 MASTER_TIMING_DIAG_LEVEL=2 输出 control/motor/current/sensor 分段耗时。
// 注意：即使 level=2，也需要手动设为 1 才打印详情，避免 Core 0 串口输出过重。
#ifndef MASTER_STATUS_TIMING_DETAIL_LOG_ENABLED
#define MASTER_STATUS_TIMING_DETAIL_LOG_ENABLED 0
#endif

// SimpleFOC 启动诊断日志开关。
// 默认值：1，只输出 init/initFOC 阶段日志。
// 注意：只允许出现在启动路径，不能进入高频控制热路径。
#ifndef MASTER_SIMPLEFOC_DEBUG_ENABLED
#define MASTER_SIMPLEFOC_DEBUG_ENABLED 1
#endif
