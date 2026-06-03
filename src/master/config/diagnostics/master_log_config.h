#pragma once

#include <stdint.h>

#include "master/config/build/master_bringup_config.h"

// 主机日志和 timing 诊断配置。
// 日志只允许在启动/Core0状态任务；timing level 会影响热路径采样开销。

// 启动配置日志开关。
// 打印启动模式、轴模式、硬件开关、周期和限制。
#ifndef MASTER_BOOT_LOG_ENABLED
#define MASTER_BOOT_LOG_ENABLED 1
#endif

// ESP-NOW 身份日志开关。
// 打印本机 STA MAC、对端 MAC 和固定信道。
#ifndef MASTER_ESPNOW_IDENTITY_LOG_ENABLED
#define MASTER_ESPNOW_IDENTITY_LOG_ENABLED MASTER_BOOT_LOG_ENABLED
#endif

// 控制定时器启动日志开关。
// 只控制成功启动提示；失败路径仍会打印错误。
#ifndef MASTER_CONTROL_TIMER_LOG_ENABLED
#define MASTER_CONTROL_TIMER_LOG_ENABLED MASTER_BOOT_LOG_ENABLED
#endif

// 低频状态日志总开关。
// 只影响 Core0 状态任务，不进入控制热路径。
#ifndef MASTER_STATUS_LOG_ENABLED
#define MASTER_STATUS_LOG_ENABLED MASTER_ENABLE_STATUS_LOG
#endif

// 状态任务打印周期，单位 ms。
// 设得过小会增加 Core0 串口格式化压力。
#ifndef MASTER_STATUS_LOOP_PERIOD_MS
#define MASTER_STATUS_LOOP_PERIOD_MS 500UL
#endif

// 摘要状态行开关。
// 输出模式、tx/ack、位置、电流、pen、链路计数和 fault。
#ifndef MASTER_STATUS_SUMMARY_LOG_ENABLED
#define MASTER_STATUS_SUMMARY_LOG_ENABLED 1
#endif

// 同步误差详情行开关。
// 输出编码器、协议坐标、从机遥测和 XY 同步误差。
#ifndef MASTER_STATUS_SYNC_LOG_ENABLED
#define MASTER_STATUS_SYNC_LOG_ENABLED 0
#endif

// timing 诊断等级：0 关闭，1 整步统计，2 分段统计。
// level 2 会增加热路径采样点，建议只短时间定位性能问题。
#ifndef MASTER_TIMING_DIAG_LEVEL
#define MASTER_TIMING_DIAG_LEVEL 2
#endif

// 整步健康统计开关。
// 记录 step_us、step_max 和 over 阈值计数。
#define MASTER_TIMING_STEP_DIAG_ENABLED (MASTER_TIMING_DIAG_LEVEL >= 1)

// 分段 timing 统计开关。
// 记录 logic、motor、move、loopFOC、current、sensor 等细项。
#define MASTER_TIMING_DETAIL_DIAG_ENABLED (MASTER_TIMING_DIAG_LEVEL >= 2)

// timing 健康行开关。
// 只关闭串口输出；采样开关由 MASTER_TIMING_DIAG_LEVEL 决定。
#ifndef MASTER_STATUS_TIMING_LOG_ENABLED
#define MASTER_STATUS_TIMING_LOG_ENABLED MASTER_TIMING_STEP_DIAG_ENABLED
#endif

// timing 分段详情行开关。
// level=2 默认打印详情行；可单独置 0 只保留采样不开串口输出。
#ifndef MASTER_STATUS_TIMING_DETAIL_LOG_ENABLED
#define MASTER_STATUS_TIMING_DETAIL_LOG_ENABLED MASTER_TIMING_DETAIL_DIAG_ENABLED
#endif

// SimpleFOC 启动诊断日志开关。
// 只允许用于 init/initFOC 启动路径。
#ifndef MASTER_SIMPLEFOC_DEBUG_ENABLED
#define MASTER_SIMPLEFOC_DEBUG_ENABLED 1
#endif
