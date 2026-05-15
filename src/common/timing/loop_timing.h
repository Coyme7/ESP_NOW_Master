// ============================================================================
// 文件：common/timing/loop_timing.h
// 作用：主从共享低频时序
// 学习要点：只放通信超时、通信任务周期、状态输出周期；高频控制周期放在各自 task config。
// ============================================================================

// 学习补充：
// - 这里放主从共同理解的低频时序，例如通信超时和状态打印周期。
// - 高频控制周期不放在 common，因为主机力反馈控制和从机执行控制可能独立调参。
// - 超时时间需要明显大于通信任务周期，否则偶发调度抖动会被误判为断链。


#pragma once

#include <stdint.h>

// 主从一致的通信和状态周期。控制周期属于各节点本地 task config。
static constexpr uint32_t COMMAND_TIMEOUT_US = 50000UL;
static constexpr uint32_t TELEMETRY_TIMEOUT_US = 50000UL;
static constexpr uint32_t COMM_LOOP_PERIOD_MS = 10UL;
static constexpr uint32_t STATUS_LOOP_PERIOD_MS = 100UL;

