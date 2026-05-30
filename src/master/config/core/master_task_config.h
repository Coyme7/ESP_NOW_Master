#pragma once

#include <Arduino.h>

#include "master/config/core/master_control_config.h"

// FreeRTOS 任务绑核。
// Core 1 跑控制热路径；Core 0 放 ESP-NOW、串口状态和后续 UI/BLE。
static constexpr BaseType_t MASTER_CONTROL_CORE = 1;
static constexpr BaseType_t MASTER_IO_CORE = 0;

// FreeRTOS 任务优先级。
// 控制任务最高，通信低于控制，状态输出最低。
static constexpr UBaseType_t MASTER_CONTROL_TASK_PRIORITY = configMAX_PRIORITIES - 1;
static constexpr UBaseType_t MASTER_COMM_TASK_PRIORITY = 3;
static constexpr UBaseType_t MASTER_STATUS_TASK_PRIORITY = 1;

// 任务栈大小，单位 byte。
// 控制栈覆盖 SimpleFOC 调用；通信和状态栈覆盖回调、包处理和串口格式化。
static constexpr uint32_t MASTER_CONTROL_TASK_STACK_BYTES = 8192;
static constexpr uint32_t MASTER_COMM_TASK_STACK_BYTES = 4096;
static constexpr uint32_t MASTER_STATUS_TASK_STACK_BYTES = 4096;

// 控制定时器配置。
// 超时后控制任务应主动输出 0A 并锁存故障。
static constexpr uint32_t MASTER_CONTROL_TIMER_PERIOD_US = MASTER_CONTROL_LOOP_PERIOD_US;
static constexpr uint32_t MASTER_CONTROL_TIMER_TIMEOUT_MS = 10UL;
