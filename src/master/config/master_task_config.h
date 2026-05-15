#pragma once

#include <Arduino.h>

// 当前阶段主机控制周期为 125us / 8kHz。
// 该值是主机本地任务节拍，不属于主从 common 协议契约。
// 如果后续从 8kHz 改回 10kHz，应同步评估 sensor_spi、current_sense、loopFOC 总耗时。
static constexpr uint32_t MASTER_CONTROL_LOOP_PERIOD_US = 125UL;
// 兼容旧代码中使用 CONTROL_LOOP_PERIOD_US 的位置，保持等价迁移。
static constexpr uint32_t CONTROL_LOOP_PERIOD_US = MASTER_CONTROL_LOOP_PERIOD_US;

// 控制任务绑核：Core 1 专门跑高频控制热路径。
static constexpr BaseType_t MASTER_CONTROL_CORE = 1;
// IO 任务绑核：Core 0 放 ESP-NOW、串口状态输出和后续 BLE/屏幕任务。
static constexpr BaseType_t MASTER_IO_CORE = 0;

// 控制任务优先级：最高优先级，保证 FOC、角度读取和力反馈按周期执行。
static constexpr UBaseType_t MASTER_CONTROL_TASK_PRIORITY = configMAX_PRIORITIES - 1;
// 通信任务优先级：低于控制任务，负责 ESP-NOW 收发和包快照处理。
static constexpr UBaseType_t MASTER_COMM_TASK_PRIORITY = 3;
// 状态任务优先级：最低，负责串口低频打印，不能影响控制。
static constexpr UBaseType_t MASTER_STATUS_TASK_PRIORITY = 1;

// 控制任务栈大小，单位 byte；需要容纳 SimpleFOC 调用、控制状态和局部变量。
static constexpr uint32_t MASTER_CONTROL_TASK_STACK_BYTES = 8192;
// 通信任务栈大小，单位 byte；需要容纳 Wi-Fi/ESP-NOW 回调和包处理。
static constexpr uint32_t MASTER_COMM_TASK_STACK_BYTES = 4096;
// 状态任务栈大小，单位 byte；需要容纳串口格式化输出。
static constexpr uint32_t MASTER_STATUS_TASK_STACK_BYTES = 4096;

// 控制定时器周期，等于主机控制周期。
// 控制任务应在每次通知后执行一次控制步，然后继续等待下一次通知。
static constexpr uint32_t MASTER_CONTROL_TIMER_PERIOD_US = MASTER_CONTROL_LOOP_PERIOD_US;
// 控制定时器失联保护，单位 ms。
// 超过该时间未收到控制通知时，应主动输出 0A 并锁存故障。
static constexpr uint32_t MASTER_CONTROL_TIMER_TIMEOUT_MS = 10UL;

