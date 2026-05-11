#include "master/master_tasks.h"

#include <Arduino.h>

#include "master/master_config.h"
#include "master/master_control.h"
#include "master/master_status.h"
#include "master/master_transport.h"

// 主机任务编排模块。
// 这里决定“什么任务跑在哪个核心、以什么节拍跑”。业务逻辑仍在 control、
// transport 和 status 模块中，避免 task loop 自身变成难以审查的大函数。

namespace {

void task_control_loop(void *pvParameters) {
    (void)pvParameters;

    // next_us 是 100 us 子步的目标时间；previous_us 用来计算真实 dt。
    // last_wake 用于每 1 ms tick 后把任务重新对齐到 FreeRTOS 节拍。
    uint32_t next_us = micros();
    uint32_t previous_us = next_us;
    TickType_t last_wake = xTaskGetTickCount();

    while (true) {
        // sdkconfig.defaults 将 FreeRTOS tick 固定为 1 ms。
        // 每个 tick 内跑 10 个 100 us 控制子步，形成约 10 kHz 控制频率。
        for (uint32_t i = 0; i < MASTER_CONTROL_STEPS_PER_TICK; ++i) {
            // 等待下一个子步时间点。这里用 taskYIELD 而不是阻塞延时，
            // 因为子步只有 100 us，普通 tick 延时粒度太粗。
            while (static_cast<int32_t>(micros() - next_us) < 0) {
                taskYIELD();
            }
            const uint32_t now_us = micros();
            const float dt_s = static_cast<float>(now_us - previous_us) * 0.000001f;
            previous_us = now_us;
            next_us += CONTROL_LOOP_PERIOD_US;

            // 10 kHz 热路径入口：只做角度读取、力反馈计算、状态标量更新和电机输出。
            runMasterControlStep(dt_s);
        }
        // 每批子步结束后让出到下一个 1 ms tick，避免控制任务永久占满核心。
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1));
    }
}

void task_comm_loop(void *pvParameters) {
    (void)pvParameters;

    // seq 是主机命令包序号，从机用它拒绝旧包并回传 ack_seq。
    uint32_t seq = 0;

    while (true) {
        // ESP-NOW 发送在 Core 0 低频任务里执行，不进入控制热路径。
        sendMasterCommand(seq++, micros());
        vTaskDelay(pdMS_TO_TICKS(COMM_LOOP_PERIOD_MS));
    }
}

void task_status_loop(void *pvParameters) {
    (void)pvParameters;

    while (true) {
        // 串口打印限频到 STATUS_LOOP_PERIOD_MS，减少对无线和控制节拍的影响。
        printMasterStatusLine();
        vTaskDelay(pdMS_TO_TICKS(STATUS_LOOP_PERIOD_MS));
    }
}

}  // namespace

void startMasterTasks() {
    // 先创建 Core 0 通信和状态任务，再创建 Core 1 控制任务。
    // 控制任务优先级最高，即便创建顺序在后，也会按 FreeRTOS 调度抢占执行。
    xTaskCreatePinnedToCore(task_comm_loop,
                            "MasterComm",
                            MASTER_COMM_TASK_STACK_BYTES,
                            NULL,
                            MASTER_COMM_TASK_PRIORITY,
                            NULL,
                            MASTER_IO_CORE);
    xTaskCreatePinnedToCore(task_status_loop,
                            "MasterStatus",
                            MASTER_STATUS_TASK_STACK_BYTES,
                            NULL,
                            MASTER_STATUS_TASK_PRIORITY,
                            NULL,
                            MASTER_IO_CORE);
    xTaskCreatePinnedToCore(task_control_loop,
                            "MasterControl",
                            MASTER_CONTROL_TASK_STACK_BYTES,
                            NULL,
                            MASTER_CONTROL_TASK_PRIORITY,
                            NULL,
                            MASTER_CONTROL_CORE);
}
