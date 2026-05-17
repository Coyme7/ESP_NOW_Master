#include "master/tasks/master_tasks.h"

#include <Arduino.h>
#include <esp_err.h>
#include <esp_timer.h>

#include "common/system_state.h"
#include "master/config/master_config.h"
#include "master/control/master_control.h"
#include "master/hardware/master_motor_hw.h"
#include "master/status/master_status.h"
#include "master/comm/master_transport.h"

namespace {

TaskHandle_t controlTaskHandle = nullptr;
esp_timer_handle_t controlTimerHandle = nullptr;

volatile uint32_t controlTimerMissedTicks = 0;
volatile uint32_t controlTimerLastDtUs = 0;
volatile uint32_t controlTimerMaxDtUs = 0;
volatile uint32_t controlTimerOver150Count = 0;
volatile uint32_t controlTimerOver200Count = 0;
volatile uint32_t controlStepLastUs = 0;
volatile uint32_t controlStepMaxUs = 0;
volatile uint32_t controlStepOverPeriodCount = 0;
volatile uint32_t controlStepOver75PctCount = 0;
volatile uint32_t controlStepOver50PctCount = 0;

const uint32_t warn_dt_us = (MASTER_CONTROL_LOOP_PERIOD_US * 3U) / 2U;
const uint32_t miss_dt_us = MASTER_CONTROL_LOOP_PERIOD_US * 2U;
const uint32_t step_over_period_us = MASTER_CONTROL_LOOP_PERIOD_US;
const uint32_t step_over_75pct_us = (MASTER_CONTROL_LOOP_PERIOD_US * 3U) / 4U;
const uint32_t step_over_50pct_us = MASTER_CONTROL_LOOP_PERIOD_US / 2U;

// 时序统计累加器：记录次数、总耗时、最近耗时和最大耗时。
struct TimingAccumulator {
    volatile uint32_t count;
    volatile uint64_t sum_us;
    volatile uint32_t last_us;
    volatile uint32_t max_us;
};

TimingAccumulator timingControlTotal = {};
TimingAccumulator timingControlLogic = {};
TimingAccumulator timingMotorTotal = {};
TimingAccumulator timingMotorMove = {};
TimingAccumulator timingMotorLoopFoc = {};
TimingAccumulator timingCurrentSense = {};
TimingAccumulator timingSensorSpi = {};

// 记录单个阶段耗时；关闭诊断宏时会被编译成空操作。
void recordTiming(TimingAccumulator &accumulator, uint32_t duration_us) {
#if MASTER_CONTROL_TIMING_DIAG_ENABLED
    accumulator.last_us = duration_us;
    accumulator.sum_us += duration_us;
    accumulator.count += 1;
    if (duration_us > accumulator.max_us) {
        accumulator.max_us = duration_us;
    }
#else
    (void)accumulator;
    (void)duration_us;
#endif
}

#if MASTER_CONTROL_TIMING_DIAG_ENABLED
// 取出一个统计窗口的 last/avg/max，并清空累计值开始下一窗口。
MasterTimingStats snapshotTiming(TimingAccumulator &accumulator) {
    const uint32_t count = accumulator.count;
    const uint64_t sum_us = accumulator.sum_us;
    const uint32_t last_us = accumulator.last_us;
    uint32_t max_us = accumulator.max_us;
    if (max_us < last_us) {
        max_us = last_us;
    }

    MasterTimingStats stats = {};
    stats.count = count;
    stats.last_us = last_us;
    stats.avg_us = (count > 0) ? static_cast<uint32_t>(sum_us / count) : 0;
    stats.max_us = max_us;

    accumulator.count = 0;
    accumulator.sum_us = 0;
    accumulator.max_us = 0;
    return stats;
}
#endif

// 控制定时器回调：只通知控制任务，不直接运行控制逻辑，避免 ISR 过重。
void IRAM_ATTR controlTimerCallback(void *arg) {
    (void)arg;

    TaskHandle_t handle = controlTaskHandle;
    if (handle == nullptr) {
        return;
    }

#ifdef CONFIG_ESP_TIMER_SUPPORTS_ISR_DISPATCH_METHOD
    BaseType_t high_task_woken = pdFALSE;
    vTaskNotifyGiveFromISR(handle, &high_task_woken);
    if (high_task_woken == pdTRUE) {
        esp_timer_isr_dispatch_need_yield();
    }
#else
    xTaskNotifyGive(handle);
#endif
}

// 创建并启动周期定时器，为控制任务提供 200us 节拍。
bool startControlTimer() {
    if (controlTimerHandle != nullptr) {
        return true;
    }

    esp_timer_create_args_t timer_args = {};
    timer_args.callback = controlTimerCallback;
    timer_args.arg = nullptr;
#ifdef CONFIG_ESP_TIMER_SUPPORTS_ISR_DISPATCH_METHOD
    timer_args.dispatch_method = ESP_TIMER_ISR;
    const char *dispatch_name = "isr";
#else
    timer_args.dispatch_method = ESP_TIMER_TASK;
    const char *dispatch_name = "task";
#endif
    timer_args.name = "MasterCtrl5k";
    timer_args.skip_unhandled_events = true;

    esp_err_t err = esp_timer_create(&timer_args, &controlTimerHandle);
    if (err != ESP_OK) {
        Serial.printf("[Master] control_timer create failed: %s\n", esp_err_to_name(err));
        return false;
    }

    err = esp_timer_start_periodic(controlTimerHandle, MASTER_CONTROL_TIMER_PERIOD_US);
    if (err != ESP_OK) {
        Serial.printf("[Master] control_timer start failed: %s\n", esp_err_to_name(err));
        esp_timer_delete(controlTimerHandle);
        controlTimerHandle = nullptr;
        return false;
    }

#if MASTER_CONTROL_TIMER_LOG_ENABLED
    Serial.printf("[Master] control_timer started period=%luus dispatch=%s\n",
                  static_cast<unsigned long>(MASTER_CONTROL_TIMER_PERIOD_US),
                  dispatch_name);
#else
    (void)dispatch_name;
#endif
    return true;
}

// 最高优先级控制任务：等待定时器通知，计算 dt，执行单步控制。
void task_control_loop(void *pvParameters) {
    (void)pvParameters;

    controlTaskHandle = xTaskGetCurrentTaskHandle();
    uint32_t previous_us = micros();

    if (!startControlTimer()) {
        addLocalFault(FAULT_MOTOR_OUTPUT_DISABLED);
        while (true) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    while (true) {
        // pending_ticks 表示两次调度之间积压的定时器通知数量，可用于判断控制任务是否漏周期。
        const uint32_t pending_ticks =
            ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(MASTER_CONTROL_TIMER_TIMEOUT_MS));
        if (pending_ticks == 0) {
            addLocalFault(FAULT_MOTOR_OUTPUT_DISABLED);
            runMasterMotorOutput(0.0f);
            previous_us = micros();
            continue;
        }

#if MASTER_TIMING_STEP_DIAG_ENABLED
        if (pending_ticks > 1) {
            controlTimerMissedTicks += pending_ticks - 1;
        }
#endif

        const uint32_t now_us = micros();
        // dt 使用实际到达时间计算，而不是固定 200us，便于在偶发抖动时让滤波/斜率按真实时间工作。
        const uint32_t dt_us = now_us - previous_us;
        previous_us = now_us;

#if MASTER_TIMING_STEP_DIAG_ENABLED
        controlTimerLastDtUs = dt_us;
        if (dt_us > controlTimerMaxDtUs) {
            controlTimerMaxDtUs = dt_us;
        }
        if (dt_us > warn_dt_us) {
            controlTimerOver150Count++;
        }
        if (dt_us > miss_dt_us) {
            controlTimerOver200Count++;
        }
        // level 1 只测完整控制步耗时；level 2 复用同一次采样，同时记录分段耗时。
        const uint32_t control_step_start_us = micros();
#endif
        runMasterControlStep(static_cast<float>(dt_us) * 0.000001f);
#if MASTER_TIMING_STEP_DIAG_ENABLED
        const uint32_t step_us = micros() - control_step_start_us;
        controlStepLastUs = step_us;
        if (step_us > controlStepMaxUs) {
            controlStepMaxUs = step_us;
        }
        if (step_us > step_over_period_us) {
            controlStepOverPeriodCount++;
        }
        if (step_us > step_over_75pct_us) {
            controlStepOver75PctCount++;
        }
        if (step_us > step_over_50pct_us) {
            controlStepOver50PctCount++;
        }
#if MASTER_CONTROL_TIMING_DIAG_ENABLED
        recordMasterTimingControlTotalUs(step_us);
#endif
#endif
    }
}

#if MASTER_ESPNOW_ENABLED
// 通信任务：低频处理遥测并发送主机命令包。
void task_comm_loop(void *pvParameters) {
    (void)pvParameters;

    uint32_t seq = 0;
    TickType_t last_wake = xTaskGetTickCount();
    while (true) {
        processMasterTelemetry();
        sendMasterCommand(seq++, micros());
        // 使用绝对周期发送，避免“处理耗时 + delay”导致 200Hz 发包节奏慢慢漂移。
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(MASTER_COMMAND_PERIOD_MS));
    }
}
#endif

// 状态任务：低频串口打印，不影响控制任务实时性。
void task_status_loop(void *pvParameters) {
    (void)pvParameters;

    while (true) {
        printMasterStatusLine();
        vTaskDelay(pdMS_TO_TICKS(MASTER_STATUS_LOOP_PERIOD_MS));
    }
}

}  // namespace

// 返回累计漏 tick 数，判断控制任务是否被阻塞或抢占。
uint32_t getMasterControlTimerMissedTicks() {
    return controlTimerMissedTicks;
}

uint32_t getMasterControlLastDtUs() {
    return controlTimerLastDtUs;
}

uint32_t getMasterControlMaxDtUs() {
    return controlTimerMaxDtUs;
}

uint32_t getMasterControlOver150Count() {
    return controlTimerOver150Count;
}

uint32_t getMasterControlOver200Count() {
    return controlTimerOver200Count;
}

// 返回控制周期健康快照，并清空窗口统计值。
MasterControlHealthSnapshot getMasterControlHealthSnapshot() {
    MasterControlHealthSnapshot snapshot = {};
    snapshot.diag_level = MASTER_TIMING_DIAG_LEVEL;
#if MASTER_TIMING_STEP_DIAG_ENABLED
    const uint32_t last_dt_us = controlTimerLastDtUs;
    uint32_t max_dt_us = controlTimerMaxDtUs;
    if (max_dt_us < last_dt_us) {
        max_dt_us = last_dt_us;
    }
    const uint32_t last_step_us = controlStepLastUs;
    uint32_t max_step_us = controlStepMaxUs;
    if (max_step_us < last_step_us) {
        max_step_us = last_step_us;
    }

    snapshot.last_dt_us = last_dt_us;
    snapshot.max_dt_us = max_dt_us;
    snapshot.step_us = last_step_us;
    snapshot.step_max_us = max_step_us;
    snapshot.step_over_period_delta = controlStepOverPeriodCount;
    snapshot.step_over_75pct_delta = controlStepOver75PctCount;
    snapshot.step_over_50pct_delta = controlStepOver50PctCount;
    snapshot.dt_over_1_5_count = controlTimerOver150Count;
    snapshot.dt_over_2_count = controlTimerOver200Count;
    snapshot.missed_ticks = controlTimerMissedTicks;

    controlTimerMaxDtUs = 0;
    controlTimerOver150Count = 0;
    controlTimerOver200Count = 0;
    controlTimerMissedTicks = 0;
    controlStepMaxUs = 0;
    controlStepOverPeriodCount = 0;
    controlStepOver75PctCount = 0;
    controlStepOver50PctCount = 0;
#endif
    return snapshot;
}

// 记录整个控制周期耗时。
void recordMasterTimingControlTotalUs(uint32_t duration_us) {
    recordTiming(timingControlTotal, duration_us);
}

// 记录纯控制逻辑耗时，不含电机 move/loopFOC。
void recordMasterTimingControlLogicUs(uint32_t duration_us) {
    recordTiming(timingControlLogic, duration_us);
}

// 记录电机输出阶段耗时，包括 move、loopFOC 和 SSI 读取。
void recordMasterTimingMotorUs(uint32_t total_us,
                               uint32_t move_us,
                               uint32_t loop_foc_us,
                               uint32_t sensor_spi_us) {
    recordTiming(timingMotorTotal, total_us);
    recordTiming(timingMotorMove, move_us);
    recordTiming(timingMotorLoopFoc, loop_foc_us);
    recordTiming(timingSensorSpi, sensor_spi_us);
}

// C 链接符号供 ADC 电流采样模块弱引用调用，避免循环 include。
extern "C" void recordMasterTimingCurrentSenseUs(uint32_t duration_us) {
    recordTiming(timingCurrentSense, duration_us);
}

// 返回控制各阶段时序统计，用于状态行显示和重构性能对比。
MasterControlTimingSnapshot getMasterControlTimingSnapshot() {
    MasterControlTimingSnapshot snapshot = {};
#if MASTER_CONTROL_TIMING_DIAG_ENABLED
    snapshot.control_total = snapshotTiming(timingControlTotal);
    snapshot.control_logic = snapshotTiming(timingControlLogic);
    snapshot.motor_total = snapshotTiming(timingMotorTotal);
    snapshot.motor_move = snapshotTiming(timingMotorMove);
    snapshot.motor_loop_foc = snapshotTiming(timingMotorLoopFoc);
    snapshot.current_sense = snapshotTiming(timingCurrentSense);
    snapshot.sensor_spi = snapshotTiming(timingSensorSpi);
#endif
    return snapshot;
}

// 创建通信、状态和控制任务；控制任务绑到专用核心并使用最高优先级。
void startMasterTasks() {
#if MASTER_ESPNOW_ENABLED
    xTaskCreatePinnedToCore(task_comm_loop,
                            "MasterComm",
                            MASTER_COMM_TASK_STACK_BYTES,
                            NULL,
                            MASTER_COMM_TASK_PRIORITY,
                            NULL,
                            MASTER_IO_CORE);
#endif
#if MASTER_STATUS_LOG_ENABLED
    xTaskCreatePinnedToCore(task_status_loop,
                            "MasterStatus",
                            MASTER_STATUS_TASK_STACK_BYTES,
                            NULL,
                            MASTER_STATUS_TASK_PRIORITY,
                            NULL,
                            MASTER_IO_CORE);
#endif
    xTaskCreatePinnedToCore(task_control_loop,
                            "MasterControl",
                            MASTER_CONTROL_TASK_STACK_BYTES,
                            NULL,
                            MASTER_CONTROL_TASK_PRIORITY,
                            NULL,
                            MASTER_CONTROL_CORE);
}
