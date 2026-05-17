#include <Arduino.h>

#include "master/config/master_config.h"
#include "master/comm/master_transport.h"
#include "master/hardware/master_hardware.h"
#include "master/tasks/master_tasks.h"

// 主机固件入口。
// 这个文件只负责“上电顺序”：初始化 Arduino 兼容层、串口、安全输出、硬件、ESP-NOW，
// 最后再启动 FreeRTOS 任务。真正的控制、通信和状态打印都拆到 master/* 模块中，
// 方便后续单独检查 200us / 5kHz 热路径和 Core 0 低频任务。
extern "C" void app_main() {
    // PlatformIO 当前使用 ESP-IDF 入口，因此需要显式启用 Arduino 运行时，
    // Serial、pinMode、digitalWrite、micros 等 Arduino API 才能正常使用。
    initArduino();
    Serial.begin(115200);

    // 安全输出必须放在最前面：任何电机初始化、Wi-Fi 初始化或任务创建前，
    // 都先把主机两路电机使能脚拉到禁用态，避免启动瞬间误输出。
    configureMasterSafeOutputs();

    // MASTER_MOTOR_HW_ENABLED=0 时这里只锁存 FAULT_MOTOR_OUTPUT_DISABLED；
    // MASTER_MOTOR_HW_ENABLED=1 时返回值表示驱动、电流采样和 initFOC 是否全部通过。
    const bool motor_ready = setupMasterMotorHardware();

    // ESP-NOW 初始化在任务启动前完成；单机旋钮/电流环测试时默认关闭，避免 Wi-Fi
    // 和发送失败回调干扰 200us / 5kHz 控制定时。
#if MASTER_ESPNOW_ENABLED
    setupMasterEspNow();
#if MASTER_ESPNOW_IDENTITY_LOG_ENABLED
    printMasterEspNowIdentity();
#endif
#else
#if MASTER_BOOT_LOG_ENABLED
    Serial.println("[Master] espnow disabled for local knob/current test");
#endif
#endif

#if MASTER_BOOT_LOG_ENABLED
    Serial.printf("[MasterConfig] mode=%s x_knob_hw=%u y_knob_hw=%u force_pen=%u ffb=%u strong=%u ble=%u status_log=%u status_period=%lums timing_level=%u timing_step=%u timing_detail=%u\n",
                  masterSyncTestModeName(),
                  MASTER_X_KNOB_HW_ENABLED ? 1 : 0,
                  MASTER_Y_KNOB_HW_ENABLED ? 1 : 0,
                  MASTER_FORCE_PEN_DOWN_FOR_TEST ? 1 : 0,
                  MASTER_FORCE_FEEDBACK_ENABLED ? 1 : 0,
                  MASTER_STRONG_TORQUE_TEST_ENABLED ? 1 : 0,
                  MASTER_BLE_MODE_ENABLED ? 1 : 0,
                  MASTER_STATUS_LOG_ENABLED ? 1 : 0,
                  static_cast<unsigned long>(MASTER_STATUS_LOOP_PERIOD_MS),
                  MASTER_TIMING_DIAG_LEVEL,
                  MASTER_TIMING_STEP_DIAG_ENABLED ? 1 : 0,
                  MASTER_TIMING_DETAIL_DIAG_ENABLED ? 1 : 0);

    // boot 行给第一次上电调试使用：确认硬件输出是否启用、旋钮角度范围、
    // 控制周期和通信周期是否与 Instruction.md 中的测试说明一致。
    Serial.printf("[Master] boot motor_hw=%u hw_status=%s force_pen=%u espnow=%u knob_center=%.1fdeg knob_range=%.0f..%.0f deg control=%luus comm=%lums vlim=%.2fV ilim=%.3fA\n",
                  motor_ready ? 1 : 0,
                  getMasterMotorHardwareStatus(),
                  MASTER_FORCE_PEN_DOWN_FOR_TEST ? 1 : 0,
                  MASTER_ESPNOW_ENABLED ? 1 : 0,
                  kMasterXAxis.center_deg,
                  kMasterXAxis.min_deg,
                  kMasterXAxis.max_deg,
                  static_cast<unsigned long>(MASTER_CONTROL_LOOP_PERIOD_US),
                  static_cast<unsigned long>(MASTER_COMMAND_PERIOD_MS),
                  kMasterMotorFoc.voltage_limit_v,
                  kMasterXAxis.haptic_current_limit_a);
#endif

    // 从这里开始进入多任务模型：
    // Core 1 运行控制热路径，Core 0 运行 ESP-NOW 与串口状态任务。
    startMasterTasks();
}
