#include <Arduino.h>

#include "common/system_state.h"
#include "common/timing/link_timing.h"
#include "master/config/master_config.h"
#include "master/comm/master_transport.h"
#include "master/hardware/master_adc1_dma_sampler.h"
#include "master/hardware/master_hardware.h"
#include "master/modes/mode_traits.h"
#include "master/modes/mode_table.h"
#include "master/tasks/master_tasks.h"

// 主机固件入口。
// 这个文件只负责“上电顺序”：初始化 Arduino 兼容层、串口、安全输出、硬件、ESP-NOW，
// 最后再启动 FreeRTOS 任务。真正的控制、通信和状态打印都拆到 master/* 模块中，
// 方便后续单独检查 run mode 派生的热路径和 Core 0 低频任务。
extern "C" void app_main() {
    // PlatformIO 当前使用 ESP-IDF 入口，因此需要显式启用 Arduino 运行时，
    // Serial、pinMode、digitalWrite、micros 等 Arduino API 才能正常使用。
    initArduino();
    Serial.begin(115200);

    // 安全输出必须放在最前面：任何电机初始化、Wi-Fi 初始化或任务创建前，
    // 都先把主机两路电机使能脚拉到禁用态，避免启动瞬间误输出。
    configureMasterSafeOutputs();

    // MASTER_ENABLE_MOTOR_HW=0 时这里只锁存 FAULT_MOTOR_OUTPUT_DISABLED；
    // MASTER_ENABLE_MOTOR_HW=1 时返回值表示驱动、电流采样和 initFOC 是否全部通过。
    const bool adc_dma_started = startMasterAdc1DmaSampler();
    const bool adc_dma_ready = adc_dma_started &&
                               waitForMasterAdc1DmaFirstFrame(100U);
    if (masterAdc1DmaSamplerRequired() && !adc_dma_ready) {
        addLocalFault(FAULT_MOTOR_OUTPUT_DISABLED);
    }

    const bool motor_ready = adc_dma_ready ? setupMasterMotorHardware() : false;

    // ESP-NOW 初始化在任务启动前完成；单机旋钮/电流环测试时默认关闭，避免 Wi-Fi
    // 和发送失败回调干扰控制热路径定时。
#if MASTER_ENABLE_ESPNOW
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
    Serial.printf("[MasterConfig] run_mode=%s run_path=%s default_app=%s init_order=%s x_encoder_req=%u y_encoder_req=%u x_motor_req=%u y_motor_req=%u x_encoder=%u y_encoder=%u x_motor=%u y_motor=%u force_pen=%u ffb=%u strong=%u ble=%u status_log=%u status_period=%lums timing_level=%u timing_step=%u timing_detail=%u\n",
                  masterRunModeName(),
                  masterRunPathName(),
                  masterStartupAppModeName(),
                  MASTER_INIT_FOC_Y_FIRST ? "Y,X" : "X,Y",
                  masterRunModeNeedsEncoderHardware(AXIS_X) ? 1 : 0,
                  masterRunModeNeedsEncoderHardware(AXIS_Y) ? 1 : 0,
                  masterRunModeNeedsMotorHardware(AXIS_X) ? 1 : 0,
                  masterRunModeNeedsMotorHardware(AXIS_Y) ? 1 : 0,
                  MASTER_ENABLE_X_ENCODER_HW ? 1 : 0,
                  MASTER_ENABLE_Y_ENCODER_HW ? 1 : 0,
                  MASTER_ENABLE_X_MOTOR_HW ? 1 : 0,
                  MASTER_ENABLE_Y_MOTOR_HW ? 1 : 0,
                  MASTER_ENABLE_FORCE_PEN_DOWN_TEST ? 1 : 0,
                  MASTER_ENABLE_FORCE_FEEDBACK ? 1 : 0,
                  MASTER_ENABLE_STRONG_TORQUE_TEST ? 1 : 0,
                  MASTER_ENABLE_BLE ? 1 : 0,
                  MASTER_STATUS_LOG_ENABLED ? 1 : 0,
                  static_cast<unsigned long>(MASTER_STATUS_LOOP_PERIOD_MS),
                  MASTER_TIMING_DIAG_LEVEL,
                  MASTER_TIMING_STEP_DIAG_ENABLED ? 1 : 0,
                  MASTER_TIMING_DETAIL_DIAG_ENABLED ? 1 : 0);

    // boot 行给第一次上电调试使用：确认硬件输出是否启用、旋钮角度范围、
    // 控制周期和通信周期是否与 Instruction.md 中的测试说明一致。
    const MasterAdc1DmaHealthSnapshot adc_health = snapshotMasterAdc1DmaHealth();
    Serial.printf("[Master] boot run_mode=%s run_path=%s default_app=%s motor_hw=%u hw_status=%s adc_dma=%u/%u/%u adc_reason=%u runtime_latch=%u seq=%lu age=%luus max_age=%luus stale_limit=%luus frame=%luB pool_frames=%lu pool_bytes=%lu per_ch=%u invalid=%lu empty=%lu overflow=%lu stale=%lu force_pen=%u espnow=%u x_center=%.1fdeg y_center=%.1fdeg x_range=%.0f..%.0fdeg y_range=%.0f..%.0fdeg control=%luus comm=%lums x_vlim=%.2fV y_vlim=%.2fV x_ilim=%.3fA y_ilim=%.3fA\n",
                  masterRunModeName(),
                  masterRunPathName(),
                  masterStartupAppModeName(),
                  motor_ready ? 1 : 0,
                  getMasterMotorHardwareStatus(),
                  adc_health.required ? 1 : 0,
                  adc_health.started ? 1 : 0,
                  adc_health.first_frame_ready ? 1 : 0,
                  static_cast<unsigned int>(adc_health.fault_reason),
                  adc_health.runtime_fault_latch_enabled ? 1 : 0,
                  static_cast<unsigned long>(adc_health.frame_sequence),
                  static_cast<unsigned long>(adc_health.latest_age_us),
                  static_cast<unsigned long>(adc_health.latest_age_max_us),
                  static_cast<unsigned long>(adc_health.stale_fault_us),
                  static_cast<unsigned long>(adc_health.frame_bytes),
                  static_cast<unsigned long>(adc_health.pool_frames),
                  static_cast<unsigned long>(adc_health.pool_bytes),
                  static_cast<unsigned int>(adc_health.samples_per_active_channel),
                  static_cast<unsigned long>(adc_health.invalid_frames),
                  static_cast<unsigned long>(adc_health.read_empty_count),
                  static_cast<unsigned long>(adc_health.pool_overflows),
                  static_cast<unsigned long>(adc_health.stale_control_cycles),
                  MASTER_ENABLE_FORCE_PEN_DOWN_TEST ? 1 : 0,
                  MASTER_ENABLE_ESPNOW ? 1 : 0,
                  kMasterXAxis.input.center_deg,
                  kMasterYAxis.input.center_deg,
                  kMasterXAxis.range.min_deg,
                  kMasterXAxis.range.max_deg,
                  kMasterYAxis.range.min_deg,
                  kMasterYAxis.range.max_deg,
                  static_cast<unsigned long>(MASTER_CONTROL_LOOP_PERIOD_US),
                  static_cast<unsigned long>(MASTER_COMMAND_PERIOD_MS),
                  kMasterXMotorFoc.voltage_limit_v,
                  kMasterYMotorFoc.voltage_limit_v,
                  kMasterXAxis.current.limit_a,
                  kMasterYAxis.current.limit_a);
#endif

    // 从这里开始进入多任务模型：
    // Core 1 运行控制热路径，Core 0 运行 ESP-NOW 与串口状态任务。
    armMasterAdc1DmaControlStartupGrace();
    startMasterTasks();
}
