#include "master/status/master_status.h"

#include <Arduino.h>

#include "common/protocol/protocol_units.h"
#include "common/system_state.h"
#include "master/comm/master_transport.h"
#include "master/config/master_config.h"
#include "master/hardware/master_encoder_hw.h"
#include "master/tasks/master_tasks.h"

// 主机低频状态行。这里只做串口打印，绝不从控制热路径调用。
void printMasterStatusLine() {
#if MASTER_STATUS_LOG_ENABLED
#if MASTER_STATUS_SUMMARY_LOG_ENABLED || MASTER_STATUS_SYNC_LOG_ENABLED
    const MasterCommandPacket command = snapshotMasterCommand();
    const SlaveTelemetryPacket telemetry = snapshotSlaveTelemetry();
    const uint32_t now_us = micros();
    const uint32_t telemetry_age_ms =
        (sysData.link.last_telemetry_seq == 0) ? 0 : ((now_us - sysData.link.last_rx_us) / 1000UL);
    const uint32_t ack_lag = static_cast<uint32_t>(command.seq - telemetry.ack_seq);
#endif

#if MASTER_STATUS_SUMMARY_LOG_ENABLED
    const uint16_t active_faults = getActiveFaultFlags();
    const uint16_t latched_faults = getLatchedFaultFlags();

    Serial.printf("[Master] mode=%s ffb=%u strong=%u timing_level=%u tx=%lu ack=%lu ack_lag=%lu x=%.1f%% y=%.1f%% pen=%u age=%lums send=%lu/%lu rx=%lu/%lu last=%u active_faults=0x%04x latched_faults=0x%04x faults=0x%04x\n",
                  masterSyncTestModeName(),
                  MASTER_FORCE_FEEDBACK_ENABLED ? 1 : 0,
                  MASTER_STRONG_TORQUE_TEST_ENABLED ? 1 : 0,
                  MASTER_TIMING_DIAG_LEVEL,
                  static_cast<unsigned long>(command.seq),
                  static_cast<unsigned long>(telemetry.ack_seq),
                  static_cast<unsigned long>(ack_lag),
                  sysData.master.x_pos,
                  sysData.master.y_pos,
                  sysData.link.pen_down ? 1 : 0,
                  static_cast<unsigned long>(telemetry_age_ms),
                  static_cast<unsigned long>(sysData.link.espnow_send_ok_count),
                  static_cast<unsigned long>(sysData.link.espnow_send_fail_count),
                  static_cast<unsigned long>(sysData.link.espnow_recv_ok_count),
                  static_cast<unsigned long>(sysData.link.espnow_recv_reject_count),
                  static_cast<unsigned int>(sysData.link.last_send_ok),
                  static_cast<unsigned int>(active_faults),
                  static_cast<unsigned int>(latched_faults),
                  static_cast<unsigned int>(sysData.link.protocol_fault_flags));
#endif

#if MASTER_STATUS_SYNC_LOG_ENABLED
    uint32_t encoder_frame = 0;
    uint16_t encoder_raw = 0;
    uint8_t encoder_status = 0;
    getMasterEncoderDiagnostics(encoder_frame, encoder_raw, encoder_status);

    const float sync_err_x_pct = sysData.master.x_pos - sysData.slave.x_pos;
    const float sync_err_y_pct = sysData.master.y_pos - sysData.slave.y_pos;
    const float sync_err_x_mm = (sync_err_x_pct * 0.01f) * PLOT_X_HALF_RANGE_MM;
    const float sync_err_y_mm = (sync_err_y_pct * 0.01f) * 125.0f;

    Serial.printf("[MasterSync] angle=%.2fdeg raw=%u stat=0x%01x frame=0x%06lx x_norm=%.3f y_norm=%.3f slave_x=%.1f%% slave_y=%.1f%% sync_err_x=%.1f%%/%.1fmm sync_err_y=%.1f%%/%.1fmm\n",
                  sysData.master.angle_deg,
                  static_cast<unsigned int>(encoder_raw),
                  static_cast<unsigned int>(encoder_status),
                  static_cast<unsigned long>(encoder_frame),
                  normToUnit(command.x_norm),
                  normToUnit(command.y_norm),
                  sysData.slave.x_pos,
                  sysData.slave.y_pos,
                  sync_err_x_pct,
                  sync_err_x_mm,
                  sync_err_y_pct,
                  sync_err_y_mm);
#endif

#if MASTER_STATUS_TIMING_LOG_ENABLED && MASTER_TIMING_STEP_DIAG_ENABLED
    const MasterControlHealthSnapshot health = getMasterControlHealthSnapshot();
    Serial.printf("[MasterTiming] diag=%u ctrl_dt=%luus ctrl_max=%luus step_us=%lu step_max=%lu over_period=%lu over_75pct=%lu over_50pct=%lu ctrl_miss=%lu dt_over_1_5=%lu dt_over_2=%lu\n",
                  static_cast<unsigned int>(health.diag_level),
                  static_cast<unsigned long>(health.last_dt_us),
                  static_cast<unsigned long>(health.max_dt_us),
                  static_cast<unsigned long>(health.step_us),
                  static_cast<unsigned long>(health.step_max_us),
                  static_cast<unsigned long>(health.step_over_period_delta),
                  static_cast<unsigned long>(health.step_over_75pct_delta),
                  static_cast<unsigned long>(health.step_over_50pct_delta),
                  static_cast<unsigned long>(health.missed_ticks),
                  static_cast<unsigned long>(health.dt_over_1_5_count),
                  static_cast<unsigned long>(health.dt_over_2_count));

#if MASTER_STATUS_TIMING_DETAIL_LOG_ENABLED && MASTER_CONTROL_TIMING_DIAG_ENABLED
    const MasterControlTimingSnapshot timing = getMasterControlTimingSnapshot();
    Serial.printf("[MasterTimingDetail] total=%lu/%lu/%lu logic=%lu/%lu/%lu motor=%lu/%lu/%lu move=%lu/%lu/%lu foc=%lu/%lu/%lu current=%lu/%lu/%lu sensor=%lu/%lu/%lu\n",
                  static_cast<unsigned long>(timing.control_total.last_us),
                  static_cast<unsigned long>(timing.control_total.avg_us),
                  static_cast<unsigned long>(timing.control_total.max_us),
                  static_cast<unsigned long>(timing.control_logic.last_us),
                  static_cast<unsigned long>(timing.control_logic.avg_us),
                  static_cast<unsigned long>(timing.control_logic.max_us),
                  static_cast<unsigned long>(timing.motor_total.last_us),
                  static_cast<unsigned long>(timing.motor_total.avg_us),
                  static_cast<unsigned long>(timing.motor_total.max_us),
                  static_cast<unsigned long>(timing.motor_move.last_us),
                  static_cast<unsigned long>(timing.motor_move.avg_us),
                  static_cast<unsigned long>(timing.motor_move.max_us),
                  static_cast<unsigned long>(timing.motor_loop_foc.last_us),
                  static_cast<unsigned long>(timing.motor_loop_foc.avg_us),
                  static_cast<unsigned long>(timing.motor_loop_foc.max_us),
                  static_cast<unsigned long>(timing.current_sense.last_us),
                  static_cast<unsigned long>(timing.current_sense.avg_us),
                  static_cast<unsigned long>(timing.current_sense.max_us),
                  static_cast<unsigned long>(timing.sensor_spi.last_us),
                  static_cast<unsigned long>(timing.sensor_spi.avg_us),
                  static_cast<unsigned long>(timing.sensor_spi.max_us));
#endif
#endif
#endif
}
