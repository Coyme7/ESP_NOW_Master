#include "master/status/master_status.h"

#include <Arduino.h>

#include "common/system_state.h"
#include "master/config/master_config.h"
#include "master/hardware/master_encoder_hw.h"
#include "master/tasks/master_tasks.h"
#include "master/comm/master_transport.h"

// 低频状态行输出。这里可以 Serial 打印，因为它运行在状态任务，不在 8kHz 控制热路径。
// 状态行是低频观测窗口：字段多是为了调试，不代表这些值都参与实时控制。
void printMasterStatusLine() {
    const MasterCommandPacket command = snapshotMasterCommand();
    const SlaveTelemetryPacket telemetry = snapshotSlaveTelemetry();

    const uint32_t now_us = micros();
    const uint32_t telemetry_age_ms =
        (sysData.link.last_telemetry_seq == 0) ? 0 : ((now_us - sysData.link.last_rx_us) / 1000UL);

    uint32_t encoder_frame = 0;
    uint16_t encoder_raw = 0;
    uint8_t encoder_status = 0;
    getMasterEncoderDiagnostics(encoder_frame, encoder_raw, encoder_status);

#if MASTER_CONTROL_HEALTH_DIAG_ENABLED || MASTER_CONTROL_TIMING_DIAG_ENABLED
    const MasterControlHealthSnapshot health = getMasterControlHealthSnapshot();
#endif
#if MASTER_CONTROL_TIMING_DIAG_ENABLED
    const MasterControlTimingSnapshot timing = getMasterControlTimingSnapshot();
#endif

#if MASTER_CONTROL_TIMING_DIAG_ENABLED
    Serial.printf("[Master] tx=%lu ack=%lu angle=%.2fdeg raw=%u stat=0x%01x frame=0x%06lx x=%.1f%% current=%.3fA iq=%.3fA id=%.3fA vq=%.3fV vd=%.3fV ctrl_dt=%luus ctrl_max=%luus overC1_5=%lu overC2=%lu ctrl_miss=%lu t_us ctrl=%lu/%lu/%lu logic=%lu/%lu/%lu motor=%lu/%lu/%lu move=%lu/%lu/%lu foc=%lu/%lu/%lu cs=%lu/%lu/%lu spi=%lu/%lu/%lu pen=%u slave_x=%.1f%% age=%lums send=%lu/%lu rx=%lu/%lu last=%u faults=0x%04x\n",
                  static_cast<unsigned long>(command.seq),
                  static_cast<unsigned long>(telemetry.ack_seq),
                  sysData.master.angle_deg,
                  static_cast<unsigned int>(encoder_raw),
                  static_cast<unsigned int>(encoder_status),
                  static_cast<unsigned long>(encoder_frame),
                  sysData.master.x_pos,
                  sysData.master.target_current_a,
                  sysData.master.current_q_a,
                  sysData.master.current_d_a,
                  sysData.master.voltage_q_v,
                  sysData.master.voltage_d_v,
                  static_cast<unsigned long>(health.last_dt_us),
                  static_cast<unsigned long>(health.max_dt_us),
                  static_cast<unsigned long>(health.over_1_5_count),
                  static_cast<unsigned long>(health.over_2_count),
                  static_cast<unsigned long>(health.missed_ticks),
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
                  static_cast<unsigned long>(timing.sensor_spi.max_us),
                  sysData.link.pen_down ? 1 : 0,
                  sysData.slave.x_pos,
                  static_cast<unsigned long>(telemetry_age_ms),
                  static_cast<unsigned long>(sysData.link.espnow_send_ok_count),
                  static_cast<unsigned long>(sysData.link.espnow_send_fail_count),
                  static_cast<unsigned long>(sysData.link.espnow_recv_ok_count),
                  static_cast<unsigned long>(sysData.link.espnow_recv_reject_count),
                  static_cast<unsigned int>(sysData.link.last_send_ok),
                  static_cast<unsigned int>(sysData.link.protocol_fault_flags));
#elif MASTER_CONTROL_HEALTH_DIAG_ENABLED
    Serial.printf("[Master] tx=%lu ack=%lu angle=%.2fdeg raw=%u stat=0x%01x frame=0x%06lx x=%.1f%% current=%.3fA iq=%.3fA id=%.3fA vq=%.3fV vd=%.3fV ctrl_dt=%luus ctrl_max=%luus overC1_5=%lu overC2=%lu ctrl_miss=%lu pen=%u slave_x=%.1f%% age=%lums send=%lu/%lu rx=%lu/%lu last=%u faults=0x%04x\n",
                  static_cast<unsigned long>(command.seq),
                  static_cast<unsigned long>(telemetry.ack_seq),
                  sysData.master.angle_deg,
                  static_cast<unsigned int>(encoder_raw),
                  static_cast<unsigned int>(encoder_status),
                  static_cast<unsigned long>(encoder_frame),
                  sysData.master.x_pos,
                  sysData.master.target_current_a,
                  sysData.master.current_q_a,
                  sysData.master.current_d_a,
                  sysData.master.voltage_q_v,
                  sysData.master.voltage_d_v,
                  static_cast<unsigned long>(health.last_dt_us),
                  static_cast<unsigned long>(health.max_dt_us),
                  static_cast<unsigned long>(health.over_1_5_count),
                  static_cast<unsigned long>(health.over_2_count),
                  static_cast<unsigned long>(health.missed_ticks),
                  sysData.link.pen_down ? 1 : 0,
                  sysData.slave.x_pos,
                  static_cast<unsigned long>(telemetry_age_ms),
                  static_cast<unsigned long>(sysData.link.espnow_send_ok_count),
                  static_cast<unsigned long>(sysData.link.espnow_send_fail_count),
                  static_cast<unsigned long>(sysData.link.espnow_recv_ok_count),
                  static_cast<unsigned long>(sysData.link.espnow_recv_reject_count),
                  static_cast<unsigned int>(sysData.link.last_send_ok),
                  static_cast<unsigned int>(sysData.link.protocol_fault_flags));
#else
    Serial.printf("[Master] tx=%lu ack=%lu angle=%.2fdeg raw=%u stat=0x%01x frame=0x%06lx x=%.1f%% current=%.3fA iq=%.3fA id=%.3fA vq=%.3fV vd=%.3fV pen=%u slave_x=%.1f%% age=%lums send=%lu/%lu rx=%lu/%lu last=%u faults=0x%04x\n",
                  static_cast<unsigned long>(command.seq),
                  static_cast<unsigned long>(telemetry.ack_seq),
                  sysData.master.angle_deg,
                  static_cast<unsigned int>(encoder_raw),
                  static_cast<unsigned int>(encoder_status),
                  static_cast<unsigned long>(encoder_frame),
                  sysData.master.x_pos,
                  sysData.master.target_current_a,
                  sysData.master.current_q_a,
                  sysData.master.current_d_a,
                  sysData.master.voltage_q_v,
                  sysData.master.voltage_d_v,
                  sysData.link.pen_down ? 1 : 0,
                  sysData.slave.x_pos,
                  static_cast<unsigned long>(telemetry_age_ms),
                  static_cast<unsigned long>(sysData.link.espnow_send_ok_count),
                  static_cast<unsigned long>(sysData.link.espnow_send_fail_count),
                  static_cast<unsigned long>(sysData.link.espnow_recv_ok_count),
                  static_cast<unsigned long>(sysData.link.espnow_recv_reject_count),
                  static_cast<unsigned int>(sysData.link.last_send_ok),
                  static_cast<unsigned int>(sysData.link.protocol_fault_flags));
#endif
}
