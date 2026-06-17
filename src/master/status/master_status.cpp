#include "master/status/master_status.h"

#include <Arduino.h>

#include "common/protocol/protocol_types.h"
#include "common/protocol/protocol_units.h"
#include "common/system_state.h"
#include "master/comm/master_transport.h"
#include "master/config/master_config.h"
#include "master/modes/auto_draw/auto_draw_mode.h"
#include "master/hardware/master_adc1_dma_sampler.h"
#include "master/hardware/master_encoder_hw.h"
#include "master/modes/mode_manager.h"
#include "master/modes/mode_protocol_map.h"
#include "master/modes/mode_table.h"
#include "master/tasks/master_tasks.h"

#include <string.h>

namespace {

const char *drawStateName(uint8_t state) {
    switch (state) {
        case DRAW_STATE_IDLE:
            return "Idle";
        case DRAW_STATE_RUNNING:
            return "Running";
        case DRAW_STATE_FINISHED:
            return "Finished";
        case DRAW_STATE_BLOCKED:
            return "Blocked";
        case DRAW_STATE_LOADING:
            return "Loading";
        default:
            return "Unknown";
    }
}

const char *trajectoryPhaseName(uint8_t flags) {
    if ((flags & TRAJECTORY_STATUS_RUNNING) != 0U) {
        return "Running";
    }
    if ((flags & TRAJECTORY_STATUS_COMPLETE) != 0U) {
        return "Complete";
    }
    if ((flags & TRAJECTORY_STATUS_READY) != 0U) {
        return "Ready";
    }
    if ((flags & TRAJECTORY_STATUS_LOADING) != 0U) {
        return "Loading";
    }
    if ((flags & TRAJECTORY_STATUS_BLOCKED) != 0U) {
        return "Blocked";
    }
    return "None";
}

void trimDiagLine(char *line) {
    size_t len = strlen(line);
    while (len > 0 && (line[len - 1] == '\r' || line[len - 1] == '\n' || line[len - 1] == ' ' || line[len - 1] == '\t')) {
        line[len - 1] = '\0';
        len--;
    }

    char *start = line;
    while (*start == ' ' || *start == '\t') {
        start++;
    }
    if (start != line) {
        memmove(line, start, strlen(start) + 1);
    }
}

void printMasterDiagHelp() {
    Serial.println("[MasterDiag] commands: help, mode, axis, paper, link, stats, fault clear, pen status, uv status, draw test, dryrun on, dryrun off");
}

void printMasterModeDiag() {
    const MasterRuntimeModeSnapshot runtime = getMasterRuntimeModeSnapshot();
    const ModeCapability capability = masterModeCapabilityForRuntime(runtime.active_mode);
    const uint8_t protocol_mode = masterProtocolModeForRuntime(runtime.active_mode);
    const uint16_t command_flags = masterCommandFlagsForRuntime(runtime.active_mode);
    Serial.printf("[MasterDiag] default_app=%s default_runtime=%s run_mode=%s run_path=%s runtime=%s requested=%s accepted=%u rejected=%u last_button=%s requests=%lu last_change=%lums buttons=g42:%u/g41:%u/g40:%u caps=0x%04x control_rate=%luhz outer_rate=%luhz protocol_mode=%u command_flags=0x%04x x_knob=%u y_knob=%u x_motor=%u y_motor=%u ffb=%u\n",
                  masterStartupAppModeName(),
                  masterRuntimeModeName(masterDefaultRuntimeMode()),
                  masterRunModeName(),
                  masterRunPathName(),
                  masterRuntimeModeName(runtime.active_mode),
                  masterRuntimeModeName(runtime.requested_mode),
                  static_cast<unsigned int>(runtime.request_accepted),
                  static_cast<unsigned int>(runtime.request_rejected),
                  masterRuntimeButtonName(runtime.last_button),
                  static_cast<unsigned long>(runtime.request_count),
                  static_cast<unsigned long>(runtime.last_change_ms),
                  static_cast<unsigned int>(runtime.manual_button_down),
                  static_cast<unsigned int>(runtime.auto_button_down),
                  static_cast<unsigned int>(runtime.bluetooth_button_down),
                  static_cast<unsigned int>(capability.flags),
                  static_cast<unsigned long>(capability.control_rate_hz),
                  static_cast<unsigned long>(capability.outer_rate_hz),
                  static_cast<unsigned int>(protocol_mode),
                  static_cast<unsigned int>(command_flags),
                  MASTER_ENABLE_X_ENCODER_HW ? 1 : 0,
                  MASTER_ENABLE_Y_ENCODER_HW ? 1 : 0,
                  MASTER_ENABLE_X_MOTOR_HW ? 1 : 0,
                  MASTER_ENABLE_Y_MOTOR_HW ? 1 : 0,
                  MASTER_ENABLE_FORCE_FEEDBACK ? 1 : 0);
}

void printMasterAxisDiag() {
    const MasterCommandPacket command = snapshotMasterCommand();
    uint32_t x_encoder_frame = 0;
    uint16_t x_encoder_raw = 0;
    uint8_t x_encoder_status = 0;
    uint32_t y_encoder_frame = 0;
    uint16_t y_encoder_raw = 0;
    uint8_t y_encoder_status = 0;
    getMasterEncoderDiagnostics(x_encoder_frame, x_encoder_raw, x_encoder_status);
    getMasterYEncoderDiagnostics(y_encoder_frame, y_encoder_raw, y_encoder_status);
    Serial.printf("[MasterDiag] axis x_angle=%.2fdeg y_angle=%.2fdeg x=%.1f%% y=%.1f%% x_norm=%d y_norm=%d boundary=%u x_boundary=%u y_boundary=%u x_i=%.3fA y_i=%.3fA x_iq/id=%.3f/%.3fA y_iq/id=%.3f/%.3fA x_raw=%u x_stat=0x%01x x_frame=0x%06lx y_raw=%u y_stat=0x%01x y_frame=0x%06lx\n",
                  sysData.master.angle_deg,
                  sysData.master.y_angle_deg,
                  sysData.master.x_pos,
                  sysData.master.y_pos,
                  command.x_norm,
                  command.y_norm,
                  sysData.master.boundary_hit ? 1 : 0,
                  sysData.master.x_boundary_hit ? 1 : 0,
                  sysData.master.y_boundary_hit ? 1 : 0,
                  sysData.master.target_current_a,
                  sysData.master.y_target_current_a,
                  sysData.master.current_q_a,
                  sysData.master.current_d_a,
                  sysData.master.y_current_q_a,
                  sysData.master.y_current_d_a,
                  static_cast<unsigned int>(x_encoder_raw),
                  static_cast<unsigned int>(x_encoder_status),
                  static_cast<unsigned long>(x_encoder_frame),
                  static_cast<unsigned int>(y_encoder_raw),
                  static_cast<unsigned int>(y_encoder_status),
                  static_cast<unsigned long>(y_encoder_frame));
}

void printMasterPaperDiag() {
    Serial.println("[MasterDiag] paper mapper is slave-owned; master sends x_norm/y_norm/pen only");
}

void printMasterLinkDiag() {
    const MasterCommandPacket command = snapshotMasterCommand();
    const SlaveTelemetryPacket telemetry = snapshotSlaveTelemetry();
    const uint32_t now_us = micros();
    const uint32_t age_ms =
        (sysData.link.last_telemetry_seq == 0) ? 0 : ((now_us - sysData.link.last_rx_us) / 1000UL);
    Serial.printf("[MasterDiag] link state=%u tx_seq=%lu ack=%lu telemetry_seq=%lu age=%lums send=%lu/%lu rx=%lu/%lu stale=%lu duplicate=%lu last_send=%u proto=%u flags=0x%04x traj_task=%u traj=%u/%u cursor=%u traj_flags=0x%02x mask=%04x:%08lx\n",
                  static_cast<unsigned int>(sysData.link.link_state),
                  static_cast<unsigned long>(command.seq),
                  static_cast<unsigned long>(telemetry.ack_seq),
                  static_cast<unsigned long>(telemetry.seq),
                  static_cast<unsigned long>(age_ms),
                  static_cast<unsigned long>(sysData.link.espnow_send_ok_count),
                  static_cast<unsigned long>(sysData.link.espnow_send_fail_count),
                  static_cast<unsigned long>(sysData.link.espnow_recv_ok_count),
                  static_cast<unsigned long>(sysData.link.espnow_recv_reject_count),
                  static_cast<unsigned long>(sysData.link.espnow_recv_stale_count),
                  static_cast<unsigned long>(sysData.link.espnow_recv_duplicate_count),
                  static_cast<unsigned int>(sysData.link.last_send_ok),
                  static_cast<unsigned int>(command.mode),
                  static_cast<unsigned int>(command.command_flags),
                  static_cast<unsigned int>(telemetry.trajectory_task_id),
                  static_cast<unsigned int>(telemetry.trajectory_received_count),
                  static_cast<unsigned int>(telemetry.trajectory_segment_count),
                  static_cast<unsigned int>(telemetry.trajectory_segment_cursor),
                  static_cast<unsigned int>(telemetry.trajectory_status_flags),
                  static_cast<unsigned int>(telemetry.trajectory_received_mask_high),
                  static_cast<unsigned long>(telemetry.trajectory_received_mask_low));
}

void printMasterStatsDiag() {
    const MasterControlHealthSnapshot health = getMasterControlHealthSnapshot();
    const MasterAdc1DmaHealthSnapshot adc = snapshotMasterAdc1DmaHealth();
    Serial.printf("[MasterDiag] stats diag=%u dt=%lu/%luus step=%lu/%luus miss=%lu over_period=%lu over75=%lu over50=%lu dt_over_1_5=%lu dt_over_2=%lu\n",
                  static_cast<unsigned int>(health.diag_level),
                  static_cast<unsigned long>(health.last_dt_us),
                  static_cast<unsigned long>(health.max_dt_us),
                  static_cast<unsigned long>(health.step_us),
                  static_cast<unsigned long>(health.step_max_us),
                  static_cast<unsigned long>(health.missed_ticks),
                  static_cast<unsigned long>(health.step_over_period_delta),
                  static_cast<unsigned long>(health.step_over_75pct_delta),
                  static_cast<unsigned long>(health.step_over_50pct_delta),
                  static_cast<unsigned long>(health.dt_over_1_5_count),
                  static_cast<unsigned long>(health.dt_over_2_count));
    Serial.printf("[MasterDiag] adc_dma required=%u started=%u first=%u fault=%u runtime_latch=%u reason=%u seq=%lu age=%luus max_age=%luus fault_age=%luus stale_limit=%luus frame=%luB pool_frames=%lu pool_bytes=%lu per_ch=%u invalid=%lu samples=%lu read_err=%lu empty=%lu overflow=%lu stale=%lu consumer=%lu/%luus\n",
                  adc.required ? 1 : 0,
                  adc.started ? 1 : 0,
                  adc.first_frame_ready ? 1 : 0,
                  adc.fault_latched ? 1 : 0,
                  adc.runtime_fault_latch_enabled ? 1 : 0,
                  static_cast<unsigned int>(adc.fault_reason),
                  static_cast<unsigned long>(adc.frame_sequence),
                  static_cast<unsigned long>(adc.latest_age_us),
                  static_cast<unsigned long>(adc.latest_age_max_us),
                  static_cast<unsigned long>(adc.fault_age_us),
                  static_cast<unsigned long>(adc.stale_fault_us),
                  static_cast<unsigned long>(adc.frame_bytes),
                  static_cast<unsigned long>(adc.pool_frames),
                  static_cast<unsigned long>(adc.pool_bytes),
                  static_cast<unsigned int>(adc.samples_per_active_channel),
                  static_cast<unsigned long>(adc.invalid_frames),
                  static_cast<unsigned long>(adc.invalid_samples),
                  static_cast<unsigned long>(adc.read_errors),
                  static_cast<unsigned long>(adc.read_empty_count),
                  static_cast<unsigned long>(adc.pool_overflows),
                  static_cast<unsigned long>(adc.stale_control_cycles),
                  static_cast<unsigned long>(adc.consumer_last_us),
                  static_cast<unsigned long>(adc.consumer_max_us));
}

void printMasterUvDiag() {
    Serial.printf("[MasterDiag] pen_req=%u slave_pen=%u uv_out=%u uv_block=%u uv_reasons=0x%04x slave_faults=0x%04x path=pen_req_to_slave_interlock_only\n",
                  sysData.link.pen_req ? 1 : 0,
                  static_cast<unsigned int>(sysData.slave.pen_state),
                  sysData.link.uv_out ? 1 : 0,
                  sysData.slave.uv_interlock_blocked ? 1 : 0,
                  static_cast<unsigned int>(sysData.slave.uv_block_reasons),
                  static_cast<unsigned int>(sysData.link.protocol_fault_flags));
}

void handleMasterDiagCommand(char *line) {
    trimDiagLine(line);
    if (line[0] == '\0') {
        return;
    }

    if (strcmp(line, "help") == 0 || strcmp(line, "?") == 0) {
        printMasterDiagHelp();
    } else if (strcmp(line, "mode") == 0) {
        printMasterModeDiag();
    } else if (strcmp(line, "axis") == 0) {
        printMasterAxisDiag();
    } else if (strcmp(line, "paper") == 0) {
        printMasterPaperDiag();
    } else if (strcmp(line, "link") == 0) {
        printMasterLinkDiag();
    } else if (strcmp(line, "stats") == 0) {
        printMasterStatsDiag();
    } else if (strcmp(line, "fault clear") == 0) {
        clearLocalFaults();
        Serial.printf("[MasterDiag] fault cleared active=0x%04x latched=0x%04x merged=0x%04x\n",
                      static_cast<unsigned int>(getActiveFaultFlags()),
                      static_cast<unsigned int>(getLatchedFaultFlags()),
                      static_cast<unsigned int>(sysData.link.protocol_fault_flags));
    } else if (strcmp(line, "pen status") == 0 || strcmp(line, "uv status") == 0) {
        printMasterUvDiag();
    } else if (strcmp(line, "draw test") == 0) {
        const MasterRuntimeModeSnapshot runtime = getMasterRuntimeModeSnapshot();
        const SlaveTelemetryPacket telemetry = snapshotSlaveTelemetry();
        const MasterAutoDrawTransportStatus tx_status = snapshotMasterAutoDrawTransportStatus();
        Serial.printf("[MasterDiag] auto_draw=%u preset=%s task=%u segments=%u accepted=%u pending=%u request_seq=%lu next_tx=%u tx_seq=%lu protocol_mode=%u flags=0x%04x default_app=%s run_mode=%s run_path=%s runtime=%s requested=%s rejected=%u draw_state=%s draw_progress=%u%% slave_task=%u slave_traj=%s rx=%u/%u cursor=%u traj_flags=0x%02x mask=%04x:%08lx\n",
                      (runtime.active_mode == MASTER_RUNTIME_MODE_AUTO_DRAW) ? 1 : 0,
                      masterAutoDrawPresetName(),
                      static_cast<unsigned int>(masterAutoDrawTaskId()),
                      static_cast<unsigned int>(masterAutoDrawSegmentCount()),
                      static_cast<unsigned int>(tx_status.mode_accepted),
                      static_cast<unsigned int>(tx_status.mode_request_pending),
                      static_cast<unsigned long>(tx_status.mode_request_seq),
                      static_cast<unsigned int>(tx_status.next_segment_index),
                      static_cast<unsigned long>(tx_status.trajectory_tx_seq),
                      static_cast<unsigned int>(masterProtocolModeForRuntime(runtime.active_mode)),
                      static_cast<unsigned int>(masterCommandFlagsForRuntime(runtime.active_mode)),
                      masterStartupAppModeName(),
                      masterRunModeName(),
                      masterRunPathName(),
                      masterRuntimeModeName(runtime.active_mode),
                      masterRuntimeModeName(runtime.requested_mode),
                      static_cast<unsigned int>(runtime.request_rejected),
                      drawStateName(sysData.slave.draw_state),
                      static_cast<unsigned int>(sysData.slave.draw_progress_pct),
                      static_cast<unsigned int>(telemetry.trajectory_task_id),
                      trajectoryPhaseName(telemetry.trajectory_status_flags),
                      static_cast<unsigned int>(telemetry.trajectory_received_count),
                      static_cast<unsigned int>(telemetry.trajectory_segment_count),
                      static_cast<unsigned int>(telemetry.trajectory_segment_cursor),
                      static_cast<unsigned int>(telemetry.trajectory_status_flags),
                      static_cast<unsigned int>(telemetry.trajectory_received_mask_high),
                      static_cast<unsigned long>(telemetry.trajectory_received_mask_low));
    } else if (strcmp(line, "dryrun on") == 0 || strcmp(line, "dryrun off") == 0) {
        const MasterRuntimeModeSnapshot runtime = getMasterRuntimeModeSnapshot();
        Serial.printf("[MasterDiag] dryrun follows runtime mode; protocol_flags=0x%04x runtime=%s default_app=%s\n",
                      static_cast<unsigned int>(masterCommandFlagsForRuntime(runtime.active_mode)),
                      masterRuntimeModeName(runtime.active_mode),
                      masterStartupAppModeName());
    } else {
        Serial.printf("[MasterDiag] unknown command: %s\n", line);
        printMasterDiagHelp();
    }
}

}  // namespace

// 主机低频状态行。这里只做串口打印，绝不从控制热路径调用。
void printMasterStatusLine() {
#if MASTER_STATUS_LOG_ENABLED
#if MASTER_STATUS_SUMMARY_LOG_ENABLED || MASTER_STATUS_SYNC_LOG_ENABLED
    const MasterCommandPacket command = snapshotMasterCommand();
#endif
#if MASTER_STATUS_SUMMARY_LOG_ENABLED
    const SlaveTelemetryPacket telemetry = snapshotSlaveTelemetry();
    const uint32_t now_us = micros();
    const uint32_t telemetry_age_ms =
        (sysData.link.last_telemetry_seq == 0) ? 0 : ((now_us - sysData.link.last_rx_us) / 1000UL);
    const uint32_t ack_lag = static_cast<uint32_t>(command.seq - telemetry.ack_seq);
#endif

#if MASTER_STATUS_SYNC_LOG_ENABLED
    uint32_t x_encoder_frame = 0;
    uint16_t x_encoder_raw = 0;
    uint8_t x_encoder_status = 0;
    uint32_t y_encoder_frame = 0;
    uint16_t y_encoder_raw = 0;
    uint8_t y_encoder_status = 0;
    getMasterEncoderDiagnostics(x_encoder_frame, x_encoder_raw, x_encoder_status);
    getMasterYEncoderDiagnostics(y_encoder_frame, y_encoder_raw, y_encoder_status);

    const float sync_err_x_pct = sysData.master.x_pos - sysData.slave.x_pos;
    const float sync_err_y_pct = sysData.master.y_pos - sysData.slave.y_pos;
    const float sync_err_x_mm = (sync_err_x_pct * 0.01f) * PLOT_X_HALF_RANGE_MM;
    const float sync_err_y_mm = (sync_err_y_pct * 0.01f) * PLOT_Y_HALF_RANGE_MM;
#endif

#if MASTER_STATUS_SUMMARY_LOG_ENABLED && MASTER_STATUS_TIMING_LOG_ENABLED && MASTER_TIMING_STEP_DIAG_ENABLED
    const MasterControlHealthSnapshot health = getMasterControlHealthSnapshot();
#endif

#if MASTER_STATUS_SUMMARY_LOG_ENABLED && MASTER_STATUS_TIMING_LOG_ENABLED && MASTER_TIMING_STEP_DIAG_ENABLED && MASTER_STATUS_TIMING_DETAIL_LOG_ENABLED && MASTER_TIMING_DETAIL_DIAG_ENABLED
    const MasterControlTimingSnapshot timing = getMasterControlTimingSnapshot();
#endif

#if MASTER_STATUS_SUMMARY_LOG_ENABLED
    const uint16_t active_faults = getActiveFaultFlags();
    const uint16_t latched_faults = getLatchedFaultFlags();
    const MasterRuntimeModeSnapshot runtime = getMasterRuntimeModeSnapshot();
    const MasterAdc1DmaHealthSnapshot adc_health = snapshotMasterAdc1DmaHealth();

    Serial.println("[Master]");
    Serial.println("  mode:");
    Serial.printf("    app=%s runtime=%s requested=%s rejected=%u\n",
                  masterStartupAppModeName(),
                  masterRuntimeModeName(runtime.active_mode),
                  masterRuntimeModeName(runtime.requested_mode),
                  static_cast<unsigned int>(runtime.request_rejected));
    Serial.printf("    run=%s path=%s button=%s\n\n",
                  masterRunModeName(),
                  masterRunPathName(),
                  masterRuntimeButtonName(runtime.last_button));

    Serial.println("  link:");
    Serial.printf("    state=%u proto=%u flags=0x%04x age=%lums\n",
                  static_cast<unsigned int>(sysData.link.link_state),
                  static_cast<unsigned int>(command.mode),
                  static_cast<unsigned int>(command.command_flags),
                  static_cast<unsigned long>(telemetry_age_ms));
    Serial.printf("    tx=%lu ack=%lu lag=%lu last=%u\n",
                  static_cast<unsigned long>(command.seq),
                  static_cast<unsigned long>(telemetry.ack_seq),
                  static_cast<unsigned long>(ack_lag),
                  static_cast<unsigned int>(sysData.link.last_send_ok));
    Serial.printf("    send=%lu/%lu rx=%lu/%lu stale=%lu duplicate=%lu\n\n",
                  static_cast<unsigned long>(sysData.link.espnow_send_ok_count),
                  static_cast<unsigned long>(sysData.link.espnow_send_fail_count),
                  static_cast<unsigned long>(sysData.link.espnow_recv_ok_count),
                  static_cast<unsigned long>(sysData.link.espnow_recv_reject_count),
                  static_cast<unsigned long>(sysData.link.espnow_recv_stale_count),
                  static_cast<unsigned long>(sysData.link.espnow_recv_duplicate_count));

    Serial.println("  axis:");
    Serial.printf("    x: pos=%.1f%% norm=%.3f boundary=%u\n",
                  sysData.master.x_pos,
                  normToUnit(command.x_norm),
                  sysData.master.x_boundary_hit ? 1 : 0);
    Serial.printf("    y: pos=%.1f%% norm=%.3f boundary=%u\n\n",
                  sysData.master.y_pos,
                  normToUnit(command.y_norm),
                  sysData.master.y_boundary_hit ? 1 : 0);

#if MASTER_STATUS_SYNC_LOG_ENABLED
    Serial.println("  sync:");
    Serial.printf("    x: angle=%.2fdeg slave=%.1f%% err=%.1f%%/%.1fmm\n",
                  sysData.master.angle_deg,
                  sysData.slave.x_pos,
                  sync_err_x_pct,
                  sync_err_x_mm);
    Serial.printf("    y: angle=%.2fdeg slave=%.1f%% err=%.1f%%/%.1fmm\n\n",
                  sysData.master.y_angle_deg,
                  sysData.slave.y_pos,
                  sync_err_y_pct,
                  sync_err_y_mm);

    Serial.println("  encoder:");
    Serial.printf("    x: raw=%u stat=0x%01x frame=0x%06lx\n",
                  static_cast<unsigned int>(x_encoder_raw),
                  static_cast<unsigned int>(x_encoder_status),
                  static_cast<unsigned long>(x_encoder_frame));
    Serial.printf("    y: raw=%u stat=0x%01x frame=0x%06lx\n\n",
                  static_cast<unsigned int>(y_encoder_raw),
                  static_cast<unsigned int>(y_encoder_status),
                  static_cast<unsigned long>(y_encoder_frame));
#endif

    Serial.println("  motor:");
    Serial.printf("    common: ffb=%u strong=%u timing=%u\n",
                  MASTER_ENABLE_FORCE_FEEDBACK ? 1 : 0,
                  MASTER_ENABLE_STRONG_TORQUE_TEST ? 1 : 0,
                  MASTER_TIMING_DIAG_LEVEL);
    Serial.printf("    adc_dma: required=%u started=%u first=%u fault=%u runtime_latch=%u reason=%u seq=%lu age=%luus max_age=%luus fault_age=%luus stale_limit=%luus frame=%luB pool_frames=%lu pool_bytes=%lu per_ch=%u invalid=%lu samples=%lu read_err=%lu empty=%lu overflow=%lu stale=%lu consumer=%lu/%luus\n",
                  adc_health.required ? 1 : 0,
                  adc_health.started ? 1 : 0,
                  adc_health.first_frame_ready ? 1 : 0,
                  adc_health.fault_latched ? 1 : 0,
                  adc_health.runtime_fault_latch_enabled ? 1 : 0,
                  static_cast<unsigned int>(adc_health.fault_reason),
                  static_cast<unsigned long>(adc_health.frame_sequence),
                  static_cast<unsigned long>(adc_health.latest_age_us),
                  static_cast<unsigned long>(adc_health.latest_age_max_us),
                  static_cast<unsigned long>(adc_health.fault_age_us),
                  static_cast<unsigned long>(adc_health.stale_fault_us),
                  static_cast<unsigned long>(adc_health.frame_bytes),
                  static_cast<unsigned long>(adc_health.pool_frames),
                  static_cast<unsigned long>(adc_health.pool_bytes),
                  static_cast<unsigned int>(adc_health.samples_per_active_channel),
                  static_cast<unsigned long>(adc_health.invalid_frames),
                  static_cast<unsigned long>(adc_health.invalid_samples),
                  static_cast<unsigned long>(adc_health.read_errors),
                  static_cast<unsigned long>(adc_health.read_empty_count),
                  static_cast<unsigned long>(adc_health.pool_overflows),
                  static_cast<unsigned long>(adc_health.stale_control_cycles),
                  static_cast<unsigned long>(adc_health.consumer_last_us),
                  static_cast<unsigned long>(adc_health.consumer_max_us));
    Serial.printf("    x: hw=%u cmd=%.3fA iq=%.3fA id=%.3fA\n",
                  MASTER_ENABLE_X_MOTOR_HW ? 1 : 0,
                  sysData.master.target_current_a,
                  sysData.master.current_q_a,
                  sysData.master.current_d_a);
    Serial.printf("       vq=%.2fV vd=%.2fV\n",
                  sysData.master.voltage_q_v,
                  sysData.master.voltage_d_v);
    Serial.printf("    y: hw=%u cmd=%.3fA iq=%.3fA id=%.3fA\n",
                  MASTER_ENABLE_Y_MOTOR_HW ? 1 : 0,
                  sysData.master.y_target_current_a,
                  sysData.master.y_current_q_a,
                  sysData.master.y_current_d_a);
    Serial.printf("       vq=%.2fV vd=%.2fV\n\n",
                  sysData.master.y_voltage_q_v,
                  sysData.master.y_voltage_d_v);

    Serial.println("  draw:");
    Serial.printf("    pen: req=%u slave=%u uv=%u reason=0x%04x\n",
                  sysData.link.pen_req ? 1 : 0,
                  static_cast<unsigned int>(sysData.slave.pen_state),
                  sysData.link.uv_out ? 1 : 0,
                  static_cast<unsigned int>(sysData.slave.uv_block_reasons));
    Serial.printf("    state=%s progress=%u%% traj=%s task=%u\n",
                  drawStateName(sysData.slave.draw_state),
                  static_cast<unsigned int>(sysData.slave.draw_progress_pct),
                  trajectoryPhaseName(telemetry.trajectory_status_flags),
                  static_cast<unsigned int>(telemetry.trajectory_task_id));
    Serial.printf("    rx=%u/%u cursor=%u flags=0x%02x\n\n",
                  static_cast<unsigned int>(telemetry.trajectory_received_count),
                  static_cast<unsigned int>(telemetry.trajectory_segment_count),
                  static_cast<unsigned int>(telemetry.trajectory_segment_cursor),
                  static_cast<unsigned int>(telemetry.trajectory_status_flags));

    Serial.println("  fault:");
    Serial.printf("    active=0x%04x latched=0x%04x protocol=0x%04x\n",
                  static_cast<unsigned int>(active_faults),
                  static_cast<unsigned int>(latched_faults),
                  static_cast<unsigned int>(sysData.link.protocol_fault_flags));

#if MASTER_STATUS_TIMING_LOG_ENABLED && MASTER_TIMING_STEP_DIAG_ENABLED
    Serial.println();
    Serial.println("  timing:");
    Serial.printf("    health: level=%u ctrl_dt=%luus ctrl_max=%luus miss=%lu\n",
                  static_cast<unsigned int>(health.diag_level),
                  static_cast<unsigned long>(health.last_dt_us),
                  static_cast<unsigned long>(health.max_dt_us),
                  static_cast<unsigned long>(health.missed_ticks));
    Serial.printf("    step: last=%luus max=%luus over=%lu/%lu/%lu dt_over=%lu/%lu\n",
                  static_cast<unsigned long>(health.step_us),
                  static_cast<unsigned long>(health.step_max_us),
                  static_cast<unsigned long>(health.step_over_period_delta),
                  static_cast<unsigned long>(health.step_over_75pct_delta),
                  static_cast<unsigned long>(health.step_over_50pct_delta),
                  static_cast<unsigned long>(health.dt_over_1_5_count),
                  static_cast<unsigned long>(health.dt_over_2_count));
#if MASTER_STATUS_TIMING_DETAIL_LOG_ENABLED && MASTER_TIMING_DETAIL_DIAG_ENABLED
    Serial.println("    detail last/avg/max(us):");
    Serial.printf("      control=%lu/%lu/%lu logic=%lu/%lu/%lu motor=%lu/%lu/%lu\n",
                  static_cast<unsigned long>(timing.control_total.last_us),
                  static_cast<unsigned long>(timing.control_total.avg_us),
                  static_cast<unsigned long>(timing.control_total.max_us),
                  static_cast<unsigned long>(timing.control_logic.last_us),
                  static_cast<unsigned long>(timing.control_logic.avg_us),
                  static_cast<unsigned long>(timing.control_logic.max_us),
                  static_cast<unsigned long>(timing.motor_total.last_us),
                  static_cast<unsigned long>(timing.motor_total.avg_us),
                  static_cast<unsigned long>(timing.motor_total.max_us));
    Serial.printf("      move=%lu/%lu/%lu foc=%lu/%lu/%lu adc_sample=%lu/%lu/%lu sensor_spi=%lu/%lu/%lu adc_dma_consumer=%lu/%lu\n",
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
                  static_cast<unsigned long>(adc_health.consumer_last_us),
                  static_cast<unsigned long>(adc_health.consumer_max_us));
#endif
#endif
    Serial.println();
#elif MASTER_STATUS_SYNC_LOG_ENABLED
    Serial.println("[MasterSync]");
    Serial.println("  sync:");
    Serial.printf("    x: angle=%.2fdeg norm=%.3f slave=%.1f%% err=%.1f%%/%.1fmm\n",
                  sysData.master.angle_deg,
                  normToUnit(command.x_norm),
                  sysData.slave.x_pos,
                  sync_err_x_pct,
                  sync_err_x_mm);
    Serial.printf("    y: angle=%.2fdeg norm=%.3f slave=%.1f%% err=%.1f%%/%.1fmm\n\n",
                  sysData.master.y_angle_deg,
                  normToUnit(command.y_norm),
                  sysData.slave.y_pos,
                  sync_err_y_pct,
                  sync_err_y_mm);

    Serial.println("  encoder:");
    Serial.printf("    x: raw=%u stat=0x%01x frame=0x%06lx\n",
                  static_cast<unsigned int>(x_encoder_raw),
                  static_cast<unsigned int>(x_encoder_status),
                  static_cast<unsigned long>(x_encoder_frame));
    Serial.printf("    y: raw=%u stat=0x%01x frame=0x%06lx\n\n",
                  static_cast<unsigned int>(y_encoder_raw),
                  static_cast<unsigned int>(y_encoder_status),
                  static_cast<unsigned long>(y_encoder_frame));
#endif
#endif
}

void processMasterDiagShell() {
    static char line[80] = {};
    static size_t len = 0;

    while (Serial.available() > 0) {
        const int ch = Serial.read();
        if (ch < 0) {
            break;
        }

        if (ch == '\r' || ch == '\n') {
            line[len] = '\0';
            handleMasterDiagCommand(line);
            len = 0;
            line[0] = '\0';
            continue;
        }

        if (len + 1 < sizeof(line)) {
            line[len++] = static_cast<char>(ch);
        } else {
            len = 0;
            line[0] = '\0';
            Serial.println("[MasterDiag] command too long");
        }
    }
}
