#include "master/comm/slave_telemetry_reader.h"

#include <Arduino.h>

#include "common/protocol/packet_codec.h"
#include "common/protocol/protocol_units.h"
#include "common/state/system_state.h"

namespace {

// 记录遥测包拒收原因，例如长度、校验、版本或序号异常。
void recordTelemetryReject(uint16_t fault) {
    // 任何拒收都增加计数并锁存 fault，串口行可区分“没收到”和“校验失败”。
    sysData.link.espnow_recv_reject_count++;
    addLocalFault(fault);
}

void recordStaleTelemetryOnly() {
    // stale 属于链路乱序或旧遥测，只计数，不覆盖状态、不锁存 fault。
    sysData.link.espnow_recv_reject_count++;
    sysData.link.espnow_recv_stale_count++;
}

void recordDuplicateTelemetryOnly() {
    // duplicate 属于链路重复遥测，只计数，不覆盖状态、不锁存 fault。
    sysData.link.espnow_recv_reject_count++;
    sysData.link.espnow_recv_duplicate_count++;
}

}  // namespace

// 校验并应用从机遥测。只有通过校验且序号更新的包才会刷新 sysData。
bool applySlaveTelemetryPacket(const SlaveTelemetryPacket &packet, int packet_len) {
    // 包长不等于约定结构体大小时，说明两端协议不同步或收到损坏包。
    if (packet_len != static_cast<int>(sizeof(SlaveTelemetryPacket))) {
        recordTelemetryReject(FAULT_PACKET_SIZE);
        return false;
    }

    // 校验失败的包不能用于更新 ack/actual，否则会把坏数据带入状态行和后续控制。
    if (!validateSlaveTelemetry(packet)) {
        const uint16_t fault = (packet.version == PROTOCOL_VERSION)
                                   ? FAULT_CHECKSUM_ERROR
                                   : FAULT_VERSION_MISMATCH;
        recordTelemetryReject(fault);
        return false;
    }

    if (sysData.link.last_telemetry_seq != 0 &&
        packet.seq == sysData.link.last_telemetry_seq) {
        recordDuplicateTelemetryOnly();
        return false;
    }

    if (sysData.link.last_telemetry_seq != 0 &&
        !isNewerSeq(packet.seq, sysData.link.last_telemetry_seq)) {
        recordStaleTelemetryOnly();
        return false;
    }

    sysData.slave.x_pos = normToPercent(packet.x_actual_norm);
    sysData.slave.y_pos = normToPercent(packet.y_actual_norm);
    sysData.slave.angle_deg = normToAngleDeg(packet.x_actual_norm);
    sysData.slave.boundary_hit = (packet.fault_flags & FAULT_BOUNDARY_HIT) != 0;
    sysData.slave.pen_state = packet.pen_state;
    sysData.slave.draw_state = packet.draw_state;
    sysData.slave.draw_progress_pct = packet.draw_progress_pct;
    sysData.slave.trajectory_task_id = packet.trajectory_task_id;
    sysData.slave.trajectory_segment_count = packet.trajectory_segment_count;
    sysData.slave.trajectory_segment_cursor = packet.trajectory_segment_cursor;
    sysData.slave.trajectory_received_count = packet.trajectory_received_count;
    sysData.slave.trajectory_status_flags = packet.trajectory_status_flags;
    sysData.slave.trajectory_received_mask_low = packet.trajectory_received_mask_low;
    sysData.slave.trajectory_received_mask_high = packet.trajectory_received_mask_high;
    sysData.slave.uv_block_reasons = packet.uv_block_reasons;
    sysData.slave.uv_interlock_blocked = packet.uv_block_reasons != UV_BLOCK_NONE;
    sysData.link.uv_out = packet.uv_out != 0;
    sysData.link.current_mode = packet.mode;
    sysData.link.link_state = packet.link_state;
    publishProtocolFaults(packet.fault_flags);
    sysData.link.last_telemetry_seq = packet.seq;
    sysData.link.last_rx_us = micros();
    sysData.link.espnow_recv_ok_count++;
    return true;
}
