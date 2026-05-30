#pragma once

#include <stdint.h>

#include "common/protocol/protocol_types.h"

// master_transport
// 职责：主机 ESP-NOW 初始化、发送 MasterCommand/TrajectorySegment、
// 接收 SlaveTelemetry，并维护低频诊断快照。
struct MasterAutoDrawTransportStatus {
    uint32_t mode_request_seq;
    uint32_t trajectory_tx_seq;
    uint8_t next_segment_index;
    uint8_t mode_request_pending;
    uint8_t mode_accepted;
};

void setupMasterEspNow();
void printMasterEspNowIdentity();
void processMasterTelemetry();
void sendMasterCommand(uint32_t seq, uint32_t now_us);

MasterCommandPacket snapshotMasterCommand();
SlaveTelemetryPacket snapshotSlaveTelemetry();
MasterAutoDrawTransportStatus snapshotMasterAutoDrawTransportStatus();
