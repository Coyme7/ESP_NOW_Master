#include "master/comm/master_transport.h"

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#include "common/protocol/packet_codec.h"
#include "common/state/system_state.h"
#include "common/timing/link_timing.h"
#include "master/comm/master_packet_builder.h"
#include "master/comm/slave_telemetry_reader.h"
#include "master/config/master_config.h"
#include "master/hardware/master_board_io.h"
#include "master/modes/auto_draw/auto_draw_mode.h"
#include "master/modes/mode_manager.h"
#include "master/modes/mode_protocol_map.h"

// 主机 ESP-NOW 传输模块。
// 这里是无线包进出的唯一位置：主机发送 MasterCommandPacket / TrajectorySegmentPacket，
// 接收 SlaveTelemetryPacket。
// 控制任务不直接调用 ESP-NOW，避免无线栈时序影响力反馈热路径。

namespace {

// txPacket/rxPacket 保存最近一次完整包，供低频状态任务读取。
// 读写固定长度结构体时用 portMUX 短临界区，避免一个任务读到另一个回调写了一半的包。
MasterCommandPacket txPacket = {};
TrajectorySegmentPacket txTrajectoryPacket = {};
SlaveTelemetryPacket rxPacket = {};
SlaveTelemetryPacket rxPendingPacket = {};
portMUX_TYPE telemetryMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE telemetryPendingMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE commandMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE trajectoryMux = portMUX_INITIALIZER_UNLOCKED;
volatile bool rxPending = false;
volatile int rxPacketLen = 0;
uint32_t trajectoryTxSeq = 0;
uint8_t nextTrajectorySegmentIndex = 0;
uint32_t autoDrawModeRequestSeq = 0;
bool autoDrawModeRequestPending = false;
bool autoDrawModeAccepted = false;

// ESP-NOW 发送完成回调：只记录成功/失败，不做复杂计算。
void onDataSent(const uint8_t *mac, esp_now_send_status_t status) {
    (void)mac;
    // ESP-NOW 发送回调只记录结果，不重发、不打印、不触碰电机。
    // 真正的链路判断交给状态行中的 send_ok/send_fail 和 ack 滞后来观察。
    if (status == ESP_NOW_SEND_SUCCESS) {
        sysData.link.espnow_send_ok_count++;
        sysData.link.last_send_ok = 1;
    } else {
        sysData.link.espnow_send_fail_count++;
        sysData.link.last_send_ok = 0;
    }
}

// ESP-NOW 接收回调：只处理从机遥测包，并把解析工作交给 telemetry reader。
void onDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
    (void)mac;

    portENTER_CRITICAL(&telemetryPendingMux);
    rxPacketLen = (incomingData != nullptr) ? len : 0;
    if (rxPacketLen > 0 && rxPacketLen <= static_cast<int>(sizeof(rxPendingPacket))) {
        memcpy(&rxPendingPacket, incomingData, static_cast<size_t>(rxPacketLen));
    }
    rxPending = true;
    portEXIT_CRITICAL(&telemetryPendingMux);
}

}  // namespace

bool seqReached(uint32_t seq, uint32_t target) {
    return seq == target || isNewerSeq(seq, target);
}

uint64_t telemetryReceivedMask(const SlaveTelemetryPacket &telemetry) {
    return static_cast<uint64_t>(telemetry.trajectory_received_mask_low) |
           (static_cast<uint64_t>(telemetry.trajectory_received_mask_high) << 32);
}

uint8_t selectNextTrajectorySegmentIndex(uint8_t segment_count) {
    if (segment_count == 0U) {
        return 0U;
    }
    if (nextTrajectorySegmentIndex >= segment_count) {
        nextTrajectorySegmentIndex = 0U;
    }

    const SlaveTelemetryPacket telemetry = snapshotSlaveTelemetry();
    const bool telemetry_matches_current_task =
        telemetry.trajectory_task_id == masterAutoDrawTaskId() &&
        telemetry.trajectory_segment_count == segment_count;
    if (telemetry_matches_current_task &&
        telemetry.trajectory_received_count < telemetry.trajectory_segment_count) {
        const uint64_t received_mask = telemetryReceivedMask(telemetry);
        for (uint8_t offset = 0U; offset < segment_count; ++offset) {
            const uint8_t index =
                static_cast<uint8_t>((nextTrajectorySegmentIndex + offset) % segment_count);
            if ((received_mask & (1ULL << index)) == 0ULL) {
                return index;
            }
        }
    }

    return nextTrajectorySegmentIndex;
}

void updateAutoDrawModeHandshake(uint32_t now_us,
                                 const MasterRuntimeModeSnapshot &runtime,
                                 const MasterCommandPacket &packet,
                                 uint8_t protocol_mode) {
    if (runtime.active_mode != MASTER_RUNTIME_MODE_AUTO_DRAW) {
        autoDrawModeRequestPending = false;
        autoDrawModeAccepted = false;
        nextTrajectorySegmentIndex = 0;
        return;
    }

    if (!autoDrawModeRequestPending) {
        autoDrawModeRequestSeq = packet.seq;
        autoDrawModeRequestPending = true;
        autoDrawModeAccepted = false;
        nextTrajectorySegmentIndex = 0;
    }

    const bool telemetry_fresh =
        sysData.link.last_telemetry_seq != 0 &&
        (now_us - sysData.link.last_rx_us) <= TELEMETRY_TIMEOUT_US;
    const SlaveTelemetryPacket telemetry = snapshotSlaveTelemetry();
    autoDrawModeAccepted =
        telemetry_fresh &&
        telemetry.mode == protocol_mode &&
        seqReached(telemetry.ack_seq, autoDrawModeRequestSeq);
}

bool shouldSendAutoDrawSegments(uint32_t now_us,
                                const MasterRuntimeModeSnapshot &runtime,
                                uint8_t protocol_mode) {
    if (runtime.active_mode != MASTER_RUNTIME_MODE_AUTO_DRAW) {
        nextTrajectorySegmentIndex = 0;
        return false;
    }
    if (masterAutoDrawSegmentCount() == 0U) {
        return false;
    }

    const bool telemetry_fresh =
        sysData.link.last_telemetry_seq != 0 &&
        (now_us - sysData.link.last_rx_us) <= TELEMETRY_TIMEOUT_US;
    (void)protocol_mode;
    if (!telemetry_fresh || !autoDrawModeAccepted) {
        nextTrajectorySegmentIndex = 0;
        return false;
    }

    const SlaveTelemetryPacket telemetry = snapshotSlaveTelemetry();
    const bool telemetry_matches_current_task =
        telemetry.trajectory_task_id == masterAutoDrawTaskId() &&
        telemetry.trajectory_segment_count == masterAutoDrawSegmentCount();
    if (telemetry_matches_current_task) {
        const bool slave_ready_or_busy =
            (telemetry.trajectory_status_flags &
             (TRAJECTORY_STATUS_READY |
              TRAJECTORY_STATUS_RUNNING |
              TRAJECTORY_STATUS_COMPLETE)) != 0U;
        const bool slave_received_all =
            telemetry.trajectory_segment_count > 0U &&
            telemetry.trajectory_received_count >= telemetry.trajectory_segment_count;
        if (slave_ready_or_busy || slave_received_all) {
            return false;
        }
    }

    const bool slave_has_task =
        sysData.slave.draw_state == DRAW_STATE_RUNNING ||
        sysData.slave.draw_state == DRAW_STATE_FINISHED;
    return !slave_has_task;
}

void sendMasterTrajectorySegment(uint32_t now_us, uint16_t command_flags) {
    const uint8_t segment_count = masterAutoDrawSegmentCount();
    if (segment_count == 0U) {
        return;
    }
    const uint8_t segment_index = selectNextTrajectorySegmentIndex(segment_count);

    MasterAutoDrawSegmentSpec segment = {};
    if (!masterAutoDrawSegmentAt(segment_index, segment)) {
        nextTrajectorySegmentIndex = 0;
        return;
    }

    TrajectorySegmentPacket packet = {};
    packet.flags = command_flags;
    packet.seq = trajectoryTxSeq++;
    packet.timestamp_us = now_us;
    packet.task_id = masterAutoDrawTaskId();
    packet.segment_index = segment_index;
    packet.segment_count = segment_count;
    packet.start_x_mm_q10 = segment.start_x_mm_q10;
    packet.start_y_mm_q10 = segment.start_y_mm_q10;
    packet.end_x_mm_q10 = segment.end_x_mm_q10;
    packet.end_y_mm_q10 = segment.end_y_mm_q10;
    packet.feed_mm_s_q10 = segment.feed_mm_s_q10;
    packet.pen_req = segment.pen_req;
    finalizeTrajectorySegment(packet);

    portENTER_CRITICAL(&trajectoryMux);
    txTrajectoryPacket = packet;
    portEXIT_CRITICAL(&trajectoryMux);

    const esp_err_t send_result =
        esp_now_send(kMasterPeerSlaveAddress, reinterpret_cast<uint8_t *>(&packet), sizeof(packet));
    if (send_result != ESP_OK) {
        sysData.link.espnow_send_fail_count++;
        sysData.link.last_send_ok = 0;
    }

    nextTrajectorySegmentIndex = static_cast<uint8_t>(segment_index + 1U);
    if (nextTrajectorySegmentIndex >= segment_count) {
        nextTrajectorySegmentIndex = 0;
    }
}

// 初始化 Wi-Fi STA、固定信道、ESP-NOW peer 和收发回调。
void setupMasterEspNow() {
    // ESP-NOW 要求 Wi-Fi 处于 STA 或 AP/STA 模式；本项目固定使用 STA。
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    (void)esp_wifi_set_ps(WIFI_PS_NONE);
    (void)esp_wifi_set_channel(MASTER_ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
    if (esp_now_init() != ESP_OK) {
        // 当前没有单独的无线初始化 fault，暂用 COMMAND_TIMEOUT 表示链路不可用。
        addLocalFault(FAULT_COMMAND_TIMEOUT);
        return;
    }

    // 回调注册后，收发结果都只更新计数和快照，不承担控制输出。
    esp_now_register_recv_cb(onDataRecv);
    esp_now_register_send_cb(onDataSent);

    // 当前阶段使用硬编码从机 MAC；固定信道，不启用加密。
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, kMasterPeerSlaveAddress, 6);
    peerInfo.channel = MASTER_ESPNOW_CHANNEL;
    peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        addLocalFault(FAULT_COMMAND_TIMEOUT);
    }
}

// 打印主机 MAC 和信道，便于从机配置 peer 地址。
void printMasterEspNowIdentity() {
    // 打印本机 STA MAC 和目标从机 MAC，方便现场确认两块板是否烧录反了或 MAC 写错。
    uint8_t local_mac[6] = {};
    const esp_err_t result = esp_wifi_get_mac(WIFI_IF_STA, local_mac);
    if (result == ESP_OK) {
        Serial.printf("[Master] sta_mac=%02x:%02x:%02x:%02x:%02x:%02x peer_slave=%02x:%02x:%02x:%02x:%02x:%02x\n",
                      local_mac[0],
                      local_mac[1],
                      local_mac[2],
                      local_mac[3],
                      local_mac[4],
                      local_mac[5],
                      kMasterPeerSlaveAddress[0],
                      kMasterPeerSlaveAddress[1],
                      kMasterPeerSlaveAddress[2],
                      kMasterPeerSlaveAddress[3],
                      kMasterPeerSlaveAddress[4],
                      kMasterPeerSlaveAddress[5]);
    }
}

// 通信任务周期调用：检查遥测超时并更新链路状态。
void processMasterTelemetry() {
    SlaveTelemetryPacket packet = {};
    int packet_len = 0;
    bool has_packet = false;

    portENTER_CRITICAL(&telemetryPendingMux);
    if (rxPending) {
        packet_len = rxPacketLen;
        if (packet_len == static_cast<int>(sizeof(packet))) {
            packet = rxPendingPacket;
        }
        rxPending = false;
        has_packet = true;
    }
    portEXIT_CRITICAL(&telemetryPendingMux);

    if (!has_packet) {
        return;
    }

    if (!applySlaveTelemetryPacket(packet, packet_len)) {
        return;
    }

    portENTER_CRITICAL(&telemetryMux);
    rxPacket = packet;
    portEXIT_CRITICAL(&telemetryMux);

}

// 通信任务周期调用：构造命令包并通过 ESP-NOW 发给固定从机。
void sendMasterCommand(uint32_t seq, uint32_t now_us) {
    // Core 0 通信任务周期调用本函数。它从 sysData 读取最新控制状态，
    // 再组包发送，不主动读取电机或执行控制。
#if MASTER_ENABLE_FORCE_PEN_DOWN_TEST
    // 当前无紫光灯、无按钮阶段用于验证落笔请求链路，所以显式强制 pen_req=1。
    // 接入真实紫光灯或主机按钮前，应关闭 MASTER_ENABLE_FORCE_PEN_DOWN_TEST。
    sysData.link.pen_req = true;
#else
    // 正式接线阶段由主机按钮控制落笔请求。
    sysData.link.pen_req = readMasterPenButtonDown();
#endif

    // 如果已经收到过遥测但之后超时，则锁存遥测超时故障，并在本周期 active_faults 中暴露。
    uint16_t active_faults = FAULT_NONE;
    if (sysData.link.last_telemetry_seq != 0 &&
        now_us - sysData.link.last_rx_us > TELEMETRY_TIMEOUT_US) {
        active_faults |= FAULT_TELEMETRY_TIMEOUT;
        sysData.link.link_state = LINK_TIMEOUT;
        addLocalFault(FAULT_TELEMETRY_TIMEOUT);
    } else if (sysData.link.last_telemetry_seq != 0) {
        sysData.link.link_state = LINK_CONNECTED;
    }

    // 发布当前本机故障视图；FAULT_NONE 不会清除已锁存故障。
    publishProtocolFaults(active_faults);

    // 协议包只携带归一化坐标和落笔状态，不携带电机角度、电流或硬件细节。
    const MasterRuntimeModeSnapshot runtime = getMasterRuntimeModeSnapshot();
    const uint8_t protocol_mode = masterProtocolModeForRuntime(runtime.active_mode);
    const uint16_t command_flags = masterCommandFlagsForRuntime(runtime.active_mode);
    sysData.link.current_mode = protocol_mode;
    MasterCommandPacket packet =
        buildMasterCommandPacket(seq,
                                 now_us,
                                 sysData.master.x_pos,
                                 sysData.master.y_pos,
                                 sysData.link.pen_req,
                                 sysData.link.current_mode,
                                 command_flags);

    // 保存最近一次发送包，状态任务会用它打印 tx 序号等信息。
    portENTER_CRITICAL(&commandMux);
    txPacket = packet;
    portEXIT_CRITICAL(&commandMux);

    // esp_now_send 只表示提交到 ESP-NOW 栈；真正发送结果会在 onDataSent 回调中统计。
    const esp_err_t send_result =
        esp_now_send(kMasterPeerSlaveAddress, reinterpret_cast<uint8_t *>(&packet), sizeof(packet));
    if (send_result != ESP_OK) {
        sysData.link.espnow_send_fail_count++;
        sysData.link.last_send_ok = 0;
    }

    updateAutoDrawModeHandshake(now_us, runtime, packet, protocol_mode);
    if (shouldSendAutoDrawSegments(now_us, runtime, protocol_mode)) {
        sendMasterTrajectorySegment(now_us, command_flags);
    }
}

// 返回最近一次发送的命令包快照，供状态输出或调试使用。
MasterCommandPacket snapshotMasterCommand() {
    // 状态任务读取发送包快照。临界区尽量短，只复制固定长度结构体。
    MasterCommandPacket packet = {};
    portENTER_CRITICAL(&commandMux);
    packet = txPacket;
    portEXIT_CRITICAL(&commandMux);
    return packet;
}

// 返回最近一次有效从机遥测包快照。
SlaveTelemetryPacket snapshotSlaveTelemetry() {
    // 状态任务读取接收遥测快照。这里不做校验，因为写入 rxPacket 前已经校验过。
    SlaveTelemetryPacket packet = {};
    portENTER_CRITICAL(&telemetryMux);
    packet = rxPacket;
    portEXIT_CRITICAL(&telemetryMux);
    return packet;
}

MasterAutoDrawTransportStatus snapshotMasterAutoDrawTransportStatus() {
    MasterAutoDrawTransportStatus status = {};
    status.mode_request_seq = autoDrawModeRequestSeq;
    status.trajectory_tx_seq = trajectoryTxSeq;
    status.next_segment_index = nextTrajectorySegmentIndex;
    status.mode_request_pending = autoDrawModeRequestPending ? 1U : 0U;
    status.mode_accepted = autoDrawModeAccepted ? 1U : 0U;
    return status;
}
