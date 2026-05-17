#include "master/comm/master_transport.h"

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#include "common/state/system_state.h"
#include "master/comm/master_packet_builder.h"
#include "master/comm/slave_telemetry_reader.h"
#include "master/config/master_config.h"
#include "master/hardware/master_board_io.h"

// 主机 ESP-NOW 传输模块。
// 这里是无线包进出的唯一位置：主机发送 MasterCommandPacket，接收 SlaveTelemetryPacket。
// 控制任务不直接调用 ESP-NOW，避免无线栈时序影响 200us / 5kHz 力反馈路径。

namespace {

// txPacket/rxPacket 保存最近一次完整包，供低频状态任务读取。
// 读写固定长度结构体时用 portMUX 短临界区，避免一个任务读到另一个回调写了一半的包。
MasterCommandPacket txPacket = {};
SlaveTelemetryPacket rxPacket = {};
SlaveTelemetryPacket rxPendingPacket = {};
portMUX_TYPE telemetryMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE telemetryPendingMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE commandMux = portMUX_INITIALIZER_UNLOCKED;
volatile bool rxPending = false;
volatile int rxPacketLen = 0;

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
#if MASTER_FORCE_PEN_DOWN_FOR_TEST
    // 当前无紫光灯、无按钮阶段用于验证落笔命令链路，所以显式强制 pen_down=1。
    // 接入真实紫光灯或主机按钮前，应关闭 MASTER_FORCE_PEN_DOWN_FOR_TEST。
    sysData.link.pen_down = true;
#else
    // 正式接线阶段由主机按钮控制落笔请求。
    sysData.link.pen_down = readMasterPenButtonDown();
#endif

    // 如果已经收到过遥测但之后超时，则锁存遥测超时故障，并在本周期 active_faults 中暴露。
    uint16_t active_faults = FAULT_NONE;
    if (sysData.link.last_telemetry_seq != 0 &&
        now_us - sysData.link.last_rx_us > TELEMETRY_TIMEOUT_US) {
        active_faults |= FAULT_TELEMETRY_TIMEOUT;
        addLocalFault(FAULT_TELEMETRY_TIMEOUT);
    }

    // 发布当前本机故障视图；FAULT_NONE 不会清除已锁存故障。
    publishProtocolFaults(active_faults);

    // 协议包只携带归一化坐标和落笔状态，不携带电机角度、电流或硬件细节。
    MasterCommandPacket packet =
        buildMasterCommandPacket(seq,
                                 now_us,
                                 sysData.master.x_pos,
                                 sysData.master.y_pos,
                                 sysData.link.pen_down,
                                 sysData.link.current_mode);

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
