#include "master/master_transport.h"

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#include "common/system_state.h"
#include "master/master_config.h"
#include "master/master_hardware.h"

// 主机 ESP-NOW 传输模块。
// 这里是无线包进出的唯一位置：主机发送 MasterCommandPacket，接收 SlaveTelemetryPacket。
// 控制任务不直接调用 ESP-NOW，避免无线栈时序影响 10 kHz 力反馈路径。

namespace {

// txPacket/rxPacket 保存最近一次完整包，供低频状态任务读取。
// 读写固定长度结构体时用 portMUX 短临界区，避免一个任务读到另一个回调写了一半的包。
MasterCommandPacket txPacket = {};
SlaveTelemetryPacket rxPacket = {};
portMUX_TYPE telemetryMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE commandMux = portMUX_INITIALIZER_UNLOCKED;

void recordTelemetryReject(uint16_t fault) {
    // 任何拒收都增加计数并锁存 fault，串口行可据此区分“没收到”和“收到了但校验失败”。
    sysData.espnow_recv_reject_count++;
    addLocalFault(fault);
}

void onDataSent(const uint8_t *mac, esp_now_send_status_t status) {
    (void)mac;
    // ESP-NOW 发送回调只记录结果，不重发、不打印、不触碰电机。
    // 真正的链路判断交给状态行中的 send_ok/send_fail 和 ack 滞后来观察。
    if (status == ESP_NOW_SEND_SUCCESS) {
        sysData.espnow_send_ok_count++;
        sysData.last_send_ok = 1;
    } else {
        sysData.espnow_send_fail_count++;
        sysData.last_send_ok = 0;
    }
}

void onDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
    (void)mac;

    // 长度先验校验：固定长度二进制包不允许短包或长包进入 memcpy 后的业务解析。
    if (len != static_cast<int>(sizeof(SlaveTelemetryPacket))) {
        recordTelemetryReject(FAULT_PACKET_SIZE);
        return;
    }

    // 复制到栈上临时对象后再校验，避免直接在回调参数内做多处读取。
    SlaveTelemetryPacket packet = {};
    memcpy(&packet, incomingData, sizeof(packet));

    // 版本/type/checksum 任一不匹配都会拒收。版本不一致通常表示两端代码或协议文档漂移。
    if (!validateSlaveTelemetry(packet)) {
        const uint16_t fault = (packet.version == PROTOCOL_VERSION)
                                   ? FAULT_CHECKSUM_ERROR
                                   : FAULT_VERSION_MISMATCH;
        recordTelemetryReject(fault);
        return;
    }

    // 序号必须向前推进，防止旧遥测覆盖较新的状态。
    if (sysData.last_telemetry_seq != 0 && !isNewerSeq(packet.seq, sysData.last_telemetry_seq)) {
        recordTelemetryReject(FAULT_STALE_SEQUENCE);
        return;
    }

    // 通过校验后才发布为最新快照。临界区只包住结构体赋值，保持回调很短。
    portENTER_CRITICAL(&telemetryMux);
    rxPacket = packet;
    portEXIT_CRITICAL(&telemetryMux);

    // 将遥测转换成适合串口显示的 sysData 标量。
    // 注意 slave_angle_deg 这里由 x_actual_norm 反推，只是监视近似值，不是从机编码器原始角。
    sysData.slave_x_pos = normToPercent(packet.x_actual_norm);
    sysData.slave_y_pos = normToPercent(packet.y_actual_norm);
    sysData.slave_angle_deg = normToAngleDeg(packet.x_actual_norm);
    sysData.boundary_hit = (packet.fault_flags & FAULT_BOUNDARY_HIT) != 0;
    sysData.current_mode = packet.mode;
    publishProtocolFaults(packet.fault_flags);
    sysData.last_telemetry_seq = packet.seq;
    sysData.last_rx_us = micros();
    sysData.espnow_recv_ok_count++;
}

}  // namespace

void setupMasterEspNow() {
    // ESP-NOW 要求 Wi-Fi 处于 STA 或 AP/STA 模式；本项目固定使用 STA。
    WiFi.mode(WIFI_STA);
    if (esp_now_init() != ESP_OK) {
        // 当前没有单独的无线初始化 fault，暂用 COMMAND_TIMEOUT 表示链路不可用。
        addLocalFault(FAULT_COMMAND_TIMEOUT);
        return;
    }

    // 回调注册后，收发结果都只更新计数和快照，不承担控制输出。
    esp_now_register_recv_cb(onDataRecv);
    esp_now_register_send_cb(onDataSent);

    // 当前阶段使用硬编码从机 MAC；channel=0 表示跟随当前 Wi-Fi 信道，不启用加密。
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, kMasterPeerSlaveAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        addLocalFault(FAULT_COMMAND_TIMEOUT);
    }
}

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

void sendMasterCommand(uint32_t seq, uint32_t now_us) {
    // Core 0 通信任务周期调用本函数。它从 sysData 读取最新控制状态，
    // 再组包发送，不主动读取电机或执行控制。
#if MASTER_FORCE_PEN_DOWN_FOR_TEST
    // 当前无紫光灯、无按钮阶段用于验证落笔命令链路，所以显式强制 pen_down=1。
    // 接入真实紫光灯或主机按钮前，应关闭 MASTER_FORCE_PEN_DOWN_FOR_TEST。
    sysData.pen_down = true;
#else
    // 正式接线阶段由主机按钮控制落笔请求。
    sysData.pen_down = readMasterPenButtonDown();
#endif

    // 如果已经收到过遥测但之后超时，则锁存遥测超时故障。
    if (sysData.last_telemetry_seq != 0 && now_us - sysData.last_rx_us > TELEMETRY_TIMEOUT_US) {
        addLocalFault(FAULT_TELEMETRY_TIMEOUT);
    }

    // 发布当前本机故障视图；FAULT_NONE 不会清除已锁存故障。
    publishProtocolFaults(FAULT_NONE);

    // 协议包只携带归一化坐标和落笔状态，不携带电机角度、电流或硬件细节。
    MasterCommandPacket packet = {};
    packet.flags = sysData.pen_down ? PACKET_FLAG_PEN_DOWN : PACKET_FLAG_NONE;
    packet.seq = seq;
    packet.timestamp_us = now_us;
    packet.x_norm = percentToNorm(sysData.master_x_pos);
    packet.y_norm = 0;
    packet.pen_down = sysData.pen_down ? 1 : 0;
    packet.mode = sysData.current_mode;
    finalizeMasterCommand(packet);

    // 保存最近一次发送包，状态任务会用它打印 tx 序号等信息。
    portENTER_CRITICAL(&commandMux);
    txPacket = packet;
    portEXIT_CRITICAL(&commandMux);

    // esp_now_send 只表示提交到 ESP-NOW 栈；真正发送结果会在 onDataSent 回调中统计。
    const esp_err_t send_result =
        esp_now_send(kMasterPeerSlaveAddress, reinterpret_cast<uint8_t *>(&packet), sizeof(packet));
    if (send_result != ESP_OK) {
        sysData.espnow_send_fail_count++;
        sysData.last_send_ok = 0;
    }
}

MasterCommandPacket snapshotMasterCommand() {
    // 状态任务读取发送包快照。临界区尽量短，只复制固定长度结构体。
    MasterCommandPacket packet = {};
    portENTER_CRITICAL(&commandMux);
    packet = txPacket;
    portEXIT_CRITICAL(&commandMux);
    return packet;
}

SlaveTelemetryPacket snapshotSlaveTelemetry() {
    // 状态任务读取接收遥测快照。这里不做校验，因为写入 rxPacket 前已经校验过。
    SlaveTelemetryPacket packet = {};
    portENTER_CRITICAL(&telemetryMux);
    packet = rxPacket;
    portEXIT_CRITICAL(&telemetryMux);
    return packet;
}
