#pragma once

#include <stdint.h>

// 主机 ESP-NOW 固定信道。
// 默认值：1，当前主从联调固定在 2.4GHz 信道 1。
// 用途：避免 channel=0 跟随 Wi-Fi 当前信道造成链路不确定。
// 风险：主从信道不一致会导致 ESP-NOW 无法通信。
// 依赖：从机 SLAVE_ESPNOW_CHANNEL 必须使用相同值。
#ifndef MASTER_ESPNOW_CHANNEL
#define MASTER_ESPNOW_CHANNEL 1
#endif

static_assert(MASTER_ESPNOW_CHANNEL >= 1 && MASTER_ESPNOW_CHANNEL <= 14,
              "MASTER_ESPNOW_CHANNEL must be in 1..14");

// 当前单轴联调使用固定从机 MAC。加入配对流程前，不在运行时自动扫描对端。
// 真实 6 字节地址定义在 master_config.cpp。
extern const uint8_t kMasterPeerSlaveAddress[6];
