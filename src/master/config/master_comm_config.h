#pragma once

#include <stdint.h>

// 主机 ESP-NOW 固定信道。
// 固定到 1 可以避免 channel=0 跟随 Wi-Fi 当前信道造成链路不确定。
// 从机必须使用相同信道。
#ifndef MASTER_ESPNOW_CHANNEL
#define MASTER_ESPNOW_CHANNEL 1
#endif

// 当前单轴联调使用固定从机 MAC。加入配对流程前，不在运行时自动扫描对端。
// 真实 6 字节地址定义在 master_config.cpp。
extern const uint8_t kMasterPeerSlaveAddress[6];

