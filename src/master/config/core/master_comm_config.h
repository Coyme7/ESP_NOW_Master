#pragma once

#include <stdint.h>

// 主机 ESP-NOW 固定信道。
// 必须与从机信道一致，避免 channel=0 跟随 Wi-Fi 当前信道。
#ifndef MASTER_ESPNOW_CHANNEL
#define MASTER_ESPNOW_CHANNEL 1
#endif

// 固定从机 MAC，字节顺序与 ESP-NOW peer 地址一致。
// 真实地址定义在 master_comm_config.cpp。
extern const uint8_t kMasterPeerSlaveAddress[6];
