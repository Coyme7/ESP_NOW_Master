#include "master/config/core/master_comm_config.h"

// 固定从机 ESP-NOW MAC。
// 更换从机开发板时替换为新从机 Wi-Fi STA MAC。
const uint8_t kMasterPeerSlaveAddress[6] = {
    0x24,
    0x58,
    0x7c,
    0xd0,
    0xab,
    0x2c,
};
