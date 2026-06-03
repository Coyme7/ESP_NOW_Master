#include "master/config/core/master_comm_config.h"

// 固定从机 ESP-NOW MAC。
// 更换从机开发板时替换为新从机 Wi-Fi STA MAC。
const uint8_t kMasterPeerSlaveAddress[6] = {
    0x3c,
    0x0f,
    0x02,
    0x6f,
    0x05,
    0x28,
};
