#pragma once

#include <stdint.h>

#include "common/protocol/protocol_types.h"

MasterCommandPacket buildMasterCommandPacket(uint32_t seq,
                                             uint32_t now_us,
                                             float master_x_percent,
                                             float master_y_percent,
                                             bool pen_down,
                                             uint8_t mode);
