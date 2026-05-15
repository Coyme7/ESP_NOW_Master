#include "master/comm/master_packet_builder.h"

#include "common/protocol/packet_codec.h"
#include "common/protocol/protocol_units.h"

// 从当前主机状态构造命令包：角度归一化值作为 target_x_norm 发给从机。
MasterCommandPacket buildMasterCommandPacket(uint32_t seq,
                                             uint32_t now_us,
                                             float master_x_percent,
                                             bool pen_down,
                                             uint8_t mode) {
    // 先清零整包，避免未显式赋值的保留字段携带栈上的随机值。
    MasterCommandPacket packet = {};
    packet.flags = pen_down ? PACKET_FLAG_PEN_DOWN : PACKET_FLAG_NONE;
    packet.seq = seq;
    packet.timestamp_us = now_us;
    packet.x_norm = percentToNorm(master_x_percent);
    packet.y_norm = 0;
    packet.pen_down = pen_down ? 1 : 0;
    packet.mode = mode;
    // finalize 会写入协议字段和 checksum，必须在所有业务字段填完后调用。
    finalizeMasterCommand(packet);
    return packet;
}

