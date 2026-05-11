#include "master/master_status.h"

#include <Arduino.h>

#include "common/system_state.h"
#include "master/master_hardware.h"
#include "master/master_transport.h"

// 主机状态打印模块。
// 这里允许 Serial.printf，因为它只在 Core 0 的低频状态任务中运行；
// 任何 10 kHz 控制路径都不应调用本函数。

void printMasterStatusLine() {
    // 读取最近一次已发送命令和最近一次已接收遥测的快照。
    // 快照函数内部使用短临界区，避免状态行读到半包。
    const MasterCommandPacket command = snapshotMasterCommand();
    const SlaveTelemetryPacket telemetry = snapshotSlaveTelemetry();

    // 如果还没收到从机遥测，age 显示 0，避免第一次启动时出现无意义的大数。
    const uint32_t now_us = micros();
    const uint32_t telemetry_age_ms =
        (sysData.last_telemetry_seq == 0) ? 0 : ((now_us - sysData.last_rx_us) / 1000UL);

    uint32_t encoder_frame = 0;
    uint16_t encoder_raw = 0;
    uint8_t encoder_status = 0;
    getMasterEncoderDiagnostics(encoder_frame, encoder_raw, encoder_status);

    // 字段含义在 Instruction.md 中有对应说明。保持一行输出是为了串口监视器易扫读。
    Serial.printf("[Master] tx=%lu ack=%lu angle=%.2fdeg raw=%u stat=0x%01x frame=0x%06lx x=%.1f%% current=%.3fA iq=%.3fA id=%.3fA vq=%.3fV vd=%.3fV pen=%u slave_x=%.1f%% age=%lums send=%lu/%lu rx=%lu/%lu last=%u faults=0x%04x\n",
                  static_cast<unsigned long>(command.seq),
                  static_cast<unsigned long>(telemetry.ack_seq),
                  sysData.master_angle_deg,
                  static_cast<unsigned int>(encoder_raw),
                  static_cast<unsigned int>(encoder_status),
                  static_cast<unsigned long>(encoder_frame),
                  sysData.master_x_pos,
                  sysData.master_target_current_a,
                  sysData.master_current_q_a,
                  sysData.master_current_d_a,
                  sysData.master_voltage_q_v,
                  sysData.master_voltage_d_v,
                  sysData.pen_down ? 1 : 0,
                  sysData.slave_x_pos,
                  static_cast<unsigned long>(telemetry_age_ms),
                  static_cast<unsigned long>(sysData.espnow_send_ok_count),
                  static_cast<unsigned long>(sysData.espnow_send_fail_count),
                  static_cast<unsigned long>(sysData.espnow_recv_ok_count),
                  static_cast<unsigned long>(sysData.espnow_recv_reject_count),
                  static_cast<unsigned int>(sysData.last_send_ok),
                  static_cast<unsigned int>(sysData.protocol_fault_flags));
}
