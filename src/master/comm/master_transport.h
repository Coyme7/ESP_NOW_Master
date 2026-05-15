#pragma once

#include "common/protocol/protocol_types.h"

// master_transport
// 职责：主机 ESP-NOW 初始化、发送 MasterCommand、接收从机遥测并维护包快照。
// 回调约束：只用短临界区复制最新固定长度 payload 并设置 pending 标志；
// 不驱动电机、不打印、不分配内存、不阻塞。

// 初始化 Wi-Fi STA、ESP-NOW、回调和固定从机 peer。
void setupMasterEspNow();

// 打印本机 STA MAC 和硬编码从机 MAC，用于烧录对象和配对地址核对。
void printMasterEspNowIdentity();

// 由 Core 0 通信任务调用，处理 ESP-NOW 回调暂存的最新遥测包。
void processMasterTelemetry();

// 由 Core 0 通信任务周期调用，打包并发送最新主机命令。
void sendMasterCommand(uint32_t seq, uint32_t now_us);

// 读取最近一次发送的主机命令快照，供状态行展示。
MasterCommandPacket snapshotMasterCommand();

// 读取最近一次接收的从机遥测快照，供状态行和链路诊断使用。
SlaveTelemetryPacket snapshotSlaveTelemetry();
