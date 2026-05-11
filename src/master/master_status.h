#pragma once

// master_status
// 职责：主机低频串口状态输出。
// 运行约束：只允许在 Core 0 状态任务中调用，不进入 10 kHz 控制路径。

// 打印一行主机状态：本机命令序号、从机 ack、角度、X 百分比、力反馈电流、
// ESP-NOW 收发计数、包龄和故障位。用于无调试器时快速判断链路状态。
void printMasterStatusLine();
