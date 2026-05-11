#pragma once

// master_tasks
// 职责：集中主机 FreeRTOS task loop 和任务创建。
// Core 1 只跑控制热路径；Core 0 处理通信和状态输出。

// 创建并启动主机所有任务。调用前必须已经完成安全输出、硬件初始化和 ESP-NOW 初始化。
void startMasterTasks();
