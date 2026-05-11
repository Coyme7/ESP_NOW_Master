#pragma once

// master_control
// 职责：主机 X 轴力反馈控制单步，包括角度读取、边界弹簧/阻尼和状态发布。
// 热路径说明：runMasterControlStep() 位于 10 kHz 控制路径，不允许日志、无线发送、
// 动态内存和阻塞等待。输出电流始终受 kMasterXAxis.haptic_current_limit_a 限制。

// dt_s 由任务调度层根据 micros() 差值传入，用于估算旋钮角速度和阻尼电流。
// 这个函数不发送 ESP-NOW；它只更新 sysData，通信任务会在 10 ms 周期读取状态发包。
void runMasterControlStep(float dt_s);
