#pragma once

#include <stdint.h>

// master_hardware
// 职责：主机侧安全输出、按钮输入、MT6701/SimpleFOC 硬件初始化和角度读取。
// 热路径说明：readMasterKnobAngleDeg() 可被 10 kHz 控制步调用，内部不得打印、
// 分配内存或等待无线事件。硬件使能失败时只设置故障并保持电机禁用。

// 上电第一步调用：关闭所有可能产生力矩的输出，并配置按钮/临时输入脚。
void configureMasterSafeOutputs();

// 初始化真实主机电机硬件。默认编译配置下不会启用硬件，只锁存禁用故障并返回 false。
bool setupMasterMotorHardware();

// 返回最近一次主机硬件初始化阶段，供启动日志判断是宏关闭、驱动失败还是 FOC 对齐失败。
const char *getMasterMotorHardwareStatus();

// 读取最近一次 MT6701 SSI 原始诊断值。只给低频状态任务调用，便于排查线序/协议/磁铁。
void getMasterEncoderDiagnostics(uint32_t &frame, uint16_t &raw_angle, uint8_t &magnetic_status);

// 读取主机旋钮角度，返回未夹紧的相对中心机械角度；控制层负责按用途夹紧。
float readMasterKnobAngleDeg();

// 读取主机中间按钮，作为落笔 pen_down 命令来源。
bool readMasterPenButtonDown();

// 条件触发的电流 PID 清零，用于边界切断或目标电流回零时消除积分/历史状态。
void resetMasterMotorCurrentPid();

// 执行力反馈电流目标。硬件关闭时为空操作，但保留同一控制路径。
void runMasterMotorOutput(float target_current_a);
