#pragma once

#include <Arduino.h>

// 上板调试快速调参区。
// 所有宏都用 #ifndef 包裹，允许 platformio.ini/build_flags 临时覆盖。

// 真实主机电机硬件总开关。
// 0：只运行算法、通信、状态机，不初始化真实驱动和 SimpleFOC 输出。
// 1：初始化电机驱动、电流采样、编码器和 SimpleFOC，允许真实力反馈输出。
// 安全建议：第一次上电或改硬件接线后，先设为 0 验证通信和状态输出。
#ifndef MASTER_MOTOR_HW_ENABLED
#define MASTER_MOTOR_HW_ENABLED 1
#endif

// 力矩控制模式选择。
// 1：使用 foc_current 电流闭环，目标量是真实 q 轴电流，适合力反馈旋钮。
// 0：使用 voltage 电压力矩模式，目标量近似为输出电压，适合低风险 bring-up。
// 当前项目主机力觉反馈以电流环为目标，因此正常联调应保持 1。
#ifndef MASTER_USE_CURRENT_SENSE
#define MASTER_USE_CURRENT_SENSE 1
#endif

// 0A 冒烟测试开关。
// 1：无论虚拟墙、阻尼、固定电流测试如何计算，真实电机目标电流都固定为 0A。
// 用途：确认电流采样、电流环方向、iq/id 偏置、vq/vd 是否稳定，不让旋钮产生主动力矩。
#ifndef MASTER_CURRENT_ZERO_CURRENT_SMOKE_TEST
#define MASTER_CURRENT_ZERO_CURRENT_SMOKE_TEST 0
#endif

// 0A 烟测时的电流控制类型。
// 1：使用 dc_current，主要用于排查电流采样方向和 SimpleFOC 电流通道。
// 0：使用 foc_current，和正常力反馈路径一致。
// 一般不改；只有确认 foc_current 路径异常时才临时切到 dc_current 对照。
#ifndef MASTER_CURRENT_ZERO_SMOKE_USE_DC_CURRENT
#define MASTER_CURRENT_ZERO_SMOKE_USE_DC_CURRENT 0
#endif

// 电流采样诊断模式。
// 1：只做短促 U/V/W 注入采样，打印 ia/ib/相电流响应，不进入正常 FOC 运行态。
// 用途：确认 shunt、gain、ADC 引脚、采样符号是否正确。
// 注意：该模式是启动诊断路径，不能和正常力反馈联调同时使用。
#ifndef MASTER_CURRENT_SENSE_DIAG_ONLY
#define MASTER_CURRENT_SENSE_DIAG_ONLY 0
#endif

// 控制环分段耗时诊断开关。
// 1：记录热路径中编码器读取、电流采样、loopFOC、总控制步等耗时，并低频输出。
// 用途：验证 200us/5kHz 控制周期是否有余量。
// 注意：诊断会调用 micros()，会有少量额外开销；正式手感测试可关闭。
#ifndef MASTER_CONTROL_TIMING_DIAG_ENABLED
#define MASTER_CONTROL_TIMING_DIAG_ENABLED 0
#endif

// 轻量级控制环健康状态输出开关。
// 默认跟随完整时序诊断开关，只输出 ctrl_dt / ctrl_max / overC / miss 等简短字段。
// 用途：正常联调时用较短状态行观察是否漏周期、是否超时。
#ifndef MASTER_CONTROL_HEALTH_DIAG_ENABLED
#define MASTER_CONTROL_HEALTH_DIAG_ENABLED MASTER_CONTROL_TIMING_DIAG_ENABLED
#endif

// 控制环状态发布分频。
// 控制任务仍按 MASTER_CONTROL_LOOP_PERIOD_US 高频运行；这里只降低 sysData 调试字段刷新频率。
// 例如 10 表示控制 5kHz 时，调试状态约 500Hz 更新，减少跨任务共享数据写入压力。
#ifndef MASTER_CONTROL_STATUS_PUBLISH_DIV
#define MASTER_CONTROL_STATUS_PUBLISH_DIV 10
#endif

// 强力矩触觉调试档开关。
// 1：使用强墙感/强阻尼/较高电流上限的调试参数。
// 0：使用保守默认参数，适合低风险首次上电。
// 注意：名字里有 TEST，但当前阶段它相当于“强力矩配置档位”。
#ifndef MASTER_STRONG_TORQUE_TEST_ENABLED
#define MASTER_STRONG_TORQUE_TEST_ENABLED 1
#endif

// 固定小电流测试开关。
// 1：绕过虚拟墙、中心阻尼、越界逻辑，直接输出 MASTER_FIXED_CURRENT_TEST_A。
// 用途：确认电流方向、力矩方向和电机响应，不用于正常力反馈手感测试。
#ifndef MASTER_FIXED_CURRENT_TEST_ENABLED
#define MASTER_FIXED_CURRENT_TEST_ENABLED 0
#endif

// 主机 ESP-NOW 通信开关。
// 1：初始化 Wi-Fi/ESP-NOW，创建通信任务，发送主机命令并接收从机遥测。
// 0：单机排查电流环、编码器或力反馈时关闭无线，减少 Core 0 干扰和日志噪声。
#ifndef MASTER_ESPNOW_ENABLED
#define MASTER_ESPNOW_ENABLED 1
#endif

// 强制落笔测试开关。
// 1：未接真实主机按钮/紫光灯控制输入时，固定发送 pen_down=1，便于验证主从通信链路。
// 0：后续接入真实按钮后，改回由按钮状态控制 pen_down。
#ifndef MASTER_FORCE_PEN_DOWN_FOR_TEST
#define MASTER_FORCE_PEN_DOWN_FOR_TEST 0
#endif

// SimpleFOC 启动诊断日志开关。
// 1：输出 init/initFOC 阶段日志，便于查看驱动、电流采样、编码器初始化情况。
// 注意：只允许出现在启动路径，不能进入高频控制热路径。
#ifndef MASTER_SIMPLEFOC_DEBUG_ENABLED
#define MASTER_SIMPLEFOC_DEBUG_ENABLED 1
#endif

// initFOC 前开环相位扫描诊断开关。
// 1：用短时间低电压扫相，观察电机是否跟随、编码器角度是否变化。
// 用途：区分“驱动没有带动电机”和“编码器没有读到变化”。
#ifndef MASTER_MOTOR_PHASE_PROBE_ENABLED
#define MASTER_MOTOR_PHASE_PROBE_ENABLED 0
#endif

// 临时正交编码器演示输入开关。
// 1：使用两个 GPIO 的中断计数代替 MT6701，用于早期无磁编码器时演示输入。
// 当前主机已使用 MT6701，正常应保持 0。
#ifndef MASTER_DEMO_QUADRATURE_ENABLED
#define MASTER_DEMO_QUADRATURE_ENABLED 0
#endif
