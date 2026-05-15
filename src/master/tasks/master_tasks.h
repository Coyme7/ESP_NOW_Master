#pragma once

#include <stdint.h>

void startMasterTasks();

uint32_t getMasterControlTimerMissedTicks();
uint32_t getMasterControlLastDtUs();
uint32_t getMasterControlMaxDtUs();
uint32_t getMasterControlOver150Count();
uint32_t getMasterControlOver200Count();

// 控制周期健康状态：关注 dt 抖动、超时和漏 tick。
struct MasterControlHealthSnapshot {
    // 最近一次控制周期 dt。
    uint32_t last_dt_us;
    // 当前统计窗口最大控制周期 dt。
    uint32_t max_dt_us;
    // dt 超过 1.5 倍目标周期的次数。
    uint32_t over_1_5_count;
    // dt 超过 2 倍目标周期的次数。
    uint32_t over_2_count;
    // 定时器通知累计漏 tick 数。
    uint32_t missed_ticks;
};

// 单个阶段的时序统计结果。
struct MasterTimingStats {
    // 统计窗口内样本数量。
    uint32_t count;
    // 最近一次耗时。
    uint32_t last_us;
    // 当前窗口平均耗时。
    uint32_t avg_us;
    // 当前窗口最大耗时。
    uint32_t max_us;
};

// 控制热路径各阶段的统计快照。
struct MasterControlTimingSnapshot {
    // 整个控制周期耗时。
    MasterTimingStats control_total;
    // 控制算法耗时，不含电机输出。
    MasterTimingStats control_logic;
    // 电机输出总耗时。
    MasterTimingStats motor_total;
    // SimpleFOC move() 耗时。
    MasterTimingStats motor_move;
    // SimpleFOC loopFOC() 耗时。
    MasterTimingStats motor_loop_foc;
    // 电流采样耗时。
    MasterTimingStats current_sense;
    // 编码器 SSI/SPI 读取耗时。
    MasterTimingStats sensor_spi;
};

void recordMasterTimingControlTotalUs(uint32_t duration_us);
void recordMasterTimingControlLogicUs(uint32_t duration_us);
void recordMasterTimingMotorUs(uint32_t total_us,
                               uint32_t move_us,
                               uint32_t loop_foc_us,
                               uint32_t sensor_spi_us);
extern "C" void recordMasterTimingCurrentSenseUs(uint32_t duration_us);
MasterControlHealthSnapshot getMasterControlHealthSnapshot();
MasterControlTimingSnapshot getMasterControlTimingSnapshot();
