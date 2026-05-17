#pragma once

#include <stdint.h>

void startMasterTasks();

uint32_t getMasterControlTimerMissedTicks();
uint32_t getMasterControlLastDtUs();
uint32_t getMasterControlMaxDtUs();
uint32_t getMasterControlOver150Count();
uint32_t getMasterControlOver200Count();

// 控制周期健康状态：关注 dt 抖动、整步耗时、周期占用和漏 tick。
// 该结构只承载 level 1 级别的轻量统计；level 2 分段耗时另走 MasterControlTimingSnapshot。
struct MasterControlHealthSnapshot {
    // 当前 timing 诊断等级，便于日志直接确认本次数据是否带分段采样开销。
    uint8_t diag_level;
    // 最近一次控制周期 dt，反映定时器通知到达间隔。
    uint32_t last_dt_us;
    // 当前统计窗口最大控制周期 dt。
    uint32_t max_dt_us;
    // 最近一次完整控制步耗时，level 1/2 有效。
    uint32_t step_us;
    // 当前统计窗口最大完整控制步耗时，level 1/2 有效。
    uint32_t step_max_us;
    // 完整控制步超过目标周期的次数，5kHz 下即 step_us > 200us。
    uint32_t step_over_period_delta;
    // 完整控制步超过 75% 目标周期的次数，5kHz 下即 step_us > 150us。
    uint32_t step_over_75pct_delta;
    // 完整控制步超过 50% 目标周期的次数，5kHz 下即 step_us > 100us。
    uint32_t step_over_50pct_delta;
    // 兼容旧日志语义：dt 超过 1.5 倍目标周期的次数。
    uint32_t dt_over_1_5_count;
    // 兼容旧日志语义：dt 超过 2 倍目标周期的次数。
    uint32_t dt_over_2_count;
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
