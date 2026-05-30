#pragma once

#include "common/protocol/protocol_units.h"
#include "master/config/build/master_bringup_config.h"
#include "master/config/types/master_haptic_wall_types.h"

static constexpr MasterCenterDampingConfig kMasterYCenterDamping = {
    MASTER_ENABLE_CENTER_DAMPING != 0, // Y 中心阻尼开关。
    -1,                                // Y 中心阻尼输出方向符号。
    0.0003f,                           // Y 中心阻尼速度增益，单位 A/(deg/s)。
    0.005f,                            // Y 速度估计滤波时间常数，单位 s。
    0.0005f,                           // Y 静止检测滤波时间常数，单位 s。
    4.0f,                              // Y 阻尼死区速度，单位 deg/s。
    20.0f,                             // Y 阻尼满量程速度，单位 deg/s。
    0.004f,                            // Y 低速库仑阻尼项，单位 A。
    12.0f,                             // Y 库仑项速度缩放，单位 deg/s。
    0.020f,                            // Y 中心阻尼电流限幅，单位 A。
};

static constexpr MasterHapticWallConfig kMasterYHapticWall = {
    0.002f,                      // Y 墙位置低通滤波时间常数，单位 s。
    PLOT_Y_HALF_RANGE_MM - 5.0f, // Y 纸面虚拟墙开始距离，单位 mm。
    PLOT_Y_HALF_RANGE_MM,        // Y 纸面虚拟墙硬限位，单位 mm。
    PLOT_Y_HALF_RANGE_MM + 5.0f, // Y raw 纸面位置安全切断距离，单位 mm。
    1.0f,                        // Y 虚拟墙释放迟滞距离，单位 mm。
#if MASTER_ENABLE_PAPER_WALL_HAPTIC
    0.01f,   // Y 入墙最小目标电流，单位 A。
    0.0005f, // Y 墙内速度阻尼增益，单位 A/(deg/s)。
    0.03f,   // Y 墙内速度阻尼电流限幅，单位 A。
#else
    0.0f, // Y 关闭纸面墙触觉时不输出最小墙电流。
    0.0f, // Y 关闭纸面墙触觉时不输出墙内阻尼。
    0.0f, // Y 关闭纸面墙触觉时墙内阻尼限幅为 0A。
#endif
    -1, // Y 虚拟墙输出方向符号。
};

static constexpr MasterHapticAxisConfig kMasterYHaptic = {
    kMasterYHapticWall,
    kMasterYCenterDamping,
};
