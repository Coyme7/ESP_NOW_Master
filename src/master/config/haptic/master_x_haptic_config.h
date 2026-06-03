#pragma once

#include "common/protocol/protocol_units.h"
#include "master/config/build/master_bringup_config.h"
#include "master/config/types/master_haptic_wall_types.h"

static constexpr MasterCenterDampingConfig kMasterXCenterDamping = {
    MASTER_ENABLE_CENTER_DAMPING != 0, // X 中心阻尼开关。
    -1,                                // X 中心阻尼输出方向符号。
    0.0005f,                           // X 中心阻尼速度增益，单位 A/(deg/s)。
    0.005f,                            // X 速度估计滤波时间常数，单位 s。
    0.0005f,                           // X 静止检测滤波时间常数，单位 s。
    4.0f,                              // X 阻尼死区速度，单位 deg/s。
    20.0f,                             // X 阻尼满量程速度，单位 deg/s。
    0.008f,                            // X 低速库仑阻尼项，单位 A。
    12.0f,                             // X 库仑项速度缩放，单位 deg/s。
    0.050f,                            // X 中心阻尼电流限幅，单位 A。
};

static constexpr MasterHapticWallConfig kMasterXHapticWall = {
    0.002f,                      // X 墙位置低通滤波时间常数，单位 s。
    PLOT_X_HALF_RANGE_MM - 5.0f, // X 纸面虚拟墙开始距离，单位 mm。
    PLOT_X_HALF_RANGE_MM,        // X 纸面虚拟墙硬限位，单位 mm。
    PLOT_X_HALF_RANGE_MM + 5.0f, // X raw 纸面位置安全切断距离，单位 mm。
    1.0f,                        // X 虚拟墙释放迟滞距离，单位 mm。
#if MASTER_ENABLE_PAPER_WALL_HAPTIC
    0.01f,  // X 入墙最小目标电流，单位 A。
    0.001f, // X 墙内速度阻尼增益，单位 A/(deg/s)。
    0.08f,  // X 墙内速度阻尼电流限幅，单位 A。
#else
    0.0f, // X 关闭纸面墙触觉时不输出最小墙电流。
    0.0f, // X 关闭纸面墙触觉时不输出墙内阻尼。
    0.0f, // X 关闭纸面墙触觉时墙内阻尼限幅为 0A。
#endif
    -1, // X 虚拟墙输出方向符号。
};

static constexpr MasterHapticAxisConfig kMasterXHaptic = {
    kMasterXHapticWall,
    kMasterXCenterDamping,
};
