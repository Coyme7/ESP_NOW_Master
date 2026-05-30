#pragma once

#include "master/config/build/master_bringup_config.h"
#include "master/config/types/master_motor_types.h"

// DengFoc / BLDCDriver3PWM EN 脚极性。
// 若板子实测 EN 低有效，必须先改这里再开启真实电机输出。
#ifndef MASTER_DRIVER_ENABLE_ACTIVE_HIGH
#define MASTER_DRIVER_ENABLE_ACTIVE_HIGH 1
#endif

// 主机 2804 无刷电机极对数。
// SimpleFOC 用它把机械角转换成电角度。
#ifndef MASTER_MOTOR_POLE_PAIRS
#define MASTER_MOTOR_POLE_PAIRS 7
#endif

// 主机 SimpleFOC 电机/电流环配置对象。
// X/Y 独立实例定义在 master_motor_config.cpp。
extern const MasterMotorFocConfig kMasterXMotorFoc;
extern const MasterMotorFocConfig kMasterYMotorFoc;
