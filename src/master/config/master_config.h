#pragma once

#include "shared_types.h"

// 运行模式与硬件边界。
#include "master/config/master_build_options.h"

// 日志、状态输出和 timing 诊断。
// 需要放在 build_options 后面，便于日志中引用运行模式和功能开关。
#include "master/config/master_log_config.h"

// 任务周期、绑核、优先级和栈。
#include "master/config/master_task_config.h"

// 硬件与算法参数。
#include "master/config/master_axis_config.h"
#include "master/config/master_comm_config.h"
#include "master/config/master_current_sense_config.h"
#include "master/config/master_haptic_config.h"
#include "master/config/master_motor_config.h"
