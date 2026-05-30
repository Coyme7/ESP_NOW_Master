#pragma once

#include "common/protocol/protocol_types.h"

// 主机配置聚合入口。
// 只组合配置类别和 typed config 声明，硬件动作和状态机不在这里实现。
#include "master/config/build/master_bringup_config.h"
#include "master/config/diagnostics/master_log_config.h"

#include "master/config/core/master_axis_config.h"
#include "master/config/core/master_comm_config.h"
#include "master/config/core/master_control_config.h"
#include "master/config/core/master_current_sense_config.h"
#include "master/config/diagnostics/master_haptic_diagnostic_config.h"
#include "master/config/haptic/master_haptic_config.h"
#include "master/config/core/master_motor_config.h"
#include "master/config/core/master_task_config.h"

// 必须放在所有分类配置之后。
#include "master/config/master_config_validate.h"
