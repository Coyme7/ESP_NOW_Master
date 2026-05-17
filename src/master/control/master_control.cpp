#include "master/control/master_control.h"

#include "common/math/clamp.h"
#include "common/protocol/protocol_units.h"
#include "common/state/system_state.h"
#include "master/config/master_config.h"
#include "master/control/master_axis_input.h"
#include "master/haptics/current_command.h"
#include "master/haptics/master_haptic_engine.h"
#include "master/hardware/master_encoder_hw.h"
#include "master/hardware/master_motor_hw.h"
#include "master/tasks/master_tasks.h"

// 主机控制模块。
// 数据流：角度输入 -> 纯算法力反馈 -> 电流请求状态机 -> sysData 监视状态 -> 硬件输出。
// 注意：这里运行在 200us / 5kHz 控制热路径，不能加入串口、ESP-NOW、动态内存或长时间等待。

namespace {

// 边界命中保持只影响状态显示，不保持墙电流，避免形成额外力矩记忆。
bool updateBoundaryHitHold(bool boundary_active, float dt_s) {
    static float hold_remaining_s = 0.0f;

    if (MASTER_HAPTIC_BLOCK_HOLD_MS <= 0) {
        hold_remaining_s = 0.0f;
        return boundary_active;
    }

    if (boundary_active) {
        // 只保持监视状态，不保持墙电流；目标电流始终由当前角度实时决定。
        hold_remaining_s = static_cast<float>(MASTER_HAPTIC_BLOCK_HOLD_MS) * 0.001f;
        return true;
    }

    if (hold_remaining_s <= 0.0f) {
        return false;
    }

    const float bounded_dt_s = (dt_s > 0.0f) ? clampFloat(dt_s, 0.0f, 0.001f) : 0.0f;
    hold_remaining_s -= bounded_dt_s;
    if (hold_remaining_s < 0.0f) {
        hold_remaining_s = 0.0f;
    }
    return hold_remaining_s > 0.0f;
}

// 当前目标电流模式编号，用于检测测试模式切换并触发 PID reset。
uint8_t currentTargetMode() {
#if MASTER_FIXED_CURRENT_TEST_ENABLED
    return 1;
#elif MASTER_CURRENT_ZERO_CURRENT_SMOKE_TEST
    return 2;
#else
    return 3;
#endif
}

}  // namespace

// 5kHz 控制热路径入口。所有耗时操作都应提前放到低频任务或启动阶段。
void runMasterControlStep(float dt_s) {
#if MASTER_CONTROL_TIMING_DIAG_ENABLED
    const uint32_t control_start_us = micros();
#endif
    static MasterAxisInputState axis_state = {};
    static MasterHapticEngineState haptic_state = {};
    static MasterCurrentCommandState current_state = {0.0f, false, 0xff};
    static uint8_t publish_div = 0;

    const float control_angle_deg = readMasterKnobAngleDeg();
    const MasterAxisInputSample axis =
        updateMasterAxisInput(axis_state, control_angle_deg, dt_s);

    const MasterHapticEngineInput haptic_input = {
        axis.control_angle_deg,
        axis.filtered_velocity_deg_s,
        normToUnit(axis.x_norm),
        normToUnit(axis.x_norm) * PLOT_X_HALF_RANGE_MM,
        dt_s,
    };
    const MasterHapticEngineOutput haptic_output =
        computeMasterHapticCommand(haptic_state, haptic_input);

    const float requested_current_a =
        MASTER_FORCE_FEEDBACK_ENABLED ? haptic_output.target_current_a : 0.0f;
    const MasterCurrentCommandInput current_input = {
        requested_current_a,
        dt_s,
        haptic_output.boundary_safety_cut,
        currentTargetMode(),
    };
    const MasterCurrentCommandOutput current_output =
        updateMasterCurrentCommand(current_state, current_input);

    if (current_output.request_pid_reset) {
        resetMasterMotorCurrentPid();
    }

    const float master_x_pos = normToPercent(axis.x_norm);
    const float master_y_pos = normToPercent(axis.y_norm);
    const bool boundary_hit = updateBoundaryHitHold(haptic_output.boundary_active, dt_s);
#if MASTER_CONTROL_STATUS_PUBLISH_DIV <= 1
    const bool publish_now = true;
#else
    publish_div++;
    const bool publish_now = publish_div >= MASTER_CONTROL_STATUS_PUBLISH_DIV;
    if (publish_now) {
        publish_div = 0;
    }
#endif
    if (publish_now) {
        sysData.master.angle_deg = axis.control_angle_deg;
        sysData.master.target_current_a = current_output.current_command_a;
        sysData.master.x_pos = master_x_pos;
        sysData.master.y_pos = master_y_pos;
        sysData.master.boundary_hit = boundary_hit;
    }

#if MASTER_CONTROL_TIMING_DIAG_ENABLED
    recordMasterTimingControlLogicUs(micros() - control_start_us);
#endif

    // 若硬件输出关闭，runMasterMotorOutput() 会丢弃目标电流；
    // 这样控制逻辑仍可运行，方便第一阶段验证状态和通信。
    runMasterMotorOutput(current_output.current_command_a);
}
