#include "master/control/master_control.h"

#include <stdint.h>

#include "common/math/clamp.h"
#include "common/protocol/protocol_units.h"
#include "common/state/system_state.h"
#include "master/config/master_config.h"
#include "master/control/master_axis_input.h"
#include "master/haptics/current_command.h"
#include "master/haptics/master_haptic_engine.h"
#include "master/hardware/master_adc1_dma_sampler.h"
#include "master/hardware/master_encoder_hw.h"
#include "master/hardware/master_motor_hw.h"
#include "master/modes/mode_guard.h"
#include "master/tasks/master_tasks.h"

// 主机控制模块。
// 数据流：角度输入 -> 纯算法力反馈 -> 电流请求状态机 -> sysData 监视状态 -> 硬件输出。
// 注意：这里运行在 MASTER_RUN_MODE 派生的控制热路径，不能加入串口、ESP-NOW、动态内存或长时间等待。

namespace {

constexpr bool masterReadsXAxisKnob() {
    return MASTER_ENABLE_X_ENCODER_HW && masterRunModeReadsEncoder(AXIS_X);
}

constexpr bool masterReadsYAxisKnob() {
    return MASTER_ENABLE_Y_ENCODER_HW &&
           masterRunModeReadsEncoder(AXIS_Y);
}

constexpr bool masterRunsXAxisHaptics() {
    return MASTER_ENABLE_FORCE_FEEDBACK &&
           MASTER_ENABLE_X_MOTOR_HW &&
           masterRunModeDrivesAxis(AXIS_X);
}

constexpr bool masterRunsYAxisHaptics() {
    return MASTER_ENABLE_FORCE_FEEDBACK &&
           MASTER_ENABLE_Y_MOTOR_HW &&
           masterRunModeDrivesAxis(AXIS_Y);
}

// 边界命中保持只影响状态显示，不保持墙电流，避免形成额外力矩记忆。
bool updateBoundaryHitHold(bool boundary_active, float dt_s) {
    static float hold_remaining_s = 0.0f;

    if (kMasterHapticDiagnostic.boundary_hold_ms == 0U) {
        hold_remaining_s = 0.0f;
        return boundary_active;
    }

    if (boundary_active) {
        // 只保持监视状态，不保持墙电流；目标电流始终由当前角度实时决定。
        hold_remaining_s = static_cast<float>(kMasterHapticDiagnostic.boundary_hold_ms) * 0.001f;
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
#if MASTER_ENABLE_FIXED_CURRENT_TEST
    return 1;
#elif MASTER_ENABLE_ZERO_CURRENT_TEST
    return 2;
#else
    return 3;
#endif
}

volatile float latest_x_current_command_a = 0.0f;
volatile float latest_y_current_command_a = 0.0f;
volatile uint32_t latest_current_command_sequence = 0;
volatile uint32_t latest_pid_reset_sequence = 0;

void publishLatestCurrentCommand(float x_current_command_a,
                                 float y_current_command_a,
                                 bool request_pid_reset) {
    const uint32_t next_sequence = latest_current_command_sequence + 1U;
    latest_x_current_command_a = x_current_command_a;
    latest_y_current_command_a = y_current_command_a;
    if (request_pid_reset) {
        latest_pid_reset_sequence = next_sequence;
    }
    latest_current_command_sequence = next_sequence;
}

}  // namespace

// 慢外环入口：读取缓存角度，计算速度估计、haptic 和电流命令。
// 该函数由独立外环任务按 1kHz 调用，禁止串口、无线、动态内存或阻塞等待。
void runMasterOuterLoopSlow(float dt_s) {
#if MASTER_TIMING_DETAIL_DIAG_ENABLED
    const uint32_t control_start_us = micros();
#endif
    static MasterAxisInputState axis_state = {};
    static MasterHapticEngineState x_haptic_state = {};
    static MasterHapticEngineState y_haptic_state = {};
    static MasterCurrentCommandState x_current_state = {0.0f, false, 0xff};
    static MasterCurrentCommandState y_current_state = {0.0f, false, 0xff};

    const float control_angle_deg = masterReadsXAxisKnob() ? readMasterKnobAngleDeg() : 0.0f;
    const float y_control_angle_deg = masterReadsYAxisKnob() ? readMasterYKnobAngleDeg() : 0.0f;
    const MasterAxisInputSample axis =
        updateMasterAxisInput(axis_state, control_angle_deg, y_control_angle_deg, dt_s);

    const MasterHapticEngineInput x_haptic_input = {
        axis.x_control_angle_deg,
        axis.x_filtered_velocity_deg_s,
        normToUnit(axis.x_norm),
        normToUnit(axis.x_norm) * PLOT_X_HALF_RANGE_MM,
        PLOT_X_HALF_RANGE_MM,
        dt_s,
    };
    const MasterHapticEngineInput y_haptic_input = {
        axis.y_control_angle_deg,
        axis.y_filtered_velocity_deg_s,
        normToUnit(axis.y_norm),
        normToUnit(axis.y_norm) * PLOT_Y_HALF_RANGE_MM,
        PLOT_Y_HALF_RANGE_MM,
        dt_s,
    };

    const MasterHapticEngineOutput x_haptic_output =
        computeMasterHapticCommand(x_haptic_state, x_haptic_input, kMasterXAxis);
    const MasterHapticEngineOutput y_haptic_output =
        computeMasterHapticCommand(y_haptic_state, y_haptic_input, kMasterYAxis);

    const MasterCurrentCommandInput x_current_input = {
        masterRunsXAxisHaptics() ? x_haptic_output.target_current_a : 0.0f,
        dt_s,
        masterRunsXAxisHaptics() && x_haptic_output.boundary_safety_cut,
        currentTargetMode(),
    };
    const MasterCurrentCommandInput y_current_input = {
        masterRunsYAxisHaptics() ? y_haptic_output.target_current_a : 0.0f,
        dt_s,
        masterRunsYAxisHaptics() && y_haptic_output.boundary_safety_cut,
        currentTargetMode(),
    };
    const MasterCurrentCommandOutput x_current_output =
        updateMasterCurrentCommand(x_current_state, x_current_input, kMasterXAxis);
    const MasterCurrentCommandOutput y_current_output =
        updateMasterCurrentCommand(y_current_state, y_current_input, kMasterYAxis);

    publishLatestCurrentCommand(x_current_output.current_command_a,
                                y_current_output.current_command_a,
                                x_current_output.request_pid_reset ||
                                    y_current_output.request_pid_reset);

    const float master_x_pos = normToPercent(axis.x_norm);
    const float master_y_pos = normToPercent(axis.y_norm);
    const bool boundary_hit =
        updateBoundaryHitHold((masterRunsXAxisHaptics() && x_haptic_output.boundary_active) ||
                                  (masterRunsYAxisHaptics() && y_haptic_output.boundary_active),
                              dt_s);
    sysData.master.angle_deg = axis.x_control_angle_deg;
    sysData.master.y_angle_deg = axis.y_control_angle_deg;
    sysData.master.target_current_a = x_current_output.current_command_a;
    sysData.master.y_target_current_a = y_current_output.current_command_a;
    sysData.master.x_pos = master_x_pos;
    sysData.master.y_pos = master_y_pos;
    sysData.master.boundary_hit = boundary_hit;
    sysData.master.x_boundary_hit = masterRunsXAxisHaptics() && x_haptic_output.boundary_active;
    sysData.master.y_boundary_hit = masterRunsYAxisHaptics() && y_haptic_output.boundary_active;

#if MASTER_TIMING_DETAIL_DIAG_ENABLED
    recordMasterTimingControlLogicUs(micros() - control_start_us);
#endif
}

// 快电流环入口：每个控制 tick 做 ADC fault gate、硬件输出和 SimpleFOC loopFOC()。
void runMasterFastCurrentLoop() {
    static uint32_t consumed_command_sequence = 0;
    static uint32_t consumed_pid_reset_sequence = 0;
    static uint32_t stale_fast_steps = MASTER_OUTER_LOOP_STALE_EVERY_N_STEPS + 1U;

    const uint32_t command_sequence = latest_current_command_sequence;
    const bool command_updated = command_sequence != consumed_command_sequence;
    if (command_updated) {
        consumed_command_sequence = command_sequence;
        stale_fast_steps = 0;
    } else if (stale_fast_steps < UINT32_MAX) {
        stale_fast_steps++;
    }

    const bool command_fresh =
        command_sequence != 0U &&
        stale_fast_steps <= MASTER_OUTER_LOOP_STALE_EVERY_N_STEPS;
    const float x_current_command_a =
        command_fresh ? latest_x_current_command_a : 0.0f;
    const float y_current_command_a =
        command_fresh ? latest_y_current_command_a : 0.0f;
    const bool update_motion_target = command_updated || !command_fresh;

    const bool adc_snapshot_ok = latchMasterAdc1DmaControlSnapshot();
    if (adc_snapshot_ok) {
        const uint32_t pid_reset_sequence = latest_pid_reset_sequence;
        if (command_updated &&
            pid_reset_sequence == command_sequence &&
            pid_reset_sequence != consumed_pid_reset_sequence) {
            resetMasterMotorCurrentPid();
            consumed_pid_reset_sequence = pid_reset_sequence;
        }
        runMasterMotorOutput(x_current_command_a,
                             y_current_command_a,
                             update_motion_target);
    } else {
        disableMasterMotorOutputsForAdcFault();
    }
}
