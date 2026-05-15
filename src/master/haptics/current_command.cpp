#include "master/haptics/current_command.h"

#include <math.h>

#include "common/math/clamp.h"
#include "master/config/master_config.h"

namespace {

// 根据目标电流变化方向选择斜率：释放/切断时允许更快卸载。
float currentRampLimitAPerS(float current_a, float target_a) {
    const float base_ramp_a_per_s = kMasterXAxis.haptic_current_ramp_a_per_s;
    const float release_ramp_a_per_s =
        (MASTER_HAPTIC_CURRENT_RELEASE_RAMP_A_PER_S > 0.0f)
            ? MASTER_HAPTIC_CURRENT_RELEASE_RAMP_A_PER_S
            : base_ramp_a_per_s;
    const bool reducing_magnitude = fabsf(target_a) < fabsf(current_a);
    const bool crossing_zero = (current_a * target_a) < 0.0f;
    return (reducing_magnitude || crossing_zero)
               ? release_ramp_a_per_s
               : base_ramp_a_per_s;
}

// 对目标电流做斜率限制，避免力矩阶跃导致手感冲击或电流环尖峰。
float rampCurrentCommand(float current_a, float target_a, float dt_s) {
    const float ramp_a_per_s = currentRampLimitAPerS(current_a, target_a);
    if (ramp_a_per_s <= 0.0f) {
        return clampFloat(target_a,
                          -kMasterXAxis.haptic_current_limit_a,
                          kMasterXAxis.haptic_current_limit_a);
    }
    if (dt_s <= 0.0f) {
        return current_a;
    }

    const float bounded_dt_s = clampFloat(dt_s, 0.0f, 0.001f);
    const float max_step_a = ramp_a_per_s * bounded_dt_s;
    const float delta_a = clampFloat(target_a - current_a, -max_step_a, max_step_a);
    return clampFloat(current_a + delta_a,
                      -kMasterXAxis.haptic_current_limit_a,
                      kMasterXAxis.haptic_current_limit_a);
}

}  // namespace

// 电流命令状态机：限幅、斜率、模式切换检测和 PID reset 请求都在这里完成。
MasterCurrentCommandOutput updateMasterCurrentCommand(MasterCurrentCommandState &state,
                                                      const MasterCurrentCommandInput &input) {
    MasterCurrentCommandOutput output = {};
    // 只有模式切换或安全切断边沿才请求 reset，避免每个周期清 PID 导致电流环无法工作。
    output.request_pid_reset =
        (state.previous_target_mode != 0xff && input.target_mode != state.previous_target_mode) ||
        (input.boundary_safety_cut && !state.previous_boundary_safety_cut);

    // 安全切断优先于斜率限制：危险越界时不慢慢降电流，而是立即命令 0A。
    if (input.boundary_safety_cut) {
        state.current_command_a = 0.0f;
    } else {
        state.current_command_a =
            rampCurrentCommand(state.current_command_a, input.target_current_a, input.dt_s);
    }

    state.previous_boundary_safety_cut = input.boundary_safety_cut;
    state.previous_target_mode = input.target_mode;
    output.current_command_a = state.current_command_a;
    return output;
}

