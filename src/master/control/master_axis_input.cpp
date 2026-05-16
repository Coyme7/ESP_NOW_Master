#include "master/control/master_axis_input.h"

#include <math.h>

#include "common/math/angle_math.h"
#include "common/math/filters.h"
#include "common/protocol/protocol_units.h"
#include "master/config/master_config.h"

namespace {

// 根据速度大小选择低通时间常数：接近静止时更快卸载速度尾巴。
float centerVelocityLpfTf(float raw_velocity_deg_s) {
    const float deadband_deg_s = fmaxf(MASTER_CENTER_DAMPING_DEADBAND_DEG_S, 0.0f);
    if (fabsf(raw_velocity_deg_s) <= deadband_deg_s) {
        return fmaxf(MASTER_CENTER_DAMPING_STILL_LPF_TF_S, 0.0f);
    }
    return fmaxf(MASTER_CENTER_DAMPING_VELOCITY_LPF_TF_S, 0.0f);
}

}  // namespace

// 把主机控制角度限制到轴范围后转换为协议归一化坐标。
int16_t masterAxisAngleDegToNorm(float angle_deg) {
    // 当前虚拟墙是实际可用行程；协议显示也应该在墙处达到 +/-100%。
    const float span_deg = kMasterXAxis.max_deg - kMasterXAxis.min_deg;
    if (span_deg <= 0.0f) {
        return 0;
    }

    const float limited = clampFloat(angle_deg, kMasterXAxis.min_deg, kMasterXAxis.max_deg);
    const float unit = ((limited - kMasterXAxis.min_deg) / span_deg) * 2.0f - 1.0f;
    const int16_t norm = unitToNorm(unit);
    const int deadband_counts =
        (MASTER_X_NORM_DEADBAND_COUNTS > 0) ? MASTER_X_NORM_DEADBAND_COUNTS : 0;
    if (norm >= -deadband_counts && norm <= deadband_counts) {
        return 0;
    }
    return norm;
}

// 每个控制周期调用：计算控制角、滤波速度和 x_norm。
MasterAxisInputSample updateMasterAxisInput(MasterAxisInputState &state,
                                            float control_angle_deg,
                                            float dt_s) {
    MasterAxisInputSample sample = {};
    sample.control_angle_deg = control_angle_deg;
    sample.clamped_angle_deg = clampFloat(control_angle_deg, kMasterXAxis.min_deg, kMasterXAxis.max_deg);

    // 用有符号最短角度差估算速度，避免跨 0/360 度时出现巨大假速度。
    const float angle_delta_deg =
        state.has_previous_angle ? signedAngleDeltaDeg(control_angle_deg, state.previous_angle_deg) : 0.0f;
    const float velocity_deg_s = (dt_s > 0.0f) ? (angle_delta_deg / dt_s) : 0.0f;
    // 速度先低通再给中心阻尼使用，减少编码器微小抖动直接变成阻尼电流。
    state.filtered_velocity_deg_s =
        lowPassFilter(velocity_deg_s,
                      state.filtered_velocity_deg_s,
                      dt_s,
                      centerVelocityLpfTf(velocity_deg_s));
    state.has_previous_angle = true;
    state.previous_angle_deg = control_angle_deg;

    sample.filtered_velocity_deg_s = state.filtered_velocity_deg_s;
    // 协议坐标使用限幅后的角度，确保发给从机的目标不会超过虚拟行程。
    sample.x_norm = masterAxisAngleDegToNorm(sample.clamped_angle_deg);
    return sample;
}
