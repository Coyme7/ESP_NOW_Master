#include "master/control/master_axis_input.h"

#include <math.h>

#include "common/math/angle_math.h"
#include "common/math/filters.h"
#include "common/protocol/protocol_units.h"
#include "master/config/master_config.h"
#include "master/modes/mode_guard.h"

namespace {

// 根据速度大小选择低通时间常数：接近静止时更快卸载速度尾巴。
float centerVelocityLpfTf(float raw_velocity_deg_s, const MasterAxisConfig &config) {
    const float deadband_deg_s = fmaxf(config.haptic.center.deadband_deg_s, 0.0f);
    if (fabsf(raw_velocity_deg_s) <= deadband_deg_s) {
        return fmaxf(config.haptic.center.still_lpf_tf_s, 0.0f);
    }
    return fmaxf(config.haptic.center.velocity_lpf_tf_s, 0.0f);
}

float updateFilteredAxisVelocity(float angle_deg,
                                 float dt_s,
                                 float &previous_angle_deg,
                                 float &filtered_velocity_deg_s,
                                 bool &has_previous_angle,
                                 const MasterAxisConfig &config) {
    const float angle_delta_deg =
        has_previous_angle ? signedAngleDeltaDeg(angle_deg, previous_angle_deg) : 0.0f;
    const float velocity_deg_s = (dt_s > 0.0f) ? (angle_delta_deg / dt_s) : 0.0f;
    filtered_velocity_deg_s =
        lowPassFilter(velocity_deg_s,
                      filtered_velocity_deg_s,
                      dt_s,
                      centerVelocityLpfTf(velocity_deg_s, config));
    has_previous_angle = true;
    previous_angle_deg = angle_deg;
    return filtered_velocity_deg_s;
}

}  // namespace

int16_t masterAxisAngleDegToNormForConfig(float angle_deg,
                                          const MasterAxisConfig &config,
                                          int deadband_counts) {
    const float span_deg = config.range.max_deg - config.range.min_deg;
    if (span_deg <= 0.0f) {
        return 0;
    }

    const float limited = clampFloat(angle_deg, config.range.min_deg, config.range.max_deg);
    const float unit = ((limited - config.range.min_deg) / span_deg) * 2.0f - 1.0f;
    const int16_t norm = unitToNorm(unit);
    if (deadband_counts < 0) {
        deadband_counts = 0;
    }
    if (norm >= -deadband_counts && norm <= deadband_counts) {
        return 0;
    }
    return norm;
}

// 把主机控制角度限制到轴范围后转换为协议归一化坐标。
int16_t masterAxisAngleDegToNorm(float angle_deg) {
    // 当前虚拟墙是实际可用行程；协议显示也应该在墙处达到 +/-100%。
    return masterAxisAngleDegToNormForConfig(angle_deg,
                                             kMasterXAxis,
                                             kMasterXAxis.input.norm_deadband_counts);
}

// 每个控制周期调用：计算控制角、滤波速度和协议坐标。
// 算法流：
//   X/Y 原始控制角 -> 轴级限幅 -> 速度估计/低通 -> 轴模式选择 -> x_norm/y_norm。
// 该函数是纯算法，不访问硬件、不发包、不打印，便于各 run mode 共用。
MasterAxisInputSample updateMasterAxisInput(MasterAxisInputState &state,
                                            float control_angle_deg,
                                            float y_control_angle_deg,
                                            float dt_s) {
    MasterAxisInputSample sample = {};
    sample.x_control_angle_deg = control_angle_deg;
    sample.x_clamped_angle_deg = clampFloat(control_angle_deg, kMasterXAxis.range.min_deg, kMasterXAxis.range.max_deg);
#if MASTER_ENABLE_Y_ENCODER_HW
    sample.y_control_angle_deg = y_control_angle_deg;
    sample.y_clamped_angle_deg = clampFloat(y_control_angle_deg, kMasterYAxis.range.min_deg, kMasterYAxis.range.max_deg);
#else
    (void)y_control_angle_deg;
    sample.y_control_angle_deg = 0.0f;
    sample.y_clamped_angle_deg = 0.0f;
#endif
    // 主显示轴默认代表 X 轴。
    sample.control_angle_deg = sample.x_control_angle_deg;
    sample.clamped_angle_deg = sample.x_clamped_angle_deg;

    // 用有符号最短角度差估算速度，避免跨 0/360 度时出现巨大假速度。
    // 速度先低通再给中心阻尼使用，减少编码器微小抖动直接变成阻尼电流。
    sample.x_filtered_velocity_deg_s =
        updateFilteredAxisVelocity(sample.x_control_angle_deg,
                                   dt_s,
                                   state.previous_angle_deg,
                                   state.filtered_velocity_deg_s,
                                   state.has_previous_angle,
                                   kMasterXAxis);
#if MASTER_ENABLE_Y_ENCODER_HW
    sample.y_filtered_velocity_deg_s =
        updateFilteredAxisVelocity(sample.y_control_angle_deg,
                                   dt_s,
                                   state.previous_y_angle_deg,
                                   state.filtered_y_velocity_deg_s,
                                   state.has_previous_y_angle,
                                   kMasterYAxis);
#else
    sample.y_filtered_velocity_deg_s = 0.0f;
#endif
    sample.filtered_velocity_deg_s = sample.x_filtered_velocity_deg_s;
    // 协议坐标使用限幅后的角度，确保发给从机的目标不会超过虚拟行程。
    const int16_t axis_norm = masterAxisAngleDegToNorm(sample.x_clamped_angle_deg);
#if MASTER_ENABLE_Y_ENCODER_HW
    const int16_t y_axis_norm =
        masterAxisAngleDegToNormForConfig(sample.y_clamped_angle_deg,
                                          kMasterYAxis,
                                          kMasterYAxis.input.norm_deadband_counts);
#endif
    sample.x_norm = masterRunModeRunsAxis(AXIS_X) ? axis_norm : 0;
    if (masterRunModeRunsAxis(AXIS_Y)) {
#if MASTER_ENABLE_Y_ENCODER_HW
        sample.y_norm = y_axis_norm;
#else
        sample.y_norm = 0;
#endif
    } else {
        sample.y_norm = 0;
    }
    return sample;
}
