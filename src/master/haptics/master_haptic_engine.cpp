#include "master/haptics/master_haptic_engine.h"

#include <math.h>
#include <stdint.h>

#include "common/math/clamp.h"
#include "common/math/filters.h"
#include "master/config/master_config.h"

namespace {

#if !MASTER_ENABLE_FIXED_CURRENT_TEST && !MASTER_ENABLE_ZERO_CURRENT_TEST
// 边界电流计算结果：区分正常墙接触、越界回推和安全切断。
struct BoundaryCurrentResult {
    float target_current_a;
    bool wall_contact;
    bool overrun_pushback;
    bool safety_cut;
};

// smootherstep 墙曲线：入口和末端斜率为 0，减少墙边缘细碎抖动。
float fastWallHardeningCurve(float depth_unit) {
    depth_unit = clampFloat(depth_unit, 0.0f, 1.0f);
    return depth_unit * depth_unit * depth_unit *
           (depth_unit * (depth_unit * 6.0f - 15.0f) + 10.0f);
}

// 根据高端/低端墙和配置方向生成带符号回推电流。
float signedWallCurrentForSide(int8_t side, float magnitude_a, const MasterAxisConfig &config) {
#if !MASTER_ENABLE_PAPER_WALL_HAPTIC
    (void)side;
    (void)magnitude_a;
    (void)config;
    return 0.0f;
#else
    const float direction_sign = (config.haptic.wall.direction_sign < 0) ? -1.0f : 1.0f;
    const float side_sign = (side > 0) ? 1.0f : -1.0f;
    const float limited_magnitude_a =
        clampFloat(magnitude_a, 0.0f, config.current.limit_a);
    return direction_sign * side_sign * limited_magnitude_a;
#endif
}

// 把墙深度系数转换为实际墙电流，并加入最小贴墙电流。
float wallCurrentForSide(int8_t side, float wall_factor, const MasterAxisConfig &config) {
    float factor = clampFloat(wall_factor, 0.0f, 1.0f);
    if (config.haptic.wall.min_current_a > 0.0f &&
        config.current.limit_a > 0.0f) {
        const float min_factor =
            clampFloat(config.haptic.wall.min_current_a /
                           config.current.limit_a,
                       0.0f,
                       1.0f);
        factor = min_factor + (1.0f - min_factor) * factor;
    }
    const float current_a =
        signedWallCurrentForSide(side, config.current.limit_a * factor, config);
    return clampFloat(current_a,
                      -config.current.limit_a,
                      config.current.limit_a);
}

// 墙内阻尼电流：根据朝墙内/墙外运动方向吸收能量，抑制边界振动。
float wallDampingCurrentForSide(int8_t side,
                                float filtered_velocity_deg_s,
                                const MasterAxisConfig &config) {
#if !MASTER_ENABLE_PAPER_WALL_HAPTIC
    (void)side;
    (void)filtered_velocity_deg_s;
    (void)config;
    return 0.0f;
#else
    if (config.haptic.wall.damping_gain_a_per_deg_s <= 0.0f ||
        config.haptic.wall.damping_limit_a <= 0.0f ||
        config.current.limit_a <= 0.0f) {
        return 0.0f;
    }

    const float inward_velocity_deg_s =
        (side > 0) ? filtered_velocity_deg_s : -filtered_velocity_deg_s;
    const float damping_magnitude_a =
        clampFloat(config.haptic.wall.damping_gain_a_per_deg_s *
                       fabsf(inward_velocity_deg_s),
                   0.0f,
                   config.haptic.wall.damping_limit_a);
    const float damping_current_a = signedWallCurrentForSide(side, damping_magnitude_a, config);
    return (inward_velocity_deg_s >= 0.0f) ? damping_current_a : -damping_current_a;
#endif
}

// 墙电流限幅时只允许保持回推方向，避免边界处出现助推脉冲。
float clampWallCurrentForSide(int8_t side, float current_a, const MasterAxisConfig &config) {
    const float wall_sign = signedWallCurrentForSide(side, 1.0f, config);
    if (wall_sign >= 0.0f) {
        return clampFloat(current_a, 0.0f, config.current.limit_a);
    }
    return clampFloat(current_a, -config.current.limit_a, 0.0f);
}

// 中心阻尼淡入系数：低速死区内为 0，超过 full speed 后为 1。
float centerDampingScale(float speed_deg_s, const MasterAxisConfig &config) {
    const float deadband_deg_s = fmaxf(config.haptic.center.deadband_deg_s, 0.0f);
    const float full_speed_deg_s =
        fmaxf(config.haptic.center.full_speed_deg_s, deadband_deg_s);
    if (speed_deg_s <= deadband_deg_s) {
        return 0.0f;
    }
    if (full_speed_deg_s <= deadband_deg_s) {
        return 1.0f;
    }
    return clampFloat((speed_deg_s - deadband_deg_s) /
                          (full_speed_deg_s - deadband_deg_s),
                      0.0f,
                      1.0f);
}

// 中心区速度阻尼：粘滞阻尼 + 平滑库仑阻尼，提升旋钮阻尼感。
float computeCenterDampingCurrent(float filtered_velocity_deg_s, const MasterAxisConfig &config) {
    if (!config.haptic.center.enabled ||
        config.haptic.center.limit_a <= 0.0f ||
        config.haptic.center.vel_scale_deg_s <= 0.0f) {
        return 0.0f;
    }

    const float damping_scale = centerDampingScale(fabsf(filtered_velocity_deg_s), config);
    if (damping_scale <= 0.0f) {
        return 0.0f;
    }

    const float direction_sign = (config.haptic.center.direction_sign < 0) ? -1.0f : 1.0f;
    const float damping_current_a =
        direction_sign *
        (config.haptic.center.gain_a_per_deg_s * filtered_velocity_deg_s +
         config.haptic.center.coulomb_a *
             tanhf(filtered_velocity_deg_s / config.haptic.center.vel_scale_deg_s)) *
        damping_scale;
    return clampFloat(damping_current_a,
                      -config.haptic.center.limit_a,
                      config.haptic.center.limit_a);
}

// 边界状态机：原始纸面位置用于安全判断，滤波纸面位置用于墙深度计算。
// 算法流：
// 原始轴位置 -> 安全切断判断 -> 硬边界回推 -> 软墙迟滞 -> 墙深度曲线 -> 阻尼叠加 -> 限幅。
BoundaryCurrentResult computeBoundaryCurrent(float raw_axis_mm,
                                             float wall_axis_mm,
                                             float paper_half_range_mm,
                                             float filtered_velocity_deg_s,
                                             int8_t &wall_contact_side,
                                             const MasterAxisConfig &config) {
    // 默认认为不在墙区；后续按“安全切断 -> 越界回推 -> 墙区迟滞”的优先级逐层覆盖。
    BoundaryCurrentResult result = {0.0f, false, false, false};
    const float axis_half_mm = fmaxf(paper_half_range_mm, 0.0f);
    const float hard_limit_mm =
        clampFloat(fabsf(config.haptic.wall.hard_limit_mm), 0.0f, axis_half_mm);
    const float wall_start_mm =
        clampFloat(fabsf(config.haptic.wall.start_mm), 0.0f, hard_limit_mm);
    const float safety_cut_mm =
        fmaxf(fabsf(config.haptic.wall.safety_cut_mm), hard_limit_mm);

    // 最高优先级：超过安全切断距离时不再输出回推电流，而是立即归零并请求上层 reset。
    if (raw_axis_mm > safety_cut_mm ||
        raw_axis_mm < -safety_cut_mm) {
        wall_contact_side = 0;
        result.safety_cut = true;
        return result;
    }

    // 第二优先级：轻微越过硬边界时给满墙电流，把旋钮推回可用范围。
    if (raw_axis_mm > hard_limit_mm) {
        wall_contact_side = 1;
        result.overrun_pushback = true;
        result.target_current_a = wallCurrentForSide(1, 1.0f, config);
        return result;
    }
    if (raw_axis_mm < -hard_limit_mm) {
        wall_contact_side = -1;
        result.overrun_pushback = true;
        result.target_current_a = wallCurrentForSide(-1, 1.0f, config);
        return result;
    }

    const float wall_zone_mm =
        fmaxf(hard_limit_mm - wall_start_mm, 0.0f);
    if (wall_zone_mm <= 0.0f) {
        wall_contact_side = 0;
        return result;
    }

    const float release_hyst_mm = fmaxf(config.haptic.wall.release_hyst_mm, 0.0f);
    const float high_wall_start = wall_start_mm;
    const float low_wall_start = -wall_start_mm;
    const float high_wall_release = high_wall_start - release_hyst_mm;
    const float low_wall_release = low_wall_start + release_hyst_mm;

    // Schmitt 迟滞：已接触墙时用 release 阈值退出，未接触时用 start 阈值进入。
    int8_t side = 0;
    if (wall_contact_side > 0) {
        side = (raw_axis_mm < high_wall_release) ? 0 : 1;
    } else if (wall_contact_side < 0) {
        side = (raw_axis_mm > low_wall_release) ? 0 : -1;
    } else if (raw_axis_mm > high_wall_start) {
        side = 1;
    } else if (raw_axis_mm < low_wall_start) {
        side = -1;
    }
    wall_contact_side = side;

    result.wall_contact = side != 0;
    // 墙深度使用滤波纸面位置，墙状态使用原始纸面位置，从而兼顾安全响应和手感稳定。
    if (result.wall_contact) {
        const float wall_depth_unit =
            (side > 0)
                ? ((wall_axis_mm - high_wall_start) / wall_zone_mm)
                : ((low_wall_start - wall_axis_mm) / wall_zone_mm);
        const float wall_current_a =
            wallCurrentForSide(side, fastWallHardeningCurve(wall_depth_unit), config);
        const float damping_current_a =
            wallDampingCurrentForSide(side, filtered_velocity_deg_s, config);
        result.target_current_a =
            clampWallCurrentForSide(side, wall_current_a + damping_current_a, config);
    }
    return result;
}
#endif

}  // namespace

// 力反馈主函数：根据角度、速度和测试模式生成未最终斜率限制的目标电流。
MasterHapticEngineOutput computeMasterHapticCommand(MasterHapticEngineState &state,
                                                    const MasterHapticEngineInput &input,
                                                    const MasterAxisConfig &config) {
    MasterHapticEngineOutput output = {};
#if MASTER_ENABLE_FIXED_CURRENT_TEST
    output.target_current_a = kMasterHapticDiagnostic.fixed_current_test_a;
#elif MASTER_ENABLE_ZERO_CURRENT_TEST
    output.target_current_a = 0.0f;
#else
    if (!state.has_filtered_wall_axis) {
        state.filtered_wall_axis_mm = input.axis_mm;
        state.has_filtered_wall_axis = true;
    } else {
        state.filtered_wall_axis_mm =
            lowPassFilter(input.axis_mm,
                          state.filtered_wall_axis_mm,
                          input.dt_s,
                          config.haptic.wall.lpf_tf_s);
    }

    const float damping_current_a =
        computeCenterDampingCurrent(input.filtered_velocity_deg_s, config);
    const BoundaryCurrentResult boundary_current =
        computeBoundaryCurrent(input.axis_mm,
                                state.filtered_wall_axis_mm,
                                input.paper_half_range_mm,
                                input.filtered_velocity_deg_s,
                                state.wall_contact_side,
                                config);

    float target_current_candidate_a = damping_current_a;
    if (boundary_current.safety_cut) {
        target_current_candidate_a = 0.0f;
    } else if (boundary_current.target_current_a != 0.0f) {
        target_current_candidate_a = boundary_current.wall_contact
                                         ? boundary_current.target_current_a + damping_current_a
                                         : boundary_current.target_current_a;
    }

    output.target_current_a =
        clampFloat(target_current_candidate_a,
                   -config.current.limit_a,
                   config.current.limit_a);
    output.boundary_active = boundary_current.wall_contact ||
                             boundary_current.overrun_pushback ||
                             boundary_current.safety_cut;
    output.boundary_safety_cut = boundary_current.safety_cut;
#endif
    return output;
}
