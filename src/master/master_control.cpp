#include "master/master_control.h"

#include <math.h>

#include "common/system_state.h"
#include "master/master_config.h"
#include "master/master_hardware.h"

// 主机控制模块。
// 数据流：角度输入 -> 边界力反馈电流 -> sysData 监视状态 -> 硬件输出。
// 注意：这里运行在 10 kHz 控制热路径，注释可以详细，但代码中不能加入串口、
// ESP-NOW、动态内存或长时间等待。

namespace {

float LPFOperator(float input, float previous, float dt_s, float Tf_s) {
    if (Tf_s <= 0.0f) {
        return input;
    }
    if (dt_s <= 0.0f) {
        return previous;
    }

    const float alpha = Tf_s / (Tf_s + dt_s);
    return alpha * previous + (1.0f - alpha) * input;
}

#if !MASTER_FIXED_CURRENT_TEST_ENABLED && !MASTER_CURRENT_ZERO_CURRENT_SMOKE_TEST
struct BoundaryCurrentResult {
    float target_current_a;
    bool wall_contact;      // 墙内侧 1 deg 接触区：(+89,+90] 或 [-90,-89)。
    bool overrun_pushback;  // 已越过 +/-90 deg，但还没到安全切断角。
    bool safety_cut;        // 已超过 min/max 外侧 cut 角度，必须撤掉电流。
};

float fastWallHardeningCurve(float depth_unit) {
    // 0 表示刚进入 89 deg，1 表示到达 90 deg。
    // ease-out cubic 让 89..90 deg 很快变硬，同时 89 deg 处没有电流突变。
    depth_unit = clampFloat(depth_unit, 0.0f, 1.0f);
    const float remaining = 1.0f - depth_unit;
    return 1.0f - remaining * remaining * remaining;
}

float wallCurrentForSide(int8_t side, float wall_factor) {
    // side 只表示高端/低端墙；方向调试只通过 MASTER_HAPTIC_DIRECTION_SIGN 翻转墙/回推电流。
    // 中心阻尼不经过这里，避免把阻尼方向误翻成助推。
    const float direction_sign = (MASTER_HAPTIC_DIRECTION_SIGN < 0) ? -1.0f : 1.0f;
    const float side_sign = (side > 0) ? 1.0f : -1.0f;
    const float current_a =
        direction_sign * side_sign * kMasterXAxis.haptic_current_limit_a *
        clampFloat(wall_factor, 0.0f, 1.0f);
    return clampFloat(current_a,
                      -kMasterXAxis.haptic_current_limit_a,
                      kMasterXAxis.haptic_current_limit_a);
}

float computeCenterDampingCurrent(float filtered_velocity_deg_s) {
#if MASTER_CENTER_DAMPING_ENABLED
    if (MASTER_CENTER_DAMPING_LIMIT_A <= 0.0f ||
        MASTER_CENTER_DAMPING_VEL_SCALE_DEG_S <= 0.0f) {
        return 0.0f;
    }

    // 符号约定：默认顺时针速度为正、正 Iq 产生逆时针阻尼。
    // 如果实测中间区变成助推，只翻转 MASTER_CENTER_DAMPING_DIRECTION_SIGN。
    const float direction_sign = (MASTER_CENTER_DAMPING_DIRECTION_SIGN < 0) ? -1.0f : 1.0f;
    const float damping_current_a =
        direction_sign *
        (MASTER_CENTER_DAMPING_GAIN_A_PER_DEG_S * filtered_velocity_deg_s +
         MASTER_CENTER_DAMPING_COULOMB_A *
             tanhf(filtered_velocity_deg_s / MASTER_CENTER_DAMPING_VEL_SCALE_DEG_S));
    return clampFloat(damping_current_a,
                      -MASTER_CENTER_DAMPING_LIMIT_A,
                      MASTER_CENTER_DAMPING_LIMIT_A);
#else
    (void)filtered_velocity_deg_s;
    return 0.0f;
#endif
}

// 虚拟硬限位几何分区：
// - [-89,+89]：中心主动阻尼，不能产生墙电流。
// - (+89,+90] / [-90,-89)：墙接触区，墙电流可叠加阻尼。
// - 超过 +/-90：越界回推；超过 min/max 外侧 cut：安全切断。
BoundaryCurrentResult computeBoundaryCurrent(float angle_deg) {
    BoundaryCurrentResult result = {0.0f, false, false, false};

    if (angle_deg > kMasterXAxis.max_deg + MASTER_HAPTIC_OVERRUN_CUT_DEG ||
        angle_deg < kMasterXAxis.min_deg - MASTER_HAPTIC_OVERRUN_CUT_DEG) {
        result.safety_cut = true;
        return result;
    }

    if (angle_deg > kMasterXAxis.max_deg) {
        result.overrun_pushback = true;
        result.target_current_a = wallCurrentForSide(1, 1.0f);
        return result;
    }
    if (angle_deg < kMasterXAxis.min_deg) {
        result.overrun_pushback = true;
        result.target_current_a = wallCurrentForSide(-1, 1.0f);
        return result;
    }

    const float wall_zone_deg = fmaxf(kMasterXAxis.boundary_soft_zone_deg, 0.0f);
    const float high_wall_start = kMasterXAxis.max_deg - wall_zone_deg;
    const float low_wall_end = kMasterXAxis.min_deg + wall_zone_deg;

    int8_t side = 0;
    float wall_depth_unit = 0.0f;
    if (wall_zone_deg > 0.0f && angle_deg > high_wall_start) {
        side = 1;
        wall_depth_unit = (angle_deg - high_wall_start) / wall_zone_deg;
    } else if (wall_zone_deg > 0.0f && angle_deg < low_wall_end) {
        side = -1;
        wall_depth_unit = (low_wall_end - angle_deg) / wall_zone_deg;
    }

    result.wall_contact = side != 0;
    if (result.wall_contact) {
        result.target_current_a = wallCurrentForSide(side, fastWallHardeningCurve(wall_depth_unit));
    }
    return result;
}
#endif

int16_t axisAngleDegToNorm(float angle_deg) {
    // 当前电流模式的虚拟墙是实际可用行程；状态/协议显示也应该在墙处达到 +/-100%。
    const float span_deg = kMasterXAxis.max_deg - kMasterXAxis.min_deg;
    if (span_deg <= 0.0f) {
        return 0;
    }

    const float limited = clampFloat(angle_deg, kMasterXAxis.min_deg, kMasterXAxis.max_deg);
    const float unit = ((limited - kMasterXAxis.min_deg) / span_deg) * 2.0f - 1.0f;
    return unitToNorm(unit);
}

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

float rampCurrentCommand(float current_a, float target_a, float dt_s) {
    if (kMasterXAxis.haptic_current_ramp_a_per_s <= 0.0f) {
        return clampFloat(target_a,
                          -kMasterXAxis.haptic_current_limit_a,
                          kMasterXAxis.haptic_current_limit_a);
    }
    if (dt_s <= 0.0f) {
        return current_a;
    }

    const float bounded_dt_s = clampFloat(dt_s, 0.0f, 0.001f);
    const float max_step_a = kMasterXAxis.haptic_current_ramp_a_per_s * bounded_dt_s;
    const float delta_a = clampFloat(target_a - current_a, -max_step_a, max_step_a);
    return clampFloat(current_a + delta_a,
                      -kMasterXAxis.haptic_current_limit_a,
                      kMasterXAxis.haptic_current_limit_a);
}

}  // namespace

void runMasterControlStep(float dt_s) {
    // previous_angle_deg 只在控制任务里使用，不跨任务共享。
    // 角速度用相邻控制步角度差估算，用于阻尼项。
    static float previous_angle_deg = 0.0f;
    static float current_command_a = 0.0f;
    static float filtered_velocity_deg_s = 0.0f;
    static bool has_previous_angle = false;
    static bool previous_boundary_safety_cut = false;
    static uint8_t previous_target_mode = 0xff;
    const float control_angle_deg = readMasterKnobAngleDeg();
    const float clamped_angle_deg = clampFloat(control_angle_deg,
                                              kMasterXAxis.min_deg,
                                              kMasterXAxis.max_deg);
    const float angle_delta_deg =
        has_previous_angle ? signedAngleDeltaDeg(control_angle_deg, previous_angle_deg) : 0.0f;
    const float velocity_deg_s = (dt_s > 0.0f) ? (angle_delta_deg / dt_s) : 0.0f;
    const float Tf = 0.005f;
    filtered_velocity_deg_s = LPFOperator(velocity_deg_s, filtered_velocity_deg_s, dt_s, Tf);
    has_previous_angle = true;
    previous_angle_deg = control_angle_deg;

    // 计算当前力反馈目标，并同步到 sysData，供状态任务打印和通信任务打包。
    // 注意：这里不因 target_current 接近 0 而 reset PID，中心区必须保持连续阻尼。
    bool boundary_active = false;
    bool boundary_safety_cut = false;
    bool reset_current_pid = false;
#if MASTER_FIXED_CURRENT_TEST_ENABLED
    const uint8_t target_mode = 1;
    const float target_current_a = MASTER_FIXED_CURRENT_TEST_A;
#elif MASTER_CURRENT_ZERO_CURRENT_SMOKE_TEST
    const uint8_t target_mode = 2;
    const float target_current_a = 0.0f;
#else
    const uint8_t target_mode = 3;
    const float damping_current_a = computeCenterDampingCurrent(filtered_velocity_deg_s);
    const BoundaryCurrentResult boundary_current = computeBoundaryCurrent(control_angle_deg);
    float target_current_candidate_a = damping_current_a;
    // 优先级：SAFETY_CUT > OVERRUN_PUSHBACK > WALL_CONTACT > CENTER_DAMPING。
    if (boundary_current.safety_cut) {
        target_current_candidate_a = 0.0f;
    } else if (boundary_current.target_current_a != 0.0f) {
        target_current_candidate_a = boundary_current.wall_contact
                                         ? boundary_current.target_current_a + damping_current_a
                                         : boundary_current.target_current_a;
    }
    const float target_current_a =
        clampFloat(target_current_candidate_a,
                   -kMasterXAxis.haptic_current_limit_a,
                   kMasterXAxis.haptic_current_limit_a);
    boundary_active = boundary_current.wall_contact ||
                      boundary_current.overrun_pushback ||
                      boundary_current.safety_cut;
    boundary_safety_cut = boundary_current.safety_cut;
#endif

    if (previous_target_mode != 0xff && target_mode != previous_target_mode) {
        reset_current_pid = true;
    }
    if (boundary_safety_cut && !previous_boundary_safety_cut) {
        reset_current_pid = true;
    }
    if (reset_current_pid) {
        resetMasterMotorCurrentPid();
    }

    if (boundary_safety_cut) {
        current_command_a = 0.0f;
    } else {
        current_command_a = rampCurrentCommand(current_command_a, target_current_a, dt_s);
    }
    previous_boundary_safety_cut = boundary_safety_cut;
    previous_target_mode = target_mode;

    sysData.master_angle_deg = control_angle_deg;
    sysData.master_target_current_a = current_command_a;

    // 协议层不传物理角度，而传 -32768..32767 的归一化坐标。
    // 通信/显示使用当前虚拟墙定义的可用行程：默认 -90..+90 deg -> -100..+100%。
    sysData.master_x_pos = normToPercent(axisAngleDegToNorm(clamped_angle_deg));
    sysData.boundary_hit = updateBoundaryHitHold(boundary_active, dt_s);

    // 若硬件输出关闭，runMasterMotorOutput() 会丢弃目标电流；
    // 这样控制逻辑仍可运行，方便第一阶段验证状态和通信。
    runMasterMotorOutput(current_command_a);
}
