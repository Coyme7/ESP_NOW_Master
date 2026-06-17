#pragma once

#include "common/protocol/protocol_types.h"
#include "common/protocol/protocol_units.h"
#include "master/modes/mode_traits.h"

// 主机配置非法组合检查。
// 普通配置文件不散落 static_assert，便于集中审计 bring-up 风险。
static_assert(COMMON_CONTRACT_VERSION == 1, "common contract version mismatch");
static_assert(sizeof(MasterCommandPacket) == 24, "MasterCommandPacket layout drifted");
static_assert(sizeof(TrajectorySegmentPacket) == 32, "TrajectorySegmentPacket layout drifted");
static_assert(sizeof(SlaveTelemetryPacket) == 44, "SlaveTelemetryPacket layout drifted");
static_assert(sizeof(MasterCommandPacket) <= 250, "ESP-NOW v1 payload limit exceeded");
static_assert(sizeof(TrajectorySegmentPacket) <= 250, "ESP-NOW v1 payload limit exceeded");
static_assert(sizeof(SlaveTelemetryPacket) <= 250, "ESP-NOW v1 payload limit exceeded");

static_assert(MASTER_RUN_MODE == MASTER_MODE_SINGLE_X_10KHZ_ID ||
                  MASTER_RUN_MODE == MASTER_MODE_SINGLE_Y_10KHZ_ID ||
                  MASTER_RUN_MODE == MASTER_MODE_DUAL_XY_5KHZ_ID,
              "invalid MASTER_RUN_MODE");

static_assert(MASTER_STARTUP_APP_MODE == MASTER_STARTUP_APP_MANUAL_DRAW_ID ||
                  MASTER_STARTUP_APP_MODE == MASTER_STARTUP_APP_AUTO_DRAW_ID ||
                  MASTER_STARTUP_APP_MODE == MASTER_STARTUP_APP_DIAGNOSTICS_ID,
              "invalid MASTER_STARTUP_APP_MODE");

static_assert(!(MASTER_ENABLE_AUTO_DRAW && !MASTER_ENABLE_ESPNOW),
              "MASTER_ENABLE_AUTO_DRAW requires MASTER_ENABLE_ESPNOW");

static_assert(!(masterRunModeNeedsMotorHardware(AXIS_X) &&
                MASTER_ENABLE_X_MOTOR_HW &&
                !MASTER_ENABLE_X_ENCODER_HW),
              "X motor output requires X encoder hardware");

static_assert(!(masterRunModeNeedsMotorHardware(AXIS_Y) &&
                MASTER_ENABLE_Y_MOTOR_HW &&
                !MASTER_ENABLE_Y_ENCODER_HW),
              "Y motor output requires Y encoder hardware");

static_assert(!(MASTER_ENABLE_FORCE_PEN_DOWN_TEST && MASTER_ENABLE_BLE),
              "forced pen test and BLE mode should not be enabled together");

static_assert(!(MASTER_ENABLE_STRONG_TORQUE_TEST &&
                (!MASTER_ENABLE_CURRENT_SENSE || !MASTER_ENABLE_FORCE_FEEDBACK)),
              "strong torque test requires current sense and force feedback");

static_assert((MASTER_ENABLE_FIXED_CURRENT_TEST +
               MASTER_ENABLE_ZERO_CURRENT_TEST +
               MASTER_ENABLE_PHASE_SCAN_TEST) <= 1,
              "dangerous motor tests are mutually exclusive");

static_assert(!(MASTER_ENABLE_ZERO_CURRENT_DC_TEST && !MASTER_ENABLE_ZERO_CURRENT_TEST),
              "MASTER_ENABLE_ZERO_CURRENT_DC_TEST requires MASTER_ENABLE_ZERO_CURRENT_TEST");

static_assert(MASTER_ESPNOW_CHANNEL >= 1 && MASTER_ESPNOW_CHANNEL <= 14,
              "MASTER_ESPNOW_CHANNEL must be in 1..14");

static_assert(MASTER_STATUS_LOOP_PERIOD_MS > 0,
              "MASTER_STATUS_LOOP_PERIOD_MS must be greater than 0");

static_assert(MASTER_CONTROL_LOOP_PERIOD_US > 0,
              "MASTER_CONTROL_LOOP_PERIOD_US must be greater than 0");
static_assert((1000UL % MASTER_CONTROL_LOOP_PERIOD_US) == 0,
              "MASTER_CONTROL_LOOP_PERIOD_US must divide 1000us FreeRTOS tick");
static_assert(MASTER_CONTROL_LOOP_PERIOD_US == masterRunModeNominalPeriodUs(),
              "MASTER_RUN_MODE frequency must match MASTER_CONTROL_LOOP_PERIOD_US");
static_assert(MASTER_OUTER_LOOP_PERIOD_US >= MASTER_CONTROL_LOOP_PERIOD_US,
              "MASTER_OUTER_LOOP_PERIOD_US must not be shorter than control loop period");
static_assert((MASTER_OUTER_LOOP_PERIOD_US % MASTER_CONTROL_LOOP_PERIOD_US) == 0,
              "MASTER_OUTER_LOOP_PERIOD_US must be divisible by control loop period");
static_assert(MASTER_OUTER_LOOP_EVERY_N_STEPS > 0,
              "MASTER_OUTER_LOOP_EVERY_N_STEPS must be greater than 0");
static_assert(MASTER_OUTER_LOOP_PERIOD_US >= 1000UL,
              "MASTER_OUTER_LOOP_PERIOD_US must not be shorter than FreeRTOS tick");
static_assert((MASTER_OUTER_LOOP_PERIOD_US % 1000UL) == 0,
              "MASTER_OUTER_LOOP_PERIOD_US must be divisible by 1000us FreeRTOS tick");
static_assert(MASTER_OUTER_LOOP_STALE_TIMEOUT_US >= MASTER_OUTER_LOOP_PERIOD_US,
              "MASTER_OUTER_LOOP_STALE_TIMEOUT_US must not be shorter than outer loop period");
static_assert((MASTER_OUTER_LOOP_STALE_TIMEOUT_US % MASTER_CONTROL_LOOP_PERIOD_US) == 0,
              "MASTER_OUTER_LOOP_STALE_TIMEOUT_US must be divisible by control loop period");
static_assert(MASTER_OUTER_LOOP_STALE_EVERY_N_STEPS > 0,
              "MASTER_OUTER_LOOP_STALE_EVERY_N_STEPS must be greater than 0");

static_assert(MASTER_CONTROL_TIMER_PERIOD_US >= MASTER_CONTROL_LOOP_PERIOD_US,
              "MASTER_CONTROL_TIMER_PERIOD_US must not be shorter than control loop period");

static_assert(MASTER_CONTROL_TIMER_TIMEOUT_MS > 0,
              "MASTER_CONTROL_TIMER_TIMEOUT_MS must be greater than 0");

static_assert(MASTER_CONTROL_STATUS_PUBLISH_DIV <= 65535,
              "MASTER_CONTROL_STATUS_PUBLISH_DIV must fit uint16_t");

static_assert(kMasterCurrentSenseAdcSinglePoolFrames > 0,
              "single-axis ADC DMA pool frames must be greater than 0");
static_assert(kMasterCurrentSenseAdcDualPoolFrames > 0,
              "dual-axis ADC DMA pool frames must be greater than 0");
static_assert(kMasterCurrentSenseAdcStartupWarmupFrames > 0,
              "ADC DMA startup warmup frames must be greater than 0");
static_assert(kMasterCurrentSenseAdcStartupGraceControlCycles > 0,
              "ADC DMA startup grace cycles must be greater than 0");
static_assert(MASTER_ADC_DMA_RUNTIME_FAULT_LATCH_ENABLED == 0 ||
                  MASTER_ADC_DMA_RUNTIME_FAULT_LATCH_ENABLED == 1,
              "MASTER_ADC_DMA_RUNTIME_FAULT_LATCH_ENABLED must be 0 or 1");

static_assert(MASTER_TIMING_DIAG_LEVEL >= 0 && MASTER_TIMING_DIAG_LEVEL <= 2,
              "MASTER_TIMING_DIAG_LEVEL must be 0, 1, or 2");
static_assert(MASTER_DIRECT_CURRENT_SETPOINT_ENABLED == 0 ||
                  MASTER_DIRECT_CURRENT_SETPOINT_ENABLED == 1,
              "MASTER_DIRECT_CURRENT_SETPOINT_ENABLED must be 0 or 1");

static_assert(MASTER_KNOB_HALF_RANGE_DEG > 0.0f,
              "MASTER_KNOB_HALF_RANGE_DEG must be greater than 0");

static_assert(PLOT_X_HALF_RANGE_MM > 0.0f && PLOT_Y_HALF_RANGE_MM > 0.0f,
              "plot half range must be greater than 0");
static_assert(A4_WIDTH_MM == A4_PORTRAIT_WIDTH_MM &&
                  A4_HEIGHT_MM == A4_PORTRAIT_HEIGHT_MM,
              "default A4 paper geometry must be portrait");

constexpr bool masterConfigDirectionSignValid(int8_t sign) {
    return sign == -1 || sign == 1;
}

constexpr bool masterConfigAxisInputValid(const MasterAxisInputConfig &input) {
    return input.center_deg >= 0.0f &&
           input.center_deg <= 360.0f &&
           input.norm_deadband_counts >= 0 &&
           masterConfigDirectionSignValid(input.axis_sign);
}

constexpr bool masterConfigAxisCurrentValid(const MasterAxisCurrentConfig &current) {
    return current.limit_a > 0.0f &&
           current.ramp_a_per_s > 0.0f &&
           current.release_a_per_s > 0.0f;
}

constexpr bool masterConfigWallValid(const MasterHapticWallConfig &wall,
                                     float axis_limit_a,
                                     float paper_half_range_mm) {
    return wall.lpf_tf_s >= 0.0f &&
           wall.start_mm >= 0.0f &&
           wall.start_mm <= wall.hard_limit_mm &&
           wall.hard_limit_mm <= paper_half_range_mm &&
           wall.hard_limit_mm <= wall.safety_cut_mm &&
           wall.release_hyst_mm >= 0.0f &&
           wall.min_current_a >= 0.0f &&
           wall.min_current_a <= axis_limit_a &&
           wall.damping_gain_a_per_deg_s >= 0.0f &&
           wall.damping_limit_a >= 0.0f &&
           wall.damping_limit_a <= axis_limit_a &&
           masterConfigDirectionSignValid(wall.direction_sign);
}

constexpr bool masterConfigCenterDampingValid(const MasterCenterDampingConfig &center,
                                              float axis_limit_a) {
    return masterConfigDirectionSignValid(center.direction_sign) &&
           center.gain_a_per_deg_s >= 0.0f &&
           center.velocity_lpf_tf_s >= 0.0f &&
           center.still_lpf_tf_s >= 0.0f &&
           center.deadband_deg_s >= 0.0f &&
           center.full_speed_deg_s >= center.deadband_deg_s &&
           center.coulomb_a >= 0.0f &&
           center.vel_scale_deg_s > 0.0f &&
           center.limit_a >= 0.0f &&
           center.limit_a <= axis_limit_a;
}

static_assert(masterConfigAxisInputValid(kMasterXAxisInput),
              "kMasterXAxisInput has invalid range or direction");
static_assert(masterConfigAxisInputValid(kMasterYAxisInput),
              "kMasterYAxisInput has invalid range or direction");

static_assert(masterConfigAxisCurrentValid(kMasterXAxisCurrent),
              "kMasterXAxisCurrent has invalid current limits");
static_assert(masterConfigAxisCurrentValid(kMasterYAxisCurrent),
              "kMasterYAxisCurrent has invalid current limits");

static_assert(masterConfigWallValid(kMasterXHaptic.wall,
                                    kMasterXAxisCurrent.limit_a,
                                    PLOT_X_HALF_RANGE_MM),
              "kMasterXHaptic wall range/current is invalid");

static_assert(masterConfigWallValid(kMasterYHaptic.wall,
                                    kMasterYAxisCurrent.limit_a,
                                    PLOT_Y_HALF_RANGE_MM),
              "kMasterYHaptic wall range/current is invalid");

static_assert(masterConfigCenterDampingValid(kMasterXHaptic.center,
                                             kMasterXAxisCurrent.limit_a),
              "kMasterXHaptic center damping is invalid");

static_assert(masterConfigCenterDampingValid(kMasterYHaptic.center,
                                             kMasterYAxisCurrent.limit_a),
              "kMasterYHaptic center damping is invalid");
