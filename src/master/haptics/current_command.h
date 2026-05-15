#pragma once

#include <stdint.h>

// 跨周期状态：保存上一周期电流命令和模式，用于斜率限制与模式切换检测。
struct MasterCurrentCommandState {
    // 上一周期输出到电机层的电流命令，单位 A。
    float current_command_a;
    // 上一周期是否处于安全切断状态。
    bool previous_boundary_safety_cut;
    // 上一周期目标模式，用于检测测试模式切换。
    uint8_t previous_target_mode;
};

// 当前周期输入：来自力反馈算法的目标电流和状态标志。
struct MasterCurrentCommandInput {
    // 力反馈算法给出的目标电流，单位 A。
    float target_current_a;
    // 当前控制周期实际 dt，单位 s。
    float dt_s;
    // 当前是否触发越界安全切断。
    bool boundary_safety_cut;
    // 当前目标电流模式编号。
    uint8_t target_mode;
};

// 当前周期输出：最终送给电机层的电流命令，以及是否请求清 PID。
struct MasterCurrentCommandOutput {
    float current_command_a;
    // 请求上层清空 q/d PID 状态，避免残余输出。
    bool request_pid_reset;
};

MasterCurrentCommandOutput updateMasterCurrentCommand(MasterCurrentCommandState &state,
                                                      const MasterCurrentCommandInput &input);
