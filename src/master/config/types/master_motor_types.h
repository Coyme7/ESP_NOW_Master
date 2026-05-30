#pragma once

struct MasterMotorPidConfig {
    float p;           // PID P 增益。
    float i;           // PID I 增益。
    float d;           // PID D 增益。
    float output_ramp; // PID 输出斜率限制。
};

struct MasterMotorCurrentLoopConfig {
    MasterMotorPidConfig q; // q 轴电流环 PID。
    MasterMotorPidConfig d; // d 轴电流环 PID。
    float lpf_tf;           // q/d 电流低通滤波时间常数，单位 s。
};

struct MasterMotorFocConfig {
    float supply_voltage_v;                 // 驱动板母线/电源电压，单位 V。
    float voltage_limit_v;                  // SimpleFOC 输出电压上限，单位 V。
    float align_voltage_v;                  // initFOC 对齐/检测使用电压，单位 V。
    MasterMotorCurrentLoopConfig current_loop; // q/d 电流环配置。
};
