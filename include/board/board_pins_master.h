#pragma once

// 主机板级引脚定义。
// 当前主从联调阶段，测试电机按 MT6701 编码器 + 2804 BLDC 电机处理。
// 修改 GPIO 前先同步 docs/pinmap_master.md，并确认没有占用启动绑带脚或保留脚。
namespace board_pins_master {

// DengFoc 双路驱动的电流采样输入，供主机 2804 力反馈电流环使用。
static constexpr int MOTOR1_CURRENT_A = 4;
static constexpr int MOTOR1_CURRENT_B = 5;
static constexpr int MOTOR2_CURRENT_A = 1;
static constexpr int MOTOR2_CURRENT_B = 2;

// 电机驱动使能脚；启动安全状态要求先拉低，再由硬件初始化流程接管。
static constexpr int MOTOR1_EN = 6;
static constexpr int MOTOR2_EN = 7;

// 三相 PWM 输出。MOTOR1 当前用于 X 旋钮测试，MOTOR2 预留给第二个旋钮轴。
static constexpr int MOTOR1_PWM_U = 15;
static constexpr int MOTOR1_PWM_V = 16;
static constexpr int MOTOR1_PWM_W = 17;

static constexpr int MOTOR2_PWM_U = 9;
static constexpr int MOTOR2_PWM_V = 10;
static constexpr int MOTOR2_PWM_W = 11;

// MT6701 编码器 SPI/SSI 类接线；每个编码器独立 CS，DO/CLK 按实际板级走线定义。
static constexpr int ENCODER1_CS = 12;
static constexpr int ENCODER1_DO = 13;
static constexpr int ENCODER1_CLK = 14;

static constexpr int ENCODER2_CS = 38;
static constexpr int ENCODER2_DO = 37;
static constexpr int ENCODER2_CLK = 36;

// 人机输入和状态输出。MAIN_BUTTON 当前映射为落笔/抬笔命令。
static constexpr int MAIN_BUTTON = 35;
static constexpr int BOOT_BUTTON = 0;
static constexpr int STATUS_RGB = 48;

// 串口和 USB 固定功能脚，保留给日志、烧录和调试。
static constexpr int UART_TX = 43;
static constexpr int UART_RX = 44;
static constexpr int USB_D_PLUS = 20;
static constexpr int USB_D_MINUS = 19;

// 当前固件未使用的可用脚位。后续启用前必须先更新 pinmap 并说明用途。
static constexpr int UNUSED_DPI_1 = 18;
static constexpr int UNUSED_DPI_2 = 8;
static constexpr int UNUSED_MODE_1 = 42;
static constexpr int UNUSED_MODE_2 = 41;
static constexpr int UNUSED_MODE_3 = 40;
static constexpr int UNUSED_BUZZER = 39;
static constexpr int UNUSED_OLED_1 = 47;
static constexpr int UNUSED_OLED_2 = 21;

// ESP32-S3 启动绑带相关脚，除非重新审查启动电平，否则不要用于新功能。
static constexpr int DO_NOT_USE_STRAP_1 = 3;
static constexpr int DO_NOT_USE_STRAP_2 = 46;
static constexpr int DO_NOT_USE_STRAP_3 = 45;

}  // namespace board_pins_master
