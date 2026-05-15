#include "master/hardware/master_board_io.h"

#include <Arduino.h>
#include <board/board_pins_master.h>

#include "master/config/master_config.h"

namespace {

volatile int32_t encoder_count = 0;

#if MASTER_DEMO_QUADRATURE_ENABLED
// 演示正交编码器中断，只在 MASTER_DEMO_QUADRATURE_ENABLED 打开时使用。
void IRAM_ATTR encoderISR() {
    if (digitalRead(MASTER_DEMO_ENCODER_PIN_A) == digitalRead(MASTER_DEMO_ENCODER_PIN_B)) {
        encoder_count++;
    } else {
        encoder_count--;
    }
}
#endif

}  // namespace

// 根据 EN 有效电平配置计算“关闭驱动”时应输出的 GPIO 电平。
int masterDriverDisabledLevel() {
    return MASTER_DRIVER_ENABLE_ACTIVE_HIGH ? LOW : HIGH;
}

// 上电最先调用：把电机 EN 拉到安全关闭态，避免初始化阶段误输出。
void configureMasterSafeOutputs() {
    // 启动安全：任何任务启动前先强制关闭两路电机使能。
    const int disabled_level = masterDriverDisabledLevel();
    pinMode(board_pins_master::MOTOR1_EN, OUTPUT);
    pinMode(board_pins_master::MOTOR2_EN, OUTPUT);
    digitalWrite(board_pins_master::MOTOR1_EN, disabled_level);
    digitalWrite(board_pins_master::MOTOR2_EN, disabled_level);
    pinMode(MASTER_DEMO_ENCODER_PIN_A, INPUT_PULLUP);
    pinMode(MASTER_DEMO_ENCODER_PIN_B, INPUT_PULLUP);
    pinMode(board_pins_master::MAIN_BUTTON, INPUT_PULLUP);
#if MASTER_DEMO_QUADRATURE_ENABLED
    // 只有显式启用临时演示输入时才挂中断，避免默认占用未确认引脚。
    attachInterrupt(digitalPinToInterrupt(MASTER_DEMO_ENCODER_PIN_A), encoderISR, FALLING);
#endif
}

// 读取主机落笔按钮；测试阶段可用宏强制 pen_down。
bool readMasterPenButtonDown() {
    // 按钮使用 INPUT_PULLUP，按下时读到 LOW。该值会进入 MasterCommandPacket.pen_down。
    return digitalRead(board_pins_master::MAIN_BUTTON) == LOW;
}

// 读取临时正交编码器计数，用于无 MT6701 的早期演示。
int32_t readMasterDemoEncoderCount() {
    return encoder_count;
}

