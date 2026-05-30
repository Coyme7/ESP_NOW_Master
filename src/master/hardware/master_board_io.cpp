#include "master/hardware/master_board_io.h"

#include <Arduino.h>
#include <board/board_pins_master.h>

#include "master/config/master_config.h"

namespace {

bool readActiveLowButton(int pin) {
    return digitalRead(pin) == LOW;
}

}  // namespace

// 根据 EN 有效电平配置计算“关闭驱动”时应输出的 GPIO 电平。
int masterDriverDisabledLevel() {
    return MASTER_DRIVER_ENABLE_ACTIVE_HIGH ? LOW : HIGH;
}

// 上电最先调用：把电机 EN 拉到安全关闭态，避免初始化阶段误输出。
void configureMasterSafeOutputs() {
    // 启动安全：任何任务启动前先强制关闭驱动板共用使能。
    const int disabled_level = masterDriverDisabledLevel();
    pinMode(board_pins_master::MOTOR_DRIVER_EN, OUTPUT);
    digitalWrite(board_pins_master::MOTOR_DRIVER_EN, disabled_level);
    pinMode(board_pins_master::MAIN_BUTTON, INPUT_PULLUP);
    pinMode(board_pins_master::MODE_BUTTON_MANUAL_DRAW, INPUT_PULLUP);
    pinMode(board_pins_master::MODE_BUTTON_AUTO_DRAW, INPUT_PULLUP);
    pinMode(board_pins_master::MODE_BUTTON_BLUETOOTH, INPUT_PULLUP);
}

// 读取主机落笔按钮；测试阶段可用宏强制 pen_req。
bool readMasterPenButtonDown() {
    // 按钮使用 INPUT_PULLUP，按下时读到 LOW。该值会进入 MasterCommandPacket.pen_req。
    return readActiveLowButton(board_pins_master::MAIN_BUTTON);
}

bool readMasterManualDrawButtonDown() {
    return readActiveLowButton(board_pins_master::MODE_BUTTON_MANUAL_DRAW);
}

bool readMasterAutoDrawButtonDown() {
    return readActiveLowButton(board_pins_master::MODE_BUTTON_AUTO_DRAW);
}

bool readMasterBluetoothButtonDown() {
    return readActiveLowButton(board_pins_master::MODE_BUTTON_BLUETOOTH);
}
