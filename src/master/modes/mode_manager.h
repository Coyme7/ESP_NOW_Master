#pragma once

#include <stdint.h>

#include "master/modes/mode_switch.h"
#include "master/modes/mode_types.h"

// 将模式切换事件映射为业务模式请求；不读取 GPIO，也不发送协议包。
void updateMasterModeFromSwitches(const MasterModeSwitchEvents &events, uint32_t now_ms);

uint8_t masterDefaultRuntimeMode();
bool masterRuntimeModeAvailable(uint8_t mode);
MasterRuntimeModeSnapshot getMasterRuntimeModeSnapshot();
const char *masterRuntimeModeName(uint8_t mode);
const char *masterRuntimeButtonName(uint8_t button);
