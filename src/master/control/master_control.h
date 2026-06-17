#pragma once

// 5kHz 快环：只做 ADC fault gate、硬件输出和 SimpleFOC loopFOC()。
void runMasterFastCurrentLoop();

// 1kHz 慢环：读取缓存角度，计算 haptic、速度估计、current command 和状态发布。
void runMasterOuterLoopSlow(float dt_s);
