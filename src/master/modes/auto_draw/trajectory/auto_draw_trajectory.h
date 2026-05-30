#pragma once

#include <stdint.h>

#define MASTER_AUTO_DRAW_PRESET_RECTANGLE 1
#define MASTER_AUTO_DRAW_PRESET_CIRCLE 2
#define MASTER_AUTO_DRAW_PRESET_STAR5 3
#define MASTER_AUTO_DRAW_PRESET_ALL 4

#ifndef MASTER_AUTO_DRAW_PRESET
#define MASTER_AUTO_DRAW_PRESET MASTER_AUTO_DRAW_PRESET_ALL
#endif

struct MasterAutoDrawSegmentSpec {
    int16_t start_x_mm_q10;
    int16_t start_y_mm_q10;
    int16_t end_x_mm_q10;
    int16_t end_y_mm_q10;
    uint16_t feed_mm_s_q10;
    uint8_t pen_req;
};

uint16_t masterAutoDrawTrajectoryTaskId();
uint8_t masterAutoDrawTrajectorySegmentCount();
const char *masterAutoDrawTrajectoryPresetName();
bool masterAutoDrawTrajectorySegmentAt(uint8_t index, MasterAutoDrawSegmentSpec &segment);
