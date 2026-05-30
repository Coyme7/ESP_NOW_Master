#include "master/modes/auto_draw/trajectory/auto_draw_trajectory.h"

#include <math.h>

namespace {

constexpr float kPi = 3.14159265358979323846f;
constexpr uint8_t kAutoDrawPresetId = static_cast<uint8_t>(MASTER_AUTO_DRAW_PRESET);
constexpr uint8_t kAutoDrawConfigRevision = 1U;
constexpr uint16_t kAutoDrawTaskId =
    (static_cast<uint16_t>(kAutoDrawPresetId) << 8) | kAutoDrawConfigRevision;
constexpr uint8_t kAutoDrawProgramCapacity = 48U;
constexpr float kDrawSpeedMmS = 160.0f;
constexpr float kLiftSpeedMmS = 160.0f;
constexpr float kRectCenterX = -68.0f;
constexpr float kCircleCenterX = 0.0f;
constexpr float kStarCenterX = 68.0f;
constexpr float kSinglePresetCenterX = 0.0f;
constexpr float kPresetCenterY = 0.0f;
constexpr float kRectWidthMm = 54.0f;
constexpr float kRectHeightMm = 34.0f;
constexpr float kCircleRadiusMm = 18.0f;
constexpr uint8_t kCircleSides = 24U;
constexpr float kStarOuterRadiusMm = 22.0f;
constexpr float kStarInnerRadiusMm = 8.5f;

struct DrawPoint {
    float x_mm;
    float y_mm;
    bool pen_down;
};

struct DrawSegment {
    DrawPoint start;
    DrawPoint end;
    float feed_mm_s;
};

DrawSegment drawProgram[kAutoDrawProgramCapacity] = {};
uint8_t drawProgramCount = 0;
bool drawProgramReady = false;

int16_t mmToQ10(float value_mm) {
    const long scaled = lroundf(value_mm * 10.0f);
    if (scaled < INT16_MIN) {
        return INT16_MIN;
    }
    if (scaled > INT16_MAX) {
        return INT16_MAX;
    }
    return static_cast<int16_t>(scaled);
}

uint16_t feedToQ10(float feed_mm_s) {
    const long scaled = lroundf(feed_mm_s * 10.0f);
    if (scaled <= 0) {
        return 1U;
    }
    if (scaled > UINT16_MAX) {
        return UINT16_MAX;
    }
    return static_cast<uint16_t>(scaled);
}

bool appendSegment(const DrawPoint &start, const DrawPoint &end, float feed_mm_s, bool pen_down) {
    if (drawProgramCount >= kAutoDrawProgramCapacity) {
        return false;
    }

    DrawSegment segment = {};
    segment.start = {start.x_mm, start.y_mm, pen_down};
    segment.end = {end.x_mm, end.y_mm, pen_down};
    segment.feed_mm_s = feed_mm_s;
    drawProgram[drawProgramCount++] = segment;
    return true;
}

void appendTravel(const DrawPoint &start, const DrawPoint &end) {
    (void)appendSegment(start, end, kLiftSpeedMmS, false);
}

DrawPoint rectangleStart(float center_x_mm, float center_y_mm) {
    return {
        center_x_mm - (kRectWidthMm * 0.5f),
        center_y_mm - (kRectHeightMm * 0.5f),
        false,
    };
}

DrawPoint circleStart(float center_x_mm, float center_y_mm) {
    return {center_x_mm + kCircleRadiusMm, center_y_mm, false};
}

DrawPoint star5Start(float center_x_mm, float center_y_mm) {
    return {center_x_mm, center_y_mm - kStarOuterRadiusMm, false};
}

void appendRectangle(float center_x_mm, float center_y_mm) {
    const float hx = kRectWidthMm * 0.5f;
    const float hy = kRectHeightMm * 0.5f;
    const DrawPoint points[5] = {
        {center_x_mm - hx, center_y_mm - hy, true},
        {center_x_mm + hx, center_y_mm - hy, true},
        {center_x_mm + hx, center_y_mm + hy, true},
        {center_x_mm - hx, center_y_mm + hy, true},
        {center_x_mm - hx, center_y_mm - hy, true},
    };
    for (uint8_t i = 1; i < 5U; ++i) {
        (void)appendSegment(points[i - 1], points[i], kDrawSpeedMmS, true);
    }
}

void appendCircle(float center_x_mm, float center_y_mm) {
    DrawPoint previous = {center_x_mm + kCircleRadiusMm, center_y_mm, true};
    for (uint8_t i = 1; i <= kCircleSides; ++i) {
        const float angle =
            (static_cast<float>(i) / static_cast<float>(kCircleSides)) * 2.0f * kPi;
        const DrawPoint next = {
            center_x_mm + cosf(angle) * kCircleRadiusMm,
            center_y_mm + sinf(angle) * kCircleRadiusMm,
            true,
        };
        (void)appendSegment(previous, next, kDrawSpeedMmS, true);
        previous = next;
    }
}

void appendStar5(float center_x_mm, float center_y_mm) {
    DrawPoint points[11] = {};
    for (uint8_t i = 0; i < 10U; ++i) {
        const bool outer = (i % 2U) == 0U;
        const float radius = outer ? kStarOuterRadiusMm : kStarInnerRadiusMm;
        const float angle = -0.5f * kPi + (static_cast<float>(i) * kPi / 5.0f);
        points[i] = {
            center_x_mm + cosf(angle) * radius,
            center_y_mm + sinf(angle) * radius,
            true,
        };
    }
    points[10] = points[0];
    for (uint8_t i = 1; i < 11U; ++i) {
        (void)appendSegment(points[i - 1], points[i], kDrawSpeedMmS, true);
    }
}

void appendConfiguredPreset() {
    const DrawPoint home = {0.0f, 0.0f, false};

#if MASTER_AUTO_DRAW_PRESET == MASTER_AUTO_DRAW_PRESET_RECTANGLE
    const DrawPoint start = rectangleStart(kSinglePresetCenterX, kPresetCenterY);
    appendTravel(home, start);
    appendRectangle(kSinglePresetCenterX, kPresetCenterY);
#elif MASTER_AUTO_DRAW_PRESET == MASTER_AUTO_DRAW_PRESET_CIRCLE
    const DrawPoint start = circleStart(kSinglePresetCenterX, kPresetCenterY);
    appendTravel(home, start);
    appendCircle(kSinglePresetCenterX, kPresetCenterY);
#elif MASTER_AUTO_DRAW_PRESET == MASTER_AUTO_DRAW_PRESET_STAR5
    const DrawPoint start = star5Start(kSinglePresetCenterX, kPresetCenterY);
    appendTravel(home, start);
    appendStar5(kSinglePresetCenterX, kPresetCenterY);
#else
    const DrawPoint rect_start = rectangleStart(kRectCenterX, kPresetCenterY);
    const DrawPoint circle_start = circleStart(kCircleCenterX, kPresetCenterY);
    const DrawPoint star_start = star5Start(kStarCenterX, kPresetCenterY);

    appendTravel(home, rect_start);
    appendRectangle(kRectCenterX, kPresetCenterY);
    appendTravel(rect_start, circle_start);
    appendCircle(kCircleCenterX, kPresetCenterY);
    appendTravel(circle_start, star_start);
    appendStar5(kStarCenterX, kPresetCenterY);
#endif
}

void ensureAutoDrawProgramReady() {
    if (drawProgramReady) {
        return;
    }

    drawProgramCount = 0;
    appendConfiguredPreset();
    drawProgramReady = drawProgramCount > 0U;
}

}  // namespace

uint16_t masterAutoDrawTrajectoryTaskId() {
    return kAutoDrawTaskId;
}

uint8_t masterAutoDrawTrajectorySegmentCount() {
    ensureAutoDrawProgramReady();
    return drawProgramCount;
}

const char *masterAutoDrawTrajectoryPresetName() {
#if MASTER_AUTO_DRAW_PRESET == MASTER_AUTO_DRAW_PRESET_RECTANGLE
    return "Rectangle";
#elif MASTER_AUTO_DRAW_PRESET == MASTER_AUTO_DRAW_PRESET_CIRCLE
    return "Circle";
#elif MASTER_AUTO_DRAW_PRESET == MASTER_AUTO_DRAW_PRESET_STAR5
    return "Star5";
#else
    return "All";
#endif
}

bool masterAutoDrawTrajectorySegmentAt(uint8_t index, MasterAutoDrawSegmentSpec &segment) {
    ensureAutoDrawProgramReady();
    if (!drawProgramReady || index >= drawProgramCount) {
        return false;
    }

    const DrawSegment &source = drawProgram[index];
    segment.start_x_mm_q10 = mmToQ10(source.start.x_mm);
    segment.start_y_mm_q10 = mmToQ10(source.start.y_mm);
    segment.end_x_mm_q10 = mmToQ10(source.end.x_mm);
    segment.end_y_mm_q10 = mmToQ10(source.end.y_mm);
    segment.feed_mm_s_q10 = feedToQ10(source.feed_mm_s);
    segment.pen_req = (source.start.pen_down || source.end.pen_down) ? 1U : 0U;
    return true;
}
