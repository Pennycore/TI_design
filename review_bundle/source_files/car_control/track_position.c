#include "track_position.h"
#include "encoder.h"

#include <stdint.h>

#define TRACK_PI                       (3.14159265358979323846f)

#define TRACK_LEFT_MM_PER_COUNT        \
    ((TRACK_PI * TRACK_WHEEL_DIAMETER_MM) / \
     TRACK_LEFT_COUNTS_PER_WHEEL_REV)

#define TRACK_RIGHT_MM_PER_COUNT       \
    ((TRACK_PI * TRACK_WHEEL_DIAMETER_MM) / \
     TRACK_RIGHT_COUNTS_PER_WHEEL_REV)

typedef struct
{
    int32_t previous_left_count;
    int32_t previous_right_count;

    float left_distance_mm;
    float right_distance_mm;
    float average_distance_mm;
    float finish_marker_start_distance_mm;
    float finish_marker_distance_mm;
    float stop_target_distance_mm;

    uint32_t finish_marker_time_ms;

    uint8_t finish_detection_enabled;
    uint8_t finish_confirmed;
    uint8_t initialized;
} TrackPositionData;

static TrackPositionData g_track;

/*
 * 计算两个编码器计数之间的绝对增量。
 *
 * 使用绝对值的原因：
 * 左右轮方向修正后理论上前进都应为正，
 * 但弯道内侧轮可能出现短暂反向或抖动。
 * 里程模块只统计车轮实际运动量。
 */
static uint32_t TrackPosition_AbsoluteDelta(int32_t current,
                                            int32_t previous)
{
    int64_t delta = (int64_t)current - (int64_t)previous;

    if (delta < 0)
    {
        delta = -delta;
    }

    if (delta > 0xFFFFFFFFLL)
    {
        return 0xFFFFFFFFU;
    }

    return (uint32_t)delta;
}

static uint32_t TrackPosition_AddTime(uint32_t value,
                                      uint32_t delta_ms)
{
    if (value > (0xFFFFFFFFU - delta_ms))
    {
        return 0xFFFFFFFFU;
    }

    return value + delta_ms;
}

static float TrackPosition_RemainingDistance(float target_mm)
{
    if (g_track.average_distance_mm >= target_mm)
    {
        return 0.0f;
    }

    return target_mm - g_track.average_distance_mm;
}

void TrackPosition_Init(void)
{
    TrackPosition_Reset();
}

void TrackPosition_Reset(void)
{
    Encoder_Value_t count;

    Encoder_GetCount(&count);

    g_track.previous_left_count = count.left;
    g_track.previous_right_count = count.right;

    g_track.left_distance_mm = 0.0f;
    g_track.right_distance_mm = 0.0f;
    g_track.average_distance_mm = 0.0f;
    g_track.finish_marker_start_distance_mm = 0.0f;
    g_track.finish_marker_distance_mm = 0.0f;
    g_track.stop_target_distance_mm = 0.0f;

    g_track.finish_marker_time_ms = 0U;

    g_track.finish_detection_enabled = 0U;
    g_track.finish_confirmed = 0U;
    g_track.initialized = 1U;
}

void TrackPosition_Update(uint8_t marker_a, uint32_t delta_ms)
{
    Encoder_Value_t count;
    uint32_t left_delta_count;
    uint32_t right_delta_count;
    float finish_enable_distance_mm;

    if (g_track.initialized == 0U)
    {
        TrackPosition_Reset();
    }

    Encoder_GetCount(&count);

    left_delta_count = TrackPosition_AbsoluteDelta(
        count.left,
        g_track.previous_left_count);

    right_delta_count = TrackPosition_AbsoluteDelta(
        count.right,
        g_track.previous_right_count);

    g_track.previous_left_count = count.left;
    g_track.previous_right_count = count.right;

    g_track.left_distance_mm +=
        (float)left_delta_count * TRACK_LEFT_MM_PER_COUNT;

    g_track.right_distance_mm +=
        (float)right_delta_count * TRACK_RIGHT_MM_PER_COUNT;

    /*
     * 左右轮平均距离作为小车沿赛道的累计里程。
     * 弯道时一侧走得多、一侧走得少，取平均更合理。
     */
    g_track.average_distance_mm =
        (g_track.left_distance_mm +
         g_track.right_distance_mm) * 0.5f;

    finish_enable_distance_mm =
        TRACK_LAP_DISTANCE_MM * TRACK_FINISH_ENABLE_RATIO;

    /*
     * 只有接近跑完一圈后，才允许识别A点终点横线。
     * 这样启动时压在A点横线上也不会立即结束。
     */
    if (g_track.average_distance_mm >= finish_enable_distance_mm)
    {
        g_track.finish_detection_enabled = 1U;
    }

    if ((g_track.finish_detection_enabled != 0U) &&
        (g_track.finish_confirmed == 0U))
    {
        if (marker_a != 0U)
        {
            if (g_track.finish_marker_time_ms == 0U)
            {
                /*
                 * 保存第一次看到横线时的里程，而不是30 ms确认后的里程。
                 * 168 mm停车距离从传感器首次到达A线开始计算。
                 */
                g_track.finish_marker_start_distance_mm =
                    g_track.average_distance_mm;
            }

            g_track.finish_marker_time_ms =
                TrackPosition_AddTime(
                    g_track.finish_marker_time_ms,
                    delta_ms);

            if (g_track.finish_marker_time_ms >=
                TRACK_FINISH_MARKER_CONFIRM_MS)
            {
                g_track.finish_confirmed = 1U;
                g_track.finish_marker_distance_mm =
                    g_track.finish_marker_start_distance_mm;
                g_track.stop_target_distance_mm =
                    g_track.finish_marker_distance_mm +
                    TRACK_SENSOR_TO_REFERENCE_MM;
            }
        }
        else
        {
            g_track.finish_marker_time_ms = 0U;
            g_track.finish_marker_start_distance_mm = 0.0f;
        }
    }
    else
    {
        g_track.finish_marker_time_ms = 0U;
    }
}

float TrackPosition_GetDistanceMm(void)
{
    return g_track.average_distance_mm;
}

float TrackPosition_GetLeftDistanceMm(void)
{
    return g_track.left_distance_mm;
}

float TrackPosition_GetRightDistanceMm(void)
{
    return g_track.right_distance_mm;
}

uint8_t TrackPosition_HasReachedB(void)
{
    return (g_track.average_distance_mm >=
            (TRACK_DISTANCE_TO_B_MM -
             TRACK_POINT_TOLERANCE_MM)) ? 1U : 0U;
}

uint8_t TrackPosition_HasReachedC(void)
{
    return (g_track.average_distance_mm >=
            (TRACK_DISTANCE_TO_C_MM -
             TRACK_POINT_TOLERANCE_MM)) ? 1U : 0U;
}

uint8_t TrackPosition_HasReachedD(void)
{
    return (g_track.average_distance_mm >=
            (TRACK_DISTANCE_TO_D_MM -
             TRACK_POINT_TOLERANCE_MM)) ? 1U : 0U;
}

uint8_t TrackPosition_IsNearFinish(void)
{
    float slowdown_start_mm =
        TRACK_LAP_DISTANCE_MM -
        TRACK_FINISH_SLOWDOWN_ADVANCE_MM;

    return (g_track.average_distance_mm >=
            slowdown_start_mm) ? 1U : 0U;
}

uint8_t TrackPosition_IsFinishDetectionEnabled(void)
{
    return g_track.finish_detection_enabled;
}

uint8_t TrackPosition_IsAtFinish(void)
{
    return g_track.finish_confirmed;
}

float TrackPosition_GetFinishMarkerDistanceMm(void)
{
    return g_track.finish_marker_distance_mm;
}

float TrackPosition_GetStopTargetDistanceMm(void)
{
    return g_track.stop_target_distance_mm;
}

float TrackPosition_GetRemainingToStop(void)
{
    if (g_track.finish_confirmed == 0U)
    {
        return 0.0f;
    }

    return TrackPosition_RemainingDistance(
        g_track.stop_target_distance_mm);
}

uint8_t TrackPosition_IsStopTargetReached(void)
{
    if (g_track.finish_confirmed == 0U)
    {
        return 0U;
    }

    return (g_track.average_distance_mm >=
            g_track.stop_target_distance_mm)
               ? 1U
               : 0U;
}

TrackPoint TrackPosition_GetCurrentPoint(void)
{
    if (g_track.finish_confirmed != 0U)
    {
        return TRACK_POINT_A_FINISH;
    }

    if (g_track.average_distance_mm >=
        (TRACK_DISTANCE_TO_D_MM -
         TRACK_POINT_TOLERANCE_MM))
    {
        return TRACK_POINT_D;
    }

    if (g_track.average_distance_mm >=
        (TRACK_DISTANCE_TO_C_MM -
         TRACK_POINT_TOLERANCE_MM))
    {
        return TRACK_POINT_C;
    }

    if (g_track.average_distance_mm >=
        (TRACK_DISTANCE_TO_B_MM -
         TRACK_POINT_TOLERANCE_MM))
    {
        return TRACK_POINT_B;
    }

    return TRACK_POINT_A_START;
}

float TrackPosition_GetRemainingToB(void)
{
    return TrackPosition_RemainingDistance(
        TRACK_DISTANCE_TO_B_MM);
}

float TrackPosition_GetRemainingToFinish(void)
{
    return TrackPosition_RemainingDistance(
        TRACK_LAP_DISTANCE_MM);
}
