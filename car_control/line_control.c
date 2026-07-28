#include "line_control.h"

#include "gray_sensor.h"
#include "pid.h"
#include "speed_control.h"

/*
 * 八个探头从左到右的位置权重。
 * 除以 3500 后，最终位置范围约为 -1.0～+1.0。
 */
static const int16_t g_lineWeights[8] = {
    -3500, -2500, -1500, -500,
      500,  1500,  2500, 3500
};

static PID_t g_linePID;
static float g_baseSpeed;
static LineControl_Status_t g_status;
static int8_t g_lastLineDirection;

static float LineControl_Limit(
    float value,
    float minimum,
    float maximum)
{
    if (value > maximum)
    {
        return maximum;
    }

    if (value < minimum)
    {
        return minimum;
    }

    return value;
}

static uint8_t LineControl_GetLineBits(uint8_t raw_sensor)
{
#if LINE_CONTROL_TRACK_BLACK_LINE
    return (uint8_t)(~raw_sensor);
#else
    return raw_sensor;
#endif
}

static float LineControl_GetPosition(
    uint8_t line_bits,
    uint8_t *active_count)
{
    int32_t weighted_sum = 0;
    uint8_t count = 0U;
    uint8_t bit;

    for (bit = 0U; bit < 8U; bit++)
    {
        if ((line_bits & (uint8_t)(1U << bit)) != 0U)
        {
#if LINE_CONTROL_BIT0_IS_LEFT
            weighted_sum += g_lineWeights[bit];
#else
            weighted_sum += g_lineWeights[7U - bit];
#endif
            count++;
        }
    }

    *active_count = count;

    if (count == 0U)
    {
        return 0.0f;
    }

    return
        ((float)weighted_sum / (float)count) /
        3500.0f;
}

static void LineControl_GetCornerTargets(
    int8_t direction,
    float *left_target,
    float *right_target)
{
    if (direction > 0)
    {
        *left_target  = LINE_CONTROL_CORNER_OUTER_SPEED;
        *right_target = LINE_CONTROL_CORNER_INNER_SPEED;
    }
    else
    {
        *left_target  = LINE_CONTROL_CORNER_INNER_SPEED;
        *right_target = LINE_CONTROL_CORNER_OUTER_SPEED;
    }
}

void LineControl_Init(void)
{
    PID_Init(
        &g_linePID,
        LINE_CONTROL_KP,
        LINE_CONTROL_KI,
        LINE_CONTROL_KD,
        LINE_CONTROL_PERIOD_SECONDS,
        -LINE_CONTROL_MAX_CORRECTION,
        LINE_CONTROL_MAX_CORRECTION);

    g_baseSpeed            = LINE_CONTROL_DEFAULT_BASE_SPEED;
    g_lastLineDirection    = 0;
    g_status.raw_sensor    = 0xFFU;
    g_status.line_bits     = 0U;
    g_status.active_count  = 0U;
    g_status.line_detected = 0U;
    g_status.update_count  = 0U;
    g_status.lost_count    = 0U;
    g_status.position      = 0.0f;
    g_status.correction    = 0.0f;
    g_status.left_target   = 0.0f;
    g_status.right_target  = 0.0f;

    SpeedControl_Stop();
}

void LineControl_Update(void)
{
    uint8_t raw_sensor;
    uint8_t line_bits;
    uint8_t active_count;
    float position;
    float correction;
    float left_target;
    float right_target;

    raw_sensor = GraySensor_Read();
    line_bits = LineControl_GetLineBits(raw_sensor);
    position = LineControl_GetPosition(line_bits, &active_count);

    g_status.raw_sensor   = raw_sensor;
    g_status.line_bits    = line_bits;
    g_status.active_count = active_count;
    g_status.update_count++;

    /*
     * 没有任何探头检测到目标线时立即停车。
     * 待硬件方向和赛道确认后，再增加记忆方向搜索策略。
     */
    if (active_count == 0U)
    {
        g_status.line_detected = 0U;
        g_status.lost_count++;
        g_status.position     = 0.0f;
        g_status.correction   = 0.0f;

        PID_Reset(&g_linePID);

        if ((g_lastLineDirection != 0) &&
            (g_status.lost_count <= LINE_CONTROL_SEARCH_MAX_CYCLES))
        {
            LineControl_GetCornerTargets(
                g_lastLineDirection,
                &left_target,
                &right_target);

            g_status.correction =
                (float)g_lastLineDirection *
                LINE_CONTROL_MAX_CORRECTION;
            g_status.left_target  = left_target;
            g_status.right_target = right_target;

            SpeedControl_SetTarget(left_target, right_target);
        }
        else
        {
            g_status.left_target  = 0.0f;
            g_status.right_target = 0.0f;
            SpeedControl_Stop();
        }

        return;
    }

    g_status.lost_count = 0U;

    if (position > LINE_CONTROL_DIRECTION_THRESHOLD)
    {
        g_lastLineDirection = 1;
    }
    else if (position < -LINE_CONTROL_DIRECTION_THRESHOLD)
    {
        g_lastLineDirection = -1;
    }

    /*
     * At a right-angle corner the continuation can cover an outer probe, or
     * briefly cover almost the entire sensor bar. Slow down and pivot around
     * the inner wheel instead of continuing straight through the corner.
     */
    if (((position >= LINE_CONTROL_CORNER_POSITION_THRESHOLD) ||
         (position <= -LINE_CONTROL_CORNER_POSITION_THRESHOLD) ||
         (active_count >= LINE_CONTROL_WIDE_LINE_COUNT)) &&
        (g_lastLineDirection != 0))
    {
        PID_Reset(&g_linePID);
        LineControl_GetCornerTargets(
            g_lastLineDirection,
            &left_target,
            &right_target);

        g_status.line_detected = 1U;
        g_status.position      = position;
        g_status.correction    =
            (float)g_lastLineDirection *
            LINE_CONTROL_MAX_CORRECTION;
        g_status.left_target   = left_target;
        g_status.right_target  = right_target;

        SpeedControl_SetTarget(left_target, right_target);
        return;
    }

    /*
     * position > 0 表示目标线位于车身右侧。
     * 此时提高左轮速度、降低右轮速度，使车辆向右修正。
     */
    correction = PID_Calculate(&g_linePID, position, 0.0f);
    left_target = g_baseSpeed + correction;
    right_target = g_baseSpeed - correction;

    /*
     * 初次调试不允许内侧轮反转，避免传感器方向错误时突然原地旋转。
     */
    left_target = LineControl_Limit(
        left_target,
        0.0f,
        LINE_CONTROL_MAX_WHEEL_SPEED);
    right_target = LineControl_Limit(
        right_target,
        0.0f,
        LINE_CONTROL_MAX_WHEEL_SPEED);

    g_status.line_detected = 1U;
    g_status.position      = position;
    g_status.correction    = correction;
    g_status.left_target   = left_target;
    g_status.right_target  = right_target;

    SpeedControl_SetTarget(left_target, right_target);
}

void LineControl_Stop(void)
{
    PID_Reset(&g_linePID);

    g_lastLineDirection      = 0;
    g_status.correction   = 0.0f;
    g_status.left_target  = 0.0f;
    g_status.right_target = 0.0f;

    SpeedControl_Stop();
}

void LineControl_SetBaseSpeed(float base_speed)
{
    g_baseSpeed = LineControl_Limit(
        base_speed,
        0.0f,
        LINE_CONTROL_MAX_WHEEL_SPEED);
}

void LineControl_SetTunings(float kp, float ki, float kd)
{
    PID_Init(
        &g_linePID,
        kp,
        ki,
        kd,
        LINE_CONTROL_PERIOD_SECONDS,
        -LINE_CONTROL_MAX_CORRECTION,
        LINE_CONTROL_MAX_CORRECTION);
}

void LineControl_GetStatus(LineControl_Status_t *status)
{
    if (status == 0)
    {
        return;
    }

    *status = g_status;
}
