#include "line_control.h"

#include "gray_sensor.h"
#include "pid.h"
#include "speed_control.h"

#if (LINE_CONTROL_BIT0_IS_LEFT > 1U)
#error "LINE_CONTROL_BIT0_IS_LEFT must be 0 or 1"
#endif

#if (LINE_CONTROL_TRACK_BLACK_LINE > 1U)
#error "LINE_CONTROL_TRACK_BLACK_LINE must be 0 or 1"
#endif

#if (LINE_CONTROL_SWAP_MOTOR_CHANNELS > 1U)
#error "LINE_CONTROL_SWAP_MOTOR_CHANNELS must be 0 or 1"
#endif

#if (LINE_CONTROL_LOST_HOLD_CYCLES > LINE_CONTROL_SEARCH_MAX_CYCLES)
#error "LOST_HOLD_CYCLES must not exceed SEARCH_MAX_CYCLES"
#endif

/* Eight sensor positions, ordered from the physical left to the right. */
static const int16_t g_lineWeights[8] = {
    -3500, -2500, -1500, -500,
      500,  1500,  2500, 3500
};

static PID_t g_linePID;
static float g_baseSpeed;
static float g_filteredPosition;
static float g_lastLeftTarget;
static float g_lastRightTarget;
static int8_t g_lastLineDirection;
static uint8_t g_positionValid;
static LineControl_Status_t g_status;

static float LineControl_Abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

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

    return ((float)weighted_sum / (float)count) / 3500.0f;
}

/*
 * left_target/right_target are physical wheel requests.  The optional swap
 * below adapts those requests to the existing driver and encoder wiring.
 */
static void LineControl_ApplyWheelTargets(
    float left_target,
    float right_target)
{
    g_lastLeftTarget  = left_target;
    g_lastRightTarget = right_target;

    g_status.left_target  = left_target;
    g_status.right_target = right_target;

#if LINE_CONTROL_SWAP_MOTOR_CHANNELS
    SpeedControl_SetTarget(right_target, left_target);
#else
    SpeedControl_SetTarget(left_target, right_target);
#endif
}

static void LineControl_StopWheels(void)
{
    g_lastLeftTarget  = 0.0f;
    g_lastRightTarget = 0.0f;
    g_status.left_target  = 0.0f;
    g_status.right_target = 0.0f;
    SpeedControl_Stop();
}

static void LineControl_GetSearchTargets(
    int8_t direction,
    float *left_target,
    float *right_target)
{
    if (direction > 0)
    {
        /* Line was on the right: left/outer wheel remains faster. */
        *left_target  = LINE_CONTROL_SEARCH_OUTER_SPEED;
        *right_target = LINE_CONTROL_SEARCH_INNER_SPEED;
    }
    else
    {
        *left_target  = LINE_CONTROL_SEARCH_INNER_SPEED;
        *right_target = LINE_CONTROL_SEARCH_OUTER_SPEED;
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

    g_baseSpeed        = LINE_CONTROL_DEFAULT_BASE_SPEED;
    g_filteredPosition = 0.0f;
    g_lastLeftTarget   = 0.0f;
    g_lastRightTarget  = 0.0f;
    g_lastLineDirection = 0;
    g_positionValid    = 0U;

    g_status.raw_sensor    = 0xFFU;
    g_status.line_bits     = 0U;
    g_status.active_count  = 0U;
    g_status.line_detected = 0U;
    g_status.search_active = 0U;
    g_status.last_direction = 0;
    g_status.update_count  = 0U;
    g_status.lost_count    = 0U;
    g_status.raw_position  = 0.0f;
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
    float raw_position;
    float correction;
    float running_base;
    float left_target;
    float right_target;

    raw_sensor = GraySensor_Read();
    line_bits = LineControl_GetLineBits(raw_sensor);
    raw_position = LineControl_GetPosition(line_bits, &active_count);

    g_status.raw_sensor   = raw_sensor;
    g_status.line_bits    = line_bits;
    g_status.active_count = active_count;
    g_status.raw_position = raw_position;
    g_status.update_count++;

    if (active_count == 0U)
    {
        g_status.line_detected = 0U;
        g_status.lost_count++;
        g_status.correction = 0.0f;

        /*
         * Hold the previous command across a very short read glitch.  This
         * avoids stop/start chatter without hiding a genuine line loss.
         */
        if ((g_positionValid != 0U) &&
            (g_status.lost_count <= LINE_CONTROL_LOST_HOLD_CYCLES))
        {
            g_status.search_active = 0U;
            LineControl_ApplyWheelTargets(
                g_lastLeftTarget,
                g_lastRightTarget);
            return;
        }

        PID_Reset(&g_linePID);

        if ((g_lastLineDirection != 0) &&
            (g_status.lost_count <= LINE_CONTROL_SEARCH_MAX_CYCLES))
        {
            LineControl_GetSearchTargets(
                g_lastLineDirection,
                &left_target,
                &right_target);

            g_status.search_active = 1U;
            g_status.correction =
                (float)g_lastLineDirection *
                (LINE_CONTROL_SEARCH_OUTER_SPEED -
                 LINE_CONTROL_SEARCH_INNER_SPEED) *
                0.5f;
            LineControl_ApplyWheelTargets(left_target, right_target);
        }
        else
        {
            g_status.search_active = 0U;
            g_positionValid = 0U;
            LineControl_StopWheels();
        }

        return;
    }

    /*
     * Do not blend a newly reacquired line position with stale data from
     * before the loss.  During normal tracking, filter one-frame bit jitter.
     */
    if ((g_positionValid == 0U) || (g_status.lost_count != 0U))
    {
        g_filteredPosition = raw_position;
        PID_Reset(&g_linePID);
    }
    else
    {
        g_filteredPosition +=
            LINE_CONTROL_POSITION_FILTER_ALPHA *
            (raw_position - g_filteredPosition);
    }

    g_positionValid = 1U;
    g_status.line_detected = 1U;
    g_status.search_active = 0U;
    g_status.lost_count = 0U;
    g_status.position = g_filteredPosition;

    if (g_filteredPosition > LINE_CONTROL_DIRECTION_THRESHOLD)
    {
        g_lastLineDirection = 1;
    }
    else if (g_filteredPosition < -LINE_CONTROL_DIRECTION_THRESHOLD)
    {
        g_lastLineDirection = -1;
    }
    g_status.last_direction = g_lastLineDirection;

    /*
     * A positive position means the line is to the right.  A positive
     * correction therefore speeds up the physical left wheel and slows the
     * physical right wheel.
     */
    correction = PID_Calculate(
        &g_linePID,
        g_filteredPosition,
        0.0f);

    /*
     * Reduce speed progressively near the edge of the sensor bar.  This is
     * continuous steering for the oval ends, not a special corner action.
     */
    running_base =
        g_baseSpeed -
        LINE_CONTROL_CURVE_SLOWDOWN *
        LineControl_Abs(g_filteredPosition);
    running_base = LineControl_Limit(
        running_base,
        LINE_CONTROL_MIN_BASE_SPEED,
        g_baseSpeed);

    left_target  = running_base + correction;
    right_target = running_base - correction;

    /* Ordinary tracking never commands a wheel to reverse. */
    left_target = LineControl_Limit(
        left_target,
        0.0f,
        LINE_CONTROL_MAX_WHEEL_SPEED);
    right_target = LineControl_Limit(
        right_target,
        0.0f,
        LINE_CONTROL_MAX_WHEEL_SPEED);

    g_status.correction = correction;
    LineControl_ApplyWheelTargets(left_target, right_target);
}

void LineControl_Stop(void)
{
    PID_Reset(&g_linePID);

    g_filteredPosition = 0.0f;
    g_lastLineDirection = 0;
    g_positionValid = 0U;
    g_status.line_detected = 0U;
    g_status.search_active = 0U;
    g_status.last_direction = 0;
    g_status.lost_count = 0U;
    g_status.raw_position = 0.0f;
    g_status.position = 0.0f;
    g_status.correction = 0.0f;

    LineControl_StopWheels();
}

void LineControl_SetBaseSpeed(float base_speed)
{
    g_baseSpeed = LineControl_Limit(
        base_speed,
        LINE_CONTROL_MIN_BASE_SPEED,
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
