#include "speed_control.h"

#include "encoder.h"
#include "motor.h"
#include "pid.h"
#include "ti_msp_dl_config.h"

#include <stdint.h>

#define SPEED_CONTROL_OUTPUT_MIN       (-100.0f)
#define SPEED_CONTROL_OUTPUT_MAX       (100.0f)
#define SPEED_CONTROL_ZERO_THRESHOLD   (0.001f)

static PID_t g_leftPID;
static PID_t g_rightPID;

static volatile float g_leftTarget;
static volatile float g_rightTarget;
static volatile int32_t g_leftActual;
static volatile int32_t g_rightActual;
static volatile float g_leftOutput;
static volatile float g_rightOutput;
static volatile uint32_t g_tickCount;

static uint32_t SpeedControl_EnterCritical(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    return primask;
}

static void SpeedControl_ExitCritical(uint32_t primask)
{
    if ((primask & 1U) == 0U)
    {
        __enable_irq();
    }
}

static int SpeedControl_IsZero(float value)
{
    return
        (value > -SPEED_CONTROL_ZERO_THRESHOLD) &&
        (value < SPEED_CONTROL_ZERO_THRESHOLD);
}

static int SpeedControl_DirectionChanged(float old_value, float new_value)
{
    return
        ((old_value > 0.0f) && (new_value < 0.0f)) ||
        ((old_value < 0.0f) && (new_value > 0.0f));
}

static float SpeedControl_Abs(float value)
{
    if (value < 0.0f)
    {
        return -value;
    }

    return value;
}

static float SpeedControl_Limit(
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

/*
 * 目标速度的符号只负责决定电机方向；PID 比较速度绝对值，只修正
 * 0～100% 的占空比。实际速度略高于目标时只会降低驱动力，不会反转。
 */
static float SpeedControl_ApplyOutputSlew(
    PID_t *pid,
    float previous,
    float requested)
{
    float limited = requested;

    if (requested > (previous + SPEED_CONTROL_OUTPUT_RISE_STEP))
    {
        limited = previous + SPEED_CONTROL_OUTPUT_RISE_STEP;
    }
    else if (requested < (previous - SPEED_CONTROL_OUTPUT_FALL_STEP))
    {
        limited = previous - SPEED_CONTROL_OUTPUT_FALL_STEP;
    }

    /*
     * The output limiter, rather than the wheel, is responsible for this
     * temporary error. Clear PID history to prevent soft-start wind-up.
     */
    if (limited != requested)
    {
        PID_Reset(pid);
    }

    return limited;
}

static float SpeedControl_CalculateWheel(
    PID_t *pid,
    float target,
    float actual)
{
    float direction;
    float target_magnitude;
    float actual_magnitude;
    float feedforward;
    float correction;
    float duty;

    if (SpeedControl_IsZero(target))
    {
        PID_Reset(pid);
        return 0.0f;
    }

    direction = (target > 0.0f) ? 1.0f : -1.0f;
    target_magnitude = SpeedControl_Abs(target);
    actual_magnitude = SpeedControl_Abs(actual);

#if COMPETITION_SEGMENTED_SPEED_FF_ENABLE
    /*
     * Tasks 4/5/6 carry the ball. Preserve the proven low-speed drive level,
     * but reduce the high-speed slope so the two wheel commands do not both
     * collapse to 100% duty in a curve.
     *
     * target <= 5: 35 + 6.0 * target
     * target >  5: 65 + 2.5 * (target - 5)
     */
    if (target_magnitude <= SPEED_CONTROL_LOW_SPEED_LIMIT)
    {
        feedforward =
            SPEED_CONTROL_MIN_DRIVE_DUTY +
            SPEED_CONTROL_LOW_SPEED_FF_GAIN * target_magnitude;
    }
    else
    {
        feedforward =
            SPEED_CONTROL_MIN_DRIVE_DUTY +
            SPEED_CONTROL_LOW_SPEED_FF_GAIN *
                SPEED_CONTROL_LOW_SPEED_LIMIT +
            SPEED_CONTROL_HIGH_SPEED_FF_GAIN *
                (target_magnitude -
                 SPEED_CONTROL_LOW_SPEED_LIMIT);
    }
#else
    /*
     * Task 2 uses the earlier aggressive response. At cruise speed 18 this
     * intentionally reaches the output limit, prioritising lap time.
     */
    feedforward =
        SPEED_CONTROL_MIN_DRIVE_DUTY +
        SPEED_CONTROL_LOW_SPEED_FF_GAIN * target_magnitude;
#endif
    correction = PID_Calculate(
        pid,
        target_magnitude,
        actual_magnitude);
    duty = SpeedControl_Limit(
        feedforward + correction,
        0.0f,
        SPEED_CONTROL_OUTPUT_MAX);

    /*
     * A low-speed command that is sufficient with the wheels lifted may not
     * overcome tire, gearbox and chassis static friction on the floor.
     * Apply a bounded breakaway command only while the encoder is stationary.
     */
    if ((actual_magnitude < SPEED_CONTROL_MOVING_THRESHOLD) &&
        (duty < SPEED_CONTROL_BREAKAWAY_DUTY))
    {
        duty = SPEED_CONTROL_BREAKAWAY_DUTY;
    }

    return direction * duty;
}

void SpeedControl_Init(void)
{
    uint32_t primask;

    PID_Init(
        &g_leftPID,
        SPEED_CONTROL_LEFT_KP,
        SPEED_CONTROL_LEFT_KI,
        SPEED_CONTROL_LEFT_KD,
        SPEED_CONTROL_PERIOD_SECONDS,
        SPEED_CONTROL_OUTPUT_MIN,
        SPEED_CONTROL_OUTPUT_MAX);

    PID_Init(
        &g_rightPID,
        SPEED_CONTROL_RIGHT_KP,
        SPEED_CONTROL_RIGHT_KI,
        SPEED_CONTROL_RIGHT_KD,
        SPEED_CONTROL_PERIOD_SECONDS,
        SPEED_CONTROL_OUTPUT_MIN,
        SPEED_CONTROL_OUTPUT_MAX);

    primask = SpeedControl_EnterCritical();

    g_leftTarget  = 0.0f;
    g_rightTarget = 0.0f;
    g_leftActual  = 0;
    g_rightActual = 0;
    g_leftOutput  = 0.0f;
    g_rightOutput = 0.0f;
    g_tickCount   = 0U;

    Motor_Stop();

    NVIC_ClearPendingIRQ(TIMER_CONTROL_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_CONTROL_INST_INT_IRQN);
    DL_TimerA_startCounter(TIMER_CONTROL_INST);

    SpeedControl_ExitCritical(primask);
}

void SpeedControl_SetTarget(float left_target, float right_target)
{
    uint32_t primask = SpeedControl_EnterCritical();

    if (SpeedControl_IsZero(left_target) ||
        SpeedControl_DirectionChanged(g_leftTarget, left_target))
    {
        PID_Reset(&g_leftPID);
    }

    if (SpeedControl_IsZero(right_target) ||
        SpeedControl_DirectionChanged(g_rightTarget, right_target))
    {
        PID_Reset(&g_rightPID);
    }

    g_leftTarget  = left_target;
    g_rightTarget = right_target;

    SpeedControl_ExitCritical(primask);

    if (SpeedControl_IsZero(left_target) &&
        SpeedControl_IsZero(right_target))
    {
        Motor_Stop();
    }
}

void SpeedControl_Stop(void)
{
    uint32_t primask = SpeedControl_EnterCritical();

    g_leftTarget  = 0.0f;
    g_rightTarget = 0.0f;
    g_leftOutput  = 0.0f;
    g_rightOutput = 0.0f;

    PID_Reset(&g_leftPID);
    PID_Reset(&g_rightPID);

    Motor_Stop();

    SpeedControl_ExitCritical(primask);
}

void SpeedControl_Update(void)
{
    Encoder_Value_t actual;
    float left_target;
    float right_target;
    float left_output;
    float right_output;
    float previous_left_output;
    float previous_right_output;
    uint32_t primask;

    Encoder_GetSpeed(&actual);

    primask = SpeedControl_EnterCritical();
    left_target  = g_leftTarget;
    right_target = g_rightTarget;
    previous_left_output = g_leftOutput;
    previous_right_output = g_rightOutput;
    SpeedControl_ExitCritical(primask);

    left_output = SpeedControl_CalculateWheel(
        &g_leftPID,
        left_target,
        (float)actual.left);

    right_output = SpeedControl_CalculateWheel(
        &g_rightPID,
        right_target,
        (float)actual.right);

    if (SpeedControl_IsZero(left_target))
    {
        left_output = 0.0f;
    }
    else
    {
        left_output = SpeedControl_ApplyOutputSlew(
            &g_leftPID,
            previous_left_output,
            left_output);
    }

    if (SpeedControl_IsZero(right_target))
    {
        right_output = 0.0f;
    }
    else
    {
        right_output = SpeedControl_ApplyOutputSlew(
            &g_rightPID,
            previous_right_output,
            right_output);
    }

    primask = SpeedControl_EnterCritical();
    g_leftActual  = actual.left;
    g_rightActual = actual.right;
    g_leftOutput  = left_output;
    g_rightOutput = right_output;
    SpeedControl_ExitCritical(primask);

    Motor_SetBoth(left_output, right_output);
}

uint32_t SpeedControl_GetTickCount(void)
{
    uint32_t tick_count;
    uint32_t primask = SpeedControl_EnterCritical();

    tick_count = g_tickCount;

    SpeedControl_ExitCritical(primask);
    return tick_count;
}

void SpeedControl_GetStatus(SpeedControl_Status_t *status)
{
    uint32_t primask;

    if (status == 0)
    {
        return;
    }

    primask = SpeedControl_EnterCritical();

    status->left_target  = g_leftTarget;
    status->right_target = g_rightTarget;
    status->left_actual  = g_leftActual;
    status->right_actual = g_rightActual;
    status->left_output  = g_leftOutput;
    status->right_output = g_rightOutput;

    SpeedControl_ExitCritical(primask);
}

void SpeedControl_SetTunings(
    float left_kp,
    float left_ki,
    float left_kd,
    float right_kp,
    float right_ki,
    float right_kd)
{
    uint32_t primask = SpeedControl_EnterCritical();

    PID_Init(
        &g_leftPID,
        left_kp,
        left_ki,
        left_kd,
        SPEED_CONTROL_PERIOD_SECONDS,
        SPEED_CONTROL_OUTPUT_MIN,
        SPEED_CONTROL_OUTPUT_MAX);

    PID_Init(
        &g_rightPID,
        right_kp,
        right_ki,
        right_kd,
        SPEED_CONTROL_PERIOD_SECONDS,
        SPEED_CONTROL_OUTPUT_MIN,
        SPEED_CONTROL_OUTPUT_MAX);

    SpeedControl_ExitCritical(primask);
}

void TIMER_CONTROL_INST_IRQHandler(void)
{
    switch (DL_TimerA_getPendingInterrupt(TIMER_CONTROL_INST))
    {
        case DL_TIMER_IIDX_ZERO:
            g_tickCount++;
            Encoder_Sample();
            SpeedControl_Update();
            break;

        default:
            break;
    }
}
