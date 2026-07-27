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
    uint32_t primask;

    Encoder_GetSpeed(&actual);

    primask = SpeedControl_EnterCritical();
    left_target  = g_leftTarget;
    right_target = g_rightTarget;
    SpeedControl_ExitCritical(primask);

    if (SpeedControl_IsZero(left_target))
    {
        PID_Reset(&g_leftPID);
        left_output = 0.0f;
    }
    else
    {
        left_output = PID_Calculate(
            &g_leftPID,
            left_target,
            (float)actual.left);
    }

    if (SpeedControl_IsZero(right_target))
    {
        PID_Reset(&g_rightPID);
        right_output = 0.0f;
    }
    else
    {
        right_output = PID_Calculate(
            &g_rightPID,
            right_target,
            (float)actual.right);
    }

    primask = SpeedControl_EnterCritical();
    g_leftActual  = actual.left;
    g_rightActual = actual.right;
    g_leftOutput  = left_output;
    g_rightOutput = right_output;
    SpeedControl_ExitCritical(primask);

    Motor_SetBoth(left_output, right_output);
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
            Encoder_Sample();
            SpeedControl_Update();
            break;

        default:
            break;
    }
}
