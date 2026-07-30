#include "stepper_motor.h"

#include "ti_msp_dl_config.h"

#include <stdint.h>

#define STEPPER_TIMER_MAX_TICKS        (65536U)
#define STEPPER_MAX_FREQUENCY_HZ       (20000U)

static volatile bool g_stepperBusy;
static volatile bool g_stepperContinuous;
static volatile uint32_t g_stepperRemainingSteps;

static uint32_t StepperMotor_EnterCritical(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    return primask;
}

static void StepperMotor_ExitCritical(uint32_t primask)
{
    if ((primask & 1U) == 0U)
    {
        __enable_irq();
    }
}

static void StepperMotor_ForceStepLow(void)
{
    DL_TimerG_setCCPOutputDisabled(
        PWM_STEPPER_INST,
        DL_TIMER_CCP_DIS_OUT_LOW,
        DL_TIMER_CCP_DIS_OUT_LOW);
}

static void StepperMotor_ConnectStepOutput(void)
{
    DL_TimerG_setCCPOutputDisabled(
        PWM_STEPPER_INST,
        DL_TIMER_CCP_DIS_OUT_SET_BY_OCTL,
        DL_TIMER_CCP_DIS_OUT_SET_BY_OCTL);
}

static void StepperMotor_StopTimer(void)
{
    DL_TimerG_stopCounter(PWM_STEPPER_INST);
    StepperMotor_ForceStepLow();
    g_stepperBusy = false;
    g_stepperContinuous = false;
    g_stepperRemainingSteps = 0U;
}

static bool StepperMotor_ConfigureFrequency(uint32_t frequency_hz)
{
    uint32_t minimum_hz;
    uint32_t period_ticks;
    uint32_t compare_ticks;

    minimum_hz =
        (PWM_STEPPER_INST_CLK_FREQ + STEPPER_TIMER_MAX_TICKS - 1U) /
        STEPPER_TIMER_MAX_TICKS;

    if ((frequency_hz < minimum_hz) ||
        (frequency_hz > STEPPER_MAX_FREQUENCY_HZ))
    {
        return false;
    }

    period_ticks =
        (PWM_STEPPER_INST_CLK_FREQ + (frequency_hz / 2U)) /
        frequency_hz;

    if ((period_ticks < 2U) || (period_ticks > STEPPER_TIMER_MAX_TICKS))
    {
        return false;
    }

    compare_ticks = period_ticks / 2U;

    DL_TimerG_setLoadValue(PWM_STEPPER_INST, period_ticks - 1U);
    DL_TimerG_setTimerCount(PWM_STEPPER_INST, period_ticks - 1U);
    DL_TimerG_setCaptureCompareValue(
        PWM_STEPPER_INST,
        compare_ticks,
        GPIO_PWM_STEPPER_C0_IDX);

    return true;
}

static bool StepperMotor_Start(
    uint32_t steps,
    uint32_t frequency_hz,
    StepperMotor_Direction_t direction,
    bool continuous)
{
    uint32_t primask = StepperMotor_EnterCritical();

    StepperMotor_StopTimer();

    if ((!continuous && (steps == 0U)) ||
        !StepperMotor_ConfigureFrequency(frequency_hz))
    {
        StepperMotor_ExitCritical(primask);
        return false;
    }

    if (direction == STEPPER_DIRECTION_POSITIVE)
    {
        DL_GPIO_setPins(GPIO_STEPPER_PORT, GPIO_STEPPER_DIR_PIN);
    }
    else
    {
        DL_GPIO_clearPins(GPIO_STEPPER_PORT, GPIO_STEPPER_DIR_PIN);
    }

    /* D36A EN is active high. Keep it high after stopping to hold position. */
    DL_GPIO_setPins(GPIO_STEPPER_PORT, GPIO_STEPPER_EN_PIN);

    g_stepperRemainingSteps = steps;
    g_stepperContinuous = continuous;
    g_stepperBusy = true;

    DL_TimerG_clearInterruptStatus(
        PWM_STEPPER_INST,
        DL_TIMERG_INTERRUPT_ZERO_EVENT);
    NVIC_ClearPendingIRQ(PWM_STEPPER_INST_INT_IRQN);
    NVIC_EnableIRQ(PWM_STEPPER_INST_INT_IRQN);

    StepperMotor_ConnectStepOutput();
    DL_TimerG_startCounter(PWM_STEPPER_INST);

    StepperMotor_ExitCritical(primask);
    return true;
}

void StepperMotor_Init(void)
{
    uint32_t primask = StepperMotor_EnterCritical();

    NVIC_DisableIRQ(PWM_STEPPER_INST_INT_IRQN);
    StepperMotor_StopTimer();
    DL_GPIO_clearPins(
        GPIO_STEPPER_PORT,
        GPIO_STEPPER_DIR_PIN | GPIO_STEPPER_EN_PIN);
    DL_TimerG_clearInterruptStatus(
        PWM_STEPPER_INST,
        DL_TIMERG_INTERRUPT_ZERO_EVENT);
    NVIC_ClearPendingIRQ(PWM_STEPPER_INST_INT_IRQN);

    StepperMotor_ExitCritical(primask);
}

bool StepperMotor_StartSteps(
    uint32_t steps,
    uint32_t frequency_hz,
    StepperMotor_Direction_t direction)
{
    return StepperMotor_Start(steps, frequency_hz, direction, false);
}

bool StepperMotor_StartContinuous(
    uint32_t frequency_hz,
    StepperMotor_Direction_t direction)
{
    return StepperMotor_Start(0U, frequency_hz, direction, true);
}

void StepperMotor_Stop(void)
{
    uint32_t primask = StepperMotor_EnterCritical();

    StepperMotor_StopTimer();

    StepperMotor_ExitCritical(primask);
}

void StepperMotor_SetEnabled(bool enabled)
{
    if (enabled)
    {
        DL_GPIO_setPins(GPIO_STEPPER_PORT, GPIO_STEPPER_EN_PIN);
    }
    else
    {
        StepperMotor_Stop();
        DL_GPIO_clearPins(GPIO_STEPPER_PORT, GPIO_STEPPER_EN_PIN);
    }
}

bool StepperMotor_IsBusy(void)
{
    return g_stepperBusy;
}

uint32_t StepperMotor_GetRemainingSteps(void)
{
    return g_stepperRemainingSteps;
}

void PWM_STEPPER_INST_IRQHandler(void)
{
    switch (DL_TimerG_getPendingInterrupt(PWM_STEPPER_INST))
    {
        case DL_TIMERG_IIDX_ZERO:
            if (!g_stepperBusy || g_stepperContinuous)
            {
                break;
            }

            if (g_stepperRemainingSteps > 0U)
            {
                g_stepperRemainingSteps--;
            }

            if (g_stepperRemainingSteps == 0U)
            {
                StepperMotor_StopTimer();
            }
            break;

        default:
            break;
    }
}
