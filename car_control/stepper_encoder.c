#include "stepper_encoder.h"

#include "ti_msp_dl_config.h"

#include <stdint.h>

static volatile int32_t g_encoderExtendedCount;
static volatile uint16_t g_encoderLastHardwareCount;
static volatile uint32_t g_encoderIndexCount;
static volatile bool g_encoderLastIndexLevel;

static volatile bool g_pwmCaptureSynced;
static volatile bool g_pwmCaptureValid;
static volatile uint32_t g_pwmPeriodTicks;
static volatile uint32_t g_pwmHighTicks;
static uint32_t g_pwmCaptureLoad;

static uint32_t StepperEncoder_EnterCritical(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    return primask;
}

static void StepperEncoder_ExitCritical(uint32_t primask)
{
    if ((primask & 1U) == 0U)
    {
        __enable_irq();
    }
}

void StepperEncoder_Init(void)
{
    uint32_t primask = StepperEncoder_EnterCritical();

    DL_TimerG_stopCounter(QEI_STEPPER_INST);
    DL_TimerG_setTimerCount(QEI_STEPPER_INST, 0U);

    g_encoderExtendedCount = 0;
    g_encoderLastHardwareCount = 0U;
    g_encoderIndexCount = 0U;
    g_encoderLastIndexLevel =
        DL_GPIO_readPins(GPIO_STEPPER_Z_PORT, GPIO_STEPPER_Z_Z_PIN) != 0U;

    g_pwmCaptureLoad =
        DL_TimerG_getLoadValue(CAPTURE_STEPPER_PWM_INST);
    g_pwmCaptureSynced = false;
    g_pwmCaptureValid = false;
    g_pwmPeriodTicks = 0U;
    g_pwmHighTicks = 0U;

    DL_TimerG_setTimerCount(
        CAPTURE_STEPPER_PWM_INST,
        g_pwmCaptureLoad);
    DL_TimerG_clearInterruptStatus(
        CAPTURE_STEPPER_PWM_INST,
        DL_TIMERG_INTERRUPT_CC1_DN_EVENT |
        DL_TIMERG_INTERRUPT_ZERO_EVENT);
    NVIC_ClearPendingIRQ(CAPTURE_STEPPER_PWM_INST_INT_IRQN);
    NVIC_EnableIRQ(CAPTURE_STEPPER_PWM_INST_INT_IRQN);

    DL_TimerG_startCounter(QEI_STEPPER_INST);
    DL_TimerG_startCounter(CAPTURE_STEPPER_PWM_INST);

    StepperEncoder_ExitCritical(primask);
}

void StepperEncoder_Update(void)
{
    uint16_t hardware_count;
    int16_t delta;
    bool index_level;
    uint32_t primask;

    hardware_count =
        (uint16_t)DL_TimerG_getTimerCount(QEI_STEPPER_INST);
    index_level =
        DL_GPIO_readPins(GPIO_STEPPER_Z_PORT, GPIO_STEPPER_Z_Z_PIN) != 0U;

    primask = StepperEncoder_EnterCritical();

    delta = (int16_t)(hardware_count - g_encoderLastHardwareCount);
    g_encoderExtendedCount += (int32_t)delta;
    g_encoderLastHardwareCount = hardware_count;

    if (index_level && !g_encoderLastIndexLevel)
    {
        g_encoderIndexCount++;
    }
    g_encoderLastIndexLevel = index_level;

    StepperEncoder_ExitCritical(primask);
}

void StepperEncoder_Reset(void)
{
    uint32_t primask = StepperEncoder_EnterCritical();

    DL_TimerG_setTimerCount(QEI_STEPPER_INST, 0U);
    g_encoderExtendedCount = 0;
    g_encoderLastHardwareCount = 0U;
    g_encoderIndexCount = 0U;

    StepperEncoder_ExitCritical(primask);
}

int32_t StepperEncoder_GetCount(void)
{
    int32_t count;
    uint32_t primask;

    StepperEncoder_Update();

    primask = StepperEncoder_EnterCritical();
    count = g_encoderExtendedCount;
    StepperEncoder_ExitCritical(primask);

    return count;
}

float StepperEncoder_GetAngleDegrees(void)
{
    return
        ((float)StepperEncoder_GetCount() * 360.0f) /
        (float)STEPPER_ENCODER_COUNTS_PER_REVOLUTION;
}

uint32_t StepperEncoder_GetIndexCount(void)
{
    uint32_t count;
    uint32_t primask = StepperEncoder_EnterCritical();

    count = g_encoderIndexCount;

    StepperEncoder_ExitCritical(primask);
    return count;
}

bool StepperEncoder_GetPwmAngleDegrees(float *angle_degrees)
{
    uint32_t period_ticks;
    uint32_t high_ticks;
    bool valid;
    uint32_t primask;

    if (angle_degrees == 0)
    {
        return false;
    }

    primask = StepperEncoder_EnterCritical();
    valid = g_pwmCaptureValid;
    period_ticks = g_pwmPeriodTicks;
    high_ticks = g_pwmHighTicks;
    StepperEncoder_ExitCritical(primask);

    if (!valid || (period_ticks == 0U) || (high_ticks > period_ticks))
    {
        return false;
    }

    *angle_degrees =
        ((float)high_ticks * 360.0f) / (float)period_ticks;
    return true;
}

void CAPTURE_STEPPER_PWM_INST_IRQHandler(void)
{
    switch (DL_TimerG_getPendingInterrupt(CAPTURE_STEPPER_PWM_INST))
    {
        case DL_TIMERG_IIDX_CC1_DN:
            if (g_pwmCaptureSynced)
            {
                uint32_t period_capture =
                    DL_TimerG_getCaptureCompareValue(
                        CAPTURE_STEPPER_PWM_INST,
                        DL_TIMER_CC_1_INDEX);
                uint32_t high_capture =
                    DL_TimerG_getCaptureCompareValue(
                        CAPTURE_STEPPER_PWM_INST,
                        DL_TIMER_CC_0_INDEX);

                g_pwmPeriodTicks = g_pwmCaptureLoad - period_capture;
                g_pwmHighTicks = g_pwmCaptureLoad - high_capture;
                g_pwmCaptureValid = true;
            }
            else
            {
                g_pwmCaptureSynced = true;
            }

            /*
             * MSPM0 timer erratum TIMER_ERR_01 requires manual reload in
             * combined pulse-width/period capture mode.
             */
            DL_TimerG_setTimerCount(
                CAPTURE_STEPPER_PWM_INST,
                g_pwmCaptureLoad);
            break;

        case DL_TIMERG_IIDX_ZERO:
            g_pwmCaptureSynced = false;
            g_pwmCaptureValid = false;
            break;

        default:
            break;
    }
}
