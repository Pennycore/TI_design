#include "encoder.h"
#include "ti_msp_dl_config.h"

#include <limits.h>
#include <stdint.h>

#if ((ENCODER_LEFT_DIRECTION != 1) && (ENCODER_LEFT_DIRECTION != -1))
#error "ENCODER_LEFT_DIRECTION must be 1 or -1"
#endif

#if ((ENCODER_RIGHT_DIRECTION != 1) && (ENCODER_RIGHT_DIRECTION != -1))
#error "ENCODER_RIGHT_DIRECTION must be 1 or -1"
#endif

#define ENCODER_LEFT_PINS  \
    (GPIO_ENCODER_LEFT_A_PIN | GPIO_ENCODER_LEFT_B_PIN)

#define ENCODER_RIGHT_PINS \
    (GPIO_ENCODER_RIGHT_A_PIN | GPIO_ENCODER_RIGHT_B_PIN)

#define ENCODER_ALL_PINS   (ENCODER_LEFT_PINS | ENCODER_RIGHT_PINS)

/*
 * 下标由“上一次 AB 状态 + 当前 AB 状态”组成。
 * 合法的单步变化得到 +1 或 -1，抖动及同时跳变得到 0。
 */
static const int8_t g_quadratureTable[16] = {
     0, -1,  1,  0,
     1,  0,  0, -1,
    -1,  0,  0,  1,
     0,  1, -1,  0
};

static volatile int32_t g_leftCount;
static volatile int32_t g_rightCount;
static volatile int32_t g_leftSpeed;
static volatile int32_t g_rightSpeed;

static int32_t g_lastLeftSample;
static int32_t g_lastRightSample;
static uint8_t g_leftPreviousState;
static uint8_t g_rightPreviousState;

static uint8_t Encoder_MakeState(
    uint32_t input,
    uint32_t a_pin,
    uint32_t b_pin)
{
    uint8_t state = 0U;

    if ((input & a_pin) != 0U)
    {
        state |= 2U;
    }

    if ((input & b_pin) != 0U)
    {
        state |= 1U;
    }

    return state;
}

/*
 * 避免有符号整数溢出。累计计数到达边界后保持在边界，
 * 调用 Encoder_Reset() 后可重新开始计数。
 */
static void Encoder_AddSaturated(
    volatile int32_t *count,
    int32_t step)
{
    if (step > 0)
    {
        if (*count < INT32_MAX)
        {
            (*count)++;
        }
    }
    else if (step < 0)
    {
        if (*count > INT32_MIN)
        {
            (*count)--;
        }
    }
}

static uint32_t Encoder_EnterCritical(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    return primask;
}

static void Encoder_ExitCritical(uint32_t primask)
{
    if ((primask & 1U) == 0U)
    {
        __enable_irq();
    }
}

void Encoder_Init(void)
{
    uint32_t input;
    uint32_t primask;

    primask = Encoder_EnterCritical();

    g_leftCount       = 0;
    g_rightCount      = 0;
    g_leftSpeed       = 0;
    g_rightSpeed      = 0;
    g_lastLeftSample  = 0;
    g_lastRightSample = 0;

    input = DL_GPIO_readPins(GPIO_ENCODER_PORT, ENCODER_ALL_PINS);
    g_leftPreviousState = Encoder_MakeState(
        input,
        GPIO_ENCODER_LEFT_A_PIN,
        GPIO_ENCODER_LEFT_B_PIN);
    g_rightPreviousState = Encoder_MakeState(
        input,
        GPIO_ENCODER_RIGHT_A_PIN,
        GPIO_ENCODER_RIGHT_B_PIN);

    DL_GPIO_clearInterruptStatus(GPIO_ENCODER_PORT, ENCODER_ALL_PINS);
    NVIC_ClearPendingIRQ(GPIO_ENCODER_INT_IRQN);
    NVIC_EnableIRQ(GPIO_ENCODER_INT_IRQN);

    Encoder_ExitCritical(primask);
}

void Encoder_Reset(void)
{
    uint32_t primask = Encoder_EnterCritical();

    g_leftCount       = 0;
    g_rightCount      = 0;
    g_leftSpeed       = 0;
    g_rightSpeed      = 0;
    g_lastLeftSample  = 0;
    g_lastRightSample = 0;

    Encoder_ExitCritical(primask);
}

void Encoder_GetCount(Encoder_Value_t *count)
{
    uint32_t primask;

    if (count == 0)
    {
        return;
    }

    primask = Encoder_EnterCritical();
    count->left  = g_leftCount;
    count->right = g_rightCount;
    Encoder_ExitCritical(primask);
}

void Encoder_GetSpeed(Encoder_Value_t *speed)
{
    uint32_t primask;

    if (speed == 0)
    {
        return;
    }

    primask = Encoder_EnterCritical();
    speed->left  = g_leftSpeed;
    speed->right = g_rightSpeed;
    Encoder_ExitCritical(primask);
}

void Encoder_Sample(void)
{
    int32_t left;
    int32_t right;
    uint32_t primask = Encoder_EnterCritical();

    left  = g_leftCount;
    right = g_rightCount;

    g_leftSpeed  = left - g_lastLeftSample;
    g_rightSpeed = right - g_lastRightSample;

    g_lastLeftSample  = left;
    g_lastRightSample = right;

    Encoder_ExitCritical(primask);
}

/*
 * MSPM0G3507 的 GPIOA/GPIOB 共用 GROUP1 中断向量。
 */
void GROUP1_IRQHandler(void)
{
    uint32_t pending;
    uint32_t input;
    uint8_t current_state;
    uint8_t transition;
    int32_t step;

    pending = DL_GPIO_getEnabledInterruptStatus(
        GPIO_ENCODER_PORT,
        ENCODER_ALL_PINS);

    if (pending == 0U)
    {
        return;
    }

    /*
     * 先清除本次标志；随后到来的边沿会再次产生中断。
     */
    DL_GPIO_clearInterruptStatus(GPIO_ENCODER_PORT, pending);
    input = DL_GPIO_readPins(GPIO_ENCODER_PORT, ENCODER_ALL_PINS);

    if ((pending & ENCODER_LEFT_PINS) != 0U)
    {
        current_state = Encoder_MakeState(
            input,
            GPIO_ENCODER_LEFT_A_PIN,
            GPIO_ENCODER_LEFT_B_PIN);
        transition =
            (uint8_t)((g_leftPreviousState << 2U) | current_state);
        step =
            (int32_t)g_quadratureTable[transition] *
            ENCODER_LEFT_DIRECTION;
        Encoder_AddSaturated(&g_leftCount, step);
        g_leftPreviousState = current_state;
    }

    if ((pending & ENCODER_RIGHT_PINS) != 0U)
    {
        current_state = Encoder_MakeState(
            input,
            GPIO_ENCODER_RIGHT_A_PIN,
            GPIO_ENCODER_RIGHT_B_PIN);
        transition =
            (uint8_t)((g_rightPreviousState << 2U) | current_state);
        step =
            (int32_t)g_quadratureTable[transition] *
            ENCODER_RIGHT_DIRECTION;
        Encoder_AddSaturated(&g_rightCount, step);
        g_rightPreviousState = current_state;
    }
}
