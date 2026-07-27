#include "motor.h"
#include "ti_msp_dl_config.h"

#include <stdint.h>

/*
 * 将数值限制在指定范围内。
 */
static float Motor_Limit(float value, float minimum, float maximum)
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
 * 求绝对值。
 */
static float Motor_Abs(float value)
{
    if (value < 0.0f)
    {
        return -value;
    }

    return value;
}

/*
 * 把0～100%的占空比转换成PWM比较值。
 *
 * 这里按照常见的向下计数PWM方式计算：
 *
 * duty = 0%   -> compare接近load
 * duty = 100% -> compare接近0
 *
 * 后面如果发现占空比现象正好相反，
 * 再调整这个函数即可。
 */
static uint32_t Motor_DutyToCompare(float duty)
{
    uint32_t load_value;
    uint32_t compare_value;

    duty = Motor_Limit(duty, 0.0f, 100.0f);

    load_value = DL_TimerA_getLoadValue(PWM_MOTOR_INST);

    compare_value =
        load_value -
        (uint32_t)(((float)load_value * duty) / 100.0f);

    return compare_value;
}

/*
 * 设置左电机PWM占空比。
 */
static void Motor_SetLeftDuty(float duty)
{
    uint32_t compare_value;

    compare_value = Motor_DutyToCompare(duty);

    DL_TimerA_setCaptureCompareValue(
        PWM_MOTOR_INST,
        compare_value,
        GPIO_PWM_MOTOR_C0_IDX
    );
}

/*
 * 设置右电机PWM占空比。
 */
static void Motor_SetRightDuty(float duty)
{
    uint32_t compare_value;

    compare_value = Motor_DutyToCompare(duty);

    DL_TimerA_setCaptureCompareValue(
        PWM_MOTOR_INST,
        compare_value,
        GPIO_PWM_MOTOR_C1_IDX
    );
}

void Motor_Init(void)
{
    /*
     * 先关闭电机输出。
     */
    DL_GPIO_clearPins(
        GPIO_MOTOR_DIR_PORT,
        GPIO_MOTOR_DIR_AIN1_PIN |
        GPIO_MOTOR_DIR_AIN2_PIN |
        GPIO_MOTOR_DIR_BIN1_PIN |
        GPIO_MOTOR_DIR_BIN2_PIN
    );

    Motor_SetLeftDuty(0.0f);
    Motor_SetRightDuty(0.0f);

    /*
     * STBY拉高，退出待机状态。
     */
    DL_GPIO_setPins(
        GPIO_MOTOR_STBY_PORT,
        GPIO_MOTOR_STBY_STBY_PIN
    );

    /*
     * 启动TIMA0 PWM定时器。
     */
    DL_TimerA_startCounter(PWM_MOTOR_INST);
}

void Motor_SetLeft(float speed)
{
    float duty;

    speed = Motor_Limit(speed, -100.0f, 100.0f);
    duty  = Motor_Abs(speed);

    if (speed > 0.0f)
    {
        /*
         * 左电机正转：
         * AIN1 = 1
         * AIN2 = 0
         */
        DL_GPIO_setPins(
            GPIO_MOTOR_DIR_PORT,
            GPIO_MOTOR_DIR_AIN1_PIN
        );

        DL_GPIO_clearPins(
            GPIO_MOTOR_DIR_PORT,
            GPIO_MOTOR_DIR_AIN2_PIN
        );
    }
    else if (speed < 0.0f)
    {
        /*
         * 左电机反转：
         * AIN1 = 0
         * AIN2 = 1
         */
        DL_GPIO_clearPins(
            GPIO_MOTOR_DIR_PORT,
            GPIO_MOTOR_DIR_AIN1_PIN
        );

        DL_GPIO_setPins(
            GPIO_MOTOR_DIR_PORT,
            GPIO_MOTOR_DIR_AIN2_PIN
        );
    }
    else
    {
        /*
         * 左电机停止：
         * AIN1 = 0
         * AIN2 = 0
         */
        DL_GPIO_clearPins(
            GPIO_MOTOR_DIR_PORT,
            GPIO_MOTOR_DIR_AIN1_PIN |
            GPIO_MOTOR_DIR_AIN2_PIN
        );
    }

    Motor_SetLeftDuty(duty);
}

void Motor_SetRight(float speed)
{
    float duty;

    speed = Motor_Limit(speed, -100.0f, 100.0f);
    duty  = Motor_Abs(speed);

    if (speed > 0.0f)
    {
        /*
         * 右电机正转：
         * BIN1 = 1
         * BIN2 = 0
         */
        DL_GPIO_setPins(
            GPIO_MOTOR_DIR_PORT,
            GPIO_MOTOR_DIR_BIN1_PIN
        );

        DL_GPIO_clearPins(
            GPIO_MOTOR_DIR_PORT,
            GPIO_MOTOR_DIR_BIN2_PIN
        );
    }
    else if (speed < 0.0f)
    {
        /*
         * 右电机反转：
         * BIN1 = 0
         * BIN2 = 1
         */
        DL_GPIO_clearPins(
            GPIO_MOTOR_DIR_PORT,
            GPIO_MOTOR_DIR_BIN1_PIN
        );

        DL_GPIO_setPins(
            GPIO_MOTOR_DIR_PORT,
            GPIO_MOTOR_DIR_BIN2_PIN
        );
    }
    else
    {
        /*
         * 右电机停止：
         * BIN1 = 0
         * BIN2 = 0
         */
        DL_GPIO_clearPins(
            GPIO_MOTOR_DIR_PORT,
            GPIO_MOTOR_DIR_BIN1_PIN |
            GPIO_MOTOR_DIR_BIN2_PIN
        );
    }

    Motor_SetRightDuty(duty);
}

void Motor_SetBoth(float left_speed, float right_speed)
{
    Motor_SetLeft(left_speed);
    Motor_SetRight(right_speed);
}

void Motor_Stop(void)
{
    Motor_SetBoth(0.0f, 0.0f);
}