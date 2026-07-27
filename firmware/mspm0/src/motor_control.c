#include "motor_control.h"

#include <stdbool.h>

#include "board_config.h"
#include "hal.h"

static int16_t clamp_pwm(int16_t value)
{
    if (value > ROBOT_MAX_PWM) {
        return ROBOT_MAX_PWM;
    }
    if (value < -ROBOT_MAX_PWM) {
        return -ROBOT_MAX_PWM;
    }
    return value;
}

static void drive_one(hal_pwm_t pwm, hal_gpio_t in1, hal_gpio_t in2, int16_t command)
{
    int16_t duty = clamp_pwm(command);

    if (duty > 0) {
        hal_gpio_write(in1, true);
        hal_gpio_write(in2, false);
        hal_pwm_write(pwm, duty);
    } else if (duty < 0) {
        hal_gpio_write(in1, false);
        hal_gpio_write(in2, true);
        hal_pwm_write(pwm, (int16_t)-duty);
    } else {
        hal_pwm_write(pwm, 0);
        hal_gpio_write(in1, false);
        hal_gpio_write(in2, false);
    }
}

void motor_standby_enable(int enable)
{
    hal_gpio_write(HAL_GPIO_MOTOR_STBY, enable != 0);
}

void motor_init(void)
{
    motor_standby_enable(1);
    motor_stop();
}

void motor_apply(int16_t left_pwm, int16_t right_pwm)
{
    motor_standby_enable(1);
    drive_one(HAL_PWM_MOTOR_LEFT,
              HAL_GPIO_MOTOR_LEFT_IN1,
              HAL_GPIO_MOTOR_LEFT_IN2,
              left_pwm);
    drive_one(HAL_PWM_MOTOR_RIGHT,
              HAL_GPIO_MOTOR_RIGHT_IN1,
              HAL_GPIO_MOTOR_RIGHT_IN2,
              right_pwm);
}

void motor_stop(void)
{
    drive_one(HAL_PWM_MOTOR_LEFT,
              HAL_GPIO_MOTOR_LEFT_IN1,
              HAL_GPIO_MOTOR_LEFT_IN2,
              0);
    drive_one(HAL_PWM_MOTOR_RIGHT,
              HAL_GPIO_MOTOR_RIGHT_IN1,
              HAL_GPIO_MOTOR_RIGHT_IN2,
              0);
}
